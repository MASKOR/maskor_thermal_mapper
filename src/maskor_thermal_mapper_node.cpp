/**
 * @maskor_thermal_mapper_node.cpp
 * @author  Fabian Nicolai  <fabian.nicolai@alumni.fh-aachen.de>
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * https://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * This code maps thermal and depth images and publishes
 * the result as mapped image.
 * Flir A325sc and Asus XTion were used as sources in our work.
 */

#include <maskor_thermal_mapper/maskor_thermal_mapper_node.h>

MappingNode::MappingNode()
    : it_(nh_), depth_image_sub_( it_, "/camera/depth/image_raw", 1 ),
      thermal_image_sub_( it_, "/flir_camera/temperature_image" , 1 ), sync( MySyncPolicy( 10 ), depth_image_sub_,  thermal_image_sub_)
{
    // ###########################################################################################
    // ##                                                                                       ##
    // ## /flir_camera/image_raw            => 16UC1, RAW, contains Kelvin, unsigend short      ##
    // ##                                                                                       ##
    // ## /flir_camera/temperature_image    => 32FC1, RAW, contains Celsius, float              ##
    // ##                                                                                       ##
    // ## /flir_camera/thermal_image_raw    => BRG8, Colored, uchar                             ##
    // ##                                                                                       ##
    // ###########################################################################################

    // Read the params from the launch file
    ros::NodeHandle privateNH("~");
    privateNH.param("package_name", package_name_, std::string("maskor_thermal_mapper"));
    privateNH.param("database_path", database_path_, std::string("/database"));
    privateNH.param("yaml_path", yaml_path_, std::string("/full_flir_ir_stereo_calib.yaml"));
    privateNH.param("publish_topic", publish_topic_, std::string("flir_camera/mapped/image"));
    privateNH.param("enable_colormap", enable_colormap_, bool(true));

    // Path to package
    pkg_path = ros::package::getPath(package_name_);
    std::cout << "Open package = " << pkg_path << std::endl;

    // Path to yaml file store
    DATABASE = pkg_path + database_path_;
    std::cout << "Open database = " << DATABASE << std::endl;

    calibration_filename = DATABASE + yaml_path_;
    std::cout << "Open calibrationfile = " << calibration_filename << std::endl;

    // Publisher
    ThermalImagePub_ = it_.advertise(publish_topic_,1);

    // Synchronised callback
    sync.registerCallback( boost::bind( &MappingNode::callback, this, _1, _2 ) );
}

MappingNode::~MappingNode()
{

}

void MappingNode::callback(const sensor_msgs::ImageConstPtr& depth_image_msg,
                           const sensor_msgs::ImageConstPtr& thermal_image_msg)
{

    cv_bridge::CvImagePtr cv_depth_image_ptr, cv_thermal_image_ptr;

    try{
        cv_depth_image_ptr   = cv_bridge::toCvCopy(depth_image_msg, "16UC1");
        cv_thermal_image_ptr = cv_bridge::toCvCopy(thermal_image_msg, "32FC1"); //16UC1 for thermal raw, bgr8 for colored
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depth_image, thermal_image;
    depth_image   = cv_depth_image_ptr->image.clone();
    thermal_image = cv_thermal_image_ptr->image.clone();

    cv::Mat depth_image_float = cv::Mat::zeros(depth_image.size(), depth_image.type());

    depth_image(cv::Rect(0,0,depth_image.cols-4,depth_image.rows-4)).copyTo(depth_image_float(cv::Rect(4,4,depth_image.cols-4,depth_image.rows-4)));

    depth_image = depth_image_float.clone();

    cv::Mat thermalImageCast = cv::Mat::ones(thermal_image.rows,thermal_image.cols, CV_8UC1);

    float newMinCelsius = 20.0;
    float newMaxCelsius = 40.0;

    for (int y = 0; y < thermal_image.rows; y++)
    {
        for (int x = 0; x < thermal_image.cols; x++)
        {
            if(thermal_image.at<float>(y,x) <= newMinCelsius){
                thermalImageCast.at<unsigned char>(y,x) = 0;
            }else if (thermal_image.at<float>(y,x) >= newMaxCelsius){
                thermalImageCast.at<unsigned char>(y,x) = 255;
            }else{

                thermalImageCast.at<unsigned char>(y,x) = (thermal_image.at<float>(y,x) - newMinCelsius) / ( (newMaxCelsius - newMinCelsius) / 255.) ;
            }
        }
    }

    thermal_image = thermalImageCast;

    //==============================================================
    // If you wish, you can apply a colormap here
    //==============================================================
    if(enable_colormap_){
        cv::applyColorMap(thermal_image, thermal_image, cv::COLORMAP_JET);
    }

    //cv::imshow("thermal_image colored", thermal_image);

    resizeImg(thermal_image, thermal_image, 640);


    cv::Mat depth_image_rect, thermal_image_rect;
    mapAndPublish(thermal_image, depth_image, thermal_image_rect, depth_image_rect);

    cv::waitKey(30);

}

void MappingNode::mapAndPublish(cv::Mat &thermal_image, cv::Mat &depth_image, cv::Mat &thermal_image_rect, cv::Mat &depth_image_rect){

    //==============================================================
    // Get Calibration Params
    //
    // In this setup:
    // l_camera = Flir
    // r_camera = Xtion
    //==============================================================
    cv::FileStorage fs(calibration_filename, cv::FileStorage::READ);
    fs["map_l1"] >> map_l1;
    fs["map_l2"] >> map_l2;
    fs["map_r1"] >> map_r1;
    fs["map_r2"] >> map_r2;
    fs["l_camera_matrix"] >> l_camera_matrix;
    fs["l_distortion_coefficients"] >> l_distortion_coefficients;
    fs["r_camera_matrix"] >> r_camera_matrix;
    fs["r_distortion_coefficients"] >> r_distortion_coefficients;
    fs["R"] >> R;
    fs["T"] >> t;


    //==============================================================
    // Rectifiy images
    //==============================================================
    cv::remap( thermal_image, thermal_image_rect, map_l1, map_l2, cv::INTER_LINEAR);
    cv::remap( depth_image, depth_image_rect, map_r1, map_r2, cv::INTER_LINEAR);

    //==============================================================
    // Undistort images
    //==============================================================
    cv::Mat thermal_image_undist, depth_image_undist;
    cv::undistort(thermal_image, thermal_image_undist, l_camera_matrix, l_distortion_coefficients);
    cv::undistort(depth_image, depth_image_undist, r_camera_matrix, r_distortion_coefficients);

    //... undistortet depth
    cv::Mat orig_depth_image = depth_image_undist.clone(); // Orig is needed for depth in mm

    cv::medianBlur(orig_depth_image, orig_depth_image, 3);

    cv::normalize(depth_image_undist, depth_image_undist, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    //cv::Mat threeDPoints;

    //==============================================================
    // Transform R and t
    //==============================================================
    cv::transpose(R,R);
    t = R * t * (-1);

    // // ----- Copy R (3x3) and t(1x3) in T (4x4)
    cv::Mat T(4, 4, CV_64FC1, cv::Scalar(0,0,0));
    //std::cout << T.rows << " " << T.cols << std::endl;
    for (int y = 0; y < R.rows; y++)
    {
        for (int x = 0; x < R.cols; x++)
        {
            T.at<double>(y,x) = R.at<double>(y,x);
        }
    }
    for (int y = 0; y < t.rows; y++)
    {
        for (int x = 0; x < t.cols; x++)
        {
            T.at<double>(y,x+3) = t.at<double>(y,x);
        }
    }
    T.at<double>(3,3) = 1.0;

    //==============================================================
    // Invert depth camera matrix for dist-1
    //==============================================================
    cv::Mat inv_r_camera_matrix = r_camera_matrix.inv();


    //==============================================================
    // Calc 3D world points from 2D depth image raw
    //==============================================================
    std::vector<cv::Point3f> vectorThreeDPoints_right;
    for (int y = 0; y < depth_image.rows; y++)
    {
        for (int x = 0; x < depth_image.cols; x++)
        {
            cv::Point3f hom_depth_point = cv::Point3f(x, y, 1);

            cv::Point3f multiplied_point;
            multiplied_point.x = hom_depth_point.x * inv_r_camera_matrix.at<double>(0,0) + hom_depth_point.y * inv_r_camera_matrix.at<double>(0,1) + hom_depth_point.z * inv_r_camera_matrix.at<double>(0,2) ;
            multiplied_point.y = hom_depth_point.x * inv_r_camera_matrix.at<double>(1,0) + hom_depth_point.y * inv_r_camera_matrix.at<double>(1,1) + hom_depth_point.z * inv_r_camera_matrix.at<double>(1,2) ;
            multiplied_point.z = hom_depth_point.x * inv_r_camera_matrix.at<double>(2,0) + hom_depth_point.y * inv_r_camera_matrix.at<double>(2,1) + hom_depth_point.z * inv_r_camera_matrix.at<double>(2,2) ;

            double r_two = multiplied_point.x * multiplied_point.x + multiplied_point.y * multiplied_point.y;
            double r_four = r_two * r_two;
            double r_six = r_two * r_four;

            double k_one   = r_distortion_coefficients.at<double>(0,0);
            double k_two   = r_distortion_coefficients.at<double>(0,1);
            double k_three = r_distortion_coefficients.at<double>(0,2);
            double k_four  = r_distortion_coefficients.at<double>(0,3);
            double k_five  = r_distortion_coefficients.at<double>(0,4);

            double left_skalar = 1 + k_one * r_two +  k_two * r_four + k_five * r_six;

            cv::Point3f left_part;
            left_part.x = multiplied_point.x * left_skalar;
            left_part.y = multiplied_point.y * left_skalar;
            left_part.z = multiplied_point.z * left_skalar * 0;

            cv::Point3f right_part;

            right_part.x = 2 * k_three * multiplied_point.x * multiplied_point.y + k_four * (r_two + 2 * (multiplied_point.x * multiplied_point.x) );
            right_part.y = 2 * k_four * multiplied_point.x * multiplied_point.y + k_three * (r_two + 2 * (multiplied_point.y * multiplied_point.y) );
            right_part.z = 1;

            cv::Point3f combined;

            combined.x = (left_part.x + right_part.x) * 1;
            combined.y = (left_part.y + right_part.y) * 1;
            combined.z = (left_part.z + right_part.z) * 1;

            float depth_mm = (float)orig_depth_image.at<short int>(y,x) /1000;
            vectorThreeDPoints_right.push_back( cv::Point3f(combined.x * depth_mm, combined.y * depth_mm, combined.z * depth_mm) );
        }
    }

    //==============================================================
    // Calc 2D Points in thermal_image from 3D world points
    //==============================================================
    std::vector<cv::Point2f> twoDPoints;
    cv::projectPoints(vectorThreeDPoints_right, R, t, l_camera_matrix, l_distortion_coefficients, twoDPoints);


    //==============================================================
    // Mapp the image
    //==============================================================
    int vectorPointer = 0;
    cv::Mat mapped_image;

    if(thermal_image_undist.type() == CV_8UC1){

        //====================
        //       MONO
        //====================
        mapped_image = cv::Mat::zeros(thermal_image.rows,thermal_image.cols, CV_8UC1);


        for (int i = 0; i < depth_image.rows; i++)
        {
            for (int j = 0; j < depth_image.cols; j++)
            {
                if(twoDPoints[vectorPointer].x >= 0 && twoDPoints[vectorPointer].x < depth_image.cols && twoDPoints[vectorPointer].y >= 0 && twoDPoints[vectorPointer].y < depth_image.rows){
                    mapped_image.at<uchar>(i,j) = thermal_image_undist.at<uchar>((int)twoDPoints[vectorPointer].y, (int)twoDPoints[vectorPointer].x); //depth_image.at<uchar>(i,j);
                }
                vectorPointer = vectorPointer + 1;
            }
        }
    }else if (thermal_image_undist.type() == CV_8UC3) {

        //====================
        //     RGB/BGR
        //====================
        mapped_image = cv::Mat::zeros(thermal_image.rows,thermal_image.cols, CV_8UC3);

        for (int i = 0; i < depth_image.rows; i++)
        {
            for (int j = 0; j < depth_image.cols; j++)
            {
                int pixelX, pixelY;
                //Is pixel inside image?
                if(     twoDPoints[vectorPointer].x >= 0 && twoDPoints[vectorPointer].x < depth_image.cols &&
                        twoDPoints[vectorPointer].y >= 0 && twoDPoints[vectorPointer].y < depth_image.rows){

                    pixelX = (int)twoDPoints[vectorPointer].x;
                    pixelY = (int)twoDPoints[vectorPointer].y;

                    mapped_image.at<cv::Vec3b>(i,j)[0] = thermal_image_undist.at<cv::Vec3b>(pixelY, pixelX)[0];
                    mapped_image.at<cv::Vec3b>(i,j)[1] = thermal_image_undist.at<cv::Vec3b>(pixelY, pixelX)[1];
                    mapped_image.at<cv::Vec3b>(i,j)[2] = thermal_image_undist.at<cv::Vec3b>(pixelY, pixelX)[2];
                }

                vectorPointer = vectorPointer + 1;
            }
        }
    }

    //==============================================================
    // Publish the mapped image
    //==============================================================
    publishThermalImage(mapped_image);

}

void MappingNode::publishThermalImage(cv::Mat thermal_image){

    //==============================================================
    // Publish resized raw thermal image
    //==============================================================
    cv_bridge::CvImage thermal_image_out_msg;
    thermal_image_out_msg.header          = std_msgs::Header(); // New Header
    thermal_image_out_msg.header.stamp    = ros::Time::now();
    thermal_image_out_msg.image           = thermal_image; //output_im;//img;//cv_ptr->image;//output_img;

    if(thermal_image.type() == CV_8UC1){
        thermal_image_out_msg.encoding        = sensor_msgs::image_encodings::MONO8; // Or whatever
    }else if (thermal_image.type() == CV_8UC3){
        thermal_image_out_msg.encoding        = sensor_msgs::image_encodings::BGR8;
    }

    ThermalImagePub_.publish(thermal_image_out_msg.toImageMsg());

}

void MappingNode::resizeImg(cv::Mat &src, cv::Mat &dest, int newsize){

    double factor = std::min(newsize/(double)src.cols,newsize/(double)src.rows);
    cv::resize(src, dest,cv::Size(src.cols*factor,src.rows*factor));
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "three_dimensional_thermal_image_mapping_node");
    MappingNode mn;

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
