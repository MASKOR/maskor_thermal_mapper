/**
 * @maskor_thermal_mapper_node.h
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

#ifndef _MASKOR_THERMAL_MAPPER_H_
#define _MASKOR_THERMAL_MAPPER_NODE_H_

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <fstream>
#include <math.h>


#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"

class MappingNode
{
public:
  MappingNode();
  ~MappingNode();

private:
    
  //ROS
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
    
  //ros::Subscriber
  typedef image_transport::SubscriberFilter ImageSubscriber;

  ImageSubscriber depth_image_sub_;
  ImageSubscriber thermal_image_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

  message_filters::Synchronizer< MySyncPolicy > sync;

  //ros::Publisher
  image_transport::Publisher ThermalImagePub_;
    
  //internal helper functions
  void callback(
      const sensor_msgs::ImageConstPtr& depth_image_msg,
      const sensor_msgs::ImageConstPtr& thermal_image_msg);
  void mapAndPublish(cv::Mat &thermal_image, cv::Mat &depth_image, cv::Mat &thermal_image_rect, cv::Mat &depth_image_rect);
  void publishThermalImage(cv::Mat thermal_image);

  //internal CV helper functions
  void resizeImg(cv::Mat &src, cv::Mat &dest, int newsize);

  //internal CV helper variables
  cv::Mat map_l1, map_l2, map_r1, map_r2, RTS_map, F, l_camera_matrix, r_camera_matrix, l_distortion_coefficients, r_distortion_coefficients, Q, R, t;

  //Path to package
  std::string pkg_path;
  
  //Path to yaml file store
  std::string DATABASE;

  //Path to calibration file
  std::string calibration_filename;

  // Param variable
  std::string package_name_;
  std::string database_path_;
  std::string yaml_path_;
  std::string publish_topic_;

  bool enable_colormap_;

};

#endif //MASKOR_THERMAL_MAPPER_NODE_H
