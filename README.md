# maskor_thermal_mapper
registeres thermal data with asus xtion depth image

How to install / use with rtabmap for thermal 3D mapping:

- install openni2 for ASUS Xtion

- clone  maskor/maskor_gige_cam to ~/catkin_ws/src and follow the install instructions inside the package
if you want to use a FLIRA325sc for example

- clone  maskor/rtabmap/fnicolai/thermal_mapping to ~/rtabmap

- clone  maskor/rtabmap_ros/fnicolai/thermal_mapping to ~/catkin_ws/src

- clone  maskor/maskor_thermal_mapper to ~/catkin_ws/src 

- run the INSTALL.sh in ~/catkin_ws/src/maskor_thermal_mapper

- launch the maskor_thermal_mapper.launch launchfile to generate thermal 3D maps

