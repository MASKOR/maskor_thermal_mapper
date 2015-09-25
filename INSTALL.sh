#!/bin/bash

echo "installing rtabmap standalone libraries to catkin_ws/devel"
cd  ~/rtabmap/build
cmake -DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel ..
make -j8
make install

echo "running catkin_make in catkin_ws"
cd ~/catkin_ws
catkin_make

echo "...done!"
