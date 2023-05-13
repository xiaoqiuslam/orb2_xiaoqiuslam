echo "Building ROS nodes"
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/q/ORB_SLAM2/Examples/ROS/ORB_SLAM2
cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
