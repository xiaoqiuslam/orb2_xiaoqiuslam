cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(CMAKE_BUILD_TYPE debug)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -std=c++11")

find_package(OpenCV QUIET)
if(OpenCV_FOUND)
   message(STATUS "Found OpenCV version " "${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}.${OpenCV_VERSION_PATCH}")
else()
   message(FATAL_ERROR "OpenCV not found. Please install OpenCV first.")
endif()

find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    message(STATUS "Found Eigen3 version ${Eigen3_VERSION}")
else()
    message(FATAL_ERROR "Eigen3 not found")
endif()

find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
)

message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
message("EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
message("Pangolin_INCLUDE_DIRS: ${Pangolin_INCLUDE_DIRS}")

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
)

# Node for monocular camera
rosbuild_add_executable(Mono src/ros_mono.cc)
target_link_libraries(Mono ${LIBS})

# Node for monocular camera (Augmented Reality Demo)
rosbuild_add_executable(MonoAR src/AR/ros_mono_ar.cc src/AR/ViewerAR.h src/AR/ViewerAR.cc)
target_link_libraries(MonoAR ${LIBS})

# Node for stereo camera
rosbuild_add_executable(Stereo src/ros_stereo.cc)
target_link_libraries(Stereo ${LIBS})

# Node for RGB-D camera
rosbuild_add_executable(RGBD src/ros_rgbd.cc)
target_link_libraries(RGBD ${LIBS})