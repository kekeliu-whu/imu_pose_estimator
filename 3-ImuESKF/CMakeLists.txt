cmake_minimum_required(VERSION 3.20)
project(imu_eskf)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        rosbag
        std_msgs
        tf)

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
        include
        src
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
        DEPENDS EIGEN3
        INCLUDE_DIRS include
)

add_executable(imu_eskf_demo main.cpp simple_estimator.cc ekf_estimator.cc ekf_estimator.h eskf_estimator.cc complementary_filter_estimator.cc)
target_link_libraries(imu_eskf_demo ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES} ${Protobuf_LIBRARIES})
