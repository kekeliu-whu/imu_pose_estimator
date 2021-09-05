cmake_minimum_required(VERSION 3.15)
project(imu_eskf)

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-march=native")

find_package(
        catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        rosbag
        std_msgs
        tf
)

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
        include
        .
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

set(
        PROJECT_SRCS
        estimator/simple_estimator.cc
        estimator/ekf_estimator.cc
        estimator/eskf_estimator.cc
        estimator/complementary_filter_estimator.cc
)

add_executable(imu_eskf_demo main.cpp ${PROJECT_SRCS})
target_link_libraries(imu_eskf_demo ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES} ${Protobuf_LIBRARIES})

include(cmake/google-test.cmake)
google_enable_testing()
add_test_library_srcs(${PROJECT_SRCS})
