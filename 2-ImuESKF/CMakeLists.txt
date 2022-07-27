cmake_minimum_required(VERSION 3.10)
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

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(
        include
        .
        src
        build
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
        DEPENDS EIGEN3
)

set(
        PROJECT_SRCS
        estimator/simple_estimator.cc
        estimator/ekf_estimator.cc
        estimator/eskf_estimator.cc
        estimator/complementary_filter_estimator.cc
)

if (Protobuf_VERBOSE)
    message(STATUS "Using Protocol Buffers ${protobuf_VERSION}")
endif ()

# protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/types.proto)

get_filename_component(hw_proto "./proto/types.proto" ABSOLUTE)
get_filename_component(hw_proto_path "${hw_proto}" PATH)

# Generated sources
set(hw_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/types.pb.cc")
set(hw_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/types.pb.h")
set(hw_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/types.grpc.pb.cc")
set(hw_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/types.grpc.pb.h")
add_custom_command(
      OUTPUT "${hw_proto_srcs}" "${hw_proto_hdrs}" "${hw_grpc_srcs}" "${hw_grpc_hdrs}"
      COMMAND ${Protobuf_PROTOC_EXECUTABLE}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${hw_proto_path}"
        --plugin=protoc-gen-grpc="${CONAN_BIN_DIRS_GRPC}/grpc_cpp_plugin"
        "${hw_proto}"
      DEPENDS "${hw_proto}")

add_executable(imu_eskf_node main.cpp ${PROJECT_SRCS})
target_link_libraries(imu_eskf_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES})

add_executable(android_ros_msg_bridge_node android_ros_msg_bridge.cc ${PROJECT_SRCS} ${PROTO_SRCS} ${hw_proto_srcs} ${hw_grpc_srcs})
target_link_libraries(android_ros_msg_bridge_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES} ${CONAN_LIBS})

include(cmake/google-test.cmake)
add_test_library_srcs(${PROJECT_SRCS})
enable_automatic_test_and_benchmark()
