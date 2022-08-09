#include "estimator/complementary_filter_estimator.h"
#include "estimator/ekf_estimator.h"
#include "estimator/eskf_estimator.h"
#include "estimator/simple_estimator.h"

#include <geometry_msgs/PoseStamped.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>

DEFINE_string(filter_type, "", "Type of the filter, value must be one of sf/ekf/eskf/cf.");

std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;
std::unique_ptr<Estimator> estimator;

void InitEstimatorOrExit() {
  if (FLAGS_filter_type == "sf") {
    estimator = std::make_unique<SimpleEstimator>();
  } else if (FLAGS_filter_type == "ekf") {
    estimator = std::make_unique<EkfEstimator>();
  } else if (FLAGS_filter_type == "eskf") {
    estimator = std::make_unique<EskfEstimator>();
  } else if (FLAGS_filter_type == "cf") {
    estimator = std::make_unique<ComplementaryFilterEstimator>();
  } else {
    std::cout << "Filter type '" << FLAGS_filter_type << "' is not one of sf/ekf/eskf/cf." << std::endl;
    exit(1);
  }
}

void chatterCallback(const sensor_msgs::Imu::ConstPtr &imu_ptr) {
  auto cur_timestamp = imu_ptr->header.stamp.toSec();

  // clang-format off
  auto pose = estimator->EstimatePose(
      cur_timestamp,
      {imu_ptr->angular_velocity.x,    imu_ptr->angular_velocity.y,    imu_ptr->angular_velocity.z},
      {imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z}
    );
  // clang-format on

  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header = imu_ptr->header;
  static_transformStamped.header.frame_id = "map";
  static_transformStamped.child_frame_id = "base_link";
  tf2::Quaternion quat(pose.x(), pose.y(), pose.z(), pose.w());
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster->sendTransform(static_transformStamped);
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  InitEstimatorOrExit();

  ros::init(argc, argv, "imu_eskf");
  ros::NodeHandle n;
  static_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
  ros::Subscriber sub = n.subscribe("/android/imu", 100, chatterCallback);

  ros::spin();

  return 0;
}
