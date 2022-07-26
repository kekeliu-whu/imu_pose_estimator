#include "estimator/complementary_filter_estimator.h"
#include "estimator/ekf_estimator.h"
#include "estimator/simple_estimator.h"

#include <geometry_msgs/PoseStamped.h>
#include <glog/logging.h>
#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>

int main(int argc, char **argv) {
  // init ros
  ros::init(argc, argv, "imu_eskf");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("imu", 10);
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  CHECK(argc == 2);

  rosbag::Bag bag;
  bag.open(argv[1]); // BagMode is Read by default

  std::unique_ptr<Estimator> estimator = std::make_unique<ComplementaryFilterEstimator>();

  for (const auto &m : rosbag::View(bag)) {
    const boost::shared_ptr<sensor_msgs::Imu> &imu_ptr = m.instantiate<sensor_msgs::Imu>();
    if (imu_ptr) {
      auto cur_timestamp = imu_ptr->header.stamp.toSec();
      auto time_interval = estimator->get_init_ok() ? cur_timestamp - estimator->get_last_timestamp() : 0;

      auto pose = estimator->EstimatePose(
          cur_timestamp, {imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z},
          {imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z});

      static_transformStamped.header = imu_ptr->header;
      static_transformStamped.header.frame_id = "map";
      static_transformStamped.child_frame_id = "base_link";
      tf2::Quaternion quat(pose.x(), pose.y(), pose.z(), pose.w());
      static_transformStamped.transform.rotation.x = quat.x();
      static_transformStamped.transform.rotation.y = quat.y();
      static_transformStamped.transform.rotation.z = quat.z();
      static_transformStamped.transform.rotation.w = quat.w();
      static_broadcaster.sendTransform(static_transformStamped);
      ros::spinOnce();

      std::this_thread::sleep_for(std::chrono::nanoseconds(int64_t(1000000000 * time_interval)));
    }
  }
  bag.close();

  return 0;
}
