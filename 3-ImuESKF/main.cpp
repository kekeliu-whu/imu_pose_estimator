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
#include <thread>

int main(int argc, char **argv) {
  // init ros
  ros::init(argc, argv, "imu_eskf");
  ros::NodeHandle n;
  ros::Publisher chatter_pub =
      n.advertise<geometry_msgs::PoseStamped>("imu", 10);

  CHECK(argc == 2);

  rosbag::Bag bag;
  bag.open(argv[1]); // BagMode is Read by default

  std::unique_ptr<Estimator> estimator = std::make_unique<ComplementaryFilterEstimator>();

  for (const auto &m : rosbag::View(bag)) {
    const boost::shared_ptr<sensor_msgs::Imu> &imu_ptr =
        m.instantiate<sensor_msgs::Imu>();
    if (imu_ptr) {
      auto cur_timestamp = imu_ptr->header.stamp.toSec();
      auto time_interval = estimator->get_init_ok()
                               ? cur_timestamp - estimator->get_last_timestamp()
                               : 0;

      auto pose = estimator->EstimatePose(
          cur_timestamp,
          {imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y,
           imu_ptr->angular_velocity.z},
          {imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y,
           imu_ptr->linear_acceleration.z});

      geometry_msgs::PoseStamped poseStamped;
      poseStamped.header.frame_id = "base_link";
      poseStamped.pose.orientation.w = pose.x();
      poseStamped.pose.orientation.x = pose.x();
      poseStamped.pose.orientation.y = pose.y();
      poseStamped.pose.orientation.z = pose.z();
      chatter_pub.publish(poseStamped);

      std::this_thread::sleep_for(
          std::chrono::nanoseconds(int64_t(1000000000 * time_interval)));
    }
  }
  bag.close();

  return 0;
}
