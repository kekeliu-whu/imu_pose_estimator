#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <thread>
#include <glog/logging.h>

using Quaterniond = Eigen::Quaterniond;
using Vec3d = Eigen::Vector3d;

namespace {
    bool g_init_ok = false;
    Quaterniond g_pose;
    double g_last_timestamp;
}

Quaterniond EstimatePose(double timestamp, const Vec3d &ang, const Vec3d &acc) {
    if (!g_init_ok) {
        g_pose = Quaterniond::Identity();
        g_init_ok = true;
        g_last_timestamp = timestamp;
        return g_pose;
    }
    double delta_t = timestamp - g_last_timestamp;
    g_pose *= Quaterniond(1, ang[0] * delta_t / 2, ang[1] * delta_t / 2, ang[2] * delta_t / 2);
    g_pose.normalize();

    g_last_timestamp = timestamp;
    return g_pose;
}

int main(int argc, char **argv) {
    // init ros
    ros::init(argc, argv, "imu_eskf");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("imu", 10);

    CHECK(argc == 2);

    rosbag::Bag bag;
    bag.open(argv[1]);  // BagMode is Read by default

    // todo change to timestamp gap
    ros::Rate loop_rate(200);
    for (rosbag::MessageInstance const m: rosbag::View(bag)) {
        const boost::shared_ptr<sensor_msgs::Imu> &imu_ptr = m.instantiate<sensor_msgs::Imu>();
        if (imu_ptr) {
            auto cur_timestamp = imu_ptr->header.stamp.toSec();
            auto time_interval = cur_timestamp - g_last_timestamp;

            auto pose = EstimatePose(
                    cur_timestamp,
                    {imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z},
                    Vec3d()
            );

            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header.frame_id = "base_link";
            poseStamped.pose.orientation.w = pose.x();
            poseStamped.pose.orientation.x = pose.x();
            poseStamped.pose.orientation.y = pose.y();
            poseStamped.pose.orientation.z = pose.z();
            chatter_pub.publish(poseStamped);

            std::this_thread::sleep_for(std::chrono::nanoseconds(int64_t(1000000000 * time_interval)));
        }
    }
    bag.close();

    return 0;
}
