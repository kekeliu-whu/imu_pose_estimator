#include "types.grpc.pb.h"
#include "types.pb.h"

#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <signal.h>

using com::example::imucollector::proto::Greeter;
using com::example::imucollector::proto::ImuData;
using google::protobuf::Empty;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

// Logic and data behind the server's behavior.
class GreeterServiceImpl final : public Greeter::Service {
public:
  GreeterServiceImpl(ros::Publisher &publisher) : publisher(publisher) {}

  Status SendImuData(ServerContext *context, const ImuData *request, Empty *reply) override {

    sensor_msgs::Imu msg;
    msg.header.stamp.sec = request->timestamp().seconds();
    msg.header.stamp.nsec = request->timestamp().nanos();
    msg.header.seq = request->seq();
    msg.angular_velocity.x = request->angular_velocity(0);
    msg.angular_velocity.y = request->angular_velocity(1);
    msg.angular_velocity.z = request->angular_velocity(2);
    msg.linear_acceleration.x = request->linear_acceleration(0);
    msg.linear_acceleration.y = request->linear_acceleration(1);
    msg.linear_acceleration.z = request->linear_acceleration(2);
    publisher.publish(msg);

    return Status::OK;
  }

private:
  ros::Publisher &publisher;
};

void RunServer(int argc, char **argv) {
  ros::init(argc, argv, "android_ros_msg_bridge_node");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::Imu>("/android/imu", 10);

  std::string server_address("0.0.0.0:32345");
  GreeterServiceImpl service{chatter_pub};

  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();
  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;

  ros::spin();
}

int main(int argc, char **argv) {
  RunServer(argc, argv);

  return 0;
}