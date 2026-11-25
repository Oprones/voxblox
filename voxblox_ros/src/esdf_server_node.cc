#include "voxblox_ros/esdf_server.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  auto node = std::make_shared<rclcpp::Node>("esdf_server");
  voxblox::EsdfServer server(node.get());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
