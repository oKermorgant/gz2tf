#include <rclcpp/rclcpp.hpp>
#include <ros_gz_world_bridge/world_bridge.h>
#include <rclcpp/executors/single_threaded_executor.hpp>

using namespace ros_gz_world_bridge;

// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);  

  auto bridge = std::make_shared<ros_gz_world_bridge::WorldBridge>();
  const auto &[urdf, links, world] = bridge->parseWorld(); {}

  if(!urdf)
  {
    RCLCPP_ERROR(bridge->get_logger(), "Could not parse world, exiting");
    return 0;
  }

  bridge->setupPoseBridge(links, world);
  auto rsp_node{bridge->initRSP(urdf)};

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(bridge);
  exec.add_node(rsp_node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
