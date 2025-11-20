#include <rclcpp/rclcpp.hpp>
#include <gz2tf/world_bridge.h>
#include <rclcpp/executors/single_threaded_executor.hpp>

using namespace gz2tf;

// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);  

  auto bridge = std::make_shared<gz2tf::WorldBridge>();
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