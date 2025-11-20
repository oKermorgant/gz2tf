#ifndef GZ2TF_WORLD_BRIDGE
#define GZ2TF_WORLD_BRIDGE

#include <rclcpp/node.hpp>
#include <robot_state_publisher/robot_state_publisher.hpp>
#include <urdf_parser/urdf_parser.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <gz2tf/sdf_parser.h>
#include <gz2tf/urdf_export.h>

#ifdef GZ_FORTRESS
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/transport/Node.hh>
#else
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#endif

namespace gz2tf
{

class WorldBridge : public rclcpp::Node
{
private:
  std::string topic_ns{"/gz_world_bridge"};

public:

  WorldBridge() : rclcpp::Node("world_bridge") {}

  DynamicModel parseWorld()
  {
    sdf::setFindCallback([&](const std::string &path) -> std::string
                         {
                           return sdf_paths::resolveURI(path);
                         });

    auto filename = declare_parameter<std::string>("file", "");
    auto world = declare_parameter<std::string>("world", "");
    const auto ignored = declare_parameter<std::vector<std::string>>("ignored", {"waves"});
    const auto only_static = declare_parameter("only_static", false);

    topic_ns = declare_parameter<std::string>("namespace", topic_ns);

    if(filename.empty() && world.empty())
    {
      RCLCPP_WARN(get_logger(), "No filename and no sdf given, getting current world");
      // get current world
      filename = "/tmp/gz_world.sdf";
      std::system("ros2 run simple_launch generate_gz_world /tmp/gz_world.sdf");
    }

    if(!filename.empty())
      world = sdf_paths::readWorld(filename);

    return parseWorldSDF(world, ignored, only_static);
  }

  robot_state_publisher::RobotStatePublisher::SharedPtr
  initRSP(urdf::ModelInterfaceConstSharedPtr model)
  {
    const auto doc{gz2tf::exportURDF(*model)};
    tinyxml2::XMLPrinter printer;
    doc->Accept(&printer);

    auto rsp_arg{rclcpp::NodeOptions()
                     .arguments({"--ros-args", "-r", "__ns:=" + topic_ns,
                                 "-p", std::string("robot_description:=") + printer.CStr()})};

    return std::make_shared<robot_state_publisher::RobotStatePublisher>(rsp_arg);
  }

  void setupPoseBridge(const LinkMap &links, const std::string &world)
  {
    const auto use_tf = declare_parameter<bool>("use_tf", true);
    static gz::transport::Node gz_node;

    std::function<void(const gz::msgs::Pose_V &)> sub_cb;

    if(use_tf)
    {
      static tf2_ros::TransformBroadcaster br(this);
      using TransformStamped = geometry_msgs::msg::TransformStamped;
      static TransformStamped tr;
      tr.header.frame_id = "world";

      sub_cb = [&](const gz::msgs::Pose_V &msg)
      {
        tr.header.stamp = get_clock()->now();
        for(int i = 0; i < msg.pose_size(); ++i)
        {
          const auto &gz_pose{msg.pose(i)};
          const auto &link{links.find(gz_pose.name())};
          if(link == links.end())
            continue;

          tr.child_frame_id = link->second;
          tr.transform.translation.x = gz_pose.position().x();
          tr.transform.translation.y = gz_pose.position().y();
          tr.transform.translation.z = gz_pose.position().z();
          tr.transform.rotation.x = gz_pose.orientation().x();
          tr.transform.rotation.y = gz_pose.orientation().y();
          tr.transform.rotation.z = gz_pose.orientation().z();
          tr.transform.rotation.w = gz_pose.orientation().w();
          br.sendTransform(tr);
        }
      };
    }
    else
    {
      using PoseStamped = geometry_msgs::msg::PoseStamped;
      static auto pose_pub = create_publisher<PoseStamped>(topic_ns + "/poses", 1);
      static PoseStamped pose;
      sub_cb = [&](const gz::msgs::Pose_V &msg)
      {
        pose.header.stamp = get_clock()->now();
        for(int i = 0; i < msg.pose_size(); ++i)
        {
          const auto &gz_pose{msg.pose(i)};
          const auto &link{links.find(gz_pose.name())};
          if(link == links.end())
            continue;

          pose.header.frame_id = link->second;
          pose.pose.position.x = gz_pose.position().x();
          pose.pose.position.y = gz_pose.position().y();
          pose.pose.position.z = gz_pose.position().z();
          pose.pose.orientation.x = gz_pose.orientation().x();
          pose.pose.orientation.y = gz_pose.orientation().y();
          pose.pose.orientation.z = gz_pose.orientation().z();
          pose.pose.orientation.w = gz_pose.orientation().w();
          pose_pub->publish(pose);
        }
      };
    }

    gz_node.Subscribe("/world/" + world + "/dynamic_pose/info", sub_cb);
  }


};

}

#endif
