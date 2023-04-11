#ifndef ROS_GZ_WORLD_BRIDGE
#define ROS_GZ_WORLD_BRIDGE

#include <rclcpp/node.hpp>
#include <robot_state_publisher/robot_state_publisher.hpp>
#include <ros_gz_world_bridge/sdf_parser.h>
#include <urdf_parser/urdf_parser.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <ignition/msgs/pose_v.pb.h>
#include <ignition/transport/Node.hh>

namespace ros_gz_world_bridge
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
      return gz::resolveURI(path);
    });

    const auto filename = declare_parameter<std::string>("file", "sydney_regatta.sdf");
    auto world = declare_parameter<std::string>("world", "");
    const auto ignored = declare_parameter<std::vector<std::string>>("ignored", {"waves"});
    const auto use_static = declare_parameter("use_static", true);

    topic_ns = declare_parameter<std::string>("namespace", topic_ns);

    if(filename.empty() && world.empty())
    {
      RCLCPP_ERROR(get_logger(), "No filename and no sdf given, exiting");
      return {};
    }

    if(!filename.empty())
      world = gz::readWorld(filename);

    return parseWorldSDF(world, ignored, use_static);
  }

  robot_state_publisher::RobotStatePublisher::SharedPtr
  initRSP(urdf::ModelInterfaceConstSharedPtr model)
  {
    const auto doc{my_urdf_export::exportURDF(*model)};
    TiXmlPrinter printer;
    doc->Accept(&printer);

    auto rsp_arg{rclcpp::NodeOptions()
          .arguments({"--ros-args", "-r", "__ns:=" + topic_ns, "-p", "robot_description:=" + printer.Str()})};

    return std::make_shared<robot_state_publisher::RobotStatePublisher>(rsp_arg);
  }

  void setupPoseBridge(const LinkMap &links, const std::string &world)
  {

    // publish pose anyway for debug
    if constexpr(true)
    {
      static tf2_ros::TransformBroadcaster br(this);
      using TransformStamped = geometry_msgs::msg::TransformStamped;
      static TransformStamped tr;
      tr.header.frame_id = "world";

      static auto tf_pub = create_wall_timer(std::chrono::seconds(1), [&]()
      {
        tr.header.stamp = get_clock()->now();
        for(auto &[_,link]: links)
        {
          tr.child_frame_id = link;
          br.sendTransform(tr);
        }
      });

      return;
    }

    const auto use_tf = declare_parameter<bool>("use_tf", false);
    static ::GZ_NS::transport::Node gz_node;

    std::function<void(const ::GZ_NS::msgs::Pose_V &)> sub_cb;

    if(use_tf)
    {
      static tf2_ros::TransformBroadcaster br(this);
      using TransformStamped = geometry_msgs::msg::TransformStamped;
      static TransformStamped tr;
      tr.header.frame_id = "world";

      sub_cb = [&](const ::GZ_NS::msgs::Pose_V &msg)
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
      sub_cb = [&](const ::GZ_NS::msgs::Pose_V &msg)
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
