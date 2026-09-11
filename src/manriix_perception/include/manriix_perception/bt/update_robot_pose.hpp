#pragma once
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace manriix_bt {

/**
 * UpdateRobotPose — Sync Action node (always SUCCESS)
 *
 * Subscribes to /odometry/filtered and writes robot_position +
 * robot_orientation to the blackboard on every tick.
 * Runs in the RootParallel alongside MissionSequence.
 *
 * New node — no Python equivalent (was a controller callback).
 */
class UpdateRobotPose : public BT::SyncActionNode {
public:
    UpdateRobotPose(const std::string& name,
                    const BT::NodeConfig& config,
                    rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub;
    nav_msgs::msg::Odometry::SharedPtr _latest_msg;
};

}  // namespace manriix_bt
