#pragma once
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <chrono>

namespace manriix_bt {

/**
 * ReturnToLastPosition — Stateful Action node (Recovery L3)
 *
 * Navigates to the last successful photo position.
 * Skips immediately if no last position stored.
 *
 * Replaces: ReturnToLastPosition in mission_bt.py
 */
class ReturnToLastPosition : public BT::StatefulActionNode {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle     = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    ReturnToLastPosition(const std::string& name,
                         const BT::NodeConfig& config,
                         rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    rclcpp::Node::SharedPtr _node;
    rclcpp_action::Client<NavigateToPose>::SharedPtr _nav_client;
    GoalHandle::SharedPtr _goal_handle;
    std::chrono::steady_clock::time_point _start;
    bool _nav_sent{false};
};

}  // namespace manriix_bt
