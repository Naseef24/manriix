#pragma once
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <chrono>

namespace manriix_bt {

/**
 * SpinAndScan — Stateful Action node (Recovery L2)
 *
 * Sends a 360° Spin goal to Nav2. ZED cameras scan for humans at all
 * orientations. Returns SUCCESS if humans detected. FAILURE after timeout.
 *
 * Replaces: SpinAndScan in mission_bt.py
 */
class SpinAndScan : public BT::StatefulActionNode {
public:
    using Spin       = nav2_msgs::action::Spin;
    using GoalHandle = rclcpp_action::ClientGoalHandle<Spin>;

    SpinAndScan(const std::string& name,
                const BT::NodeConfig& config,
                rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    rclcpp::Node::SharedPtr _node;
    rclcpp_action::Client<Spin>::SharedPtr _spin_client;
    GoalHandle::SharedPtr _goal_handle;
    std::chrono::steady_clock::time_point _start;
    bool _spin_sent{false};
};

}  // namespace manriix_bt
