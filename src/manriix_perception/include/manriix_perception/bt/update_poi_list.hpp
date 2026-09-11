#pragma once
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <manriix_perception/msg/optimal_position.hpp>

namespace manriix_bt {

/**
 * UpdatePOIList — Sync Action node (always SUCCESS)
 *
 * Subscribes to /poi_manager/optimal_positions and /poi_manager/all_pois.
 * Writes available_targets and all_pois to the blackboard on every tick.
 * Runs in the RootParallel alongside MissionSequence.
 *
 * New node — no Python equivalent (was controller callbacks).
 */
class UpdatePOIList : public BT::SyncActionNode {
public:
    UpdatePOIList(const std::string& name,
                  const BT::NodeConfig& config,
                  rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::Subscription<manriix_perception::msg::OptimalPosition>::SharedPtr _opt_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _all_sub;
    std::string _latest_optimal;
    std::string _latest_all;
};

}  // namespace manriix_bt
