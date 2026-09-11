#pragma once
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <vector>

namespace manriix_bt {

class NavigateMultiPOI : public BT::StatefulActionNode {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle     = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    enum class NavResult { PENDING, SUCCEEDED, CANCELED, ABORTED };

    NavigateMultiPOI(const std::string& name,
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

    int        _attempt{0};
    bool       _goal_sent{false};
    bool       _goal_active{false};
    NavResult  _nav_result{NavResult::PENDING};
    std::chrono::steady_clock::time_point _nav_start;
    nlohmann::json _failed_positions;

    // Multi-POI retry support
    nlohmann::json _poi_list;
    int _current_poi_index{0};

    nlohmann::json getCurrentTarget();
    BT::NodeStatus onNavFailed(const nlohmann::json& target);
    void cancelNavGoal();
};

}  // namespace manriix_bt