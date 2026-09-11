#include "manriix_perception/bt/return_to_last_position.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>

namespace manriix_bt {

ReturnToLastPosition::ReturnToLastPosition(const std::string& name,
                                           const BT::NodeConfig& config,
                                           rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), _node(node) {}

BT::PortsList ReturnToLastPosition::providedPorts() {
    return {
        BT::InputPort<std::string>("last_successful_position"),
        BT::InputPort<std::string>("available_targets"),
        BT::InputPort<bool>("teleop_paused", false, ""),
        BT::InputPort<bool>("recovery_lock", false, ""),
        BT::InputPort<double>("nav_timeout_sec", 90.0, ""),
    };
}

BT::NodeStatus ReturnToLastPosition::onStart() {
    _start     = std::chrono::steady_clock::now();
    _nav_sent  = false;
    _goal_handle = nullptr;

    // No last position stored — skip L3
    auto last_pos = getInput<std::string>("last_successful_position");
    if (!last_pos || last_pos->empty() || *last_pos == "null") {
        RCLCPP_WARN(_node->get_logger(),
                    "ReturnToLastPosition: No last position stored — skipping L3");
        return BT::NodeStatus::FAILURE;
    }

    // Create nav client and send goal
    _nav_client = rclcpp_action::create_client<NavigateToPose>(
        _node, "navigate_to_pose");

    if (!_nav_client->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(_node->get_logger(),
                    "ReturnToLastPosition: Nav2 not available");
        return BT::NodeStatus::FAILURE;
    }

    // Parse JSON position: {"x": 1.23, "y": 4.56}
    double x = 0.0, y = 0.0;
    try {
        auto pos_json = nlohmann::json::parse(*last_pos);
        x = pos_json["x"].get<double>();
        y = pos_json["y"].get<double>();
    } catch (...) {
        return BT::NodeStatus::FAILURE;
    }

    auto goal = NavigateToPose::Goal{};
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp    = _node->get_clock()->now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.w = 1.0;

    _nav_client->async_send_goal(goal);
    _nav_sent = true;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReturnToLastPosition::onRunning() {
    auto teleop = getInput<bool>("teleop_paused").value_or(false);
    auto locked = getInput<bool>("recovery_lock").value_or(false);
    if (teleop || locked) {
        return BT::NodeStatus::SUCCESS;
    }

    auto targets = getInput<std::string>("available_targets").value_or("[]");
    if (targets != "[]" && !targets.empty()) {
        return BT::NodeStatus::SUCCESS;
    }

    auto now    = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - _start).count();
    double timeout = getInput<double>("nav_timeout_sec").value_or(90.0);

    if (elapsed > timeout) {
        return BT::NodeStatus::FAILURE;   // escalate to L4
    }

    return BT::NodeStatus::RUNNING;
}

void ReturnToLastPosition::onHalted() {
    if (_nav_client) {
        _nav_client->async_cancel_all_goals();
    }
}

}  // namespace manriix_bt
