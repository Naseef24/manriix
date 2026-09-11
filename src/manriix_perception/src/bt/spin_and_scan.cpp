#include "manriix_perception/bt/spin_and_scan.hpp"

namespace manriix_bt {

SpinAndScan::SpinAndScan(const std::string& name,
                         const BT::NodeConfig& config,
                         rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), _node(node) {}

BT::PortsList SpinAndScan::providedPorts() {
    return {
        BT::InputPort<std::string>("available_targets"),
        BT::InputPort<bool>("teleop_paused", false, ""),
        BT::InputPort<bool>("recovery_lock", false, ""),
        BT::InputPort<double>("scan_timeout_sec", 30.0, ""),
    };
}

BT::NodeStatus SpinAndScan::onStart() {
    _start      = std::chrono::steady_clock::now();
    _spin_sent  = false;
    _goal_handle = nullptr;

    // Create spin client
    _spin_client = rclcpp_action::create_client<Spin>(_node, "spin");

    if (!_spin_client->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_WARN(_node->get_logger(),
                    "SpinAndScan: Spin server not available — skipping L2");
        return BT::NodeStatus::FAILURE;
    }

    auto goal = Spin::Goal{};
    goal.target_yaw = 6.283185f;   // 360°

    auto send_goal_opts = rclcpp_action::Client<Spin>::SendGoalOptions{};
    _spin_client->async_send_goal(goal, send_goal_opts);
    _spin_sent = true;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SpinAndScan::onRunning() {
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
    double timeout = getInput<double>("scan_timeout_sec").value_or(30.0);

    if (elapsed > timeout) {
        return BT::NodeStatus::FAILURE;   // escalate to L3
    }

    return BT::NodeStatus::RUNNING;
}

void SpinAndScan::onHalted() {
    if (_spin_client) {
        _spin_client->async_cancel_all_goals();
    }
}

}  // namespace manriix_bt
