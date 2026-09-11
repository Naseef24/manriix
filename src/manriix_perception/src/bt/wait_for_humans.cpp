#include "manriix_perception/bt/wait_for_humans.hpp"

namespace manriix_bt {

WaitForHumans::WaitForHumans(const std::string& name,
                             const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config) {}

BT::PortsList WaitForHumans::providedPorts() {
    return {
        BT::InputPort<std::string>("available_targets"),
        BT::InputPort<bool>("teleop_paused", false, ""),
        BT::InputPort<bool>("recovery_lock", false, ""),
        BT::InputPort<double>("wait_duration_sec", 60.0, ""),
    };
}

BT::NodeStatus WaitForHumans::onStart() {
    _start = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForHumans::onRunning() {
    // Interrupt conditions — exit recovery cleanly
    auto teleop = getInput<bool>("teleop_paused").value_or(false);
    auto locked = getInput<bool>("recovery_lock").value_or(false);
    if (teleop || locked) {
        return BT::NodeStatus::SUCCESS;
    }

    // Humans detected — resume
    auto targets = getInput<std::string>("available_targets").value_or("[]");
    if (targets != "[]" && !targets.empty()) {
        return BT::NodeStatus::SUCCESS;
    }

    // Check wait timeout
    auto now = std::chrono::steady_clock::now();
    double elapsed  = std::chrono::duration<double>(now - _start).count();
    double duration = getInput<double>("wait_duration_sec").value_or(60.0);

    if (elapsed >= duration) {
        return BT::NodeStatus::FAILURE;   // escalate to L2
    }

    return BT::NodeStatus::RUNNING;
}

void WaitForHumans::onHalted() {
    // Nothing to cancel
}

}  // namespace manriix_bt
