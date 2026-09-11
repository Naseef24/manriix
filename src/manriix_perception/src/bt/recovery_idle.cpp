#include "manriix_perception/bt/recovery_idle.hpp"

namespace manriix_bt {

RecoveryIdle::RecoveryIdle(const std::string& name,
                           const BT::NodeConfig& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList RecoveryIdle::providedPorts() {
    return {
        BT::InputPort<std::string>("available_targets"),
        BT::InputPort<bool>("teleop_paused", false, ""),
        BT::InputPort<bool>("recovery_lock", false, ""),
    };
}

BT::NodeStatus RecoveryIdle::tick() {
    // Teleop or lock released — exit recovery
    auto teleop = getInput<bool>("teleop_paused").value_or(false);
    auto locked = getInput<bool>("recovery_lock").value_or(false);
    if (teleop || locked) {
        return BT::NodeStatus::SUCCESS;
    }

    // Humans detected — resume mission
    auto targets = getInput<std::string>("available_targets").value_or("[]");
    if (targets != "[]" && !targets.empty()) {
        return BT::NodeStatus::SUCCESS;
    }

    // Still waiting — operator must intervene
    return BT::NodeStatus::RUNNING;
}

}  // namespace manriix_bt
