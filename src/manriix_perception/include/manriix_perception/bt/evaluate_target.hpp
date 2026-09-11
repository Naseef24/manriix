#pragma once
#include <behaviortree_cpp/behavior_tree.h>
#include <chrono>

namespace manriix_bt {

/**
 * EvaluateTarget — Condition node
 *
 * Gate at the start of every tick. Returns SUCCESS when a valid target
 * exists and all guards pass. Returns RUNNING while waiting for humans
 * (60s patience timer). Returns FAILURE to escalate to recovery.
 *
 * Replaces: EvaluateTarget in mission_bt.py
 */
class EvaluateTarget : public BT::ConditionNode {
public:
    EvaluateTarget(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    std::chrono::steady_clock::time_point _no_target_since;
    bool _patience_timer_active{false};
    static constexpr double PATIENCE_SEC = 120.0;
    static constexpr double COOLDOWN_SEC = 120.0;
};

}  // namespace manriix_bt
