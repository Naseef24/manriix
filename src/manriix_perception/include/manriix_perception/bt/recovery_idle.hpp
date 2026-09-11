#pragma once
#include <behaviortree_cpp/behavior_tree.h>

namespace manriix_bt {

/**
 * RecoveryIdle — Condition node (Recovery L4)
 *
 * Waits indefinitely. Returns SUCCESS when humans reappear or
 * teleop/recovery_lock interrupts. Otherwise always RUNNING.
 *
 * Replaces: RecoveryIdle in mission_bt.py
 */
class RecoveryIdle : public BT::ConditionNode {
public:
    RecoveryIdle(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

}  // namespace manriix_bt
