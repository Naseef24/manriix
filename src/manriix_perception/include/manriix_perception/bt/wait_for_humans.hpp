#pragma once
#include <behaviortree_cpp/behavior_tree.h>
#include <chrono>

namespace manriix_bt {

/**
 * WaitForHumans — Stateful Action node (Recovery L1)
 *
 * Waits in place for wait_duration_sec. Returns SUCCESS if humans
 * appear or teleop/lock interrupt. Returns FAILURE after timeout.
 *
 * Replaces: WaitForHumans in mission_bt.py
 */
class WaitForHumans : public BT::StatefulActionNode {
public:
    WaitForHumans(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    std::chrono::steady_clock::time_point _start;
};

}  // namespace manriix_bt
