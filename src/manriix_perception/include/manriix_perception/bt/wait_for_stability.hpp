#pragma once
#include <behaviortree_cpp/behavior_tree.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <string>

namespace manriix_bt {

class WaitForStability : public BT::StatefulActionNode {
public:
    WaitForStability(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    std::chrono::steady_clock::time_point _start;
    std::chrono::steady_clock::time_point _last_move_time;
    double _last_x{0.0};
    double _last_y{0.0};
    bool   _has_last_pos{false};
};

}  // namespace manriix_bt