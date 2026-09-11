#include "manriix_perception/bt/wait_for_stability.hpp"
#include <cmath>

namespace manriix_bt {

WaitForStability::WaitForStability(const std::string& name,
                                   const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config) {}

BT::PortsList WaitForStability::providedPorts() {
    return {
        BT::InputPort<std::string>("robot_position"),
        BT::InputPort<double>("stability_threshold", 0.05, ""),
        BT::InputPort<double>("stability_wait_time", 2.0, ""),
        BT::InputPort<double>("timeout_sec", 30.0, ""),
    };
}

BT::NodeStatus WaitForStability::onStart() {
    _start          = std::chrono::steady_clock::now();
    _last_move_time = _start;
    _has_last_pos   = false;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForStability::onRunning() {
    auto robot_pos = getInput<std::string>("robot_position");
    if (!robot_pos || robot_pos->empty()) {
        return BT::NodeStatus::FAILURE;
    }

    auto now       = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - _start).count();
    double timeout = getInput<double>("timeout_sec").value_or(30.0);
    double stable  = getInput<double>("stability_wait_time").value_or(2.0);
    double thresh  = getInput<double>("stability_threshold").value_or(0.05);

    // Timeout — proceed anyway
    if (elapsed > timeout) {
        return BT::NodeStatus::SUCCESS;
    }

    // Parse current position
    double cx = 0.0, cy = 0.0;
    try {
        auto j = nlohmann::json::parse(*robot_pos);
        cx = j["x"].get<double>();
        cy = j["y"].get<double>();
    } catch (...) {
        return BT::NodeStatus::RUNNING;
    }

    // Compare with last known position
    if (_has_last_pos) {
        double dx = cx - _last_x;
        double dy = cy - _last_y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist > thresh) {
            // Robot moved — reset stability timer
            _last_move_time = now;
            _last_x = cx;
            _last_y = cy;
        }
    } else {
        _last_x = cx;
        _last_y = cy;
        _has_last_pos = true;
    }

    // Stable long enough → proceed
    double stable_elapsed =
        std::chrono::duration<double>(now - _last_move_time).count();
    if (stable_elapsed >= stable) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void WaitForStability::onHalted() {
    // Nothing to cancel
}

}  // namespace manriix_bt