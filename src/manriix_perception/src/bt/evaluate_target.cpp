#include "manriix_perception/bt/evaluate_target.hpp"
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace manriix_bt {

EvaluateTarget::EvaluateTarget(const std::string& name,
                               const BT::NodeConfig& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList EvaluateTarget::providedPorts() {
    return {
        BT::InputPort<bool>("teleop_paused", false, ""),
        BT::InputPort<bool>("recovery_lock", false, ""),
        BT::InputPort<double>("recovery_cooldown_until", 0.0, ""),
        BT::InputPort<bool>("camera_active", false, ""),
        BT::InputPort<std::string>("available_targets", "[]"),
        BT::InputPort<std::string>("robot_position",   ""),
        BT::OutputPort<std::string>("current_target"),
        BT::BidirectionalPort<std::string>("recently_photographed"),
    };
}

BT::NodeStatus EvaluateTarget::tick() {
    // ── Guard checks — read from blackboard ───────────────────────
    if (getInput<bool>("teleop_paused").value_or(false))
        return BT::NodeStatus::FAILURE;

    if (getInput<bool>("recovery_lock").value_or(false))
        return BT::NodeStatus::FAILURE;

    if (getInput<bool>("camera_active").value_or(false))
        return BT::NodeStatus::FAILURE;

    // Recovery cooldown — compare unix timestamp against now
    double cooldown_until = getInput<double>("recovery_cooldown_until").value_or(0.0);
    if (cooldown_until > 0.0) {
        auto now = std::chrono::system_clock::now();
        double now_sec = std::chrono::duration<double>(
            now.time_since_epoch()).count();
        if (now_sec < cooldown_until)
            return BT::NodeStatus::FAILURE;
    }

    // ── Check available targets ────────────────────────────────────
    std::string targets_json =
        getInput<std::string>("available_targets").value_or("[]");

    bool has_targets = false;
    try {
        auto j = nlohmann::json::parse(targets_json);
        has_targets = j.is_array() && !j.empty();
    } catch (...) {
        has_targets = false;
    }

    // ── No targets — 60s patience timer ───────────────────────────
    if (!has_targets) {
        auto now = std::chrono::steady_clock::now();

        if (!_patience_timer_active) {
            _no_target_since      = now;
            _patience_timer_active = true;
        }

        double elapsed = std::chrono::duration<double>(
            now - _no_target_since).count();

        if (elapsed >= PATIENCE_SEC) {
            _patience_timer_active = false;
            return BT::NodeStatus::FAILURE;   // escalate to recovery
        }

        return BT::NodeStatus::RUNNING;       // wait for humans
    }

    // Targets exist — reset patience timer
    _patience_timer_active = false;

    // ── Select best target by priority_score ──────────────────────
    try {
        auto targets = nlohmann::json::parse(targets_json);

        // Find highest priority_score
        nlohmann::json best = targets[0];
        for (auto& t : targets) {
            if (t.value("priority_score", 0.0) >
                best.value("priority_score", 0.0)) {
                best = t;
            }
        }

        // ── 120s distance-based cooldown check ───────────────────
        // Handles POI position drift — checks 1.5m radius around
        // each recently photographed position instead of exact key match
        std::string recent_json =
            getInput<std::string>("recently_photographed").value_or("{}");
        try {
            auto recent = nlohmann::json::parse(recent_json);
            auto pos    = best["position"];
            double bx   = pos[0].get<double>();
            double by   = pos[1].get<double>();

            auto now_sys = std::chrono::system_clock::now();
            double now_sec = std::chrono::duration<double>(
                now_sys.time_since_epoch()).count();

            for (auto& [key, val] : recent.items()) {
                double shot_at = val.get<double>();
                if (now_sec - shot_at >= COOLDOWN_SEC) continue;

                // Parse key format "[x,y]"
                try {
                    auto coords = nlohmann::json::parse(key);
                    double rx = coords[0].get<double>();
                    double ry = coords[1].get<double>();
                    double dx = bx - rx;
                    double dy = by - ry;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    if (dist < 1.5) {
                        return BT::NodeStatus::FAILURE;  // within cooldown zone
                    }
                } catch (...) {}
            }
        } catch (...) {}

        // ── Already at POI? (< 0.8m) ─────────────────────────────
        std::string robot_pos_json =
            getInput<std::string>("robot_position").value_or("");
        if (!robot_pos_json.empty()) {
            try {
                auto rp  = nlohmann::json::parse(robot_pos_json);
                auto pos = best["position"];
                double dx = rp["x"].get<double>() - pos[0].get<double>();
                double dy = rp["y"].get<double>() - pos[1].get<double>();
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist < 0.8) {
                    setOutput("current_target", best.dump());
                    return BT::NodeStatus::SUCCESS;
                }
            } catch (...) {}
        }

        // ── Write selected target to blackboard ───────────────────
        setOutput("current_target", best.dump());
        return BT::NodeStatus::SUCCESS;

    } catch (...) {
        return BT::NodeStatus::FAILURE;
    }
}

}  // namespace manriix_bt
