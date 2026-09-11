#include "manriix_perception/bt/navigate_multi_poi.hpp"
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <sstream>

namespace manriix_bt {

NavigateMultiPOI::NavigateMultiPOI(const std::string& name,
                                   const BT::NodeConfig& config,
                                   rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), _node(node) {}

BT::PortsList NavigateMultiPOI::providedPorts() {
    return {
        BT::InputPort<std::string>("current_target"),
        BT::InputPort<std::string>("all_pois", "[]", "Full POI list for retry"),
        BT::InputPort<std::string>("robot_position"),
        BT::InputPort<int>("max_poi_retries", 5, ""),
        BT::InputPort<double>("arrival_threshold", 0.5, ""),
        BT::InputPort<double>("per_goal_timeout", 60.0, ""),
        BT::InputPort<double>("abs_nav_timeout", 180.0, ""),
        BT::OutputPort<std::string>("failed_positions"),
    };
}

BT::NodeStatus NavigateMultiPOI::onStart() {
    _attempt       = 0;
    _goal_sent     = false;
    _goal_active   = false;
    _goal_handle   = nullptr;
    _failed_positions.clear();
    _current_poi_index = 0;

    // Build POI list: try all_pois first, fall back to current_target
    _poi_list = nlohmann::json::array();
    auto all_pois_json = getInput<std::string>("all_pois");
    if (all_pois_json && !all_pois_json->empty() && *all_pois_json != "[]") {
        try {
            auto parsed = nlohmann::json::parse(*all_pois_json);
            if (parsed.is_array() && !parsed.empty()) {
                _poi_list = parsed;
                RCLCPP_INFO(_node->get_logger(),
                    "NavigateMultiPOI: %zu POIs available for retry",
                    _poi_list.size());
            }
        } catch (...) {}
    }

    // If all_pois was empty/invalid, use current_target as single-element list
    if (_poi_list.empty()) {
        auto target_json = getInput<std::string>("current_target");
        if (target_json && !target_json->empty() && *target_json != "null") {
            try {
                auto t = nlohmann::json::parse(*target_json);
                _poi_list.push_back(t);
            } catch (...) {
                return BT::NodeStatus::FAILURE;
            }
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

    // Create Nav2 action client
    _nav_client = rclcpp_action::create_client<NavigateToPose>(
        _node, "navigate_to_pose");

    return BT::NodeStatus::RUNNING;
}

nlohmann::json NavigateMultiPOI::getCurrentTarget() {
    if (_poi_list.empty()) return {};
    int idx = _current_poi_index % static_cast<int>(_poi_list.size());
    return _poi_list[idx];
}

BT::NodeStatus NavigateMultiPOI::onRunning() {
    // Get current target from POI list
    nlohmann::json target = getCurrentTarget();
    if (target.empty() || !target.contains("position"))
        return BT::NodeStatus::FAILURE;

    auto robot_json = getInput<std::string>("robot_position");
    nlohmann::json robot_pos;
    try {
        if (robot_json && !robot_json->empty())
            robot_pos = nlohmann::json::parse(*robot_json);
    } catch (...) {}

    double tx = target["position"][0].get<double>();
    double ty = target["position"][1].get<double>();

    // Arrival check
    if (!robot_pos.is_null()) {
        double rx   = robot_pos["x"].get<double>();
        double ry   = robot_pos["y"].get<double>();
        double dist = std::sqrt((rx-tx)*(rx-tx) + (ry-ty)*(ry-ty));
        double threshold = getInput<double>("arrival_threshold").value_or(0.5);

        if (dist <= threshold) {
            if (_goal_handle) cancelNavGoal();
            setOutput("failed_positions", nlohmann::json(_failed_positions).dump());
            return BT::NodeStatus::SUCCESS;
        }
    }

    // Send goal if not yet sent
    if (!_goal_sent) {
        if (!_nav_client->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(_node->get_logger(),
                "NavigateMultiPOI: Nav2 not available — attempt %d", _attempt+1);
            return onNavFailed(target);
        }

        auto goal = NavigateToPose::Goal{};
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp    = _node->get_clock()->now();
        goal.pose.pose.position.x = tx;
        goal.pose.pose.position.y = ty;
        goal.pose.pose.position.z = 0.0;

        double yaw = target.value("orientation", 0.0);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.pose.pose.orientation.x = q.x();
        goal.pose.pose.orientation.y = q.y();
        goal.pose.pose.orientation.z = q.z();
        goal.pose.pose.orientation.w = q.w();

        auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
        send_opts.goal_response_callback =
            [this](const GoalHandle::SharedPtr& gh) {
                if (!gh || !gh->get_status()) {
                    RCLCPP_WARN(_node->get_logger(),
                        "NavigateMultiPOI: goal REJECTED");
                    _goal_active = false;
                } else {
                    _goal_handle = gh;
                    _goal_active = true;
                }
            };
        send_opts.result_callback =
            [this](const GoalHandle::WrappedResult& result) {
                using Status = rclcpp_action::ResultCode;
                if (result.code == Status::SUCCEEDED) {
                    _nav_result = NavResult::SUCCEEDED;
                } else if (result.code == Status::CANCELED) {
                    _nav_result = NavResult::CANCELED;
                } else {
                    _nav_result = NavResult::ABORTED;
                }
                _goal_active = false;
            };

        _nav_client->async_send_goal(goal, send_opts);
        _nav_start  = std::chrono::steady_clock::now();
        _nav_result = NavResult::PENDING;
        _goal_sent  = true;
        _goal_active = true;

        RCLCPP_INFO(_node->get_logger(),
            "NavigateMultiPOI: Attempt #%d, POI %d/%zu → (%.2f, %.2f)",
            _attempt+1, _current_poi_index+1, _poi_list.size(), tx, ty);

        return BT::NodeStatus::RUNNING;
    }

    // Check Nav2 result
    if (_nav_result == NavResult::SUCCEEDED) {
        setOutput("failed_positions", nlohmann::json(_failed_positions).dump());
        return BT::NodeStatus::SUCCESS;
    }

    if (_nav_result == NavResult::ABORTED) {
        RCLCPP_WARN(_node->get_logger(),
            "NavigateMultiPOI: Nav2 ABORTED on attempt #%d", _attempt+1);
        return onNavFailed(target);
    }

    // Timeout checks
    auto now     = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - _nav_start).count();
    double per_timeout = getInput<double>("per_goal_timeout").value_or(60.0);
    double abs_timeout = getInput<double>("abs_nav_timeout").value_or(180.0);

    if (elapsed > abs_timeout) {
        RCLCPP_WARN(_node->get_logger(),
            "NavigateMultiPOI: Absolute timeout on attempt #%d", _attempt+1);
        cancelNavGoal();
        return onNavFailed(target);
    }

    if (elapsed > per_timeout) {
        RCLCPP_WARN(_node->get_logger(),
            "NavigateMultiPOI: Per-goal timeout on attempt #%d", _attempt+1);
        cancelNavGoal();
        return onNavFailed(target);
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateMultiPOI::onNavFailed(const nlohmann::json& target) {
    // Mark position failed
    auto pos = target["position"];
    _failed_positions.push_back({
        {"x", pos[0]}, {"y", pos[1]},
        {"timestamp", std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count()}
    });

    _attempt++;
    int max_retries = getInput<int>("max_poi_retries").value_or(5);

    if (_attempt >= max_retries) {
        RCLCPP_WARN(_node->get_logger(),
            "NavigateMultiPOI: Max retries (%d) reached — handing to recovery",
            max_retries);
        setOutput("failed_positions", nlohmann::json(_failed_positions).dump());
        return BT::NodeStatus::FAILURE;
    }

    // Advance to next POI in the list
    _current_poi_index++;
    if (_current_poi_index >= static_cast<int>(_poi_list.size())) {
        RCLCPP_WARN(_node->get_logger(),
            "NavigateMultiPOI: All %zu POIs exhausted — handing to recovery",
            _poi_list.size());
        setOutput("failed_positions", nlohmann::json(_failed_positions).dump());
        return BT::NodeStatus::FAILURE;
    }

    // Reset for next POI
    _goal_sent   = false;
    _goal_active = false;
    _goal_handle = nullptr;
    _nav_result  = NavResult::PENDING;

    auto next = getCurrentTarget();
    RCLCPP_INFO(_node->get_logger(),
        "NavigateMultiPOI: Switching to POI %d/%zu (%.2f, %.2f)",
        _current_poi_index+1, _poi_list.size(),
        next["position"][0].get<double>(),
        next["position"][1].get<double>());

    return BT::NodeStatus::RUNNING;
}

void NavigateMultiPOI::cancelNavGoal() {
    if (_nav_client && _goal_handle) {
        _nav_client->async_cancel_goal(_goal_handle);
        _goal_handle = nullptr;
    }
    _goal_active = false;
}

void NavigateMultiPOI::onHalted() {
    cancelNavGoal();
}

}  // namespace manriix_bt