#include "manriix_perception/bt/take_photo_action.hpp"
#include <nlohmann/json.hpp>

namespace manriix_bt {

TakePhotoAction::TakePhotoAction(const std::string& name,
                                 const BT::NodeConfig& config,
                                 rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), _node(node) {}

BT::PortsList TakePhotoAction::providedPorts() {
    return {
        BT::InputPort<int>("group_size", 0, ""),
        BT::InputPort<double>("requested_duration", 0.0, ""),
        BT::InputPort<std::string>("current_target", "{current_target}", ""),
        BT::InputPort<std::string>("robot_position", "{robot_position}", ""),
        // BT::InputPort<std::string>("recently_photographed", "{recently_photographed}", ""),
        BT::BidirectionalPort<std::string>("recently_photographed"),
        BT::BidirectionalPort<bool>("camera_active"),
        BT::OutputPort<int>("photos_taken"),
        BT::OutputPort<std::string>("last_successful_position"),
        // BT::OutputPort<std::string>("recently_photographed"),
    };
}

BT::NodeStatus TakePhotoAction::onStart() {
    _result_received = false;
    _photos_taken    = 0;
    _actual_duration = 0.0;
    _cooldown_written = false;

    // Create TakePhoto action client
    _client = rclcpp_action::create_client<TakePhoto>(_node, "take_photo");

    if (!_client->wait_for_action_server(std::chrono::seconds(3))) {
        RCLCPP_WARN(_node->get_logger(),
            "TakePhotoAction: action server not available");
        setOutput("camera_active", false);
        return BT::NodeStatus::FAILURE;
    }

    auto goal = TakePhoto::Goal{};
    goal.group_size        = getInput<int>("group_size").value_or(0);
    goal.requested_duration = getInput<double>("requested_duration").value_or(0.0);

    auto send_opts = rclcpp_action::Client<TakePhoto>::SendGoalOptions{};

    send_opts.goal_response_callback =
        [this](const GoalHandle::SharedPtr& gh) {
            if (!gh) {
                RCLCPP_WARN(_node->get_logger(),
                    "TakePhotoAction: goal REJECTED by PIS");
                _result_received = true;  // treat as done, FAILURE path
                _photos_taken    = 0;
            } else {
                _goal_handle = gh;
                RCLCPP_INFO(_node->get_logger(),
                    "TakePhotoAction: goal ACCEPTED");
            }
        };

    send_opts.feedback_callback =
        [this](GoalHandle::SharedPtr,
               const std::shared_ptr<const TakePhoto::Feedback> fb) {
            RCLCPP_DEBUG(_node->get_logger(),
                "TakePhotoAction: %.1fs elapsed, group=%d, formation=%s",
                fb->elapsed_time,
                fb->current_group_size,
                fb->formation_type.c_str());
        };

    send_opts.result_callback =
        [this](const GoalHandle::WrappedResult& result) {
            _photos_taken    = result.result->photos_taken;
            _actual_duration = result.result->actual_duration;
            _result_received = true;

            RCLCPP_INFO(_node->get_logger(),
                "TakePhotoAction: result — photos=%d, duration=%.1fs, "
                "interrupted=%s, reason=%s",
                _photos_taken,
                _actual_duration,
                result.result->interrupted ? "true" : "false",
                result.result->stop_reason.c_str());
        };

    _client->async_send_goal(goal, send_opts);
    setOutput("camera_active", true);

    RCLCPP_INFO(_node->get_logger(),
        "TakePhotoAction: goal sent to PIS");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakePhotoAction::onRunning() {
    if (!_result_received)
        return BT::NodeStatus::RUNNING;

    // Result received — write cooldown once, then wait for PIS to complete
    // camera_active is cleared by PIS status subscriber in mission_controller_cpp
    // when PIS publishes "completed". This prevents BT restarting before
    // the real photo session finishes.
    if (!_cooldown_written) {
        setOutput("photos_taken", _photos_taken);

        // Always write cooldown regardless of photos_taken
        auto current_target_json = getInput<std::string>("current_target");
        if (current_target_json && !current_target_json->empty()) {
            try {
                auto target = nlohmann::json::parse(*current_target_json);
                auto pos = target["position"];
                double x = std::round(pos[0].get<double>() * 10.0) / 10.0;
                double y = std::round(pos[1].get<double>() * 10.0) / 10.0;

                std::string recent_json =
                    getInput<std::string>("recently_photographed").value_or("{}");
                auto recent = nlohmann::json::parse(recent_json);

                std::ostringstream key_ss;
                key_ss << "[" << x << "," << y << "]";
                auto now_sec = std::chrono::duration<double>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
                recent[key_ss.str()] = now_sec;
                setOutput("recently_photographed", recent.dump());

                RCLCPP_INFO(_node->get_logger(),
                    "TakePhotoAction: cooldown set for [%.1f,%.1f] (photos=%d)",
                    x, y, _photos_taken);
            } catch (...) {}
        }

        if (_photos_taken > 0) {
            auto robot_json = getInput<std::string>("robot_position");
            if (robot_json && !robot_json->empty()) {
                setOutput("last_successful_position", *robot_json);
            }
            RCLCPP_INFO(_node->get_logger(),
                "TakePhotoAction: session complete — %d photos taken",
                _photos_taken);
        } else {
            RCLCPP_WARN(_node->get_logger(),
                "TakePhotoAction: zero photos — cooldown set, waiting for PIS");
        }

        _cooldown_written = true;
    }

    // Wait for camera_active to be cleared by PIS status subscriber
    auto camera_active = getInput<bool>("camera_active").value_or(true);
    if (camera_active) {
        return BT::NodeStatus::RUNNING;  // PIS still running
    }

    return BT::NodeStatus::SUCCESS;
}

void TakePhotoAction::onHalted() {
    if (_client && _goal_handle) {
        RCLCPP_INFO(_node->get_logger(),
            "TakePhotoAction: cancelling goal (halted)");
        _client->async_cancel_goal(_goal_handle);
        _goal_handle = nullptr;
    }
    setOutput("camera_active", false);
    _result_received = false;
}

}  // namespace manriix_bt
