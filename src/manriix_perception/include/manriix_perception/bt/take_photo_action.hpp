#pragma once
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <manriix_perception/action/take_photo.hpp>

namespace manriix_bt {

/**
 * TakePhotoAction — Stateful Action node
 *
 * Sends a TakePhoto action goal to PhotoIntelligenceSystem.
 * RUNNING while session is in progress.
 * SUCCESS when photos_taken > 0 and session completes.
 * Writes camera_active, photos_taken, last_successful_position to blackboard.
 *
 * Replaces: TakePhotoAction in mission_bt.py
 */
class TakePhotoAction : public BT::StatefulActionNode {
public:
    using TakePhoto  = manriix_perception::action::TakePhoto;
    using GoalHandle = rclcpp_action::ClientGoalHandle<TakePhoto>;

    TakePhotoAction(const std::string& name,
                    const BT::NodeConfig& config,
                    rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    rclcpp::Node::SharedPtr _node;
    rclcpp_action::Client<TakePhoto>::SharedPtr _client;
    GoalHandle::SharedPtr _goal_handle;
    bool _result_received{false};
    int  _photos_taken{0};
    bool _cooldown_written{false};
    double _actual_duration{0.0};
};

}  // namespace manriix_bt
