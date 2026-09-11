#include "manriix_perception/bt/update_robot_pose.hpp"
#include <cstdio>
#include "manriix_perception/bt/update_poi_list.hpp"
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace manriix_bt {

UpdateRobotPose::UpdateRobotPose(const std::string& name,
                                 const BT::NodeConfig& config,
                                 rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), _node(node) {
    _sub = _node->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered",
        rclcpp::QoS(rclcpp::KeepLast(3))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            _latest_msg = msg;
        });
}

BT::PortsList UpdateRobotPose::providedPorts() {
    return {
        BT::OutputPort<std::string>("robot_position"),
        BT::OutputPort<double>("robot_orientation"),
    };
}

BT::NodeStatus UpdateRobotPose::tick() {
    if (!_latest_msg)
        return BT::NodeStatus::SUCCESS;

    double x = _latest_msg->pose.pose.position.x;
    double y = _latest_msg->pose.pose.position.y;

    tf2::Quaternion q(
        _latest_msg->pose.pose.orientation.x,
        _latest_msg->pose.pose.orientation.y,
        _latest_msg->pose.pose.orientation.z,
        _latest_msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    nlohmann::json pos_json = {{"x", x}, {"y", y}};
    setOutput("robot_position",    pos_json.dump());
    setOutput("robot_orientation", yaw);
    return BT::NodeStatus::SUCCESS;
}

UpdatePOIList::UpdatePOIList(const std::string& name,
                             const BT::NodeConfig& config,
                             rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), _node(node) {

    _opt_sub = _node->create_subscription<manriix_perception::msg::OptimalPosition>(
        "/poi_manager/optimal_positions",
        rclcpp::QoS(rclcpp::KeepLast(5))
            .reliability(rclcpp::ReliabilityPolicy::Reliable),
        [this](manriix_perception::msg::OptimalPosition::SharedPtr msg) {
            nlohmann::json target = {
                {"position",        {msg->x, msg->y}},
                {"orientation",     msg->orientation},
                {"priority_score",  msg->priority_score},
                {"cluster_id",      0},
                {"stability_score", 0.5}
            };
            nlohmann::json arr = nlohmann::json::array();
            arr.push_back(target);
            _latest_optimal = arr.dump();
            { FILE* _f=fopen("/tmp/bt_debug.log","a"); if(_f){ fprintf(_f, "UPOI_CB: got optimal, _latest=%s\n", _latest_optimal.substr(0,80).c_str()); fclose(_f); } }
        });

    _all_sub = _node->create_subscription<std_msgs::msg::String>(
        "/poi_manager/all_pois",
        rclcpp::QoS(rclcpp::KeepLast(3))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort),
        [this](std_msgs::msg::String::SharedPtr msg) {
            _latest_all = msg->data;
        });
}

BT::PortsList UpdatePOIList::providedPorts() {
    return {
        BT::OutputPort<std::string>("available_targets"),
        BT::OutputPort<std::string>("all_pois"),
    };
}

BT::NodeStatus UpdatePOIList::tick() {
    if (!_latest_optimal.empty())
    { FILE* _f=fopen("/tmp/bt_debug.log","a"); if(_f){ fprintf(_f, "UPOI_TICK: latest_opt=%s\n", _latest_optimal.empty() ? "EMPTY" : _latest_optimal.substr(0,60).c_str()); fclose(_f); } }
        setOutput("available_targets", _latest_optimal);
    if (!_latest_all.empty())
        setOutput("all_pois", _latest_all);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace manriix_bt
