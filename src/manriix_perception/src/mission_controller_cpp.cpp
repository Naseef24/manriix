#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "manriix_perception/bt/evaluate_target.hpp"
#include "manriix_perception/bt/navigate_multi_poi.hpp"
#include "manriix_perception/bt/wait_for_stability.hpp"
#include "manriix_perception/bt/take_photo_action.hpp"
#include "manriix_perception/bt/wait_for_humans.hpp"
#include "manriix_perception/bt/spin_and_scan.hpp"
#include "manriix_perception/bt/return_to_last_position.hpp"
#include "manriix_perception/bt/recovery_idle.hpp"
#include "manriix_perception/msg/optimal_position.hpp"

using namespace manriix_bt;

class MissionControllerCpp : public rclcpp::Node {
public:
    MissionControllerCpp() : Node("mission_controller_cpp") {
        RCLCPP_INFO(get_logger(), "MissionControllerCpp starting...");
    }

    void initialize() {
        auto node_ptr = shared_from_this();

        // Register BT nodes — no updaters, they run in pre-tick
        _factory.registerNodeType<EvaluateTarget>("EvaluateTarget");

        _factory.registerBuilder<NavigateMultiPOI>(
            "NavigateMultiPOI",
            [node_ptr](const std::string& name, const BT::NodeConfig& cfg) {
                return std::make_unique<NavigateMultiPOI>(name, cfg, node_ptr);
            });

        _factory.registerNodeType<WaitForStability>("WaitForStability");

        _factory.registerBuilder<TakePhotoAction>(
            "TakePhotoAction",
            [node_ptr](const std::string& name, const BT::NodeConfig& cfg) {
                return std::make_unique<TakePhotoAction>(name, cfg, node_ptr);
            });

        _factory.registerNodeType<WaitForHumans>("WaitForHumans");

        _factory.registerBuilder<SpinAndScan>(
            "SpinAndScan",
            [node_ptr](const std::string& name, const BT::NodeConfig& cfg) {
                return std::make_unique<SpinAndScan>(name, cfg, node_ptr);
            });

        _factory.registerBuilder<ReturnToLastPosition>(
            "ReturnToLastPosition",
            [node_ptr](const std::string& name, const BT::NodeConfig& cfg) {
                return std::make_unique<ReturnToLastPosition>(name, cfg, node_ptr);
            });

        _factory.registerNodeType<RecoveryIdle>("RecoveryIdle");

        // Load XML tree
        std::string pkg_share =
            ament_index_cpp::get_package_share_directory("manriix_perception");
        std::string xml_path = pkg_share + "/bt_trees/mission_tree.xml";

        try {
            _tree = _factory.createTreeFromFile(xml_path);
            RCLCPP_INFO(get_logger(), "Mission BT loaded");
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Failed to load BT XML: %s", e.what());
            throw;
        }

        // File logger
        try {
            _file_logger = std::make_unique<BT::FileLogger2>(
                _tree, "/tmp/manriix_bt_trace.btlog");
            RCLCPP_INFO(get_logger(), "BT file logger active");
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "BT file logger failed: %s", e.what());
        }

        // Groot2 live publisher (port 1667)
        try {
            _groot2_publisher = std::make_unique<BT::Groot2Publisher>(
                _tree, 1667);
            RCLCPP_INFO(get_logger(), "Groot2 publisher active on port 1667");
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Groot2 publisher failed: %s", e.what());
        }

        // === Subscriptions — data stored in members, written to BB in pre-tick ===

        // Odometry → robot_position, robot_orientation
        _odom_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered",
            rclcpp::QoS(rclcpp::KeepLast(3))
                .reliability(rclcpp::ReliabilityPolicy::BestEffort),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                double x = msg->pose.pose.position.x;
                double y = msg->pose.pose.position.y;
                tf2::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                nlohmann::json pos = {{"x", x}, {"y", y}};
                _robot_position_json = pos.dump();
                _robot_orientation = yaw;
                _has_odom = true;
            });

        // POI optimal positions → available_targets
        _poi_opt_sub = create_subscription<manriix_perception::msg::OptimalPosition>(
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
                _available_targets_json = arr.dump();
            });

        // All POIs → all_pois
        _poi_all_sub = create_subscription<std_msgs::msg::String>(
            "/poi_manager/all_pois",
            rclcpp::QoS(rclcpp::KeepLast(3))
                .reliability(rclcpp::ReliabilityPolicy::BestEffort),
            [this](std_msgs::msg::String::SharedPtr msg) {
                _all_pois_json = msg->data;
            });

        // PIS status → camera_active
        _pis_status_sub = create_subscription<std_msgs::msg::String>(
            "/photo/intelligence/status", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "completed" || msg->data == "idle") {
                    _camera_active = false;
                    RCLCPP_INFO(get_logger(),
                        "PIS session %s — camera_active cleared",
                        msg->data.c_str());
                }
            });

        // Teleop override → teleop_paused
        _teleop_sub = create_subscription<std_msgs::msg::Bool>(
            "/teleop/override", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                _teleop_paused = msg->data;
                if (msg->data) {
                    RCLCPP_WARN(get_logger(), "Teleop override ACTIVE");
                } else {
                    RCLCPP_INFO(get_logger(), "Teleop override released");
                }
            });

        // 5Hz tick timer
        _tick_timer = create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() { tickBT(); });

        RCLCPP_INFO(get_logger(),
            "MissionControllerCpp active — ticking at 5Hz");
    }

private:
    void tickBT() {
        try {
            // === Pre-tick: write subscription data to blackboard ===
            auto bb = _tree.rootBlackboard();

            if (_has_odom) {
                bb->set<std::string>("robot_position", _robot_position_json);
                bb->set<double>("robot_orientation", _robot_orientation);
            }

            if (!_available_targets_json.empty()) {
                bb->set<std::string>("available_targets", _available_targets_json);
            }

            if (!_all_pois_json.empty()) {
                bb->set<std::string>("all_pois", _all_pois_json);
            }

            bb->set<bool>("camera_active", _camera_active);
            bb->set<bool>("teleop_paused", _teleop_paused);

            // === Tick the tree ===
            _tree.tickOnce();

            // === Post-tick: read camera_active back (TakePhoto may set it) ===
            _camera_active = bb->get<bool>("camera_active");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "BT tick error: %s", e.what());
        }
    }

    // BT
    BT::BehaviorTreeFactory          _factory;
    BT::Tree                         _tree;
    std::unique_ptr<BT::FileLogger2> _file_logger;
    std::unique_ptr<BT::Groot2Publisher> _groot2_publisher;
    rclcpp::TimerBase::SharedPtr     _tick_timer;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
    rclcpp::Subscription<manriix_perception::msg::OptimalPosition>::SharedPtr _poi_opt_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _poi_all_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _pis_status_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _teleop_sub;

    // Latest data from subscriptions (written to BB in pre-tick)
    std::string _robot_position_json;
    double      _robot_orientation{0.0};
    bool        _has_odom{false};
    std::string _available_targets_json;
    std::string _all_pois_json;
    bool        _camera_active{false};
    bool        _teleop_paused{false};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionControllerCpp>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
