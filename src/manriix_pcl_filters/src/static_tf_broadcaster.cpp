#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class StaticTFBroadcaster : public rclcpp::Node
{
public:
    StaticTFBroadcaster() : Node("static_tf_broadcaster")
    {
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        publishTransforms();

        RCLCPP_INFO(this->get_logger(), "Static TFs published (map → odom → base_link → camera)");
    }

private:

    void publishTransforms()
    {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        // -------------------------
        // map -> odom
        // -------------------------
        geometry_msgs::msg::TransformStamped t1;
        t1.header.stamp = this->now();
        t1.header.frame_id = "map";
        t1.child_frame_id = "odom";

        t1.transform.translation.x = 0.0;
        t1.transform.translation.y = 0.0;
        t1.transform.translation.z = 0.0;

        t1.transform.rotation.w = 1.0;

        transforms.push_back(t1);

        // -------------------------
        // odom -> base_link
        // -------------------------
        geometry_msgs::msg::TransformStamped t2;
        t2.header.stamp = this->now();
        t2.header.frame_id = "odom";
        t2.child_frame_id = "base_link";

        t2.transform.translation.x = 0.0;
        t2.transform.translation.y = 0.0;
        t2.transform.translation.z = 0.0;

        t2.transform.rotation.w = 1.0;

        transforms.push_back(t2);

        // -------------------------
        // base_link -> camera
        // -------------------------
        geometry_msgs::msg::TransformStamped t3;
        t3.header.stamp = this->now();
        t3.header.frame_id = "base_link";
        t3.child_frame_id = "oak_right_camera_optical_frame";

        t3.transform.translation.x = 0.0;
        t3.transform.translation.y = 0.0;
        t3.transform.translation.z = 0.2;

        t3.transform.rotation.w = 1.0;

        transforms.push_back(t3);

        broadcaster_->sendTransform(transforms);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}