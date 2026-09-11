#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>

#include <pcl_conversions/pcl_conversions.h>

class PassThroughNode : public rclcpp::Node
{
public:
    PassThroughNode() : Node("pcl_passthrough_node")
    {
        auto qos = rclcpp::SensorDataQoS();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/oak/points",
            qos,
            std::bind(&PassThroughNode::callback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/oak/points_filtered",
            qos
        );

        RCLCPP_INFO(this->get_logger(), "PCL PassThrough + Light Noise Filter Started");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 to PCL cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::fromROSMsg(*msg, *cloud);

        // -----------------------------------
        // 1. Remove NaN / invalid points
        // -----------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clean(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud_clean, indices);

        // -----------------------------------
        // 2. PassThrough Z: forward distance
        // -----------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_z(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::PassThrough<pcl::PointXYZI> pass_z;
        pass_z.setInputCloud(cloud_clean);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(0.5, 2.5);
        pass_z.filter(*cloud_z);

        // -----------------------------------
        // 3. PassThrough X: left/right width
        // -----------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_x(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::PassThrough<pcl::PointXYZI> pass_x;
        pass_x.setInputCloud(cloud_z);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-2.0, 2.0);
        pass_x.filter(*cloud_x);

        // -----------------------------------
        // 4. PassThrough Y: vertical band
        // -----------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_y(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::PassThrough<pcl::PointXYZI> pass_y;
        pass_y.setInputCloud(cloud_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-0.5, 0.5);
        pass_y.filter(*cloud_y);

        // Convert back to ROS PointCloud2
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_y, output);
        output.header = msg->header;

        pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PassThroughNode>());
    rclcpp::shutdown();
    return 0;
}











// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>

// #include <pcl/point_types.h>
// #include <pcl/filters/passthrough.h>

// #include <pcl_conversions/pcl_conversions.h>

// // ⭐ IMPORTANT: QoS fix header
// #include <rclcpp/qos.hpp>

// class PassThroughNode : public rclcpp::Node
// {
// public:
//     PassThroughNode() : Node("pcl_passthrough_node")
//     {
//         // ⭐ SENSOR QoS (CRITICAL FIX)
//         auto qos = rclcpp::SensorDataQoS();

//         sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             "/oak/points",
//             qos,
//             std::bind(&PassThroughNode::callback, this, std::placeholders::_1)
//         );

//         pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//             "/oak/points_filtered",
//             qos
//         );

//         RCLCPP_INFO(this->get_logger(), "PCL PassThrough Node Started (QoS FIXED)");
//     }

// private:
//     void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//     {
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
//             new pcl::PointCloud<pcl::PointXYZI>
//         );

//         pcl::fromROSMsg(*msg, *cloud);

//         pcl::PassThrough<pcl::PointXYZI> pass;
//         pass.setInputCloud(cloud);

//         // =========================
//         // FILTER (YOUR REQUIREMENT)
//         // =========================

//         pass.setFilterFieldName("z");
//         pass.setFilterLimits(0.7, 3.0);

//         pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
//             new pcl::PointCloud<pcl::PointXYZI>
//         );

//         pass.filter(*filtered);

//         sensor_msgs::msg::PointCloud2 output;
//         pcl::toROSMsg(*filtered, output);

//         output.header = msg->header;

//         pub_->publish(output);
//     }

//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<PassThroughNode>());
//     rclcpp::shutdown();
//     return 0;
// }