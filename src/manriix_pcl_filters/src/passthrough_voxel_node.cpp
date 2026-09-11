#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>

class PassThroughVoxelNode : public rclcpp::Node
{
public:
    PassThroughVoxelNode() : Node("pcl_passthrough_voxel_node")
    {
        auto qos = rclcpp::SensorDataQoS();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/oak/points",
            qos,
            std::bind(&PassThroughVoxelNode::callback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/oak/points_filtered",
            qos
        );

        RCLCPP_INFO(
            this->get_logger(),
            "PCL PassThrough + VoxelGrid node started"
        );
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // ------------------------------------------------
        // 1. Convert ROS PointCloud2 to PCL PointXYZI cloud
        // ------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::fromROSMsg(*msg, *cloud_raw);

        if (cloud_raw->empty()) {
            return;
        }

        // ------------------------------------------------
        // 2. Remove NaN / invalid points
        // ------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clean(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        std::vector<int> valid_indices;
        pcl::removeNaNFromPointCloud(*cloud_raw, *cloud_clean, valid_indices);

        if (cloud_clean->empty()) {
            return;
        }

        // ------------------------------------------------
        // 3. PassThrough Z filter
        //    Z = forward depth in OAK optical frame
        // ------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_z(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::PassThrough<pcl::PointXYZI> pass_z;
        pass_z.setInputCloud(cloud_clean);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(0.5, 2.5);
        pass_z.filter(*cloud_z);

        if (cloud_z->empty()) {
            return;
        }

        // ------------------------------------------------
        // 4. PassThrough X filter
        //    X = left/right width
        // ------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_x(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::PassThrough<pcl::PointXYZI> pass_x;
        pass_x.setInputCloud(cloud_z);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-2.0, 2.0);
        pass_x.filter(*cloud_x);

        if (cloud_x->empty()) {
            return;
        }

        // ------------------------------------------------
        // 5. PassThrough Y filter
        //    Y = vertical image direction in optical frame
        //    This is only a rough vertical crop.
        //    Real floor separation comes later with RANSAC.
        // ------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_y(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::PassThrough<pcl::PointXYZI> pass_y;
        pass_y.setInputCloud(cloud_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-0.5, 0.5);
        pass_y.filter(*cloud_y);

        if (cloud_y->empty()) {
            return;
        }

        // ------------------------------------------------
        // 6. Voxel Grid downsampling
        //    Paper used 0.07 m voxel edge length.
        // ------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(cloud_y);
        voxel.setLeafSize(0.07f, 0.07f, 0.07f);
        voxel.filter(*cloud_voxel);

        if (cloud_voxel->empty()) {
            return;
        }

        // ------------------------------------------------
        // 7. Convert back to ROS PointCloud2 and publish
        // ------------------------------------------------
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_voxel, output);
        output.header = msg->header;

        pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PassThroughVoxelNode>());
    rclcpp::shutdown();
    return 0;
}