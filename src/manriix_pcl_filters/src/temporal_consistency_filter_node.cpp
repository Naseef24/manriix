#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/kdtree/kdtree_flann.h>

class TemporalConsistencyFilter : public rclcpp::Node
{
public:
    TemporalConsistencyFilter()
    : Node("temporal_consistency_filter")
    {
        auto qos = rclcpp::SensorDataQoS();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/oak/points",
            qos,
            std::bind(&TemporalConsistencyFilter::callback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/oak/points_temporal_filtered",
            qos
        );

        RCLCPP_INFO(this->get_logger(), "Temporal Consistency Filter Started");
    }

private:

    pcl::PointCloud<pcl::PointXYZI>::Ptr prev_cloud_;

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::fromROSMsg(*msg, *cloud);

        if (!prev_cloud_)
        {
            prev_cloud_ = cloud;
            return;
        }

        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(prev_cloud_);

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        const float radius_threshold = 0.08f;

        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);

        for (const auto &pt : cloud->points)
        {
            // ✅ FINAL SAFE CHECK (NO PCL isFinite)
            if (!std::isfinite(pt.x) ||
                !std::isfinite(pt.y) ||
                !std::isfinite(pt.z))
            {
                continue;
            }

            if (kdtree.nearestKSearch(pt, 1, nn_indices, nn_dists) > 0)
            {
                if (nn_dists[0] < radius_threshold * radius_threshold)
                {
                    filtered->points.push_back(pt);
                }
            }
        }

        filtered->width = filtered->points.size();
        filtered->height = 1;
        filtered->is_dense = true;

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*filtered, output);
        output.header = msg->header;

        pub_->publish(output);

        prev_cloud_ = cloud;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemporalConsistencyFilter>());
    rclcpp::shutdown();
    return 0;
}