#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <vector>
#include <cmath>
#include <unordered_set>

class DBSCANObstacleNode : public rclcpp::Node
{
public:
    DBSCANObstacleNode()
    : Node("dbscan_obstacle_node")
    {
        auto qos = rclcpp::SensorDataQoS();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/oak/points_temporal_filtered",
            qos,
            std::bind(&DBSCANObstacleNode::callback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/oak/stable_obstacles",
            qos
        );

        RCLCPP_INFO(this->get_logger(), "DBSCAN (custom) Node Started");
    }

private:

    struct Pt
    {
        float x, y, z;
    };

    float dist(const Pt &a, const Pt &b)
    {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        float dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    void regionQuery(int idx,
                     const std::vector<Pt> &pts,
                     std::vector<int> &neighbors,
                     float eps)
    {
        for (size_t i = 0; i < pts.size(); i++)
        {
            if (dist(pts[idx], pts[i]) < eps)
                neighbors.push_back(i);
        }
    }

    void expandCluster(int idx,
                       int cluster_id,
                       const std::vector<Pt> &pts,
                       std::vector<int> &labels,
                       float eps,
                       int minPts)
    {
        std::vector<int> seeds;
        regionQuery(idx, pts, seeds, eps);

        if (seeds.size() < minPts)
        {
            labels[idx] = -1;
            return;
        }

        labels[idx] = cluster_id;

        std::unordered_set<int> seed_set(seeds.begin(), seeds.end());

        for (auto it = seed_set.begin(); it != seed_set.end(); ++it)
        {
            int pt_idx = *it;

            if (labels[pt_idx] == -1)
                labels[pt_idx] = cluster_id;

            if (labels[pt_idx] != 0)
                continue;

            labels[pt_idx] = cluster_id;

            std::vector<int> result;
            regionQuery(pt_idx, pts, result, eps);

            if (result.size() >= minPts)
            {
                seed_set.insert(result.begin(), result.end());
            }
        }
    }

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::fromROSMsg(*msg, *cloud);

        std::vector<Pt> pts;
        for (auto &p : cloud->points)
        {
            if (!std::isfinite(p.x) ||
                !std::isfinite(p.y) ||
                !std::isfinite(p.z))
                continue;

            pts.push_back({p.x, p.y, p.z});
        }

        std::vector<int> labels(pts.size(), 0);

        float eps = 0.15f;
        int minPts = 6;

        int cluster_id = 1;

        for (size_t i = 0; i < pts.size(); i++)
        {
            if (labels[i] != 0)
                continue;

            expandCluster(i, cluster_id, pts, labels, eps, minPts);
            cluster_id++;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr out(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        for (size_t i = 0; i < pts.size(); i++)
        {
            if (labels[i] > 0)
            {
                pcl::PointXYZI p;
                p.x = pts[i].x;
                p.y = pts[i].y;
                p.z = pts[i].z;
                p.intensity = labels[i];

                out->points.push_back(p);
            }
        }

        out->width = out->points.size();
        out->height = 1;
        out->is_dense = true;

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*out, output);
        output.header = msg->header;

        pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DBSCANObstacleNode>());
    rclcpp::shutdown();
    return 0;
}