#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>

#include <cmath>
#include <string>
#include <vector>
#include <array>

class PassThroughVoxelRansacClusterNode : public rclcpp::Node
{
public:
    PassThroughVoxelRansacClusterNode()
        : Node("clustering_node")
    {
        this->declare_parameter<std::string>("input_topic", "/oak/points");
        this->declare_parameter<std::string>("filtered_topic", "/oak/points_filtered");
        this->declare_parameter<std::string>("ground_topic", "/oak/ground_plane");
        this->declare_parameter<std::string>("obstacles_topic", "/oak/obstacles");
        this->declare_parameter<std::string>("clusters_topic", "/oak/obstacle_clusters");

        // OAK optical frame:
        // x = left/right
        // y = vertical image direction
        // z = forward depth
        this->declare_parameter<double>("z_min", 0.5);
        this->declare_parameter<double>("z_max", 2.5);
        this->declare_parameter<double>("x_min", -2.0);
        this->declare_parameter<double>("x_max", 2.0);

        // Keep wide enough so RANSAC can still see floor.
        this->declare_parameter<double>("y_min", -0.8);
        this->declare_parameter<double>("y_max", 1.0);

        // Paper used 0.07 m.
        this->declare_parameter<double>("voxel_leaf", 0.07);

        this->declare_parameter<double>("ransac_distance_threshold", 0.04);
        this->declare_parameter<int>("ransac_max_iterations", 100);

        this->declare_parameter<bool>("use_axis_constraint", true);
        this->declare_parameter<double>("axis_x", 0.0);
        this->declare_parameter<double>("axis_y", 1.0);
        this->declare_parameter<double>("axis_z", 0.0);
        this->declare_parameter<double>("eps_angle_deg", 35.0);

        // Signed distance from ground plane.
        this->declare_parameter<double>("min_obstacle_height", 0.06);
        this->declare_parameter<double>("max_obstacle_height", 0.60);

        // Sparse noise filter before clustering.
        this->declare_parameter<double>("noise_radius", 0.15);
        this->declare_parameter<int>("noise_min_neighbors", 3);

        // Euclidean clustering.
        // Increase min_cluster_size if small noise still appears.
        this->declare_parameter<double>("cluster_tolerance", 0.18);
        this->declare_parameter<int>("min_cluster_size", 20);
        this->declare_parameter<int>("max_cluster_size", 50000);

        input_topic_ = this->get_parameter("input_topic").as_string();
        filtered_topic_ = this->get_parameter("filtered_topic").as_string();
        ground_topic_ = this->get_parameter("ground_topic").as_string();
        obstacles_topic_ = this->get_parameter("obstacles_topic").as_string();
        clusters_topic_ = this->get_parameter("clusters_topic").as_string();

        rclcpp::SensorDataQoS qos;

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            qos,
            std::bind(&PassThroughVoxelRansacClusterNode::callback, this, std::placeholders::_1)
        );

        filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            filtered_topic_,
            qos
        );

        ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ground_topic_,
            qos
        );

        obstacles_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            obstacles_topic_,
            qos
        );

        clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            clusters_topic_,
            qos
        );

        RCLCPP_INFO(this->get_logger(), "PCL PassThrough + VoxelGrid + RANSAC + Euclidean Clustering node started");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud_raw);

        if (cloud_raw->empty()) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZI>);
        std::vector<int> valid_indices;
        pcl::removeNaNFromPointCloud(*cloud_raw, *cloud_clean, valid_indices);

        if (cloud_clean->empty()) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> pass_z;
        pass_z.setInputCloud(cloud_clean);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(
            this->get_parameter("z_min").as_double(),
            this->get_parameter("z_max").as_double()
        );
        pass_z.filter(*cloud_z);

        if (cloud_z->empty()) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_x(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> pass_x;
        pass_x.setInputCloud(cloud_z);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(
            this->get_parameter("x_min").as_double(),
            this->get_parameter("x_max").as_double()
        );
        pass_x.filter(*cloud_x);

        if (cloud_x->empty()) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_y(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> pass_y;
        pass_y.setInputCloud(cloud_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(
            this->get_parameter("y_min").as_double(),
            this->get_parameter("y_max").as_double()
        );
        pass_y.filter(*cloud_y);

        if (cloud_y->empty()) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZI>);

        const float leaf = static_cast<float>(
            this->get_parameter("voxel_leaf").as_double()
        );

        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(cloud_y);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*cloud_voxel);

        if (cloud_voxel->empty()) {
            return;
        }

        publishCloudXYZI(cloud_voxel, msg->header, filtered_pub_);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);

        const bool use_axis_constraint =
            this->get_parameter("use_axis_constraint").as_bool();

        if (use_axis_constraint) {
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);

            Eigen::Vector3f axis(
                static_cast<float>(this->get_parameter("axis_x").as_double()),
                static_cast<float>(this->get_parameter("axis_y").as_double()),
                static_cast<float>(this->get_parameter("axis_z").as_double())
            );

            if (axis.norm() > 0.001f) {
                axis.normalize();
            } else {
                axis = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
            }

            constexpr double pi = 3.14159265358979323846;
            const double eps_angle_deg =
                this->get_parameter("eps_angle_deg").as_double();

            seg.setAxis(axis);
            seg.setEpsAngle(eps_angle_deg * pi / 180.0);
        } else {
            seg.setModelType(pcl::SACMODEL_PLANE);
        }

        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(
            this->get_parameter("ransac_max_iterations").as_int()
        );
        seg.setDistanceThreshold(
            this->get_parameter("ransac_distance_threshold").as_double()
        );

        seg.setInputCloud(cloud_voxel);
        seg.segment(*ground_inliers, *coefficients);

        if (ground_inliers->indices.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "RANSAC could not find a ground plane."
            );
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud_voxel);
        extract.setIndices(ground_inliers);

        extract.setNegative(false);
        extract.filter(*ground_cloud);

        extract.setNegative(true);
        extract.filter(*raw_obstacle_cloud);

        publishCloudXYZI(ground_cloud, msg->header, ground_pub_);

        if (raw_obstacle_cloud->empty()) {
            publishCloudXYZI(raw_obstacle_cloud, msg->header, obstacles_pub_);
            publishEmptyRGBCloud(msg->header);
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr height_filtered_obstacles(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        const double min_h = this->get_parameter("min_obstacle_height").as_double();
        const double max_h = this->get_parameter("max_obstacle_height").as_double();

        if (coefficients->values.size() < 4) {
            return;
        }

        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];
        double d = coefficients->values[3];

        double denom = std::sqrt(a * a + b * b + c * c);

        if (denom < 1e-6) {
            return;
        }

        a /= denom;
        b /= denom;
        c /= denom;
        d /= denom;

        // Camera origin side should be positive.
        if (d < 0.0) {
            a = -a;
            b = -b;
            c = -c;
            d = -d;
        }

        for (const auto& p : raw_obstacle_cloud->points) {
            const double signed_distance_to_ground =
                a * p.x + b * p.y + c * p.z + d;

            if (signed_distance_to_ground >= min_h &&
                signed_distance_to_ground <= max_h) {
                height_filtered_obstacles->points.push_back(p);
            }
        }

        height_filtered_obstacles->width =
            static_cast<uint32_t>(height_filtered_obstacles->points.size());
        height_filtered_obstacles->height = 1;
        height_filtered_obstacles->is_dense = true;

        if (height_filtered_obstacles->empty()) {
            publishCloudXYZI(height_filtered_obstacles, msg->header, obstacles_pub_);
            publishEmptyRGBCloud(msg->header);
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr clean_obstacles(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_filter;
        radius_filter.setInputCloud(height_filtered_obstacles);
        radius_filter.setRadiusSearch(
            this->get_parameter("noise_radius").as_double()
        );
        radius_filter.setMinNeighborsInRadius(
            this->get_parameter("noise_min_neighbors").as_int()
        );
        radius_filter.filter(*clean_obstacles);

        clean_obstacles->width =
            static_cast<uint32_t>(clean_obstacles->points.size());
        clean_obstacles->height = 1;
        clean_obstacles->is_dense = true;

        publishCloudXYZI(clean_obstacles, msg->header, obstacles_pub_);

        if (clean_obstacles->empty()) {
            publishEmptyRGBCloud(msg->header);
            return;
        }

        // ------------------------------------------------
        // Euclidean Clustering
        // ------------------------------------------------
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZI>
        );

        tree->setInputCloud(clean_obstacles);

        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(
            this->get_parameter("cluster_tolerance").as_double()
        );
        ec.setMinClusterSize(
            this->get_parameter("min_cluster_size").as_int()
        );
        ec.setMaxClusterSize(
            this->get_parameter("max_cluster_size").as_int()
        );
        ec.setSearchMethod(tree);
        ec.setInputCloud(clean_obstacles);
        ec.extract(cluster_indices);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloured_clusters(
            new pcl::PointCloud<pcl::PointXYZRGB>
        );

        const std::array<std::array<uint8_t, 3>, 8> colours = {{
            {{255, 0, 0}},
            {{0, 255, 0}},
            {{0, 0, 255}},
            {{255, 255, 0}},
            {{255, 0, 255}},
            {{0, 255, 255}},
            {{255, 128, 0}},
            {{128, 0, 255}}
        }};

        int cluster_id = 0;

        for (const auto& indices : cluster_indices) {
            const auto& colour = colours[cluster_id % colours.size()];

            for (const auto& idx : indices.indices) {
                const auto& src = clean_obstacles->points[idx];

                pcl::PointXYZRGB dst;
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.r = colour[0];
                dst.g = colour[1];
                dst.b = colour[2];

                coloured_clusters->points.push_back(dst);
            }

            cluster_id++;
        }

        coloured_clusters->width =
            static_cast<uint32_t>(coloured_clusters->points.size());
        coloured_clusters->height = 1;
        coloured_clusters->is_dense = true;

        publishCloudRGB(coloured_clusters, msg->header, clusters_pub_);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Detected clusters: %zu | obstacle candidate points: %zu",
            cluster_indices.size(),
            clean_obstacles->points.size()
        );
    }

    void publishCloudXYZI(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const std_msgs::msg::Header& header,
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher
    )
    {
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header = header;
        publisher->publish(output);
    }

    void publishCloudRGB(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
        const std_msgs::msg::Header& header,
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher
    )
    {
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header = header;
        publisher->publish(output);
    }

    void publishEmptyRGBCloud(const std_msgs::msg::Header& header)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>
        );
        empty_cloud->width = 0;
        empty_cloud->height = 1;
        empty_cloud->is_dense = true;
        publishCloudRGB(empty_cloud, header, clusters_pub_);
    }

    std::string input_topic_;
    std::string filtered_topic_;
    std::string ground_topic_;
    std::string obstacles_topic_;
    std::string clusters_topic_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PassThroughVoxelRansacClusterNode>());
    rclcpp::shutdown();
    return 0;
}