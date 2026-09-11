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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>

#include <cmath>
#include <string>
#include <vector>

class PassThroughVoxelRansacNode : public rclcpp::Node
{
public:
    PassThroughVoxelRansacNode()
        : Node("pcl_passthrough_voxel_ransac_node")
    {
        // -----------------------------
        // Topics
        // -----------------------------
        this->declare_parameter<std::string>("input_topic", "/oak/points");
        this->declare_parameter<std::string>("filtered_topic", "/oak/points_filtered");
        this->declare_parameter<std::string>("ground_topic", "/oak/ground_plane");
        this->declare_parameter<std::string>("obstacles_topic", "/oak/obstacles");

        // -----------------------------
        // PassThrough ROI parameters
        //
        // OAK optical frame:
        // x = left/right
        // y = vertical image direction
        // z = forward depth
        // -----------------------------
        this->declare_parameter<double>("z_min", 0.5);
        this->declare_parameter<double>("z_max", 2.5);

        this->declare_parameter<double>("x_min", -2.0);
        this->declare_parameter<double>("x_max", 2.0);

        // Keep this wider so RANSAC can still see the floor.
        this->declare_parameter<double>("y_min", -0.8);
        this->declare_parameter<double>("y_max", 1.0);

        // Paper used 0.07 m.
        this->declare_parameter<double>("voxel_leaf", 0.07);

        // -----------------------------
        // RANSAC parameters
        // -----------------------------
        this->declare_parameter<double>("ransac_distance_threshold", 0.04);
        this->declare_parameter<int>("ransac_max_iterations", 100);

        // Axis constraint helps RANSAC find floor instead of wall.
        // In OAK optical frame, vertical is usually Y.
        this->declare_parameter<bool>("use_axis_constraint", true);
        this->declare_parameter<double>("axis_x", 0.0);
        this->declare_parameter<double>("axis_y", 1.0);
        this->declare_parameter<double>("axis_z", 0.0);

        // Increase this if camera is tilted strongly.
        this->declare_parameter<double>("eps_angle_deg", 35.0);

        // -----------------------------
        // Obstacle candidate filtering
        //
        // These are signed distances from the RANSAC ground plane.
        // min_obstacle_height removes ground residue.
        // max_obstacle_height removes high clutter.
        // -----------------------------
        this->declare_parameter<double>("min_obstacle_height", 0.06);
        this->declare_parameter<double>("max_obstacle_height", 0.60);

        // Radius outlier removal for sparse/floating noise.
        this->declare_parameter<double>("noise_radius", 0.15);
        this->declare_parameter<int>("noise_min_neighbors", 3);

        input_topic_ = this->get_parameter("input_topic").as_string();
        filtered_topic_ = this->get_parameter("filtered_topic").as_string();
        ground_topic_ = this->get_parameter("ground_topic").as_string();
        obstacles_topic_ = this->get_parameter("obstacles_topic").as_string();

        rclcpp::SensorDataQoS qos;

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            qos,
            std::bind(&PassThroughVoxelRansacNode::callback, this, std::placeholders::_1)
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

        RCLCPP_INFO(this->get_logger(), "PCL PassThrough + VoxelGrid + RANSAC + signed obstacle cleanup node started");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Filtered topic: %s", filtered_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Ground topic: %s", ground_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Obstacles topic: %s", obstacles_topic_.c_str());
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // ------------------------------------------------
        // 1. Convert ROS PointCloud2 to PCL cloud
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
        // 3. PassThrough Z: forward depth
        // ------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_z(
            new pcl::PointCloud<pcl::PointXYZI>
        );

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

        // ------------------------------------------------
        // 4. PassThrough X: left/right width
        // ------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_x(
            new pcl::PointCloud<pcl::PointXYZI>
        );

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

        // ------------------------------------------------
        // 5. PassThrough Y: vertical crop
        //    Keep wider than obstacle layer so RANSAC can see floor.
        // ------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_y(
            new pcl::PointCloud<pcl::PointXYZI>
        );

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

        // ------------------------------------------------
        // 6. Voxel Grid downsampling
        // ------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel(
            new pcl::PointCloud<pcl::PointXYZI>
        );

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

        // Publish PassThrough + voxel cloud for debugging
        publishCloud(cloud_voxel, msg->header, filtered_pub_);

        // ------------------------------------------------
        // 7. RANSAC ground plane segmentation
        // ------------------------------------------------
        pcl::ModelCoefficients::Ptr coefficients(
            new pcl::ModelCoefficients
        );

        pcl::PointIndices::Ptr ground_inliers(
            new pcl::PointIndices
        );

        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);

        const bool use_axis_constraint =
            this->get_parameter("use_axis_constraint").as_bool();

        if (use_axis_constraint) {
            // Plane perpendicular to vertical axis.
            // This helps choose floor instead of wall.
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

            const double eps_angle_rad =
                eps_angle_deg * pi / 180.0;

            seg.setAxis(axis);
            seg.setEpsAngle(eps_angle_rad);
        } else {
            // Fallback: finds largest plane.
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
                "RANSAC could not find a ground plane. Publishing filtered cloud as obstacles."
            );

            publishCloud(cloud_voxel, msg->header, obstacles_pub_);
            return;
        }

        // ------------------------------------------------
        // 8. Extract ground plane and raw non-ground points
        // ------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_obstacle_cloud(
            new pcl::PointCloud<pcl::PointXYZI>
        );

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud_voxel);
        extract.setIndices(ground_inliers);

        // Ground plane inliers
        extract.setNegative(false);
        extract.filter(*ground_cloud);

        // Non-ground outliers
        extract.setNegative(true);
        extract.filter(*raw_obstacle_cloud);

        publishCloud(ground_cloud, msg->header, ground_pub_);

        if (raw_obstacle_cloud->empty()) {
            publishCloud(raw_obstacle_cloud, msg->header, obstacles_pub_);
            return;
        }

        // ------------------------------------------------
        // 9. Keep only obstacle candidates ABOVE the floor
        //
        // IMPORTANT:
        // This uses signed distance, not abs distance.
        //
        // abs(distance) keeps both:
        //   above-floor points
        //   below-floor points
        //
        // Signed distance lets us remove below-floor artifacts.
        //
        // Plane equation:
        //   ax + by + cz + d = 0
        //
        // We orient the plane normal so the camera side is positive.
        // Since the camera is above the floor, positive distance means
        // "above ground" for our obstacle candidate layer.
        // ------------------------------------------------
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

        // Normalize plane coefficients
        a /= denom;
        b /= denom;
        c /= denom;
        d /= denom;

        // Camera origin is approximately (0, 0, 0).
        // Plane value at camera origin = d.
        // If camera side is negative, flip normal so camera side becomes positive.
        const double camera_side_value = d;

        if (camera_side_value < 0.0) {
            a = -a;
            b = -b;
            c = -c;
            d = -d;
        }

        for (const auto& p : raw_obstacle_cloud->points) {
            const double signed_distance_to_ground =
                a * p.x + b * p.y + c * p.z + d;

            // Keep only points on camera side of ground plane.
            // This removes below-floor points.
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
            publishCloud(height_filtered_obstacles, msg->header, obstacles_pub_);
            return;
        }

        // ------------------------------------------------
        // 10. Radius Outlier Removal
        //     Removes isolated floating noise points.
        // ------------------------------------------------
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

        // ------------------------------------------------
        // 11. Publish cleaned obstacle candidate cloud
        // ------------------------------------------------
        publishCloud(clean_obstacles, msg->header, obstacles_pub_);
    }

    void publishCloud(
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

    std::string input_topic_;
    std::string filtered_topic_;
    std::string ground_topic_;
    std::string obstacles_topic_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PassThroughVoxelRansacNode>());
    rclcpp::shutdown();
    return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>

// #include <pcl/point_types.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/extract_indices.h>

// #include <pcl/ModelCoefficients.h>
// #include <pcl/PointIndices.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>

// #include <pcl_conversions/pcl_conversions.h>

// #include <Eigen/Core>

// class PassThroughVoxelRansacNode : public rclcpp::Node
// {
// public:
//     PassThroughVoxelRansacNode()
//         : Node("pcl_passthrough_voxel_ransac_node")
//     {
//         // -----------------------------
//         // Parameters
//         // -----------------------------
//         this->declare_parameter<std::string>("input_topic", "/oak/points");
//         this->declare_parameter<std::string>("filtered_topic", "/oak/points_filtered");
//         this->declare_parameter<std::string>("ground_topic", "/oak/ground_plane");
//         this->declare_parameter<std::string>("obstacles_topic", "/oak/obstacles");

//         // OAK optical frame:
//         // x = left/right
//         // y = vertical image direction
//         // z = forward depth
//         this->declare_parameter<double>("z_min", 0.5);
//         this->declare_parameter<double>("z_max", 2.5);
//         this->declare_parameter<double>("x_min", -2.0);
//         this->declare_parameter<double>("x_max", 2.0);

//         // Keep this wider for RANSAC, otherwise floor can be removed before plane fitting.
//         this->declare_parameter<double>("y_min", -0.8);
//         this->declare_parameter<double>("y_max", 1.0);

//         // Paper used 0.07 m.
//         this->declare_parameter<double>("voxel_leaf", 0.07);

//         this->declare_parameter<double>("ransac_distance_threshold", 0.04);
//         this->declare_parameter<int>("ransac_max_iterations", 100);

//         // Axis constraint helps RANSAC choose floor instead of wall.
//         // In OAK optical frame, vertical is usually Y.
//         this->declare_parameter<bool>("use_axis_constraint", true);
//         this->declare_parameter<double>("axis_x", 0.0);
//         this->declare_parameter<double>("axis_y", 1.0);
//         this->declare_parameter<double>("axis_z", 0.0);

//         // 35 degrees tolerance. Increase if camera is tilted.
//         this->declare_parameter<double>("eps_angle_deg", 35.0);

//         input_topic_ = this->get_parameter("input_topic").as_string();
//         filtered_topic_ = this->get_parameter("filtered_topic").as_string();
//         ground_topic_ = this->get_parameter("ground_topic").as_string();
//         obstacles_topic_ = this->get_parameter("obstacles_topic").as_string();

//         rclcpp::SensorDataQoS qos;

//         sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             input_topic_,
//             qos,
//             std::bind(&PassThroughVoxelRansacNode::callback, this, std::placeholders::_1)
//         );

//         filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//             filtered_topic_,
//             qos
//         );

//         ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//             ground_topic_,
//             qos
//         );

//         obstacles_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//             obstacles_topic_,
//             qos
//         );

//         RCLCPP_INFO(this->get_logger(), "PCL PassThrough + VoxelGrid + RANSAC node started");
//         RCLCPP_INFO(this->get_logger(), "Input: %s", input_topic_.c_str());
//         RCLCPP_INFO(this->get_logger(), "Filtered: %s", filtered_topic_.c_str());
//         RCLCPP_INFO(this->get_logger(), "Ground: %s", ground_topic_.c_str());
//         RCLCPP_INFO(this->get_logger(), "Obstacles: %s", obstacles_topic_.c_str());
//     }

// private:
//     void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//     {
//         // ------------------------------------------------
//         // 1. Convert ROS PointCloud2 to PCL cloud
//         // ------------------------------------------------
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(
//             new pcl::PointCloud<pcl::PointXYZI>
//         );

//         pcl::fromROSMsg(*msg, *cloud_raw);

//         if (cloud_raw->empty()) {
//             return;
//         }

//         // ------------------------------------------------
//         // 2. Remove NaN / invalid points
//         // ------------------------------------------------
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clean(
//             new pcl::PointCloud<pcl::PointXYZI>
//         );

//         std::vector<int> valid_indices;
//         pcl::removeNaNFromPointCloud(*cloud_raw, *cloud_clean, valid_indices);

//         if (cloud_clean->empty()) {
//             return;
//         }

//         // ------------------------------------------------
//         // 3. PassThrough Z: forward depth
//         // ------------------------------------------------
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_z(
//             new pcl::PointCloud<pcl::PointXYZI>
//         );

//         pcl::PassThrough<pcl::PointXYZI> pass_z;
//         pass_z.setInputCloud(cloud_clean);
//         pass_z.setFilterFieldName("z");
//         pass_z.setFilterLimits(
//             this->get_parameter("z_min").as_double(),
//             this->get_parameter("z_max").as_double()
//         );
//         pass_z.filter(*cloud_z);

//         if (cloud_z->empty()) {
//             return;
//         }

//         // ------------------------------------------------
//         // 4. PassThrough X: left/right width
//         // ------------------------------------------------
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_x(
//             new pcl::PointCloud<pcl::PointXYZI>
//         );

//         pcl::PassThrough<pcl::PointXYZI> pass_x;
//         pass_x.setInputCloud(cloud_z);
//         pass_x.setFilterFieldName("x");
//         pass_x.setFilterLimits(
//             this->get_parameter("x_min").as_double(),
//             this->get_parameter("x_max").as_double()
//         );
//         pass_x.filter(*cloud_x);

//         if (cloud_x->empty()) {
//             return;
//         }

//         // ------------------------------------------------
//         // 5. PassThrough Y: vertical crop
//         //    Kept wider so floor remains available for RANSAC.
//         // ------------------------------------------------
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_y(
//             new pcl::PointCloud<pcl::PointXYZI>
//         );

//         pcl::PassThrough<pcl::PointXYZI> pass_y;
//         pass_y.setInputCloud(cloud_x);
//         pass_y.setFilterFieldName("y");
//         pass_y.setFilterLimits(
//             this->get_parameter("y_min").as_double(),
//             this->get_parameter("y_max").as_double()
//         );
//         pass_y.filter(*cloud_y);

//         if (cloud_y->empty()) {
//             return;
//         }

//         // ------------------------------------------------
//         // 6. Voxel Grid downsampling
//         // ------------------------------------------------
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel(
//             new pcl::PointCloud<pcl::PointXYZI>
//         );

//         const float leaf = static_cast<float>(
//             this->get_parameter("voxel_leaf").as_double()
//         );

//         pcl::VoxelGrid<pcl::PointXYZI> voxel;
//         voxel.setInputCloud(cloud_y);
//         voxel.setLeafSize(leaf, leaf, leaf);
//         voxel.filter(*cloud_voxel);

//         if (cloud_voxel->empty()) {
//             return;
//         }

//         // Publish filtered + voxel cloud for debugging
//         publishCloud(cloud_voxel, msg->header, filtered_pub_);

//         // ------------------------------------------------
//         // 7. RANSAC ground plane segmentation
//         // ------------------------------------------------
//         pcl::ModelCoefficients::Ptr coefficients(
//             new pcl::ModelCoefficients
//         );

//         pcl::PointIndices::Ptr ground_inliers(
//             new pcl::PointIndices
//         );

//         pcl::SACSegmentation<pcl::PointXYZI> seg;
//         seg.setOptimizeCoefficients(true);

//         const bool use_axis_constraint =
//             this->get_parameter("use_axis_constraint").as_bool();

//         if (use_axis_constraint) {
//             // Plane perpendicular to vertical axis.
//             // This tries to find a floor-like plane instead of any random wall.
//             seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);

//             Eigen::Vector3f axis(
//                 static_cast<float>(this->get_parameter("axis_x").as_double()),
//                 static_cast<float>(this->get_parameter("axis_y").as_double()),
//                 static_cast<float>(this->get_parameter("axis_z").as_double())
//             );

//             if (axis.norm() > 0.001f) {
//                 axis.normalize();
//             } else {
//                 axis = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
//             }

//             const double eps_angle_deg =
//                 this->get_parameter("eps_angle_deg").as_double();

//             const double eps_angle_rad =
//                 eps_angle_deg * M_PI / 180.0;

//             seg.setAxis(axis);
//             seg.setEpsAngle(eps_angle_rad);
//         } else {
//             // Unconstrained fallback: finds largest plane.
//             seg.setModelType(pcl::SACMODEL_PLANE);
//         }

//         seg.setMethodType(pcl::SAC_RANSAC);
//         seg.setMaxIterations(
//             this->get_parameter("ransac_max_iterations").as_int()
//         );
//         seg.setDistanceThreshold(
//             this->get_parameter("ransac_distance_threshold").as_double()
//         );

//         seg.setInputCloud(cloud_voxel);
//         seg.segment(*ground_inliers, *coefficients);

//         if (ground_inliers->indices.empty()) {
//             RCLCPP_WARN_THROTTLE(
//                 this->get_logger(),
//                 *this->get_clock(),
//                 2000,
//                 "RANSAC could not find a ground plane. Publishing all filtered points as obstacles."
//             );

//             publishCloud(cloud_voxel, msg->header, obstacles_pub_);
//             return;
//         }

//         // ------------------------------------------------
//         // 8. Extract ground plane and obstacles separately
//         // ------------------------------------------------
//         pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(
//             new pcl::PointCloud<pcl::PointXYZI>
//         );

//         pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud(
//             new pcl::PointCloud<pcl::PointXYZI>
//         );

//         pcl::ExtractIndices<pcl::PointXYZI> extract;
//         extract.setInputCloud(cloud_voxel);
//         extract.setIndices(ground_inliers);

//         // Ground plane inliers
//         extract.setNegative(false);
//         extract.filter(*ground_cloud);

//         // Non-ground outliers = candidate obstacles
//         extract.setNegative(true);
//         extract.filter(*obstacle_cloud);

//         publishCloud(ground_cloud, msg->header, ground_pub_);
//         publishCloud(obstacle_cloud, msg->header, obstacles_pub_);
//     }

//     void publishCloud(
//         const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
//         const std_msgs::msg::Header& header,
//         const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher
//     )
//     {
//         sensor_msgs::msg::PointCloud2 output;
//         pcl::toROSMsg(*cloud, output);
//         output.header = header;
//         publisher->publish(output);
//     }

//     std::string input_topic_;
//     std::string filtered_topic_;
//     std::string ground_topic_;
//     std::string obstacles_topic_;

//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<PassThroughVoxelRansacNode>());
//     rclcpp::shutdown();
//     return 0;
// }