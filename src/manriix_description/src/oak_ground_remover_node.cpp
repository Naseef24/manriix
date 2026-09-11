// ============================================================================
// oak_ground_remover_node.cpp
//
// Subscribes to a PointCloud2 topic (typically /oak/points_reliable from the
// qos_bridge), runs PCL RANSAC plane segmentation constrained to near-horizontal
// orientation, and publishes two outputs:
//   - <output_obstacles>  : non-ground points only  (consumed by STVL)
//   - <output_ground>     : ground points only      (debug/viz)
//
// Performance is logged every N seconds: input rate, output rate, RANSAC
// latency (mean & p95), inlier fraction.
//
// Standalone diagnostic experiment — NOT integrated into Manriix nav stack.
// Run alongside the existing OAK pipeline; delete this file freely if not used.
//
// File: ~/manriix2_ws/src/manriix_description/src/oak_ground_remover_node.cpp
// ============================================================================

#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>       // pcl::removeNaNFromPointCloud
#include <pcl/filters/passthrough.h>  // pcl::PassThrough
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class OakGroundRemover : public rclcpp::Node
{
public:
  OakGroundRemover() : Node("oak_ground_remover")
  {
    // ----- Parameters -----
    input_topic_      = declare_parameter<std::string>("input_topic", "/oak/points_reliable");
    obstacles_topic_  = declare_parameter<std::string>("obstacles_topic", "/oak/points_obstacles");
    ground_topic_     = declare_parameter<std::string>("ground_topic", "/oak/points_ground");
    publish_ground_   = declare_parameter<bool>("publish_ground", true);

    voxel_leaf_size_  = declare_parameter<double>("voxel_leaf_size", 0.05);   // 5 cm
    plane_distance_threshold_ = declare_parameter<double>("plane_distance_threshold", 0.05);  // 5 cm
    max_iterations_   = declare_parameter<int>("max_iterations", 100);
    epsilon_angle_deg_= declare_parameter<double>("epsilon_angle_deg", 15.0);
    min_inliers_      = declare_parameter<int>("min_inliers", 200);

    // ----- Range / spatial clipping (camera optical frame: +Y down, +Z forward) -----
    // These are the new filter parameters that kill wall-blob false detections
    // and below-floor stereo noise. Values for OAK mounted ~0.23 m above floor.
    min_range_         = declare_parameter<double>("min_range", 0.20);          // Z min (m)
    max_range_         = declare_parameter<double>("max_range", 4.00);          // Z max (m) — was 8.0
    floor_clip_m_      = declare_parameter<double>("floor_clip_m", 0.40);       // Y max (drop noise below floor: floor is at Y≈0.23 in optical)
    ceiling_clip_m_    = declare_parameter<double>("ceiling_clip_m", 1.50);     // |Y| above camera origin (drop ceiling noise)
    obstacle_max_range_ = declare_parameter<double>("obstacle_max_range", 4.0); // Z cap on FINAL obstacle cloud

    // ----- Euclidean clustering to reject sparse stereo noise -----
    cluster_tolerance_   = declare_parameter<double>("cluster_tolerance", 0.10);  // 10 cm grouping radius
    cluster_min_size_    = declare_parameter<int>("cluster_min_size", 30);        // Min points per cluster
    cluster_max_size_    = declare_parameter<int>("cluster_max_size", 25000);

    // Axis along which the plane should be perpendicular. For OAK point clouds
    // in oak_d_pro_w_right_camera_optical_frame:
    //   X = right, Y = down, Z = forward  (camera optical convention)
    // The floor's normal is ≈ -Y in optical frame.
    axis_x_ = declare_parameter<double>("axis_x", 0.0);
    axis_y_ = declare_parameter<double>("axis_y", -1.0);
    axis_z_ = declare_parameter<double>("axis_z", 0.0);

    stats_log_period_ = declare_parameter<double>("stats_log_period_sec", 5.0);

    // ----- ROS interfaces -----
    auto qos = rclcpp::QoS(5).reliable();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&OakGroundRemover::onCloud, this, std::placeholders::_1));

    pub_obstacles_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacles_topic_, qos);
    if (publish_ground_) {
      pub_ground_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic_, qos);
    }

    stats_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(stats_log_period_ * 1000.0)),
      std::bind(&OakGroundRemover::logStats, this));

    RCLCPP_INFO(get_logger(),
      "oak_ground_remover started:\n"
      "  input:           %s (RELIABLE)\n"
      "  obstacles out:   %s\n"
      "  ground out:      %s%s\n"
      "  voxel leaf:      %.3f m\n"
      "  plane distance:  %.3f m\n"
      "  axis (optical):  (%.1f, %.1f, %.1f)\n"
      "  epsilon angle:   %.1f deg\n"
      "  max iterations:  %d\n"
      "  min inliers:     %d",
      input_topic_.c_str(),
      obstacles_topic_.c_str(),
      ground_topic_.c_str(), publish_ground_ ? "" : " (disabled)",
      voxel_leaf_size_,
      plane_distance_threshold_,
      axis_x_, axis_y_, axis_z_,
      epsilon_angle_deg_,
      max_iterations_,
      min_inliers_);
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const auto t_start = std::chrono::steady_clock::now();

    // Log frame_id once so we know what frame the cloud is in
    if (!frame_logged_) {
      RCLCPP_INFO(get_logger(), "First cloud received — frame_id='%s'  (verify axis is correct for this frame)", msg->header.frame_id.c_str());
      frame_logged_ = true;
    }

    // ----- 1. Convert ROS → PCL -----
    auto cloud_raw = std::make_shared<CloudT>();
    pcl::fromROSMsg(*msg, *cloud_raw);
    const size_t n_raw = cloud_raw->size();

    if (n_raw == 0) {
      return;
    }

    // ----- 1b. Strip NaN/inf points (OAK clouds are not dense) -----
    auto cloud_clean = std::make_shared<CloudT>();
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*cloud_raw, *cloud_clean, nan_indices);
    const size_t n_clean = cloud_clean->size();

    if (n_clean < static_cast<size_t>(min_inliers_)) {
      return;
    }

    // ----- DIAGNOSTIC: log bounding box of the cleaned cloud (once) -----
    if (!bbox_logged_) {
      float xmin = 1e30f, xmax = -1e30f;
      float ymin = 1e30f, ymax = -1e30f;
      float zmin = 1e30f, zmax = -1e30f;
      size_t n_inf = 0;
      for (const auto & p : cloud_clean->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
          n_inf++;
          continue;
        }
        xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
        zmin = std::min(zmin, p.z); zmax = std::max(zmax, p.z);
      }
      RCLCPP_INFO(get_logger(),
        "DIAG  cleaned cloud bbox: x=[%.2f, %.2f]  y=[%.2f, %.2f]  z=[%.2f, %.2f]  "
        "non-finite remaining=%zu / %zu",
        xmin, xmax, ymin, ymax, zmin, zmax, n_inf, cloud_clean->size());
      bbox_logged_ = true;
    }

    // ----- 1c. PassThrough filters: clip to sane sensor range -----
    // OAK in optical frame: X=right, Y=down, Z=forward.
    // Stereo at long range produces garbage points up to 65 m and below/above floor.
    // We aggressively clip in Y (height) and Z (depth) to drop these BEFORE RANSAC.

    // Z clip — depth range we trust
    auto cloud_zclipped = std::make_shared<CloudT>();
    {
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(cloud_clean);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(
          static_cast<float>(min_range_),
          static_cast<float>(max_range_));
      pass.filter(*cloud_zclipped);
    }

    // Y clip — drop below-floor and above-ceiling noise
    // In camera optical frame: +Y is DOWN. Floor is at Y ≈ +camera_height (0.23m).
    // Keep:  Y in [-ceiling_clip_m_, floor_clip_m_]
    //        i.e. from ceiling_clip_m_ ABOVE camera down to floor_clip_m_ below camera
    // Note: floor_clip_m_ should be slightly larger than camera mount height
    //       so the floor itself is kept (RANSAC needs floor points).
    auto cloud_clipped = std::make_shared<CloudT>();
    {
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(cloud_zclipped);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(
          static_cast<float>(-ceiling_clip_m_),
          static_cast<float>(floor_clip_m_));
      pass.filter(*cloud_clipped);
    }

    if (cloud_clipped->size() < static_cast<size_t>(min_inliers_)) {
      return;
    }

    // ----- 2. Voxel downsample -----
    auto cloud_ds = std::make_shared<CloudT>();
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_clipped);    // <-- changed from cloud_clean
    vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    vg.filter(*cloud_ds);
    const size_t n_ds = cloud_ds->size();

    // ----- 3. RANSAC plane segmentation (perpendicular to axis) -----
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations_);
    seg.setDistanceThreshold(plane_distance_threshold_);
    seg.setAxis(Eigen::Vector3f(
      static_cast<float>(axis_x_),
      static_cast<float>(axis_y_),
      static_cast<float>(axis_z_)));
    seg.setEpsAngle(epsilon_angle_deg_ * M_PI / 180.0);
    seg.setInputCloud(cloud_ds);
    seg.segment(*inliers, *coeffs);

    const size_t n_inliers = inliers->indices.size();

    bool plane_found = (static_cast<int>(n_inliers) >= min_inliers_);

    // ----- 4. Split into ground / obstacles -----
    auto cloud_obstacles = std::make_shared<CloudT>();
    auto cloud_ground    = std::make_shared<CloudT>();

    if (plane_found) {
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(cloud_ds);
      extract.setIndices(inliers);

      // Non-ground = setNegative(true)
      extract.setNegative(true);
      extract.filter(*cloud_obstacles);

      if (publish_ground_) {
        extract.setNegative(false);
        extract.filter(*cloud_ground);
      }
    } else {
      // No plane found — pass everything through as obstacles
      *cloud_obstacles = *cloud_ds;
    }

    // ----- 4b. Final obstacle range cap -----
    // Even after RANSAC, distant noisy stereo points can produce phantom
    // obstacles. Drop any obstacle point further than obstacle_max_range_.
    {
      auto obs_culled = std::make_shared<CloudT>();
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(cloud_obstacles);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(
          static_cast<float>(min_range_),
          static_cast<float>(obstacle_max_range_));
      pass.filter(*obs_culled);
      cloud_obstacles = obs_culled;
    }

    // ----- 4c. Euclidean clustering: drop sparse stereo noise -----
    // Real obstacles produce dense point clusters. Stereo noise produces
    // 1-3 scattered points. Cluster the obstacle cloud and keep only
    // clusters with enough points to be a real object.
    if (cloud_obstacles->size() >= static_cast<size_t>(cluster_min_size_)) {
      pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
      tree->setInputCloud(cloud_obstacles);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance(cluster_tolerance_);
      ec.setMinClusterSize(cluster_min_size_);
      ec.setMaxClusterSize(cluster_max_size_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud_obstacles);
      ec.extract(cluster_indices);

      auto obs_clustered = std::make_shared<CloudT>();
      for (const auto & indices : cluster_indices) {
        for (int idx : indices.indices) {
          obs_clustered->push_back(cloud_obstacles->points[idx]);
        }
      }

      // Update stats counters
      last_n_clusters_ = cluster_indices.size();
      last_n_obs_after_cluster_ = obs_clustered->size();

      cloud_obstacles = obs_clustered;
    } else {
      // Not enough points to cluster — drop them all as noise
      last_n_clusters_ = 0;
      last_n_obs_after_cluster_ = 0;
      cloud_obstacles->clear();
    }

    // ----- 5. Publish -----
    sensor_msgs::msg::PointCloud2 out_obs;
    pcl::toROSMsg(*cloud_obstacles, out_obs);
    out_obs.header = msg->header;
    pub_obstacles_->publish(out_obs);

    if (publish_ground_ && plane_found) {
      sensor_msgs::msg::PointCloud2 out_gnd;
      pcl::toROSMsg(*cloud_ground, out_gnd);
      out_gnd.header = msg->header;
      pub_ground_->publish(out_gnd);
    }

    // ----- 6. Stats -----
    const auto t_end = std::chrono::steady_clock::now();
    const double dt_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    latency_window_.push_back(dt_ms);
    if (latency_window_.size() > 200) latency_window_.pop_front();

    ++callback_count_;
    last_n_raw_ = n_raw;
    last_n_clean_ = n_clean;
    last_n_ds_ = n_ds;
    last_n_inliers_ = n_inliers;
    last_plane_found_ = plane_found;
    if (plane_found) {
      last_coeffs_[0] = coeffs->values[0];
      last_coeffs_[1] = coeffs->values[1];
      last_coeffs_[2] = coeffs->values[2];
      last_coeffs_[3] = coeffs->values[3];
    }
  }

  void logStats()
  {
    if (callback_count_ == 0) {
      RCLCPP_WARN(get_logger(), "No clouds received yet on '%s'.", input_topic_.c_str());
      return;
    }

    // Compute mean + p95 latency
    double sum = 0.0;
    std::vector<double> sorted(latency_window_.begin(), latency_window_.end());
    for (double v : sorted) sum += v;
    const double mean_ms = sum / sorted.size();
    std::sort(sorted.begin(), sorted.end());
    const double p95_ms = sorted[static_cast<size_t>(0.95 * (sorted.size() - 1))];

    const double rate_hz = callback_count_ / stats_log_period_;
    const double inlier_pct = last_n_ds_ > 0
        ? 100.0 * static_cast<double>(last_n_inliers_) / static_cast<double>(last_n_ds_)
        : 0.0;

    RCLCPP_INFO(get_logger(),
      "STATS  rate=%.1f Hz  raw=%zu  clean=%zu  ds=%zu  inliers=%zu (%.0f%%)  "
      "clusters=%zu  obs_pts=%zu  "
      "latency mean=%.1f ms p95=%.1f ms  plane_found=%s  coeffs=(%.2f, %.2f, %.2f, %.2f)",
      rate_hz, last_n_raw_, last_n_clean_, last_n_ds_, last_n_inliers_, inlier_pct,
      last_n_clusters_, last_n_obs_after_cluster_,
      mean_ms, p95_ms,
      last_plane_found_ ? "YES" : "NO",
      last_coeffs_[0], last_coeffs_[1], last_coeffs_[2], last_coeffs_[3]);

    callback_count_ = 0;
  }

  // Params
  std::string input_topic_;
  std::string obstacles_topic_;
  std::string ground_topic_;
  bool publish_ground_;
  double voxel_leaf_size_;
  double plane_distance_threshold_;
  int max_iterations_;
  double epsilon_angle_deg_;
  int min_inliers_;
  double axis_x_, axis_y_, axis_z_;
  double stats_log_period_;

  // New filter params
  double min_range_;
  double max_range_;
  double floor_clip_m_;
  double ceiling_clip_m_;
  double obstacle_max_range_;

  // Clustering params
  double cluster_tolerance_;
  int cluster_min_size_;
  int cluster_max_size_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // Stats
  bool frame_logged_ = false;
  bool bbox_logged_ = false;
  size_t callback_count_ = 0;
  size_t last_n_raw_ = 0;
  size_t last_n_clean_ = 0;
  size_t last_n_ds_ = 0;
  size_t last_n_inliers_ = 0;
  bool last_plane_found_ = false;
  std::array<double, 4> last_coeffs_ = {0, 0, 0, 0};
  std::deque<double> latency_window_;

  // Cluster stats
  size_t last_n_clusters_ = 0;
  size_t last_n_obs_after_cluster_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OakGroundRemover>());
  rclcpp::shutdown();
  return 0;
}