#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudFusion
{
private:
    ros::NodeHandle nh_;
    ros::Publisher fused_cloud_pub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> realsense_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> velodyne_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string velodyne_frame_;
    std::string realsense_frame_;

public:
    PointCloudFusion() : nh_("~"), tf_listener_(tf_buffer_)
    {
        fused_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fused_point_cloud", 1);

        realsense_sub_.subscribe(nh_, "/camera/depth/color/points", 1);
        velodyne_sub_.subscribe(nh_, "/velodyne_points", 1);

        nh_.param<std::string>("velodyne_frame", velodyne_frame_, "velodyne");
        nh_.param<std::string>("realsense_frame", realsense_frame_, "camera_depth_optical_frame");

        sync_.reset(new Sync(MySyncPolicy(10), realsense_sub_, velodyne_sub_));
        sync_->registerCallback(boost::bind(&PointCloudFusion::cloudCallback, this, _1, _2));

        // Set up ICP
        icp_.setMaxCorrespondenceDistance(0.05);
        icp_.setMaximumIterations(50);
        icp_.setTransformationEpsilon(1e-8);
        icp_.setEuclideanFitnessEpsilon(1);

        // Set up voxel filter
        voxel_filter_.setLeafSize(0.05f, 0.05f, 0.05f);

        // Set up statistical outlier removal
        sor_.setMeanK(50);
        sor_.setStddevMulThresh(1.0);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& realsense_msg, 
                       const sensor_msgs::PointCloud2ConstPtr& velodyne_msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr realsense_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr realsense_xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr fused_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(*realsense_msg, *realsense_cloud);
        pcl::fromROSMsg(*velodyne_msg, *velodyne_cloud);

        // Convert RealSense cloud to XYZ for alignment
        pcl::copyPointCloud(*realsense_cloud, *realsense_xyz_cloud);

        // Apply voxel grid filter
        voxel_filter_.setInputCloud(realsense_xyz_cloud);
        voxel_filter_.filter(*realsense_xyz_cloud);
        voxel_filter_.setInputCloud(velodyne_cloud);
        voxel_filter_.filter(*velodyne_cloud);

        // Apply statistical outlier removal
        sor_.setInputCloud(realsense_xyz_cloud);
        sor_.filter(*realsense_xyz_cloud);
        sor_.setInputCloud(velodyne_cloud);
        sor_.filter(*velodyne_cloud);

        // Transform Velodyne cloud to RealSense frame
        Eigen::Affine3d transform;
        try {
            transform = tf2::transformToEigen(tf_buffer_.lookupTransform(realsense_frame_, velodyne_frame_, ros::Time(0)));
            pcl::transformPointCloud(*velodyne_cloud, *velodyne_cloud, transform);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        // Align clouds using ICP
        icp_.setInputSource(realsense_xyz_cloud);
        icp_.setInputTarget(velodyne_cloud);
        icp_.align(*aligned_cloud);

        // Fuse clouds
        *fused_cloud = *realsense_cloud;  // Start with colored RealSense cloud
        pcl::PointXYZRGB colored_point;
        for (const auto& point : velodyne_cloud->points) {
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = 255;  // Color Velodyne points red for distinction
            colored_point.g = 0;
            colored_point.b = 0;
            fused_cloud->points.push_back(colored_point);
        }
        fused_cloud->width = fused_cloud->points.size();
        fused_cloud->height = 1;

        // Publish fused cloud
        sensor_msgs::PointCloud2 fused_msg;
        pcl::toROSMsg(*fused_cloud, fused_msg);
        fused_msg.header = realsense_msg->header;
        fused_cloud_pub_.publish(fused_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_fusion_node");
    PointCloudFusion fusion_node;
    ros::spin();
    return 0;
}