// Headers 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/pose_array.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/pca.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Geometry>

#include "robot_interfaces/msg/block_pose.hpp"

class PclColorDetector : public rclcpp::Node {
public:
    PclColorDetector() : Node("obj_perception") {

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth_camera/points", 10, 
            std::bind(&PclColorDetector::cloud_callback, this, std::placeholders::_1));

        block_pub_ = this->create_publisher<robot_interfaces::msg::BlockPose>(
            "/detected_blocks", 10);


        filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/filtered_blocks_cloud", 10);

        table_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/table_cloud", 10);

        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/block_poses", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Node started");
    }

private:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Got point cloud %d %d", msg->width, msg->height); 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

       pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr plane_coeffs(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.08);
        seg.setMaxIterations(1000);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *plane_coeffs);

        if(inliers->indices.empty())
        {   RCLCPP_WARN(this->get_logger(), "No indices found for the RANSAC "); 
            return; 
        }

        PointCloudT::Ptr filtered_cloud(new PointCloudT);
        pcl::ExtractIndices<PointT> extract_ransac;
        extract_ransac.setInputCloud(cloud);
        extract_ransac.setIndices(inliers);
        extract_ransac.setNegative(true);
        extract_ransac.filter(*filtered_cloud);

        PointCloudT::Ptr table_cloud(new PointCloudT);
        pcl::ExtractIndices<PointT> extract_table;
        extract_table.setInputCloud(cloud);
        extract_table.setIndices(inliers);
        extract_table.setNegative(false);
        extract_table.filter(*table_cloud);

        sensor_msgs::msg::PointCloud2 ransac_msg;
        pcl::toROSMsg(*table_cloud, ransac_msg);
        ransac_msg.header = msg->header;
        table_cloud_pub_->publish(ransac_msg);


        pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
        reg.setInputCloud(filtered_cloud);
        reg.setSearchMethod(tree);
        reg.setDistanceThreshold(0.015);  
        reg.setPointColorThreshold(200);   
        reg.setRegionColorThreshold(200);  
        reg.setMinClusterSize(500); 
        reg.setMaxClusterSize(8000); 

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        uint8_t block_id = 0;

        geometry_msgs::msg::PoseArray block_poses_array;
        block_poses_array.header.frame_id = "base_link"; 
        block_poses_array.header.stamp = msg->header.stamp;

        for (const auto& cluster_indices : clusters) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_block(new pcl::PointCloud<pcl::PointXYZRGB>);
            long r_total = 0, g_total = 0, b_total = 0;

            for (int index : cluster_indices.indices) {
                auto pt = filtered_cloud->points[index];
                single_block->points.push_back(pt);
                r_total += pt.r; g_total += pt.g; b_total += pt.b;
            }

            int count = single_block->points.size();
            int r_avg = r_total / count, g_avg = g_total / count, b_avg = b_total / count;
            
            std::string color_id = "Unknown";
            if (r_avg > g_avg && r_avg > b_avg && r_avg > 100) color_id = "R";
            else if (g_avg > r_avg && g_avg > b_avg && g_avg > 100) color_id = "G";
            else if (b_avg > r_avg && b_avg > g_avg && b_avg > 100) color_id = "B";

            if (color_id == "Unknown") continue;


            *output_cloud += *single_block;

            pcl::PCA<pcl::PointXYZRGB> pca;
            pca.setInputCloud(single_block);

            Eigen::Vector4f centroid = pca.getMean();
            Eigen::Matrix3f rotation_matrix = pca.getEigenVectors();
            Eigen::Quaternionf q(rotation_matrix);
            q.normalize();

            geometry_msgs::msg::PoseStamped pose_cam;
            pose_cam.header.frame_id = msg->header.frame_id; 
            pose_cam.header.stamp = this->get_clock()->now();
            pose_cam.pose.position.x = centroid[0];
            pose_cam.pose.position.y = centroid[1];
            pose_cam.pose.position.z = centroid[2];
            pose_cam.pose.orientation.x = q.x();
            pose_cam.pose.orientation.y = q.y();
            pose_cam.pose.orientation.z = q.z();
            pose_cam.pose.orientation.w = q.w();

            geometry_msgs::msg::PoseStamped pose_base;
            try {
                pose_base = tf_buffer_->transform(pose_cam, "base_link", tf2::durationFromSec(0.5));

                robot_interfaces::msg::BlockPose final_msg;
                final_msg.color = color_id;
                final_msg.pose = pose_base;
                final_msg.block_id = block_id; 
                block_pub_->publish(final_msg);

                pose_base.pose.orientation.x = 0; 
                pose_base.pose.orientation.y = 0;
                pose_base.pose.orientation.z = 0; 
                pose_base.pose.orientation.w = 1;

                block_poses_array.poses.push_back(pose_base.pose);

                RCLCPP_INFO(this->get_logger(), 
                "Block ID: %d | Colour ID: %s | X:[%.3f] Y:[%.3f] Z:[%.3f]",
                static_cast<int>(block_id),
                color_id.c_str(),
                pose_base.pose.position.x, 
                pose_base.pose.position.y,
                pose_base.pose.position.z); 

            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform pose: %s", ex.what());
                continue; 
            }

            block_id++;
        }
        if(!block_poses_array.poses.empty())
        {
            pose_array_pub_->publish(block_poses_array); 
        }

        if (!output_cloud->points.empty()) {
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*output_cloud, output_msg);
            output_msg.header = msg->header; 
            filtered_cloud_pub_->publish(output_msg);
        }
    }
    //Pointer definitons
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<robot_interfaces::msg::BlockPose>::SharedPtr block_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr table_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_; 

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PclColorDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
