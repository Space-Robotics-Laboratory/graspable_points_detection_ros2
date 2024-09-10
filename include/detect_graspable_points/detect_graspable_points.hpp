/*!
 * @file detect_graspable_points.hpp
 * @author Naomasa Takada, Jitao Zheng
 * @brief Describes the header of graspable target detection algorithm.
 */

#ifndef DETECT_GRASPABLE_POINTS_HPP
#define DETECT_GRASPABLE_POINTS_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp> 
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <string>
#include <sensor_msgs/msg/point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.h> 
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp> 
#include <geometry_msgs/msg/pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "Eigen/Core"
#include "Eigen/Dense"
#include <eigen3/Eigen/Eigenvalues>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include "pcl/features/normal_3d.h"
#include "pcl/features/principal_curvatures.h"
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include<libInterpolate/AnyInterpolator.hpp>
#include<libInterpolate/Interpolate.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/time.hpp>

#include "settings.hpp"


class detect_graspable_points : public rclcpp::Node
{
public:
	detect_graspable_points();
    ~detect_graspable_points(); // ~ is a bitwise NOT operator, e.g. NOT 011100 = 100011
private:
	/**
	 * @brief This function is called when a point cloud is received to be processed
	 */
	void mapReceivedCallBack(const sensor_msgs::msg::PointCloud2 cloud_msg);

	/**
	 * @brief : compute centroid point of input point cloud and normal std::vector of regression plane
	 * @param raw_pcd : input point cloud
	 * @param centroid : coordinate of centroid (output)
	 * @param normal_vector_of_plane : normal std::vector of regression plane (output)
	 * @return none
	 */	
	void pcd_least_squares_plane(const pcl::PointCloud<pcl::PointXYZ> raw_pcd, Eigen::Vector4f &centroid,  Eigen::Vector3f &normal_vector_of_plane);

	/**
	 * @brief : transform coordinate of pointcloud to terrain base frame
	 * @param raw_pcd : input point cloud
	 * @param transformed_point_cloud : transormed point cloud (output)
	 * @param centroid_vector_of_plane : coordinate of centroid (output)
	 * @param rotation_matrix : rotation matrix used for the coordinates transformation (output)
	 * @return none
	 */	
	void pcd_transform ( const pcl::PointCloud<pcl::PointXYZ>  raw_pcd, pcl::PointCloud<pcl::PointXYZ> &transformed_point_cloud, Eigen::Vector4f &centroid_vector_of_plane, Eigen::Matrix3f &rotation_matrix);

	/**
	 * @brief : homogenize the point cloud and interpolate occlusion points
	 * @param raw_pcd : input point cloud
	 * @param interpolated_point_cloud : interpolated point cloud (output)
	 * @return none
	 */		
	void pcd_interpolate (const pcl::PointCloud<pcl::PointXYZ>  raw_pcd,pcl::PointCloud<pcl::PointXYZ> &interpolated_point_cloud);

	/**
	 * @brief : turns the point_cloud into a 3D array of 0 and 1, 0 being void and 1 solid
	 * @param input_pcd : input point cloud
	 * @return none
	 */		
	pcl::PointCloud<pcl::PointXYZ> pcd_voxelize (const pcl::PointCloud<pcl::PointXYZ>  &input_pcd);

	/**
	 * @brief : execute downsampling to input pointcloud (downsampling means the number of pointcloud is reduced)
	 * @param cloud_msg : pointer of sensor_msgs::PointCloud2 (input)
	 * @param frame_id : reference frame of this point
	 * @param filtered_points : downsampling points (output)
	 * @return none
	 */		
	void downsampling(const sensor_msgs::msg::PointCloud2 & cloud_msg, const std::string frame_id, pcl::PointCloud<pcl::PointXYZ> &filtered_points);

	/**
	 * @brief : broadcast relative position between two frames
	 */
	void tf_broadcast(const std::string frame_id);

	/**
	 * @brief : broadcast relative position between two frames
	 */
	void tf_broadcast_from_pose(const std::string parant_frame_id, const std::string child_frame_id_to, geometry_msgs::msg::Pose relative_pose_between_frame);

	/**
	 * @function name : vox_extract() 
	 * @brief : extracts a pcl of the gripper size (see settings.hpp)
	 * @param position_reference_point this point is seen as the cneter of the top surface of the extraction box
	 */	
	pcl::PointCloud<pcl::PointXYZ> vox_extract(const pcl::PointCloud<pcl::PointXYZ>& input_pcl,const pcl::PointXYZ& position_reference_point); 

	/**
	 * @brief : tries to match the gripper mask on the points of input pcl
	 * @return point cloud with I as the graspability value
	 */	
	pcl::PointCloud<pcl::PointXYZI> voxel_matching(pcl::PointCloud<pcl::PointXYZ>& input_pcl);
	/**
	 * @brief : returns the centroid of each cluster detected
	 * @param input_pcl
	 * @return centroid of each cluster in pcl format 
	 */												
	pcl::PointCloud<pcl::PointXYZ> clusterize(const pcl::PointCloud<pcl::PointXYZ> &input_pcl);
	/**
	 * @brief : returns a msg ready to be published for visualizing the grapsability of each point
	 */
	sensor_msgs::msg::PointCloud2 visualizeRainbow(pcl::PointCloud<pcl::PointXYZI> &input_pcl);
	/**
	 * @brief : convert the input pcl to a msg that only keep point above graspability threshold
	 */
	sensor_msgs::msg::PointCloud2 to_msg_format(const pcl::PointCloud<pcl::PointXYZI> input_pcl);
	/**
	 * @brief : verify the graspability of the input cloud 
	 * @param input_pcl /!\ the input must have the same size as the gripper mask
	 * @param centroid this point must be on the center of the top surface of the box that inscribes the point cloud
	 */
	float checkGraspability(const pcl::PointCloud<pcl::PointXYZ> &input_pcd,const pcl::PointXYZ &centroid);
	/**
	 * @brief : returns the input pcl without the point that are too far from the normal plane 
	 */
	pcl::PointCloud<pcl::PointXYZ> deleteLowPoints(const pcl::PointCloud<pcl::PointXYZ> &input_pcl);
	/**
	 * @brief returns the value of the deviation from the normal plane
	 */
	float deviationFromPlane(const pcl::PointCloud<pcl::PointXYZ> &input_cloud)const;

private:
	std::string camera_frame;
	float global_z_thrshold;
  	// Node Handle declaration
	
	//Subcriber
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;

	//Publishers
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_points_pub_ ;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr interpolate_point_pub_ ;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr peaks_pub_ ;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr graspability_map_pub_ ;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pub_ ;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_point_pub_ ;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debugging_pub_ ;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_ ;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr point_visualization_marker_pub_ ;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_ ;
	tf2_ros::TransformBroadcaster dynamic_tf=tf2_ros::TransformBroadcaster(this);

	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcast_;
};


#endif
