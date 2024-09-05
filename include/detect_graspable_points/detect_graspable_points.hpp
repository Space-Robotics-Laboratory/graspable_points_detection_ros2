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

//declaring the parameter before the function that needs it
	struct Subscripts {
    std::vector<int> x;
    std::vector<int> y;
    std::vector<int> z;
	};

	struct MatchingSettings
	{	
		float voxel_size;
		int threshold;
		std::string delete_lower_targets;
		float delete_lower_targets_threshold;
		float searching_radius_for_normal_and_curvature;
		int extra_sheet;
		float graspability_threshold;
	};

	/**
	 * @brief This function is called when a point cloud is received to be processed
	 * 
	 * @param cloud_msg
	 */
	void mapReceivedCallBack(const sensor_msgs::msg::PointCloud2 cloud_msg);

	/**
	 * @function name : pcd_least_squares_plane()
	 * @brief : compute centroid point of input point cloud and normal std::vector of regression plane
	 * @param raw_pcd : input point cloud
	 * @param centroid : coordinate of centroid (output)
	 * @param normal_vector_of_plane : normal std::vector of regression plane (output)
	 * @return none
	 */	
	void pcd_least_squares_plane(const pcl::PointCloud<pcl::PointXYZ> raw_pcd, Eigen::Vector4f &centroid,  Eigen::Vector3f &normal_vector_of_plane);

	/**
	 * @function name : pcd_transform()
	 * @brief : transform coordinate of pointcloud to terrain base frame
	 * @param raw_pcd : input point cloud
	 * @param transformed_point_cloud : transormed point cloud (output)
	 * @param centroid_vector_of_plane : coordinate of centroid (output)
	 * @param rotation_matrix : rotation matrix used for the coordinates transformation (output)
	 * @return none
	 */	
	void pcd_transform ( const pcl::PointCloud<pcl::PointXYZ>  raw_pcd, pcl::PointCloud<pcl::PointXYZ> &transformed_point_cloud, Eigen::Vector4f &centroid_vector_of_plane, Eigen::Matrix3f &rotation_matrix);

	/**
	 * @function name : pcd_interpolate()
	 * @brief : homogenize the point cloud and interpolate occlusion points
	 * @param raw_pcd : input point cloud
	 * @param interpolated_point_cloud : interpolated point cloud (output)
	 * @return none
	 */		
	void pcd_interpolate (const pcl::PointCloud<pcl::PointXYZ>  raw_pcd,pcl::PointCloud<pcl::PointXYZ> &interpolated_point_cloud);

	/**
	 * @function name : pcd_voxelize()
	 * @brief : turns the point_cloud into a 3D array of 0 and 1, 0 being void and 1 solid
	 * @param input_pcd : input point cloud
	 * @return none
	 */		
	std::vector<std::vector<std::vector<int>>> pcd_voxelize (const pcl::PointCloud<pcl::PointXYZ>  input_pcd);

	/**
	 * @function name : downsampling() 
	 * @brief : execute downsampling to input pointcloud (downsampling means the number of pointcloud is reduced)
	 * @param cloud_msg : pointer of sensor_msgs::PointCloud2 (input)
	 * @param frame_id : reference frame of this point
	 * @param filtered_points : downsampling points (output)
	 * @return none
	 */		
	void downsampling(const sensor_msgs::msg::PointCloud2 & cloud_msg, const std::string frame_id, pcl::PointCloud<pcl::PointXYZ> &filtered_points);

	void tf_broadcast(const std::string frame_id);

	void tf_broadcast_from_pose(const std::string parant_frame_id, const std::string child_frame_id_to, geometry_msgs::msg::Pose relative_pose_between_frame);

	/**
	 * @function name : visualizePoint() 
	 * @brief : publish a 3D point (x, y, z) with respect to frame_id.
	 * @param x : x value of position std::vector
	 * @param y : y value of position std::vector
	 * @param z : z value of position std::vector
	 * @param object_name : object name of this point. this shows what this point is.
	 * @param frame_id : reference frame of this point
	 * @return none
	 */		
	void visualizePoint(const double x, const double y, const double z, const std::string object_name, const std::string frame_id);


	/**
	 * @function name : visualizeVector() 
	 * @brief : publish an arrow from vector_of_start_point to vector_of_end_point
	 * @param vector_of_start_point : inital point of the arrow
	 * @param vector_of_end_point : endpoint point of the arrow
	 * @param object_name : object name of this point. this shows what this point is.
	 * @param frame_id : reference frame of this point
	 * @return none
	 */	
	void visualizeVector(const Eigen::Vector3f &vector_of_start_point, const Eigen::Vector3f &vector_of_end_point, const std::string object_name, const std::string frame_id);


	/**
	 * @function name : vox_evaluate() 
	 * @brief : compares two voxel arrays of the same size and returns a value between 1 and 100
	 * @param subset_of_voxel_array : l*m*n voxelgrid
	 * @param gripper_mask : l*m*n voxelgrid
	 * @return graspability: the probability of graspability of the respective point
	 */	
	float vox_evaluate(const int number_of_points, const std::vector<std::vector<std::vector<int>>>& subset_of_voxel_array);

	/**
	 * @function name : vox_clip() 
	 * @brief : clips the searched array at all sides by setting the values to zero. 
		We want to prevent the gripper mask from protruding from the voxel array during the matching algorithm. 
		We can limit the range of searching by using this function because the values in the area around the outer edges are set 0. 
		Then we will compare  3-dimensional arrays based on solid(1) voxels located in the inside area having room for gripper-mask.
	 * @param x: number of surfaces set 0 in x-direction
	 * @param y : number of surfaces set 0 in y-direction
	 * @param search_voxel_array : search voxelgrid
	 * @return searchVoxelArray: search voxelgrid having the room of a gripper mask around the outer edges
	 */	
	std::vector<std::vector<std::vector<int>>> vox_clip(const int x, const int y, std::vector<std::vector<std::vector<int>>> search_voxel_array);

	/**
	 * @function name : vox_extract() 
	 * @brief : extracts an array of voxels of the specified size from a larger voxel array.
		We want to compare the gripper mask with the extracted voxel array of the same size as the gripper mask.
	 * @param voxel_array : m*n*l matrix, 3-dimensional voxelgrid 
	 * @param position_reference_point : 1*3 std::vector, position of the extracted voxel array, xyz 
	 * @param size_extracting: 1*3 std::vector, size of the of the extracted voxelgrid ,i*i*j
	 * @return extractev_voxel_array: i*i*j matrix, extracted 3-dimensional voxel array (i*i*j matrix)
	 */	
	std::vector<std::vector<std::vector<int>>> vox_extract(const std::vector<std::vector<std::vector<int>>>& voxel_array ,const std::vector<int>& position_reference_point, std::array<int,3> size_extracting);

	/**
	 * @function name : gripper_mask() 
	 * @brief : makes the gripper_mask, which is the 3-dimensional array composed of 0 and 1 considering geometric parameters of the gripper.
	 * @param voxel_size: length of one side of voxel [m]
	 * @param gripper_param : geometric parameters of the gripper which are set in the config file.
	 * @return gripper_mask : n*n*m array composed of 0 and 1.
	 */	
	void creategrippermask(int (&gripper_mask)[gripper_mask_size][gripper_mask_size][gripper_mask_height]);
	/**
	 * @function name : gripper_mask() 
	 * @brief : makes the gripper_mask, which is the 3-dimensional array composed of 0 and 1 considering geometric parameters of the gripper.
	 * @param voxel_size: length of one side of voxel [m]
	 * @param gripper_param : geometric parameters of the gripper which are set in the config file.
	 * @return gripper_mask : n*n*m array composed of 0 and 1.
	 */	

	/**
	 * @function name : voxel_matching() 
	 * @brief : finds subset voxel arrays in a major voxel array that matches a gripper mask voxel array.
	 * @param terrain_matrix : 3-dimensional array composed of 0 and 1
	 * @return voxel_array_of_graspable_points : 4*n matrix, containing subscripts of "grippable" points and "grippability" at those points  
        1st,2nd, and 3rd rows indicate subscripts in x-y-z directions
		4th row indicates the number of solid
		voxels, but that of the sub-graspable
		points is 1
	 */	
	std::vector<std::vector<int>> voxel_matching(std::vector<std::vector<std::vector<int>>>& terrain_matrix,std::array<float,3> &offset_vector);

	/**
	 * @function name : pcd_re_transform() 
	 * @brief : returns the coordinates of the voxel array to the original input coordinate system
	 * @param voxel_coordinates_of_graspable_points : 4*n matrix. Each column indicates a matched voxel. 1st, 2nd, and 3rd raws indicate x y z subscripts in voxel array.
	 * @param voxel_size : length of one side of voxel used in voxelize [m]
	 * @param rotation_matrix
	 * @param centroid_vector_of_plane
	 * @return graspable_points: std::vector<std::vector<float>>
	 */	
	std::vector<std::vector<float>> pcd_re_transform(std::vector<std::vector<int>> voxel_coordinates_of_graspable_points, std::array<float,3> offset_vector);

	/**
	 * @brief : returns the centroid of each cluster detected
	 * @param input_pcl
	 * @return centroid of each cluster in pcl format 
	 */												
	pcl::PointCloud<pcl::PointXYZ> clusterize(const pcl::PointCloud<pcl::PointXYZ> &input_pcl);

	void save3DVectorToFile(const int (&vector3D)[gripper_mask_size][gripper_mask_size][gripper_mask_height], const std::string& filename);

	sensor_msgs::msg::PointCloud2 visualizeRainbow(std::vector<std::vector<float>> array);

	sensor_msgs::msg::PointCloud2 to_msg_format(const std::vector<std::vector<float>> array);

	std::array<float,3> getMinValues(const pcl::PointCloud<pcl::PointXYZ>& pointCloud);

private:
	std::string camera_frame;
	int gripper_mask_[gripper_mask_size][gripper_mask_size][gripper_mask_height];
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
