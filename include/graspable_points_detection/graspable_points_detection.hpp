// Copyright (c) 2024 Space Robotics Lab -- Tohoku University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file graspable_points_detection.hpp
 * @author Naomasa Takada, Jitao Zheng
 * @brief Describes the header of graspable target detection algorithm.
 */

#ifndef GRASPABLE_POINTS_DETECTION__GRASPABLE_POINTS_DETECTION_HPP_
#define GRASPABLE_POINTS_DETECTION__GRASPABLE_POINTS_DETECTION_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "graspable_points_detection/gripper_params.hpp"
#include "graspable_points_detection/matching_params.hpp"
#include "graspable_points_detection/visibility_control.hpp"
namespace graspable_points_detection
{

class GraspablePointsDetection : public rclcpp::Node
{
public:
  GRASPABLE_POINTS_DETECTION_PUBLIC
  explicit GraspablePointsDetection(const rclcpp::NodeOptions & options);

  GRASPABLE_POINTS_DETECTION_PUBLIC
  virtual ~GraspablePointsDetection();

private:
  /**
   * @brief Structure holding a voxel index (integer) of a graspable point.
   */
  struct GraspablePointVoxel
  {
    int x;
    int y;
    int z;
    float score;
  };

  /**
   * @brief Structure holding real numbers [m] in the physical coordinate system of a graspable point.
   */
  struct GraspablePointPhysical
  {
    float x;
    float y;
    float z;
    float score;
  };

  /**
   * @brief Callback function for the point cloud subscriber.
   *
   * @param received_cloud_msg
   */
  void pointCloudCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr received_cloud_msg);

  /**
   * @brief Main pipeline to execute the graspable points detection algorithm.
   *
   * @param received_cloud_msg
   */
  void detectGraspablePoints(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & received_cloud_msg);

  /**
     * @brief Downsample the input point cloud using a voxel grid filter to reduce computation time.
     *
     * @param input_cloud_msg Input ROS point cloud message.
     * @param downsampled_cloud Output PCL point cloud after downsampling.
     */
  void downsamplePointCloud(
    const sensor_msgs::msg::PointCloud2 & input_cloud_msg,
    pcl::PointCloud<pcl::PointXYZ> & downsampled_cloud);

  /**
     * @brief Estimate the normal vector of the regression plane from the given point cloud.
     *
     * @param input_cloud Point cloud to compute the regression plane from.
     * @param centroid Centroid of the input point cloud.
     * @param normal Output normal vector of the estimated plane.
     */
  void estimateRegressionPlaneNormal(
    const pcl::PointCloud<pcl::PointXYZ> & input_cloud, const Eigen::Vector4f & centroid,
    Eigen::Vector3f & normal);

  /**
     * @brief Transform and align the point cloud to the regression plane, and broadcasts the transformation (TF).
     *
     * @param input_cloud Input point cloud.
     * @param transformed_cloud Output point cloud aligned to the regression plane.
     * @param parent_frame_id Frame ID of the input point cloud.
     * @param child_frame_id Frame ID of the generated regression plane.
     */
  void alignPointCloudAndBroadcastTF(
    const pcl::PointCloud<pcl::PointXYZ> & input_cloud,
    pcl::PointCloud<pcl::PointXYZ> & transformed_cloud, const std::string & parent_frame_id,
    const std::string & child_frame_id);

  /**
     * @brief Interpolate the point cloud to fill gaps and densify the surface representation.
     *
     * @param input_cloud Input aligned point cloud.
     * @param interpolated_cloud Output interpolated point cloud.
     */
  void interpolatePointCloud(
    const pcl::PointCloud<pcl::PointXYZ> & input_cloud,
    pcl::PointCloud<pcl::PointXYZ> & interpolated_cloud);

  /**
     * @brief Convert the point cloud into a 3D grid (voxel matrix) for collision and grasp evaluation.
     *
     * @param input_cloud Input point cloud.
     * @return std::vector<std::vector<std::vector<int>>> 3D boolean/integer matrix representing occupied voxels.
     */
  std::vector<std::vector<std::vector<int>>> voxelizePointCloud(
    const pcl::PointCloud<pcl::PointXYZ> & input_cloud);

  /**
   * @brief Extract the minimum X, Y, and Z values from the point cloud for coordinate offsetting.
   *
   * @param cloud Input point cloud.
   * @return std::array<float, 3> Array containing {min_x, min_y, min_z}.
   */
  std::array<float, 3> getMinValues(const pcl::PointCloud<pcl::PointXYZ> & cloud);

  /**
   * @brief Evaluates the graspability score for each voxel by convolving the terrain matrix with the gripper mask.
   *
   * @param terrain_matrix 3D voxel matrix representing the environment.
   * @param offset_vector Vector used to correct the origin shift caused by voxelization.
   * @return std::vector<GraspablePointVoxel> List of graspable candidates in voxel coordinate space.
   */
  std::vector<GraspablePointVoxel> evaluateVoxelMatching(
    const std::vector<std::vector<std::vector<int>>> & terrain_matrix,
    const std::array<float, 3> & offset_vector);

  /**
   * @brief Transforms the grasp candidates from voxel index coordinates back to physical metric coordinates.
   *
   * @param voxel_points List of grasp candidates in voxel space.
   * @param offset_vector Offset vector to restore the original physical positions.
   * @return std::vector<GraspablePointPhysical> List of grasp candidates in physical coordinate space.
   */
  std::vector<GraspablePointPhysical> retransformToPhysical(
    const std::vector<GraspablePointVoxel> & voxel_points,
    const std::array<float, 3> & offset_vector);

  /**
   * @brief Filters out candidates that do not meet the minimum score or height thresholds.
   *
   * @param points List of raw physical grasp candidates.
   * @return std::vector<GraspablePointPhysical> List of validated grasp candidates.
   */
  std::vector<GraspablePointPhysical> extractValidGraspablePoints(
    const std::vector<GraspablePointPhysical> & points);

  /**
   * @brief Visualizes all grasp candidates using a color gradient map based on their scores.
   *
   * @param points List of physical grasp candidates.
   */
  void visualizeGraspabilityScoreMap(const std::vector<GraspablePointPhysical> & points);

  /**
   * @brief Visualizes only the high-scoring, validated grasp candidates.
   *
   * @param points List of validated grasp candidates.
   */
  void visualizeHighGraspabilityScoreMap(const std::vector<GraspablePointPhysical> & points);

  /**
   * @brief Clusters the valid grasp points and extracts the centroids to determine the final approach poses.
   *
   * @param valid_points List of validated grasp candidates.
   */
  void extractClusterCentroids(const std::vector<GraspablePointPhysical> & valid_points);

  /**
   * @brief Publishes normal vector.
   *
   * @param direction_vector The direction of the vector.
   * @param origin_point The starting point of the vector.
   * @param frame_id The TF frame ID the vector belongs to.
   * @param object_name The namespace/name for the marker.
   */
  void visualizeVector(
    const Eigen::Vector3f & direction_vector, const Eigen::Vector3f & origin_point,
    const std::string & frame_id, const std::string & object_name);

  /**
   * @brief Broadcasts the coordinate transformation (TF) of the calculated regression plane.
   *
   * @param translation 3D translation vector (centroid).
   * @param rotation_matrix 3x3 rotation matrix of the plane.
   * @param parent_frame Parent TF frame name.
   * @param child_frame Child TF frame name.
   */
  void broadcastRegressionPlaneTF(
    const Eigen::Vector4f & translation, const Eigen::Matrix3f & rotation_matrix,
    const std::string & parent_frame, const std::string & child_frame);

  /**
   * @brief Generates the 3D voxel mask representing the physical volume of the robot's gripper.
   *
   */
  void createGripperMask();

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr interpolated_point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr graspability_score_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr high_graspability_score_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr graspable_poses_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graspable_points_marker_pub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr normal_vector_marker_pub_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Variables
  std::vector<std::vector<std::vector<int>>> gripper_mask_;

  std::string received_cloud_frame_id_;
  const std::string kRegressionPlaneFrameId_ = "regression_plane_frame";

  builtin_interfaces::msg::Time msg_stamp_;
};

}  // namespace graspable_points_detection

#endif  // GRASPABLE_POINTS_DETECTION__GRASPABLE_POINTS_DETECTION_HPP_
