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

#include "graspable_points_detection/gripper_params.hpp"
#include "graspable_points_detection/matching_params.hpp"
#include "graspable_points_detection/visibility_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
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
  // Structure holding a voxel index (integer) of graspable point
  struct GraspablePointVoxel
  {
    int x;
    int y;
    int z;
    float score;
  };

  // Structure holding real numbers ([m]) in the physical coordinate system of graspable point
  struct GraspablePointPhysical
  {
    float x;
    float y;
    float z;
    float score;
  };

  void pointCloudCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr received_cloud_msg);

  void detectGraspablePoints(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & received_cloud_msg);

  void downsamplePointCloud(
    const sensor_msgs::msg::PointCloud2 & input_cloud_msg,
    pcl::PointCloud<pcl::PointXYZ> & downsampled_cloud);

  void estimateRegressionPlaneNormal(
    const pcl::PointCloud<pcl::PointXYZ> & input_cloud, const Eigen::Vector4f & centroid,
    Eigen::Vector3f & normal);

  void alignPointCloudAndBroadcastTF(
    const pcl::PointCloud<pcl::PointXYZ> & input_cloud,
    pcl::PointCloud<pcl::PointXYZ> & transformed_cloud, const std::string & parent_frame_id,
    const std::string & child_frame_id);

  void interpolatePointCloud(
    const pcl::PointCloud<pcl::PointXYZ> & input_cloud,
    pcl::PointCloud<pcl::PointXYZ> & interpolated_cloud);

  std::vector<std::vector<std::vector<int>>> voxelizePointCloud(
    const pcl::PointCloud<pcl::PointXYZ> & input_cloud);

  std::array<float, 3> getMinValues(const pcl::PointCloud<pcl::PointXYZ> & cloud);

  std::vector<GraspablePointVoxel> evaluateVoxelMatching(
    const std::vector<std::vector<std::vector<int>>> & terrain_matrix,
    const std::array<float, 3> & offset_vector);

  std::vector<GraspablePointPhysical> retransformToPhysical(
    const std::vector<GraspablePointVoxel> & voxel_points,
    const std::array<float, 3> & offset_vector);

  std::vector<GraspablePointPhysical> extractValidGraspablePoints(
    const std::vector<GraspablePointPhysical> & points);

  void visualizeGraspabilityScoreMap(const std::vector<GraspablePointPhysical> & points);

  void visualizeHighGraspabilityScoreMap(const std::vector<GraspablePointPhysical> & points);

  void extractClusterCentroids(const std::vector<GraspablePointPhysical> & valid_points);

  void visualizeVector(
    const Eigen::Vector3f & direction_vector, const Eigen::Vector3f & origin_point,
    const std::string & frame_id, const std::string & object_name);

  void broadcastRegressionPlaneTF(
    const Eigen::Vector4f & translation, const Eigen::Matrix3f & rotation_matrix,
    const std::string & parent_frame, const std::string & child_frame);

  void createGripperMask();

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr interpolated_point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr graspability_score_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr high_graspability_score_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_graspable_points_pub_;

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
