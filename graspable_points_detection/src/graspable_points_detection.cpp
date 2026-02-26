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
  * \file graspable_points_detection.hpp
  * \author
  * Jitao Zheng (jitao.zheng at tum.de)
  * Ringeval-Meusnier Antonin (ringeval at insta-toulouse.fr)
  * Taku Okawara (taku.okawara.t3 at dc.tohoku.ac.jp)
  * Kentaro Uno (unoken at tohoku.ac.jp)
  * \brief Every function of the graspable target detection algorithm
*/

#include "graspable_points_detection/graspable_points_detection.hpp"

#include <algorithm>
#include <cmath>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

// libInterpolate
// #include <libInterpolate/AnyInterpolator.hpp>
#include <libInterpolate/Interpolate.hpp>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#define DEBUG true

namespace
{

using namespace graspable_points_detection;

// Calculate the ratio of voxel size and 1mm in order to keep the gripper size in real world regardless of voxel size
constexpr float kRatio = 1.0f / (kVoxelSize * 1000.0f);

// Reduce or magnify the gripper's parameters to fit voxel's dimension
// Change demensions from [mm] to [voxels]
const float kPalmDiameter = std::round(kGripperParams.palm_diameter * kRatio);
const float kPalmDiameterOfFingerJoints =
  std::round(kGripperParams.palm_diameter_of_finger_joints * kRatio);
const float kFingerLength = std::round(kGripperParams.finger_length * kRatio);
const float kSpineLength = std::round(kGripperParams.spine_length * kRatio);
const float kSpineDepth = std::round(kGripperParams.spine_depth * kRatio);
const float kOpeningSpineRadius = std::round(kGripperParams.opening_spine_radius * kRatio);
const float kOpeningSpineDepth = std::round(kGripperParams.opening_spine_depth * kRatio);
const float kClosingHeight = std::round(kGripperParams.closing_height * kRatio);
const float kMarginOfTopSolidDiameter =
  std::round(kGripperParams.margin_of_top_solid_diameter * kRatio);
const float kInsideMarginOfBottomVoidDiameter =
  std::round(kGripperParams.inside_margin_of_bottom_void_diameter * kRatio);

// Set the gripper-mask size
const float kGripperMaskHalfSize =
  (kPalmDiameterOfFingerJoints / 2.0f) + kFingerLength + kSpineLength;
const int kGripperMaskSize = static_cast<int>(2.0f * kGripperMaskHalfSize) + 1;
const int kGripperMaskHeight = static_cast<int>(kClosingHeight);

// Calculate the parameters to determine solid area and void area
const float kGripperMaskTopSolidRadius =
  std::round((kPalmDiameter + kMarginOfTopSolidDiameter) / 2.0f);

const float kGripperMaskClearance = std::round(
  (static_cast<float>(kGripperMaskSize) - kPalmDiameter) / 2.0f *
  std::tan((90.0f - kGripperParams.opening_angle) * (M_PI / 180.0)));

const float kGripperMaskBottomVoidRadius = std::round(
  (kPalmDiameter / 2.0f) +
  (static_cast<float>(kGripperMaskHeight) *
   std::tan(kGripperParams.closing_angle * (M_PI / 180.0))) -
  kInsideMarginOfBottomVoidDiameter);

// Penalty threshold for scores [%]
const float kMinScoreForPenalty = 60.0f;

// Color map for graspability score map
const std::vector<std::array<int, 3>> kGraspabilityColorMap = {
  {255, 0, 0},    // 0.0: red (Score 0)
  {255, 38, 0},   // 0.1
  {255, 77, 0},   // 0.2
  {255, 115, 0},  // 0.3
  {255, 154, 0},  // 0.4
  {255, 193, 0},  // 0.5: orange
  {255, 255, 0},  // 0.6: yellow
  {214, 255, 0},  // 0.7
  {99, 255, 0},   // 0.8
  {8, 144, 0},    // 0.9
  {8, 144, 0}     // 1.0: green
};
const int kColorMapSize = kGraspabilityColorMap.size();

}  // anonymous namespace

namespace graspable_points_detection
{

GraspablePointsDetection::GraspablePointsDetection(const rclcpp::NodeOptions & options)
: rclcpp::Node("graspable_points_detection", options)
{
  // Parameters
  this->declare_parameter<std::string>("subscribed_topic_name", "/map_point_cloud");

  std::string topic_name = this->get_parameter("subscribed_topic_name").as_string();

  // Publishers
  downsampled_point_cloud_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/downsampled_points", 1);
  interpolated_point_cloud_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/interpolated_points", 1);
  graspability_score_map_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/graspability_score_map", 1);
  high_graspability_score_map_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/high_graspability_score_map", 1);
  clustered_graspable_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/graspable_points", 1);

  normal_vector_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("~/normal_vector", 1);

  // Subscriber
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_name, 1,
    std::bind(&GraspablePointsDetection::pointCloudCallBack, this, std::placeholders::_1));

  // TF
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  createGripperMask();

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

GraspablePointsDetection::~GraspablePointsDetection()
{
  RCLCPP_INFO(this->get_logger(), "/%s node is destructed.", this->get_name());
}

void GraspablePointsDetection::pointCloudCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr received_cloud_msg)
{
  received_cloud_frame_id_ = received_cloud_msg->header.frame_id;
  msg_stamp_ = received_cloud_msg->header.stamp;

  detectGraspablePoints(received_cloud_msg);
}

void GraspablePointsDetection::detectGraspablePoints(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & received_cloud_msg)
{
  RCLCPP_INFO(this->get_logger(), "========== Detecting Graspable Points... ==========");
  auto start_time = std::chrono::high_resolution_clock::now();

  // === Downsampling ===

  pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
  downsamplePointCloud(*received_cloud_msg, downsampled_cloud);

  if (downsampled_cloud.points.size() < 3) {
    // No faces. Process will have segmentation fault.
    // RCLCPP_WARN(this->get_logger(), "");
    return;
  }

  // === Transformation to Regression Plane ===

  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  alignPointCloudAndBroadcastTF(
    downsampled_cloud, transformed_cloud, received_cloud_frame_id_, kRegressionPlaneFrameId_);

  // === Interpolation ===

  pcl::PointCloud<pcl::PointXYZ> interpolated_cloud;
  interpolatePointCloud(transformed_cloud, interpolated_cloud);

  // Get minimum values for re-transform
  std::array<float, 3> offset_vec_for_retransform = getMinValues(interpolated_cloud);

  // === Voxelization ===

  auto voxel_matrix = voxelizePointCloud(interpolated_cloud);

  // === Voxel Matching ===

  std::vector<GraspablePointVoxel> voxel_grasp_candidates =
    evaluateVoxelMatching(voxel_matrix, offset_vec_for_retransform);

  // === Re-transformation ===

  // NOTE: Re-transform only to the regression plane frame, NOT to robot-based frame, for better visualization
  // If you want to further retransform to input camera depth optical frame, you have to modify the function
  std::vector<GraspablePointPhysical> physical_grasp_candidates =
    retransformToPhysical(voxel_grasp_candidates, offset_vec_for_retransform);

  // === Graspable Points Filtering ===

  // High-scored graspable point candidates
  std::vector<GraspablePointPhysical> valid_grasp_candidates =
    extractValidGraspablePoints(physical_grasp_candidates);

  // === Visualization ===

  // Graspability score map (Criterion I)
  visualizeGraspabilityScoreMap(physical_grasp_candidates);

  // Curvature Combined (Criterion II)
  // High graspability score map
  visualizeHighGraspabilityScoreMap(valid_grasp_candidates);

  // === Clustering ===

  extractClusterCentroids(valid_grasp_candidates);

  auto end_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  RCLCPP_INFO(
    this->get_logger(), "Total time for graspable points detection: %ld ms",
    elapsed_time.count() / 1000);
  RCLCPP_INFO(this->get_logger(), "========== Detected Graspable Points! =============");
}

void GraspablePointsDetection::downsamplePointCloud(
  const sensor_msgs::msg::PointCloud2 & input_cloud_msg,
  pcl::PointCloud<pcl::PointXYZ> & downsampled_cloud)
{
  // VoxelGrid filtering
  // Ref: http://www.pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid

  // Convert to PCL data type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(input_cloud_msg, *cloud);

  // === Perform the actual filtering ===

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);

  const auto kVoxelSize = graspable_points_detection::kVoxelSize;
  sor.setLeafSize(kVoxelSize, kVoxelSize, kVoxelSize);

  sor.filter(downsampled_cloud);

#if DEBUG
  // Convert to ROS msg and publish
  sensor_msgs::msg::PointCloud2 downsampled_cloud_msg;
  pcl::toROSMsg(downsampled_cloud, downsampled_cloud_msg);
  downsampled_cloud_msg.header = input_cloud_msg.header;
  downsampled_cloud_msg.header.frame_id = received_cloud_frame_id_;
  downsampled_point_cloud_pub_->publish(downsampled_cloud_msg);
#endif
}

void GraspablePointsDetection::estimateRegressionPlaneNormal(
  const pcl::PointCloud<pcl::PointXYZ> & input_cloud, const Eigen::Vector4f & centroid,
  Eigen::Vector3f & normal)
{
  Eigen::Matrix3f covariance_matrix;
  pcl::computeCovarianceMatrixNormalized(input_cloud, centroid, covariance_matrix);

  // Compute normals by Principal Component Analysis (PCA)
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> pca(covariance_matrix, Eigen::ComputeEigenvectors);

  // HACK: SelfAdjointEigenSolver automatically sorts and outputs eigenvalues in ascending order.
  normal = pca.eigenvectors().col(0);

#if DEBUG
  Eigen::Vector3f centroid3f = centroid.head<3>();
  visualizeVector(normal, centroid3f, received_cloud_frame_id_, "normal_vector");
#endif
}

void GraspablePointsDetection::alignPointCloudAndBroadcastTF(
  const pcl::PointCloud<pcl::PointXYZ> & input_cloud,
  pcl::PointCloud<pcl::PointXYZ> & transformed_cloud, const std::string & parent_frame_id,
  const std::string & child_frame_id)
{
#if DEBUG
  auto start_time = std::chrono::high_resolution_clock::now();
#endif

  Eigen::Vector4f centroid;
  Eigen::Matrix3f rotation_matrix;

  Eigen::Vector3f normal_vector;

  pcl::compute3DCentroid(input_cloud, centroid);

  estimateRegressionPlaneNormal(input_cloud, centroid, normal_vector);
  Eigen::Vector3f centroid3f = centroid.head<3>();

  // Adjusting the direction of the normal vector
  if (centroid3f.dot(normal_vector) > 0) {
    normal_vector = -normal_vector;
  }

  Eigen::Vector3f y_vector = centroid3f.cross(normal_vector).normalized();
  Eigen::Vector3f x_vector = normal_vector.cross(y_vector).normalized();

  rotation_matrix.col(0) = x_vector;
  rotation_matrix.col(1) = -y_vector;
  rotation_matrix.col(2) = normal_vector;

  // Homogeneous Transformation Matrix
  // Represent (R^T * p - R^T * c) as a single matrix
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = rotation_matrix.transpose();
  transform.block<3, 1>(0, 3) = -rotation_matrix.transpose() * centroid3f;

  pcl::transformPointCloud(input_cloud, transformed_cloud, transform);

#if DEBUG
  auto end_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  RCLCPP_INFO(this->get_logger(), "Elapsed time for transformation: %ld µs", elapsed_time.count());
#endif

  broadcastRegressionPlaneTF(centroid, rotation_matrix, parent_frame_id, child_frame_id);
}

void GraspablePointsDetection::interpolatePointCloud(
  const pcl::PointCloud<pcl::PointXYZ> & input_cloud,
  pcl::PointCloud<pcl::PointXYZ> & interpolated_cloud)
{
#if DEBUG
  auto start_time = std::chrono::high_resolution_clock::now();
#endif

  if (input_cloud.empty()) {
    return;
  }

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(input_cloud, min_pt, max_pt);

  float x_width = max_pt.x - min_pt.x;
  float y_width = max_pt.y - min_pt.y;

  std::vector<float> x, y, z;
  x.reserve(input_cloud.size());
  y.reserve(input_cloud.size());
  z.reserve(input_cloud.size());

  for (const auto & point : input_cloud.points) {
    x.push_back(point.x);
    y.push_back(point.y);
    z.push_back(point.z);
  }

  _2D::LinearDelaunayTriangleInterpolator<float> delaunay_interpolator;
  delaunay_interpolator.setData(x, y, z);

  // Compute grid size
  float grid_size = 1.0f /
    (std::round(std::sqrt(input_cloud.size() / (x_width * y_width) * (5.0f / 3.0f)) * 10000.0f) /
     10000.0f);

  if constexpr (graspable_points_detection::kArtificiallyAddPoints) {
    float min_grid = graspable_points_detection::kVoxelSize * 2.0f;
    while (grid_size > min_grid) {
      grid_size /= 1.2f;
    }
  }

  // Grid Generation and Interpolation
  int est_size = std::ceil(x_width / grid_size) * std::ceil(y_width / grid_size);
  interpolated_cloud.reserve(est_size);

  for (float curr_x = min_pt.x; curr_x <= max_pt.x; curr_x += grid_size) {
    for (float curr_y = min_pt.y; curr_y <= max_pt.y; curr_y += grid_size) {
      float interp_z = delaunay_interpolator(curr_x, curr_y);

      if (interp_z != 0.0f && !std::isnan(interp_z)) {
        interpolated_cloud.push_back(pcl::PointXYZ(curr_x, curr_y, interp_z));
      }
    }
  }

#if DEBUG
  // Convert to ROS msg and publish
  sensor_msgs::msg::PointCloud2 interpolated_cloud_msg;
  pcl::toROSMsg(interpolated_cloud, interpolated_cloud_msg);
  interpolated_cloud_msg.header.frame_id = kRegressionPlaneFrameId_;
  interpolated_cloud_msg.header.stamp = msg_stamp_;
  interpolated_point_cloud_pub_->publish(interpolated_cloud_msg);

  auto end_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  RCLCPP_INFO(this->get_logger(), "Elapsed time for interpolation: %ld µs", elapsed_time.count());
#endif
}

std::vector<std::vector<std::vector<int>>> GraspablePointsDetection::voxelizePointCloud(
  const pcl::PointCloud<pcl::PointXYZ> & input_cloud)
{
#if DEBUG
  auto start_time = std::chrono::high_resolution_clock::now();
#endif

  if (input_cloud.empty()) {
    return {};
  }

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(input_cloud, min_pt, max_pt);

  const float kVoxelSize = graspable_points_detection::kVoxelSize;

  int x_size = static_cast<int>(std::floor((max_pt.x - min_pt.x) / kVoxelSize)) + 1;
  int y_size = static_cast<int>(std::floor((max_pt.y - min_pt.y) / kVoxelSize)) + 1;
  int z_size = static_cast<int>(std::floor((max_pt.z - min_pt.z) / kVoxelSize)) + 1;

  std::vector<std::vector<std::vector<int>>> voxelized_grid(
    x_size, std::vector<std::vector<int>>(y_size, std::vector<int>(z_size, 0)));

  for (const auto & point : input_cloud.points) {
    // offset the point by the minimum coordinate so minimum is now 0 then divide by resolution to know in which voxel the point is.
    int idx_x = static_cast<int>((point.x - min_pt.x) / kVoxelSize + (kVoxelSize / 4.0f));
    int idx_y = static_cast<int>((point.y - min_pt.y) / kVoxelSize + (kVoxelSize / 4.0f));
    int idx_z = static_cast<int>((point.z - min_pt.z) / kVoxelSize + (kVoxelSize / 4.0f));

    // Bounds checking to avoid segmentation fault.
    if (
      idx_x >= 0 && idx_x < x_size && idx_y >= 0 && idx_y < y_size && idx_z >= 0 &&
      idx_z < z_size) {
      voxelized_grid[idx_x][idx_y][idx_z] = 1;
    }
  }

#if DEBUG
  auto end_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  RCLCPP_INFO(this->get_logger(), "Elapsed time for voxelization: %ld µs", elapsed_time.count());
#endif

  return voxelized_grid;
}

std::array<float, 3> GraspablePointsDetection::getMinValues(
  const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  if (cloud.empty()) {
    return {0.0f, 0.0f, 0.0f};
  }

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(cloud, min_pt, max_pt);

  return {min_pt.x, min_pt.y, min_pt.z};
}

std::vector<GraspablePointsDetection::GraspablePointVoxel>
GraspablePointsDetection::evaluateVoxelMatching(
  const std::vector<std::vector<std::vector<int>>> & terrain_matrix,
  const std::array<float, 3> & offset_vector)
{
#if DEBUG
  auto start_time = std::chrono::high_resolution_clock::now();
#endif

  std::vector<GraspablePointVoxel> voxel_grasp_candidates;

  if (terrain_matrix.empty() || terrain_matrix[0].empty() || terrain_matrix[0][0].empty()) {
    return voxel_grasp_candidates;
  }

  const int size_x = terrain_matrix.size();
  const int size_y = terrain_matrix[0].size();
  const int size_z = terrain_matrix[0][0].size();

  // === Search Range Computation ===

  using namespace graspable_points_detection;

  // Half size of gripper mask
  int half_mask_x = kGripperMaskSize / 2;  // ?: static_cast<int>(kGripperMaskHalfSize);?
  int half_mask_y = kGripperMaskSize / 2;

  // To prevent the gripper from protruding beyond the point cloud boundary,
  // the search range is narrowed inward by the mask radius plus one voxel (safety margin).
  const int clip_margin_x = half_mask_x + 1;
  const int clip_margin_y = half_mask_y + 1;

  // Index for starting and ending the search
  const int search_start_x = clip_margin_x;
  const int search_end_x = size_x - clip_margin_x;

  const int search_start_y = clip_margin_y;
  const int search_end_y = size_y - clip_margin_y;

  int start_z = static_cast<int>(
    std::abs(offset_vector[2]) / kVoxelSize + kDeleteLowerTargetsThreshold / kVoxelSize);

  // Offset amount to raise the z-coordinate for searching to prevent the gripper from penetrating below the ground (Z=0)
  int z_shift = kGripperMaskHeight - kExtraSheet;

  // Lambda expression to check for out-of-bounds access
  auto is_within_bounds = [&](int x, int y, int z) {
    return x >= 0 && x < size_x && y >= 0 && y < size_y && z >= 0 && z < size_z;
  };

  // === Main Loop for Searching ===

  for (int cx = search_start_x; cx < search_end_x; ++cx) {
    for (int cy = search_start_y; cy < search_end_y; ++cy) {
      for (int cz = start_z; cz < size_z; ++cz) {
        if (terrain_matrix[cx][cy][cz] == 0) {
          continue;
        }

        // === Direct Evaluation Using 3D Convolution (Sliding Window) ===
        // Overlay the gripper mask directly onto the target region (convolution) and
        // count both the “total number of voxels within the region” and the “proper points that do not interfere with the gripper” in one batch.

        int total_matching_voxels = 0;  // Total number of voxels within the region
        int num_proper_points = 0;      // Number of proper points that don't interfere with gripper

        for (int mx = 0; mx < kGripperMaskSize; ++mx) {
          for (int my = 0; my < kGripperMaskSize; ++my) {
            for (int mz = 0; mz < kGripperMaskHeight; ++mz) {
              // Compute where the mask coordinates (mx, my, mz) correspond within the terrain matrix
              int target_x = cx - half_mask_x + mx;
              int target_y = cy - half_mask_y + my;
              int target_z = cz - z_shift + mz;

              if (is_within_bounds(target_x, target_y, target_z)) {
                int val = terrain_matrix[target_x][target_y][target_z];
                total_matching_voxels += val;
                num_proper_points += val * gripper_mask_[mx][my][mz];
              }
            }
          }
        }

        if (total_matching_voxels == 0) {
          continue;
        }

        // === Graspability Score Computation ===

        float graspability_score =
          (static_cast<float>(num_proper_points) / total_matching_voxels) * 100.0f;

        // The computed graspability score is reduced by a penalty ratio
        // if the number of solid terrain voxels inside the subset is below a threshold (Threshold of Solid Voxels, TSV).
        // We only penalize those points which have an erroneously high graspability score.
        if (total_matching_voxels <= kThreshold && graspability_score >= kMinScoreForPenalty) {
          float penalty =
            ((kThreshold - total_matching_voxels) / static_cast<float>(kThreshold)) * 100.0f;
          graspability_score -= penalty;
        }

        // Store voxel coordinate and score of graspable point
        if (graspability_score >= 0.0f) {
          GraspablePointVoxel pt;
          pt.x = cx;
          pt.y = cy;
          pt.z = cz;
          pt.score = graspability_score;
          voxel_grasp_candidates.push_back(pt);
        }
      }
    }
  }

#if DEBUG
  RCLCPP_INFO(
    this->get_logger(), "Size of graspable points after voxel matching: %ld",
    voxel_grasp_candidates.size());
  auto end_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  RCLCPP_INFO(this->get_logger(), "Elapsed time for voxel matching: %ld µs", elapsed_time.count());
#endif

  return voxel_grasp_candidates;
}

std::vector<GraspablePointsDetection::GraspablePointPhysical>
GraspablePointsDetection::retransformToPhysical(
  const std::vector<GraspablePointVoxel> & voxel_points, const std::array<float, 3> & offset_vector)
{
  std::vector<GraspablePointPhysical> physical_points;

  if (voxel_points.empty()) {
    return physical_points;
  }

  physical_points.reserve(voxel_points.size());

  using namespace graspable_points_detection;

  // Re-transformation using the parameters in the voxelization step
  for (const auto & pt : voxel_points) {
    GraspablePointPhysical phys_pt;

    // ?: Is the calculation "- (kVoxelSize / 4.0f)" necessary?
    phys_pt.x = (static_cast<float>(pt.x) - (kVoxelSize / 4.0f)) * kVoxelSize + offset_vector[0];
    phys_pt.y = (static_cast<float>(pt.y) - (kVoxelSize / 4.0f)) * kVoxelSize + offset_vector[1];
    phys_pt.z = (static_cast<float>(pt.z) - (kVoxelSize / 4.0f)) * kVoxelSize + offset_vector[2];
    phys_pt.score = pt.score;

    physical_points.push_back(phys_pt);
  }

  return physical_points;
}

std::vector<GraspablePointsDetection::GraspablePointPhysical>
GraspablePointsDetection::extractValidGraspablePoints(
  const std::vector<GraspablePointPhysical> & points)
{
  std::vector<GraspablePointPhysical> valid_points;
  valid_points.reserve(points.size());

  using namespace graspable_points_detection;

  for (const auto & pt : points) {
    // Extract only points where the score is above the threshold and the Z-coordinate is above the lower threshold.
    if (pt.score >= kGraspabilityThreshold && pt.z > kDeleteLowerTargetsThreshold) {
      valid_points.push_back(pt);
    }
  }

  return valid_points;
}

void GraspablePointsDetection::visualizeGraspabilityScoreMap(
  const std::vector<GraspablePointPhysical> & points)
{
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  pcl_cloud.reserve(points.size());

  using namespace graspable_points_detection;

  for (const auto & pt : points) {
    pcl::PointXYZRGB p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;

    // If the Z-axis is below the lower threshold, color is forcibly set to “white”.
    if (pt.z < kDeleteLowerTargetsThreshold) {
      p.r = p.g = p.b = 255;
    }
    // Apply a gradient from red to green based on the score.
    else {
      // Convert scores (0-100) to percentages (t) ranging from 0.0 to 1.0
      float t = std::clamp(pt.score / 100.0f, 0.0f, 1.0f);

      // Calculate the array index (floating-point)
      float float_idx = t * (kColorMapSize - 1);
      int idx1 = static_cast<int>(std::floor(float_idx));
      int idx2 = std::min(idx1 + 1, kColorMapSize - 1);

      // Calculate the ratio between two keyframes
      float local_t = float_idx - idx1;

      // Calculate color using Lerp
      p.r = static_cast<uint8_t>(
        kGraspabilityColorMap[idx1][0] +
        (kGraspabilityColorMap[idx2][0] - kGraspabilityColorMap[idx1][0]) * local_t);
      p.g = static_cast<uint8_t>(
        kGraspabilityColorMap[idx1][1] +
        (kGraspabilityColorMap[idx2][1] - kGraspabilityColorMap[idx1][1]) * local_t);
      // p.b = static_cast<uint8_t>(
      //   kOriginalColorMap[idx1][2] +
      //   (kOriginalColorMap[idx2][2] - kOriginalColorMap[idx1][2]) * local_t);
      p.b = 0;
    }

    pcl_cloud.push_back(p);
  }

  sensor_msgs::msg::PointCloud2 graspability_score_cloud_msg;
  pcl::toROSMsg(pcl_cloud, graspability_score_cloud_msg);
  graspability_score_cloud_msg.header.frame_id = kRegressionPlaneFrameId_;
  graspability_score_cloud_msg.header.stamp = msg_stamp_;

  graspability_score_map_pub_->publish(graspability_score_cloud_msg);
}

void GraspablePointsDetection::visualizeHighGraspabilityScoreMap(
  const std::vector<GraspablePointPhysical> & valid_points)
{
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  pcl_cloud.reserve(valid_points.size());

  using namespace graspable_points_detection;

  for (const auto & pt : valid_points) {
    pcl::PointXYZRGB p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    // Magenta
    p.r = 128;
    p.g = 0;
    p.b = 128;

    pcl_cloud.push_back(p);
  }

#if DEBUG
  RCLCPP_INFO(
    this->get_logger(), "Number of graspable points befor clustering: %ld", pcl_cloud.size());
#endif

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(pcl_cloud, cloud_msg);
  cloud_msg.header.frame_id = kRegressionPlaneFrameId_;
  cloud_msg.header.stamp = msg_stamp_;

  high_graspability_score_map_pub_->publish(cloud_msg);
}

void GraspablePointsDetection::extractClusterCentroids(
  const std::vector<GraspablePointPhysical> & valid_points)
{
#if DEBUG
  auto start_time = std::chrono::high_resolution_clock::now();
#endif

  if (valid_points.empty()) {
    return;
  }

  // Convert to PCL cloud for clustering
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(valid_points.size());
  for (const auto & pt : valid_points) {
    cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  using namespace graspable_points_detection;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(kPalmDiameter * 0.001f);
  ec.setMinClusterSize(1);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);

  // Compute centroids (= graspable_points)
  pcl::PointCloud<pcl::PointXYZ> graspable_points;
  graspable_points.reserve(cluster_indices.size());

  for (const auto & indices : cluster_indices) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, indices.indices, centroid);
    graspable_points.push_back(pcl::PointXYZ(centroid[0], centroid[1], centroid[2]));
  }

  // Convert to ROS msg and publish
  sensor_msgs::msg::PointCloud2 graspable_points_msg;
  pcl::toROSMsg(graspable_points, graspable_points_msg);
  graspable_points_msg.header.frame_id = kRegressionPlaneFrameId_;
  graspable_points_msg.header.stamp = msg_stamp_;

  clustered_graspable_points_pub_->publish(graspable_points_msg);

#if DEBUG
  RCLCPP_INFO(
    this->get_logger(), "Number of graspable points after clustering: %ld",
    graspable_points.size());
  auto end_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  RCLCPP_INFO(this->get_logger(), "Elapsed time for voxel matching: %ld µs", elapsed_time.count());
#endif
}

void GraspablePointsDetection::visualizeVector(
  const Eigen::Vector3f & direction_vector, const Eigen::Vector3f & origin_point,
  const std::string & frame_id, const std::string & object_name)
{
  if (direction_vector.norm() < 1e-6) {
    return;
  }

  constexpr float kArrowLength = 0.5f;

  geometry_msgs::msg::Point start_pt;
  start_pt.x = origin_point.x();
  start_pt.y = origin_point.y();
  start_pt.z = origin_point.z();

  geometry_msgs::msg::Point end_pt;
  end_pt.x = origin_point.x() + (direction_vector.x() * kArrowLength);
  end_pt.y = origin_point.y() + (direction_vector.y() * kArrowLength);
  end_pt.z = origin_point.z() + (direction_vector.z() * kArrowLength);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = msg_stamp_;
  marker.ns = object_name;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.points.resize(2);
  marker.points[0] = start_pt;
  marker.points[1] = end_pt;

  // Arrow width [mm] (x: axis width, y: arrowhead width, z: arrowhead length)
  marker.scale.x = 0.005;
  marker.scale.y = 0.01;
  marker.scale.z = 0.02;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

  normal_vector_marker_pub_->publish(marker);
}

void GraspablePointsDetection::broadcastRegressionPlaneTF(
  const Eigen::Vector4f & translation, const Eigen::Matrix3f & rotation_matrix,
  const std::string & parent_frame, const std::string & child_frame)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = msg_stamp_;
  tf.header.frame_id = parent_frame;
  tf.child_frame_id = child_frame;

  tf.transform.translation.x = translation.x();
  tf.transform.translation.y = translation.y();
  tf.transform.translation.z = translation.z();

  Eigen::Quaternionf q(rotation_matrix);
  q.normalize();

  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(tf);
}

void GraspablePointsDetection::createGripperMask()
{
#if DEBUG
  auto start_time = std::chrono::high_resolution_clock::now();
#endif

  gripper_mask_.assign(
    kGripperMaskSize,
    std::vector<std::vector<int>>(kGripperMaskSize, std::vector<int>(kGripperMaskHeight, 0)));

  float opening_depth = kOpeningSpineDepth;
  if (opening_depth == 1.0f) {
    opening_depth = 2.0f;
  }

  for (int z = 0; z < kGripperMaskHeight; ++z) {
    float z_val = static_cast<float>(z + 1);

    // Compute radius of inner cone and outer solid area
    float graspable_radius = kGripperMaskTopSolidRadius +
      (kGripperMaskHalfSize - kGripperMaskTopSolidRadius) * static_cast<float>(z) /
        (kGripperMaskClearance - 1.0f);

    float unreachable_radius = kGripperMaskHalfSize -
      std::round(kGripperMaskHalfSize - (kOpeningSpineRadius + kSpineDepth)) *
        static_cast<float>(z) / (opening_depth - 1.0f);

    for (int y = 0; y < kGripperMaskSize; ++y) {
      float dy = kGripperMaskHalfSize - static_cast<float>(y);

      for (int x = 0; x < kGripperMaskSize; ++x) {
        float dx = kGripperMaskHalfSize - static_cast<float>(x);

        // Distance from center of layer
        float dist = std::hypot(dx, dy);

        bool is_solid = false;

        // Interference determination logic expression
        // Judges whether it is a solid(1) region or not
        if (z_val <= kGripperMaskClearance) {
          // Conditions for the lower half
          if (
            dist < graspable_radius ||
            dist > unreachable_radius *
                2.0f  //added a times 2 to prevent having walls in the gripper shape
          ) {
            is_solid = true;
          }
        } else {
          // Conditions for the upper half
          if (
            dist >
              kGripperMaskBottomVoidRadius ||  //added condition to ave a bigger hole at the bottom of gripper
            z_val < std::round(kGripperMaskClearance * 1.75f)) {
            is_solid = true;
          }
        }

        if (is_solid) {
          // Set the element as 1
          // Flip the gripper mask vertically
          gripper_mask_[x][y][(kGripperMaskHeight - 1) - z] = 1;
        }
      }
    }
  }

#if DEBUG
  auto end_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  RCLCPP_INFO(
    this->get_logger(), "Elapsed time for creation of gripper mask: %ld µs", elapsed_time.count());
#endif
}

}  // namespace graspable_points_detection

RCLCPP_COMPONENTS_REGISTER_NODE(graspable_points_detection::GraspablePointsDetection)
