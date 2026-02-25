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
  * Kentaro Uno (unoken at astro.mech.tohoku.ac.jp)
  * \brief Every function of the graspable target detection algorithm
*/

#include "graspable_points_detection/graspable_points_detection.hpp"

#include <cmath>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
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

}  // anonymous namespace

namespace graspable_points_detection
{

GraspablePointsDetection::GraspablePointsDetection(const rclcpp::NodeOptions & options)
: rclcpp::Node("graspable_points_detection", options)
{
  // Publishers
  downsampled_point_cloud_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/downsampled_points", 1);
  interpolated_point_cloud_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/interpolated_points", 1);

  normal_vector_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("~/normal_vector", 1);

  // Subscriber
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/merged_pcd", 1,  // TODO: Parameterize topic name
    std::bind(&GraspablePointsDetection::pointCloudCallBack, this, std::placeholders::_1));

  // TF
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

#if DEBUG
  auto start_mask = std::chrono::high_resolution_clock::now();
#endif
  createGripperMask();
#if DEBUG
  auto stop_mask = std::chrono::high_resolution_clock::now();
  auto duration_mask =
    std::chrono::duration_cast<std::chrono::microseconds>(stop_mask - start_mask);
  std::cout << "Time for creation of gripper mask in µs : " << duration_mask.count() << std::endl;
#endif

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

GraspablePointsDetection::~GraspablePointsDetection()
{
  RCLCPP_INFO(this->get_logger(), "/%s node is destructed.", this->get_name());
}

void GraspablePointsDetection::pointCloudCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr received_cloud_msg)
{
  RCLCPP_INFO(this->get_logger(), "========== Detecting Graspable Points... ==========");

  msg_stamp_ = received_cloud_msg->header.stamp;

  // === Downsample ===

  pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
  downsamplePointCloud(*received_cloud_msg, downsampled_cloud);

  if (downsampled_cloud.points.size() < 3) {
    // HACK: No faces. Process will have segmentation fault.
    // RCLCPP_WARN(this->get_logger(), "");
    return;
  }

  // === Transform ===

#if DEBUG
  auto start_transform = std::chrono::high_resolution_clock::now();
#endif

  Eigen::Vector4f centroid_point;
  Eigen::Matrix3f rotation_matrix;
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  alignPointCloudToRegressionPlane(
    downsampled_cloud, transformed_cloud, centroid_point, rotation_matrix);

#if DEBUG
  auto stop_transform = std::chrono::high_resolution_clock::now();
  auto duration_transform =
    std::chrono::duration_cast<std::chrono::microseconds>(stop_transform - start_transform);
  std::cout << "Time for transformation in µs : " << duration_transform.count() << std::endl;
#endif

  // === Interpolate ===

#if DEBUG
  auto start_interp = std::chrono::high_resolution_clock::now();
  broadcastRegressionPlaneTF(centroid_point, rotation_matrix, "map", "regression_plane_frame");
#endif

  pcl::PointCloud<pcl::PointXYZ> interpolated_cloud;
  interpolatePointCloud(transformed_cloud, interpolated_cloud);

#if DEBUG
  auto stop_interp = std::chrono::high_resolution_clock::now();
  auto duration_interp =
    std::chrono::duration_cast<std::chrono::microseconds>(stop_interp - start_interp);
  std::cout << "Time for interpolation in µs : " << duration_interp.count() << std::endl;
#endif

  // === Voxelization ===

#if DEBUG
  auto start_voxel = std::chrono::high_resolution_clock::now();
#endif

  std::vector<std::vector<std::vector<int>>> voxel_matrix;
  voxel_matrix = voxelizePointCloud(interpolated_cloud);

#if DEBUG
  auto stop_voxel = std::chrono::high_resolution_clock::now();
  auto duration_voxel =
    std::chrono::duration_cast<std::chrono::microseconds>(stop_voxel - start_voxel);
  std::cout << "Time for voxelization in µs : " << duration_voxel.count() << std::endl;
#endif
}

void GraspablePointsDetection::downsamplePointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud_msg,
  pcl::PointCloud<pcl::PointXYZ> & downsampled_cloud)
{
  // VoxelGrid filtering
  // Ref: http://www.pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid

  // Convert to PCL data type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud_msg, *cloud);

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
  downsampled_cloud_msg.header = cloud_msg.header;
  downsampled_cloud_msg.header.frame_id = "map";  // TODO: Change frame_id
  downsampled_point_cloud_pub_->publish(downsampled_cloud_msg);
#endif
}

void GraspablePointsDetection::estimateRegressionPlaneNormal(
  const pcl::PointCloud<pcl::PointXYZ> & raw_cloud, const Eigen::Vector4f & centroid,
  Eigen::Vector3f & normal)
{
  Eigen::Matrix3f covariance_matrix;
  pcl::computeCovarianceMatrixNormalized(raw_cloud, centroid, covariance_matrix);

  // Compute normals by Principal Component Analysis (PCA)
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> pca(covariance_matrix, Eigen::ComputeEigenvectors);

  // HACK: SelfAdjointEigenSolver automatically sorts and outputs eigenvalues in ascending order.
  normal = pca.eigenvectors().col(0);

#if DEBUG
  Eigen::Vector3f centroid3f = centroid.head<3>();
  visualizeVector(normal, centroid3f, "map", "normal_vector");  // TODO: Change frame_id
#endif
}

void GraspablePointsDetection::alignPointCloudToRegressionPlane(
  const pcl::PointCloud<pcl::PointXYZ> & raw_cloud,
  pcl::PointCloud<pcl::PointXYZ> & transformed_cloud, Eigen::Vector4f & centroid,
  Eigen::Matrix3f & rotation_matrix)
{
  Eigen::Vector3f normal_vector;

  pcl::compute3DCentroid(raw_cloud, centroid);

  estimateRegressionPlaneNormal(raw_cloud, centroid, normal_vector);
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

  pcl::transformPointCloud(raw_cloud, transformed_cloud, transform);
}

void GraspablePointsDetection::interpolatePointCloud(
  const pcl::PointCloud<pcl::PointXYZ> & raw_cloud,
  pcl::PointCloud<pcl::PointXYZ> & interpolated_cloud)
{
  if (raw_cloud.empty()) {
    return;
  }

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(raw_cloud, min_pt, max_pt);

  float x_width = max_pt.x - min_pt.x;
  float y_width = max_pt.y - min_pt.y;

  std::vector<float> x, y, z;
  x.reserve(raw_cloud.size());
  y.reserve(raw_cloud.size());
  z.reserve(raw_cloud.size());

  for (const auto & point : raw_cloud.points) {
    x.push_back(point.x);
    y.push_back(point.y);
    z.push_back(point.z);
  }

  _2D::LinearDelaunayTriangleInterpolator<float> delaunay_interpolator;
  delaunay_interpolator.setData(x, y, z);

  // Compute grid size
  float grid_size = 1.0f /
    (std::round(std::sqrt(raw_cloud.size() / (x_width * y_width) * (5.0f / 3.0f)) * 10000.0f) /
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
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(interpolated_cloud, msg);
  msg.header.frame_id = "regression_plane_frame";  // TODO: Change frame_id
  msg.header.stamp = msg_stamp_;
  interpolated_point_cloud_pub_->publish(msg);
#endif
}

std::vector<std::vector<std::vector<int>>> GraspablePointsDetection::voxelizePointCloud(
  const pcl::PointCloud<pcl::PointXYZ> & input_cloud)
{
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

  return voxelized_grid;
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
  marker.header.stamp = this->now();
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
  const Eigen::Vector4f & centroid, const Eigen::Matrix3f & rotation_matrix,
  const std::string & parent_frame, const std::string & child_frame)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->now();
  tf.header.frame_id = parent_frame;
  tf.child_frame_id = child_frame;

  tf.transform.translation.x = centroid.x();
  tf.transform.translation.y = centroid.y();
  tf.transform.translation.z = centroid.z();

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
}

}  // namespace graspable_points_detection

RCLCPP_COMPONENTS_REGISTER_NODE(graspable_points_detection::GraspablePointsDetection)
