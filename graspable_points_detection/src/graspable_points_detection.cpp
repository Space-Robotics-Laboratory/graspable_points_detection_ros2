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

// PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#define DEBUG true

namespace graspable_points_detection
{

GraspablePointsDetection::GraspablePointsDetection(const rclcpp::NodeOptions & options)
: rclcpp::Node("graspable_points_detection", options)
{
  // Publishers
  downsampled_point_clout_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/downsampled_points", 1);

  // Subscriber
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/merged_pcd", 1,  // TODO: Parameterize topic name
    std::bind(&GraspablePointsDetection::pointCloudCallBack, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "/%s node is constructed.", this->get_name());
}

GraspablePointsDetection::~GraspablePointsDetection()
{
  RCLCPP_INFO(this->get_logger(), "/%s node is destructed.", this->get_name());
}

void GraspablePointsDetection::pointCloudCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud_msg)
{
  RCLCPP_INFO(this->get_logger(), "========== Detecting Graspable Points... ==========");

  pcl::PointCloud<pcl::PointXYZ> downsampled_pcd;
  downsample(*point_cloud_msg, downsampled_pcd);
}

void GraspablePointsDetection::downsample(
  const sensor_msgs::msg::PointCloud2 & pcd_msg, pcl::PointCloud<pcl::PointXYZ> & downsampled_pcd)
{
  // VoxelGrid filtering
  // Ref: http://www.pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid

  // Convert to PCL data type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(pcd_msg, *cloud);

  // === Perform the actual filtering ===

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);

  auto kVoxelSize = graspable_points_detection::kVoxelSize;
  sor.setLeafSize(kVoxelSize, kVoxelSize, kVoxelSize);

  sor.filter(downsampled_pcd);

  // Convert to ROS msg and publish
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(downsampled_pcd, output);
  output.header = pcd_msg.header;
  output.header.frame_id = "map";
  downsampled_point_clout_pub_->publish(output);
}

}  // namespace graspable_points_detection

RCLCPP_COMPONENTS_REGISTER_NODE(graspable_points_detection::GraspablePointsDetection)
