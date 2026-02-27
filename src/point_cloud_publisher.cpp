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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#define DEBUG false

using namespace std::chrono_literals;

class PointCloudPublisherNode : public rclcpp::Node
{
public:
  PointCloudPublisherNode() : Node("point_cloud_publisher_node")
  {
    // Parameters
    this->declare_parameter<std::string>("pcd_file_name", "limbero_testfield_quartered.pcd");
    this->declare_parameter<int>("publish_rate", 1000);
    this->declare_parameter<std::string>("frame_id", "map");

    std::string file_name = this->get_parameter("pcd_file_name").as_string();
    int loop_rate_ms = this->get_parameter("publish_rate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();

    // Publisher
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_point_cloud", 1);

    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("graspable_points_detection");
    std::string pcd_path = pkg_share_dir + "/data/" + file_name;

    RCLCPP_INFO(this->get_logger(), "Loading PCD file from: data/%s", file_name.c_str());

    pcl::PointCloud<pcl::PointXYZ> map_cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, map_cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read PCD file: %s", pcd_path.c_str());
      return;
    }

    pcl::toROSMsg(map_cloud, map_cloud_msg_);
    map_cloud_msg_.header.frame_id = frame_id_;

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(loop_rate_ms),
      std::bind(&PointCloudPublisherNode::publishMapPointCloud, this));

    RCLCPP_INFO(this->get_logger(), "Successfully loaded: %s", file_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing point cloud in %d ms", loop_rate_ms);
  }

private:
  void publishMapPointCloud()
  {
    map_cloud_msg_.header.stamp = this->now();
    map_pub_->publish(map_cloud_msg_);

    RCLCPP_DEBUG(this->get_logger(), "Published point cloud under frame: %s", frame_id_.c_str());
  }

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  // Variables
  sensor_msgs::msg::PointCloud2 map_cloud_msg_;
  std::string frame_id_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
