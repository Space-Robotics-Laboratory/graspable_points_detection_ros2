#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.h>

#include <fstream>
#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/filesystem_helper.hpp>

#define DEBUG false

using namespace std::chrono_literals;

class publish_pcl_node : public rclcpp::Node
{
public:
  publish_pcl_node() : Node("publish_pcl_node")
  {
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_pcd", 1);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(kTimerLoopRate_),
      std::bind(&publish_pcl_node::publish_message, this));

    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("detect_graspable_points");
    rcpputils::fs::path pkg_install_dir =
      rcpputils::fs::path(pkg_share_dir).parent_path().parent_path();
    std::string pcd_data_path = pkg_install_dir.string() + "/include/pcd_data/" + kMapName_;
#if DEBUG
    std::cout << "pcd_data_path = " << pcd_data_path << std::endl;
#endif  // DEBUG

    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_data_path, pcl_map_);
    pcl::toROSMsg(pcl_map_, msg_map_);
  }

private:
  void publish_message()
  {
    msg_map_.header.frame_id = "camera_depth_optical_frame";
    msg_map_.header.stamp = this->now();
    map_pub_->publish(msg_map_);
    std::cout << kMapName_ << " published under the frame :" << msg_map_.header.frame_id
              << std::endl;
  }

  rclcpp::TimerBase::SharedPtr timer_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

  // Variables
  pcl::PointCloud<pcl::PointXYZ> pcl_map_;
  sensor_msgs::msg::PointCloud2 msg_map_;
  int kTimerLoopRate_ = 500;  // [ms]

  // Please change for the name of the map you want to send for testing
  std::string kMapName_ = "limbero_testfield_quarted.pcd";
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<publish_pcl_node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
