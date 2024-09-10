
#include <iostream> 
#include <fstream> 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

using namespace std::chrono_literals;

class publish_pcl_node : public rclcpp::Node{
    public:
        publish_pcl_node() : Node ("publish_pcl_node")
        {
            map_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_pcd",1);
            timer_ = this->create_wall_timer(500ms, std::bind(&publish_pcl_node::publish_message, this));
            pcl::io::loadPCDFile<pcl::PointXYZ>("/home/antonin/ros2_ws/src/graspable_points_detection_ros2/include/pcd_data/"+name,pcl_map);
            pcl::toROSMsg(pcl_map, msg_map);
        }
    private: 
        void publish_message()
        {
            msg_map.header.frame_id = "camera_depth_optical_frame";
            msg_map.header.stamp = this->now();
            map_pub_->publish(msg_map);
            std::cout <<name<<" published under the frame :"<<msg_map.header.frame_id << std::endl;
        }
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        pcl::PointCloud<pcl::PointXYZ> pcl_map;
        sensor_msgs::msg::PointCloud2 msg_map;

        //please change for the name of the map you want to send for testing
        std::string name ="limbero_testfield_quarted.pcd";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<publish_pcl_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
