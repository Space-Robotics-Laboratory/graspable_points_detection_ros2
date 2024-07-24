
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
            pub1  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_pcd",1);
            timer_ = this->create_wall_timer(500ms, std::bind(&publish_pcl_node::publish_message, this));
            pcl::io::loadPCDFile<pcl::PointXYZ>("/home/antonin/linked_ws/src/graspable_points_detection_ros2/pcd_data/"+name,cloud);
            pcl::toROSMsg(cloud, output1);
        }
    private: 
        void publish_message()
        {
            // Set the header information
            output1.header.frame_id = "camera_depth_optical_frame";
            output1.header.stamp = this->now();
            // Publish the point cloud
            pub1->publish(output1);
            std::cout <<name<<" published under the frame :"<<output1.header.frame_id << std::endl;
        }
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub1;
        rclcpp::TimerBase::SharedPtr timer_;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        sensor_msgs::msg::PointCloud2 output1;

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
