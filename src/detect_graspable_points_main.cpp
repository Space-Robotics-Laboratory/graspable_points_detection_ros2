#include "detect_graspable_points/detect_graspable_points.hpp"

//create node publisher class
class Detect_grasapable_point_node : public rclcpp::Node
{
  public:
    Detect_grasapable_point_node()
    : Node("Detect_grasapable_point_node")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/merged_pcd", 10, std::bind(&Detect_grasapable_point_node::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2 & msg) const
    {
      detect_graspable_points detect_graspable_points(msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};


//main function that will run the detection
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Detect_grasapable_point_node>());
  rclcpp::shutdown();
  return 0;
}

