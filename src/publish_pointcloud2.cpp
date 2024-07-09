
#include <iostream> 
#include <fstream> 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>


using namespace std;
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Rate loop_rate(1);
    rclcpp::Node publish_pcl_node =rclcpp::Node("publish_pcl_node");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub1 = publish_pcl_node.create_publisher<sensor_msgs::msg::PointCloud2> ("merged_pcd", 1);

    while (rclcpp::ok())
    {   

        // Create a point cloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        vector<int> indices;

        // ****** SCAR-E Maps (Large maps, set voxel size to 0.01!) ******* //

        // ****** Scene 1 ******* //

        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/scanned_cloud_primitive.pcd",cloud);

        // ****** Scene 2 ******* //

        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/simulated_cloud.pcd",cloud);
        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/simulated_cloud_pre-scan.pcd",cloud);

        // ****** Scene 3 ******* //

        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/scanned_cloud_Apr8.pcd",cloud);

        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/testfield_apr8_upscaled.pcd",cloud);


        // ****** Scene 4 ******* //

        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/scanned_cloud_realtime_1.5x.pcd",cloud);
        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/Artificial_rocks_pose_1.pcd",cloud);
        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/artificial_rocks.pcd",cloud);
        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/scanned_cloud_realtime.pcd",cloud);

        // ****** HubRobo Maps (small maps, set voxel size to 0.001!) ******* //

        pcl::io::loadPCDFile<pcl::PointXYZ>("/home/srl-limb/ros2_ws/src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/realsense_octree.pcd",cloud);


        // ****** Cloud of curvatures ******* //
        // Remove that if the bug in curvature detection is fixed


        // Convert the point cloud to a ROS message
        sensor_msgs::msg::PointCloud2 output1;
        pcl::toROSMsg(cloud, output1);

        // Set the header information
        output1.header.frame_id = "regression_plane_frame";
        output1.header.stamp = publish_pcl_node.now();


        // Publish the point cloud
        pub1->publish(output1);

        cout <<"PointCloud published under the frame'camera_depth_optical_frame'" << endl;

        // Sleep to maintain the 1 Hz publishing rate
        loop_rate.sleep();
    }


    return 0;


}