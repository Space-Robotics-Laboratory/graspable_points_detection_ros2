/*!
  * \file detect_graspable_points.hpp
  * \author 
  * Jitao Zheng (jitao.zheng@tum.de)
  * Ringeval-Meusnier Antonin (ringeval@insta-toulouse.fr)
  * Taku Okawara (taku.okawara.t3 at dc.tohoku.ac.jp)
  * Kentaro Uno (unoken at astro.mech.tohoku.ac.jp)
  * \brief Every function of the graspable target detection algorithm
*/

#include "detect_graspable_points/detect_graspable_points.hpp"
#include <pcl/filters/filter.h>
#include <unistd.h>
#include <stdio.h>

#define DEBUG_MOD true
#define TEST_MOD false
#define VOXEL_SIZE 0.004

#define MY_PRINT(x) std::cout<<#x<< "=" << x <<std::endl


/*! ******************************
 ***       Constructor       *****
 *********************************/

detect_graspable_points::detect_graspable_points()
: Node("detect_graspable_point")
{
  std::string topic_prefix = "/"+ std::string(this->get_name());

  //Subscriber
  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/merged_pcd",1,std::bind(&detect_graspable_points::mapReceivedCallBack,this, std::placeholders::_1));

  //Publisher
  downsampled_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (topic_prefix + "/down_sample_points", 1);
  interpolate_point_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (topic_prefix + "/interpolated_point", 1);
  peaks_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (topic_prefix + "/peaks_of_convex_surface", 1);
  graspability_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (topic_prefix + "/graspability_map", 1);
  combined_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (topic_prefix + "/graspable_points", 1); //graspable point
  transformed_point_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (topic_prefix + "/transformed_point", 1);
  point_visualization_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker> (topic_prefix + "/point_visualization_marker", 1);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker> (topic_prefix + "/normal_vector", 1);
  
  //static broadcaster for now since the computation is quite long and the message are comming one by one not by a constant stream thus leading to sometime messge here for too long and crashing nodes
  tf_static_broadcast_= std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  //debugging publisher to know if the input map is wrong in the first place
  debugging_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (topic_prefix + "/received_map_checking", 1);
}



/*! ******************************
 ***        Destructor       *****
 *********************************/
detect_graspable_points::~detect_graspable_points()
{

}

//******** CALLBACK *******//

void detect_graspable_points::mapReceivedCallBack(const sensor_msgs::msg::PointCloud2 received_cloud_msg)
{
  #if DEBUG_MOD
  debugging_pub_->publish(received_cloud_msg);
  #endif
  // ****** PARAMETERS ******* //

  // ****** SCAR-E Gripper ******* //
/*
  GripperParam gripper_param = { //in [mm]!
		71, // palm_diameter
		92, // palm_diameter_of_finger_joints
		40, // finger_length
		41, // spine_length
		5, // spine_depth
		85, // opening_angle
		10, // closing_angle
		136, // opening_spine_radius, the distance from the center of palm to the furthest point of the spines
		5, // opening_spine_depth
		90, // closing_height, Vertical distance between the tip of the spine and the bottom of the palm when closed
		4, // margin_of_top_solid_diameter
		2, // inside_margin_of_bottom_void_diameter
	}; 
*/


  // ****** HubRobo Gripper ******* //
/*
    GripperParam gripper_param = { //in [mm]!
    32, // palm_diameter
    28, // palm_diameter_of_finger_joints
    15, // finger_length
    15, // spine_length
    5, // spine_depth
    75, // opening_angle
    30, // closing_angle
    37, // opening_spine_radius, the distance from the center of palm to the furthest point of the spines
    5, // opening_spine_depth
    16, // closing_height, Vertical distance between the tip of the spine and the bottom of the palm when closed
    4, // margin_of_top_solid_diameter
    2, // inside_margin_of_bottom_void_diameter
  };
*/

  // ****** LIMBERO ******* //

  GripperParam gripper_param = { //in [mm]!
		62, // palm_diameter
		69, // palm_diameter_of_finger_joints
		26, // finger_length
		25, // spine_length
		12, // spine_depth
		79, // opening_angle
		42, // closing_angle
		77, // opening_spine_radius, the distance from the center of palm to the furthest point of the spines
		4, // opening_spine_depth
		27, // closing_height, Vertical distance between the tip of the spine and the bottom of the palm when closed
		4, // margin_of_top_solid_diameter
		2, // inside_margin_of_bottom_void_diameter
	};


  // ****** General Parameters ******* //

  MatchingSettings matching_settings ={
    VOXEL_SIZE, // voxel size [m]
    150, // threshold of numbers of solid voxels within the subset (TSV) (SCAR-E: 120)
    "on", // delete the targets whose z positions are lower than a limit value: on or off
    0.015, // [m] Lower threshold of targets (Apr8_realtime: 0.025, Scanned: -0.05, Simulated: -0.07, primitive: 0.01, leaning_bouldering_holds: 0.01)
    0.07, // [m] Searching radius for the curvature (SCAR-E: 0.09, HubRobo: 0.03)
    3, // size of extra sheet above the top layer of gripper mask (H_add)(SCAR-E: 1)
    90 // Graspability threshold. Above which we can call it graspable with great certainty
  };

  #if (TEST_MOD)
  std::vector<int> x = {0,1,7,1,3,4,4,3,4,9};
  std::vector<int> y = {0,1,0,5,3,3,4,4,2,2};
  std::vector<int> z = {0,1,5,1,3,3,4,4,6,5};

  std::vector<std::vector<std::vector<int>>> voxel_for_test;
  voxel_for_test.resize(10, std::vector<std::vector<int>>(10, std::vector<int>(7,0)));
  for (int i=0; i<10;i++ ){
    voxel_for_test[x[i]][y[i]][z[i]]=1;
  }
  //clip and substract etc...
  std::string filename = "/home/antonin/linked_ws/src/graspable_points_detection_ros2/pcd_data/test_raw.csv";
  save3DVectorToFile(voxel_for_test,filename);
  voxel_for_test = vox_clip(1,1,voxel_for_test);
  filename = "/home/antonin/linked_ws/src/graspable_points_detection_ros2/pcd_data/test_clipped.csv";
  save3DVectorToFile(voxel_for_test,filename);

  for (int x=0; x<voxel_for_test.size(); x++) 
  {
    for (int y=0; y<voxel_for_test[0].size(); y++)
    {
      for (int z=0; z<voxel_for_test[0][0].size(); z++)
      {
        if (voxel_for_test[x][y][z] != 0)
        {
          std::cout<<"clipped: "<<" x: "<< x << " y: "<< y<<" z: "<< z<<std::endl;
        }
      }
    }
  }
  #endif

  #if (!TEST_MOD)
  // ************************** //
  std::cout <<"************************ START ************************" << std::endl;
  auto start_overall = std::chrono::high_resolution_clock::now();
  camera_frame = received_cloud_msg.header.frame_id; //get the input cloud frame id for later process

  // ****** Downsampling ******* //
  pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
  downsampling(received_cloud_msg, "downsampling_frame", downsampled_cloud, matching_settings.voxel_size);

  // ****** remove all the NaNs from pcd ******* //
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(downsampled_cloud, downsampled_cloud, indices);
  indices.clear();


  // ****** get the min values from the pcd, for offset re-transform ******* //
  if(downsampled_cloud.points.size() > 0) // If downsampled_points.points.size()==0, process will have segmentation fault
  {
    // ****** Transform ******* //
    Eigen::Vector4f centroid_point;
    Eigen::Vector3f normal_vector;
    Eigen::Matrix3f rotation_matrix;
    pcl::PointCloud<pcl::PointXYZ> transformed_pcl;
    #if DEBUG_MOD
    auto start_transform = std::chrono::high_resolution_clock::now();
    #endif
    pcd_transform(downsampled_cloud,transformed_pcl,centroid_point,rotation_matrix);  // /!\ modified downsampled_points to real cloud data 
    pcl::removeNaNFromPointCloud(transformed_pcl, transformed_pcl, indices);
    indices.clear();
    #if DEBUG_MOD
    auto stop_transform = std::chrono::high_resolution_clock::now();
    auto duration_transform = std::chrono::duration_cast<std::chrono::microseconds>(stop_transform - start_transform);
    std::cout <<"Time for pcd_transform in µs : " << duration_transform.count() << std::endl;
    #endif


    // ****** Testing and Debug of Transformation ******* //

    // The following description are used for just visualization of pcd_transform()
    // Just instisute the relative pose between the camera and the regression_plane calculated by pcd_transform()
    geometry_msgs::msg::Pose relative_pose_between_camera_and_regression_plane;
    relative_pose_between_camera_and_regression_plane.position.x = centroid_point[0];
    relative_pose_between_camera_and_regression_plane.position.y = centroid_point[1];
    relative_pose_between_camera_and_regression_plane.position.z = centroid_point[2];
    Eigen::Quaternionf relative_quat_between_camera_and_regression_plane(rotation_matrix);

    // considering normalization for avoiding an error
    relative_pose_between_camera_and_regression_plane.orientation.x = relative_quat_between_camera_and_regression_plane.normalized().x();
    relative_pose_between_camera_and_regression_plane.orientation.y = relative_quat_between_camera_and_regression_plane.normalized().y();
    relative_pose_between_camera_and_regression_plane.orientation.z = relative_quat_between_camera_and_regression_plane.normalized().z();
    relative_pose_between_camera_and_regression_plane.orientation.w = relative_quat_between_camera_and_regression_plane.normalized().w();

    // set a frame between a camera and the regression_plane calculated by pcd_transform()
    tf_broadcast_from_pose(received_cloud_msg.header.frame_id, "regression_plane_frame", relative_pose_between_camera_and_regression_plane);

    #if DEBUG_MOD
    // publish the 3D centroid point for debug
    visualizePoint(centroid_point[0], centroid_point[1], centroid_point[2], "centroid_point", received_cloud_msg.header.frame_id);
    #endif
    // finish the description related to pcd_transform().

    std::vector<std::vector<std::vector<int>>> voxel_matrix;  // important that size needs to be 0 in declaration for the resize to work properly
    std::vector<float> offset_vector_for_retransform;
    pcl::PointCloud<pcl::PointXYZRGB> peak_cloud;
    sensor_msgs::msg::PointCloud2 peak_coordinates;


    // ****** Interpolate ******* //
    pcl::PointCloud<pcl::PointXYZ> interpolated_pcl;
    #if DEBUG_MOD
    auto start_interp = std::chrono::high_resolution_clock::now();
    #endif
    pcd_interpolate(transformed_pcl,interpolated_pcl);
    pcl::removeNaNFromPointCloud(interpolated_pcl, interpolated_pcl, indices);
    indices.clear();
    #if DEBUG_MOD   
    auto stop_interp = std::chrono::high_resolution_clock::now();
    auto duration_interp = std::chrono::duration_cast<std::chrono::microseconds>(stop_interp - start_interp);
    std::cout <<"Time for pcd_interpolate in µs : " << duration_interp.count() << std::endl;

    // publish the pointcloud with respect to regrssion_plane_frame
    sensor_msgs::msg::PointCloud2 interpolated_point_cloud_for_pub; 
    pcl::toROSMsg(interpolated_pcl, interpolated_point_cloud_for_pub); 
    interpolated_point_cloud_for_pub.header.frame_id="regression_plane_frame";
    interpolated_point_cloud_for_pub.header.stamp = this->now();
    interpolate_point_pub_->publish(interpolated_point_cloud_for_pub);
    #endif

    // ****** Find the peaks ******* //
    #if DEBUG_MOD
    auto start_peaks= std::chrono::high_resolution_clock::now();
    #endif      
    detectTerrainPeaks(interpolated_pcl, peak_cloud,peak_coordinates, matching_settings);
    #if DEBUG_MOD   
    auto stop_peaks = std::chrono::high_resolution_clock::now();
    auto duration_peaks = std::chrono::duration_cast<std::chrono::microseconds>(stop_peaks - start_peaks);
    std::cout <<"Time for detectTerrainPeaks in µs : " << duration_peaks.count() << std::endl;
    #endif

    // ****** Voxelize ******* //
    #if DEBUG_MOD
    auto start_voxel = std::chrono::high_resolution_clock::now();
    #endif
    voxel_matrix = pcd_voxelize(interpolated_pcl, matching_settings.voxel_size);
    #if DEBUG_MOD
    auto stop_voxel = std::chrono::high_resolution_clock::now();
    auto duration_voxel = std::chrono::duration_cast<std::chrono::microseconds>(stop_voxel - start_voxel);
    std::cout <<"Time for pcd_voxelize in µs : " << duration_voxel.count() << std::endl;
    #endif

    //get minimum values for re-transform later
    offset_vector_for_retransform = getMinValues(interpolated_pcl);

    // ****** Gripper mask ******* //
    std::vector<std::vector<std::vector<int>>> gripper_mask(0, std::vector<std::vector<int>>(0, std::vector<int>(0,0)));
    gripper_mask = creategrippermask(gripper_param, matching_settings.voxel_size);
    #if DEBUG_MOD
    std::cout<<"Mask created!"<<std::endl;
    #endif

    // ****** Voxel matching ******* //
    // create an empty 2d array ready to use. it will be 4 columns: x, y, z and graspability score
    std::vector<std::vector<int>> graspable_points(0, std::vector<int>(0,0));
    #if DEBUG_MOD
    auto start_matching = std::chrono::high_resolution_clock::now();
    #endif
    graspable_points = voxel_matching(voxel_matrix, gripper_mask, matching_settings);
    #if DEBUG_MOD
    std::cout<<"Size of graspable after voxel_matching"<<graspable_points[0].size() <<std::endl;
    auto stop_matching = std::chrono::high_resolution_clock::now();
    auto duration_matching = std::chrono::duration_cast<std::chrono::microseconds>(stop_matching - start_matching);
    std::cout <<"Time for voxel_matching in µs : " << duration_matching.count() << std::endl;
    #endif

    // ****** Re-transform ******* //

    std::vector<std::vector<float>> graspable_points_after_retransform;
    // NOTE: Re-transform only to the regression plane frame, NOT to robot-based frame, for better visualization
    // If you want to further retransform to input camera depth optical frame, you have to modify the function
    graspable_points_after_retransform = pcd_re_transform(graspable_points, matching_settings.voxel_size, offset_vector_for_retransform);

    // ****** Visualization ******* //
    // ****** Color Gradient (Criterion I) ******* //
    // Turn the graspabiliy score to color gradient. Publish the pointcloud with respect to regression_plane_frame
    sensor_msgs::msg::PointCloud2 marker_of_graspable_points;
    marker_of_graspable_points = visualizeRainbow(graspable_points_after_retransform, matching_settings);
    graspability_map_pub_->publish(marker_of_graspable_points);

    // ****** Curvature Combined (Criterion II) ******* //
    sensor_msgs::msg::PointCloud2 results_of_combined_analysis;
    // Searching distance for intersection between convex peaks and graspability maxima
    float distance_threshold = matching_settings.voxel_size;
    // combine Graspability maxima with convex shape analysis. output the intersection of both.
    results_of_combined_analysis = combinedAnalysis(graspable_points_after_retransform, peak_cloud, distance_threshold, matching_settings);
    combined_pub_->publish(results_of_combined_analysis);

    // ****** Overall time consumption ******* //
    auto stop_overall = std::chrono::high_resolution_clock::now();
    auto duration_overall = std::chrono::duration_cast<std::chrono::microseconds>(stop_overall - start_overall);
    std::cout <<"Total time in ms : " << duration_overall.count()/1000 << std::endl;
    std::cout <<"************************  END  ************************" << std::endl;
  }
  #endif
}


// ****** SECONDARY FUNCTIONS ******* //

std::vector<float> detect_graspable_points::getMinValues(const pcl::PointCloud<pcl::PointXYZ>& pointCloud)
{
  std::vector<float> minValues(3);
  std::vector<float> x,y,z;
  pcl::PointXYZ point; //variable for storing point values temporary before adding to pcl
  for (long unsigned int i = 0; i < pointCloud.size(); ++i)
  {
    point = pointCloud.points[i];
    x.push_back(point.x);
    y.push_back(point.y);
    z.push_back(point.z);
  }
  minValues[0]= *min_element(x.begin(),x.end());
  minValues[1]= *min_element(y.begin(),y.end());
  minValues[2]= *min_element(z.begin(),z.end());
  return minValues;
}

void detect_graspable_points::tf_broadcast(const std::string frame_id)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = camera_frame;
  transformStamped.child_frame_id = frame_id;
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  tf_static_broadcast_->sendTransform(transformStamped);
}

void detect_graspable_points::tf_broadcast_from_pose(const std::string parant_frame_id, const std::string child_frame_id_to, geometry_msgs::msg::Pose relative_pose_between_frame)
{
  //static tf2_ros::TransformBroadcaster br;
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = parant_frame_id;
  transformStamped.child_frame_id = child_frame_id_to;
  transformStamped.transform.translation.x = relative_pose_between_frame.position.x;
  transformStamped.transform.translation.y = relative_pose_between_frame.position.y;
  transformStamped.transform.translation.z = relative_pose_between_frame.position.z;
  transformStamped.transform.rotation.x = relative_pose_between_frame.orientation.x;
  transformStamped.transform.rotation.y = relative_pose_between_frame.orientation.y;
  transformStamped.transform.rotation.z = relative_pose_between_frame.orientation.z;
  transformStamped.transform.rotation.w = relative_pose_between_frame.orientation.w;
  tf_static_broadcast_->sendTransform(transformStamped);
}

void detect_graspable_points::save3DVectorToFile(const std::vector<std::vector<std::vector<int>>>& vector3D, const std::string& filename) 
{
  std::ofstream outFile(filename);
  if (!outFile) 
  {
    std::cerr << "Failed to open file for writing: " << filename << std::endl;
    return;
  }
  for (long unsigned int x=0; x<vector3D.size(); x++) 
  {
    for (long unsigned int y=0; y<vector3D[0].size(); y++)
    {
      for (long unsigned int z=0; z<vector3D[0][0].size(); z++)
      {
        if (vector3D[x][y][z] != 0)
        {
          outFile << x << "," << y << "," << z << "\n";
        }
      }
    }
  }
  outFile.close();
  #if DEBUG_MOD
  std::cout << "3D std::vector saved to file: " << filename << std::endl;
  #endif
}

void detect_graspable_points::visualizePoint(const double x, const double y, const double z, const std::string object_name, const std::string frame_id)
{
  visualization_msgs::msg::Marker visualization_point;
	visualization_point.header.frame_id = frame_id;
  visualization_point.ns = object_name;
  visualization_point.id = 0;
  visualization_point.type = visualization_msgs::msg::Marker::SPHERE;
  visualization_point.scale.x = 0.05;	// 0.0 -> warning
  visualization_point.scale.y = 0.05;	// 0.0 -> warning
  visualization_point.scale.z = 0.05;	// 0.0 -> warning
  visualization_point.color.r = 1.0f;
  visualization_point.color.g = 1.0f;
  visualization_point.color.b = 0.0f;
  visualization_point.color.a = 0.8f;
  visualization_point.pose.position.x = x;
  visualization_point.pose.position.y = y;
  visualization_point.pose.position.z = z;
  visualization_point.pose.orientation.x = 0.0;
  visualization_point.pose.orientation.y = 0.0;
  visualization_point.pose.orientation.z = 0.0;
  visualization_point.pose.orientation.w = 1.0;  
  visualization_point.header.stamp = this->now();
  visualization_point.lifetime = rclcpp::Duration::from_nanoseconds(0);
  for(int i=0; i<10; i++)
  {
    point_visualization_marker_pub_->publish(visualization_point);
    rclcpp::Duration::from_seconds(0.1);
  }
}

void detect_graspable_points::visualizeVector(const Eigen::Vector3f &vector_of_start_point, const Eigen::Vector3f &vector_of_end_point, const std::string object_name, const std::string frame_id)
{
  geometry_msgs::msg::Vector3 arrow;  // config arrow shape
  arrow.x = 0.02;
  arrow.y = 0.04;
  arrow.z = 0.1;

  geometry_msgs::msg::Point initial_point;
  initial_point.x = vector_of_end_point[0];
  initial_point.y = vector_of_end_point[1];
  initial_point.z = vector_of_end_point[2];

  geometry_msgs::msg::Point end_point;
  end_point.x = vector_of_start_point[0] + initial_point.x;
  end_point.y = vector_of_start_point[1] + initial_point.y;
  end_point.z = vector_of_start_point[2] + initial_point.z;

  if(abs(vector_of_start_point.norm()) > 0)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = object_name;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.points.resize(2);
    marker.points[0] = initial_point;
    marker.points[1] = end_point;
    marker.scale = arrow;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);
    marker_pub_->publish(marker);
  }
}

std::vector<int> subtractInteger(const std::vector<int>& vector, int num) 
{
  std::vector<int> result;  
  // Perform subtraction for each element
  for (int element : vector) 
  {
    result.push_back(element - num);
  }
  return result;
}

// ****** DOWNSAMPLING ******* //

void detect_graspable_points::downsampling(const sensor_msgs::msg::PointCloud2 & cloud_msg, const std::string frame_id, pcl::PointCloud<pcl::PointXYZ> &filtered_points, const float cube_size)
{
  // VoxelGrid filtering
  // Ref: http://www.pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid

  // Container for original & filtered data
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  // Convert to PCL data type
  pcl_conversions::toPCL(cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (cube_size, cube_size, cube_size);
  sor.filter (*cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::msg::PointCloud2 output;
  pcl_conversions::moveFromPCL(*cloud_filtered, output);
  output.header.frame_id = frame_id;
  // Publish the data
  downsampled_points_pub_->publish(output);

  // get filtered_points as this function's output
  // But this is cause of segmentation fault
  // pcl::fromROSMsg (output, filtered_points);

  // convert from pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ> through sensor_msgs::PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered_by_PC1;
  //sensor_msgs::convertPointCloud2ToPointCloud(output, cloud_filtered_by_PC1);

  pcl::fromROSMsg (output, cloud_filtered_by_PC1);
  if(cloud_filtered_by_PC1.points.size() > 0)
  {
    for(auto &filtered_point_by_PC1 : cloud_filtered_by_PC1.points)
    {
      pcl::PointXYZ filtered_point;
      filtered_point.x = filtered_point_by_PC1.x;
      filtered_point.y = filtered_point_by_PC1.y;
      filtered_point.z = filtered_point_by_PC1.z;
      filtered_points.push_back(filtered_point);
    }
  }
  // publish TF for visualization
  // to shift positions of rendering point clouds
  tf_broadcast(frame_id);
}

// ****** REGRESSION PLANE ******* //

//Function for least square on pcd
void detect_graspable_points::pcd_least_squares_plane(const pcl::PointCloud<pcl::PointXYZ> raw_pcd, Eigen::Vector4f &centroid,  Eigen::Vector3f &normal_vector_of_plane )
{
  Eigen::MatrixXf pcd_matrix = raw_pcd.getMatrixXfMap(3,4,0);//Create a matrix with points as value
  Eigen::MatrixXf deviation_matrix(3,raw_pcd.size());
  Eigen::MatrixXf deviation_matrix_transposed(raw_pcd.size(),3);
  Eigen::Vector3f eigen_values; 
  Eigen::Matrix3f product_deviation_matrix;  // This matrix is used for calculation eigenvector
  Eigen::Vector3f centroid_vector_of_plane3f;

  //compute the centroid
  pcl::compute3DCentroid(raw_pcd,centroid);
  for(long unsigned int i=0; i< raw_pcd.size()-1; i++)
  {
    deviation_matrix(0,i)=(pcd_matrix(0,i) - centroid(0));
    deviation_matrix(1,i)=(pcd_matrix(1,i) - centroid(1));
    deviation_matrix(2,i)=(pcd_matrix(2,i) - centroid(2)); // substracting the centroid (std::vector) to each column of the matrix, one column being a point
  }
  deviation_matrix_transposed = deviation_matrix.transpose();
  product_deviation_matrix=deviation_matrix*deviation_matrix_transposed;
  
  //EigenSolver computes the eigen vectors and eigen values but returns them as complex float arrays
  Eigen::EigenSolver<Eigen::MatrixXf> es(product_deviation_matrix);

  //transform complex float std::vector in float std::vector
  Eigen::Vector3cf complex_eigen_values = es.eigenvalues();
  eigen_values(0)= real(complex_eigen_values(0));
  eigen_values(1)= real(complex_eigen_values(1));
  eigen_values(2)= real(complex_eigen_values(2));
  
  //sort the smallest eigen values and its corresponding index 
  float a = eigen_values(0);
  int index=0;
  if (eigen_values(1) < a) {
    index = 1;
    a=eigen_values(1);
  }
  if (eigen_values(2) < a) {
    index = 2;
  }

  //choose std::vector corresponding to the smallest eigen value and convert complex float to float
  Eigen::Vector3cf complex_normal_vector = es.eigenvectors().col(index);
  normal_vector_of_plane(0)= real(complex_normal_vector(0));
  normal_vector_of_plane(1)= real(complex_normal_vector(1));
  normal_vector_of_plane(2)= real(complex_normal_vector(2));

  centroid_vector_of_plane3f[0]= centroid[0];
  centroid_vector_of_plane3f[1]= centroid[1];
  centroid_vector_of_plane3f[2]= centroid[2]; 

  visualizeVector(normal_vector_of_plane, centroid_vector_of_plane3f, "normal_vector", camera_frame);
}

// ****** TRANSFORMATION ******* //
void detect_graspable_points::pcd_transform ( const pcl::PointCloud<pcl::PointXYZ>  raw_pcd, pcl::PointCloud<pcl::PointXYZ> &transformed_point_cloud, Eigen::Vector4f &centroid_vector_of_plane, Eigen::Matrix3f &rotation_matrix) {

	Eigen::Vector3f normal_vector_of_plane;
	Eigen::Vector3f y_vector_of_plane;
	Eigen::Vector3f x_vector_of_plane;
  Eigen::Vector3f centroid_vector_of_plane3f;
  Eigen::MatrixXf transformed_point_cloud_matrix(3,raw_pcd.size());
  Eigen::MatrixXf raw_pcd_matrix = raw_pcd.getMatrixXfMap(3,4,0);

	pcd_least_squares_plane(raw_pcd ,centroid_vector_of_plane, normal_vector_of_plane);

  //transfering values from 4f std::vector to 3f std::vector so .dot computes
  centroid_vector_of_plane3f<< centroid_vector_of_plane[0], centroid_vector_of_plane[1] , centroid_vector_of_plane[2]; 

	float innerproduct_of_centroid_normal_vector = centroid_vector_of_plane3f.dot(normal_vector_of_plane);
	if (innerproduct_of_centroid_normal_vector > 0) // changes direction of std::vector to be from ground to sky if needed
  {
		normal_vector_of_plane = - normal_vector_of_plane;
	}

	y_vector_of_plane = centroid_vector_of_plane3f.cross(normal_vector_of_plane); // .cross realize cross product
	y_vector_of_plane = y_vector_of_plane/y_vector_of_plane.norm(); //normalize std::vector
	x_vector_of_plane = normal_vector_of_plane.cross(y_vector_of_plane);

  //assign values to rotation matrix
	rotation_matrix.col(0) = x_vector_of_plane;
	rotation_matrix.col(1) = -y_vector_of_plane; // if no minus sign flipp problem occurs on y axis
	rotation_matrix.col(2) = normal_vector_of_plane;

  //operate the frame transformation
  Eigen::Matrix3f rotation_matrix_transposed;
  rotation_matrix_transposed=rotation_matrix.transpose();
  Eigen::MatrixXf raw_pcd_transposed(raw_pcd.size(),3);
  raw_pcd_transposed =raw_pcd_matrix.transpose();
  Eigen::VectorXf one(raw_pcd.size()) ;
  for (long unsigned int i=0;i<raw_pcd.size();i++)
  {
    one(i)=1;
  }
  transformed_point_cloud_matrix=rotation_matrix_transposed*raw_pcd_matrix - (rotation_matrix_transposed*centroid_vector_of_plane3f*one.transpose());
  //assign matrix values to point that is fed in tranformed pcl
  for (long unsigned int i = 0; i < raw_pcd.size(); ++i)
  {
    transformed_point_cloud.push_back (pcl::PointXYZ (transformed_point_cloud_matrix(0,i), transformed_point_cloud_matrix(1,i), transformed_point_cloud_matrix(2,i)));
  }
} 

// ****** INTERPOLATION ******* //
// Diagrams for better understanding:
// /HubRobo/tools/detect_graspable_points/fig_for_understanding/your_image
void detect_graspable_points::pcd_interpolate (const pcl::PointCloud<pcl::PointXYZ>  raw_pcd, pcl::PointCloud<pcl::PointXYZ> &interpolated_point_cloud)
{
  std::vector<double> x,y,z;
  pcl::PointXYZ point;
  // assign points into vectors for interpolation
  for (long unsigned int i = 0; i < raw_pcd.size(); ++i)
  {
    point = raw_pcd.points[i];
    x.push_back(point.x);
    y.push_back(point.y);
    z.push_back(point.z);
  }
  // create interpolator
  // .setData add all the known values x,y,z (f(x,y)=z) to the interpolator class
  _2D::LinearDelaunayTriangleInterpolator<double> delaunay_interpolator;
  delaunay_interpolator.setData(x,y,z);

  // operation for homogenization and keeping as many points as in input
  double min_y= *min_element(y.begin(),y.end());
  double min_x= *min_element(x.begin(),x.end());
  double max_y= *max_element(y.begin(),y.end());
  double max_x= *max_element(x.begin(),x.end());
  double x_width = max_x-min_x;
  double y_width = max_y-min_y;

  //double grid_size = 1/(round(sqrt(raw_pcd.size()/(x_width*y_width)*(5/3))));
  double grid_size = 1/(round(sqrt(raw_pcd.size() / (x_width * y_width)*(5/3) ) * 10000) / 10000);
  while (grid_size> VOXEL_SIZE*2){
    grid_size=grid_size/1.2; //not change it too fast to stay close to the target value, if changed manually like grid_size= VOXEL_SIZE*2 causes segfault in voxelization
  }

  // creates the regular std::vector x and y for the grid
  std::vector<double> x_grid_vector, y_grid_vector;
  for (double i = min_y; i < max_y; i+=grid_size)
  {
    y_grid_vector.push_back(i);
  }

  for (double i = min_x; i < max_x; i+=grid_size)
  {
    x_grid_vector.push_back(i);
  }
  // interpolate the values based on x and y vectors like a grid
  double interp_z;
  for (long unsigned int i = 0; i < x_grid_vector.size(); ++i) 
  {
    for (long unsigned int j = 0; j < y_grid_vector.size(); ++j) 
    {
      interp_z= delaunay_interpolator(x_grid_vector[i],y_grid_vector[j]);  // interpolate z at coorinates x,y from given data set
      if (interp_z != 0)          // if the point is interpolated at z=0 then it is not part of original range of data so we don't add it
      {
        interpolated_point_cloud.push_back(pcl::PointXYZ(x_grid_vector[i],y_grid_vector[j],interp_z)); //add the interpolated point
      }
      
    }
  }
}

// ****** VOXELIZATION ******* //
std::vector<std::vector<std::vector<int>>> detect_graspable_points::pcd_voxelize (const pcl::PointCloud<pcl::PointXYZ>  input_pcd, const float cube_size){
  
  //voxel grid creates voxels of said size and then only keeps the centroid of voxels that have points in them
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;

  std::vector<float> x,y,z;
  pcl::PointXYZ point; //variable for storing point values temporary before adding to pcl
  for (long unsigned int i = 0; i < input_pcd.size(); ++i)
  {
    point = input_pcd.points[i];
    x.push_back(point.x);
    y.push_back(point.y);
    z.push_back(point.z);
  }

  float min_y= *min_element(y.begin(),y.end());
  float min_x= *min_element(x.begin(),x.end());
  float max_y= *max_element(y.begin(),y.end());
  float max_x= *max_element(x.begin(),x.end());
  float min_z= *min_element(z.begin(),z.end());
  float max_z= *max_element(z.begin(),z.end());

  //size for the 3d array that will receive the voxels informations 
  int xmatrix_size,ymatrix_size,zmatrix_size;

  //size of the matrix is based on the range of data and how many cubes can fit inside
  //ex : from 0 to 9 you can fit 3 cube of size 3
  xmatrix_size= trunc((max_x-min_x)/cube_size)+1;
  ymatrix_size= trunc((max_y-min_y)/cube_size)+1;
  zmatrix_size= trunc((max_z-min_z)/cube_size)+1;
  // change data type for adding it to class
  pcl::PCLPointCloud2 point_cloud;
  pcl::PCLPointCloud2::Ptr point_cloud_2format (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2(input_pcd,point_cloud);
  *point_cloud_2format=point_cloud;

  voxel_grid.setInputCloud(point_cloud_2format);
  voxel_grid.setLeafSize(cube_size,cube_size,cube_size);

  // compute a filter to only keep centroid point of voxel
  pcl::PointCloud<pcl::PointXYZ> pcl_after_filtering;
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  voxel_grid.filter(*cloud_filtered);  //fails if the pointcloud is too big
  pcl::fromPCLPointCloud2 (*cloud_filtered, pcl_after_filtering);
  std::vector<std::vector<std::vector<int>>> voxelized_pcd(xmatrix_size, std::vector<std::vector<int>>(ymatrix_size, std::vector<int>(zmatrix_size,0)));
  int voxel_index_x=0;
  int voxel_index_y=0;
  int voxel_index_z=0;
  for (auto &point:pcl_after_filtering.points)
  {
    // offset the point by the minimum coordinate so minimum is now 0 then divide by resolution to know in which voxel the point is. 
    // trunc it to only keep integer part. refer to : /HubRobo/tools/detect_graspable_points/fig_for_understanding/step_of_voxelize.png
    voxel_index_x= trunc((point.x-min_x)/cube_size + cube_size/4);
    voxel_index_y= trunc((point.y-min_y)/cube_size + cube_size/4);
    voxel_index_z= trunc((point.z-min_z)/cube_size + cube_size/4);
    voxelized_pcd[voxel_index_x][voxel_index_y][voxel_index_z]=1;
  }
  x.clear();
  y.clear();
  z.clear();
  std::cout<<"After loop"<<std::endl;
  return voxelized_pcd;
}

// ****** GRASPABILITY SCORE ******* //
float detect_graspable_points::vox_evaluate(const int number_of_points, const std::vector<std::vector<std::vector<int>>>& subset_of_voxel_array, const std::vector<std::vector<std::vector<int>>>& gripper_mask) {
  int number_of_proper_points = 0;
  const int x_size = subset_of_voxel_array.size();
  const int y_size = subset_of_voxel_array[0].size();
  const int z_size = subset_of_voxel_array[0][0].size();
  for (int x = 0; x < x_size; x++) {
    for (int y = 0; y < y_size; y++) {
      for (int z = 0; z < z_size; z++) {
        number_of_proper_points += subset_of_voxel_array[x][y][z] * gripper_mask[x][y][z];
      }
    }
  }
  float graspability = (static_cast<float>(number_of_proper_points) / number_of_points) * 100.0;
  return graspability;
}

// ****** VOXEL CLIP ******* //
std::vector<std::vector<std::vector<int>>> detect_graspable_points::vox_clip(const int x, const int y, std::vector<std::vector<std::vector<int>>> search_voxel_array) {

//crop in x direction
  for (long unsigned int i=search_voxel_array.size()-x; i<search_voxel_array.size();++i)
  {
    for (long unsigned int j=0; j<search_voxel_array[0].size();++j)
    {
      for (long unsigned int k=0; k<search_voxel_array[0][0].size();++k)
        {
          search_voxel_array[i][j][k]=0;
        }
    }
  }
  for (int i=0; i<x; ++i)
  {
    for (long unsigned int j=0; j<search_voxel_array[0].size();++j)
    {
      for (long unsigned int k=0; k<search_voxel_array[0][0].size();++k)
        {
          search_voxel_array[i][j][k]=0;
        }
    }
  }
//crop in y direction
  for (long unsigned int i = 0 ; i < search_voxel_array.size() ; ++i)
  {
    for (long unsigned int j = search_voxel_array[0].size()-y ; j<search_voxel_array[0].size();++j)
    {
      for (long unsigned int k=0; k<search_voxel_array[0][0].size();++k)
        {
          search_voxel_array[i][j][k]=0;
        }
    }
  }
  for (long unsigned int i = 0 ; i < search_voxel_array.size() ; ++i)
  {
    for (int j = 0; j<y; ++j)
    {
      for (long unsigned int k=0; k<search_voxel_array[0][0].size();++k)
        {
          search_voxel_array[i][j][k]=0;
        }
    }
  }
  return search_voxel_array;
}

// ****** VOXEL EXTRACT ******* //
std::vector<std::vector<std::vector<int>>> detect_graspable_points::vox_extract(const std::vector<std::vector<std::vector<int>>>& voxel_array,
                                                                                const std::vector<int>& position_reference_point,
                                                                                const std::vector<int>& size_extracting) 
{
  int size_x = size_extracting[0];
  int size_y = size_extracting[1];
  int size_z = size_extracting[2];
  int ref_x = position_reference_point[0];
  int ref_y = position_reference_point[1];
  int ref_z = position_reference_point[2];
  std::vector<std::vector<std::vector<int>>> extracted_voxel_array(size_x, std::vector<std::vector<int>>(size_y, std::vector<int>(size_z, 0)));
  for (int i = 0; i < size_x; ++i) 
  {
    for (int j = 0; j < size_y; ++j) 
    {
      for (int k = 0; k < size_z; ++k) 
      {
        extracted_voxel_array[i][j][k] = voxel_array[ref_x + i][ref_y + j][ref_z + k];
      }
    }
  }
  return extracted_voxel_array;
}

// ****** GRIPPER MASK ******* //
std::vector<std::vector<std::vector<int>>> detect_graspable_points::creategrippermask(GripperParam gripper_param, float voxel_size)
{
  // Calculate the ratio of voxel size and 1mm in order to keep the gripper size in real world regardless of voxel size
  float ratio = 1 / (voxel_size * 1000);
  // Reduce or magnify the gripper's parameters to fit voxel's dimension
  // Change demensions from [mm] to [voxels]
  float palm_diameter = std::round(gripper_param.palm_diameter * ratio);
  float palm_diameter_of_finger_joints = std::round(gripper_param.palm_diameter_of_finger_joints * ratio);
  float finger_length = std::round(gripper_param.finger_length * ratio);
  float spine_length = std::round(gripper_param.spine_length * ratio);
  float spine_depth = std::round(gripper_param.spine_depth * ratio);
  float opening_spine_radius = std::round(gripper_param.opening_spine_radius * ratio);
  float opening_spine_depth = std::round(gripper_param.opening_spine_depth * ratio);
  float closing_height = std::round(gripper_param.closing_height * ratio);
  float margin_of_top_solid_diameter = std::round(gripper_param.margin_of_top_solid_diameter * ratio);
  float inside_margin_of_bottom_void_diameter = std::round(gripper_param.inside_margin_of_bottom_void_diameter * ratio);

  // Set the gripper-mask size
  float gripper_mask_half_size = (palm_diameter_of_finger_joints / 2) + finger_length + spine_length;
  float gripper_mask_size = 2 * gripper_mask_half_size + 1;
  float gripper_mask_height = closing_height;

  // Calculate the parameters to determine solid area and void area
  float gripper_mask_top_solid_radius = std::round((palm_diameter + margin_of_top_solid_diameter) / 2);
  float gripper_mask_clearance = std::round((gripper_mask_size - palm_diameter) / 2 * std::tan((90 - gripper_param.opening_angle)*(M_PI/180.0)));
  float gripper_mask_bottom_void_radius = std::round(palm_diameter / 2 + (gripper_mask_height * std::tan((gripper_param.closing_angle)*(M_PI/180.0))) - inside_margin_of_bottom_void_diameter);

  // Prepare a 3-dimensional array composed of 0
  std::vector<std::vector<std::vector<int>>> gripper_mask(gripper_mask_size, std::vector<std::vector<int>>(gripper_mask_size, std::vector<int>(gripper_mask_height, 0)));
  float grippable_radius = 0;
  float unreachble_radius = 0;
  float distance_from_center_of_layer = 0;

  //out
/*
  std::cout<<"*****VALUES*****"<<std::endl;
  MY_PRINT(ratio);
  MY_PRINT(palm_diameter);
  MY_PRINT(palm_diameter_of_finger_joints);
  MY_PRINT(finger_length);
  MY_PRINT(spine_length);
  MY_PRINT(spine_depth);
  MY_PRINT(opening_spine_radius);
  MY_PRINT(opening_spine_depth);
  MY_PRINT(closing_height);
  MY_PRINT(margin_of_top_solid_diameter);
  MY_PRINT(inside_margin_of_bottom_void_diameter);
  MY_PRINT(gripper_mask_half_size);
  MY_PRINT(gripper_mask_size);
  MY_PRINT(gripper_mask_height);
  MY_PRINT(gripper_mask_top_solid_radius);
  MY_PRINT(gripper_mask_clearance);
  MY_PRINT(gripper_mask_bottom_void_radius);
  std::cout<<"*****VALUES*****"<<std::endl;
*/

  std::ofstream outFile("/home/antonin/linked_ws/src/graspable_points_detection_ros2/pcd_data/Values.txt");

  // Make the gripper_mask by setting the elements of 1
  if (opening_spine_depth ==1){opening_spine_depth =2;} //if the spinde depth is one then there is a division by 0
  for (int z_subscript = 1; z_subscript < gripper_mask_height+1; ++z_subscript)
  {
    // Calculate radius of inner cone and outer solid area
    grippable_radius = gripper_mask_top_solid_radius + (gripper_mask_half_size - gripper_mask_top_solid_radius) * (z_subscript-1) / (gripper_mask_clearance-1);
    unreachble_radius = gripper_mask_half_size - std::round(gripper_mask_half_size - (opening_spine_radius + spine_depth)) * (z_subscript-1) / (opening_spine_depth-1);
    for (int y_subscript = 1; y_subscript < gripper_mask_size+1; ++y_subscript)
    {
      for (int x_subscript = 1; x_subscript < gripper_mask_size+1; ++x_subscript)
      {   // Caculate the distance from center of layer
        distance_from_center_of_layer = std::sqrt(std::pow(gripper_mask_half_size+1 - x_subscript, 2) + std::pow(gripper_mask_half_size+1 - y_subscript, 2));
        // Judges whether it is a solid(1) region or not
        if ((z_subscript <= gripper_mask_clearance && distance_from_center_of_layer < grippable_radius) ||
            (z_subscript<= gripper_mask_clearance && distance_from_center_of_layer > unreachble_radius*2 ) || //added a times 2 to prevent having walls in the gripper shape
            (z_subscript > gripper_mask_clearance && z_subscript != gripper_mask_height && distance_from_center_of_layer > gripper_mask_bottom_void_radius) || //added conditio to ave a bigger hole at the bottom of gripper
            (z_subscript < std::round(gripper_mask_clearance*1.75) && z_subscript > gripper_mask_clearance) ||
            (z_subscript == gripper_mask_height && distance_from_center_of_layer > gripper_mask_bottom_void_radius))
        {
          // Set the element as 1
          // gripper_mask[x_subscript-1][y_subscript-1][z_subscript-1] = 1;
          // Flip the gripper mask vertically
          gripper_mask[x_subscript-1][y_subscript-1][gripper_mask_height-z_subscript] = 1;
        }
      }
    }
  }
  std::string filename = "/home/antonin/linked_ws/src/graspable_points_detection_ros2/pcd_data/GripperMask.csv";
  save3DVectorToFile(gripper_mask, filename);
  return gripper_mask;
}


// ****** CURVATURE ANALYSIS ******* //

void detect_graspable_points::detectTerrainPeaks(pcl::PointCloud<pcl::PointXYZ> input_cloud,pcl::PointCloud<pcl::PointXYZRGB> &peak_visualization_cloud, sensor_msgs::msg::PointCloud2 &cloud_msg, 
                                                const MatchingSettings& matching_settings)
  {
  // This function carries out a convex peak detection using curvature analysis. First, we compute the surface normal vectors
  // of each point in a defined radius "searching_radius_for_normal_and_curvature". Next, we compute the respective principal
  // curvatures k1 and k2. Convex peaks are defined as points with an positive k1, k2 and k1*k2=K (Gaussian curvature).
  
  // Create a normal estimation object
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(input_cloud.makeShared());

  // Create a KD-Tree for searching
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  // Compute normals
  ne.setRadiusSearch(matching_settings.searching_radius_for_normal_and_curvature); // Adjust the radius as needed
  ne.compute(*cloud_normals);

  // Create a curvature estimation object
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> est;
  est.setInputCloud(input_cloud.makeShared());
  est.setInputNormals(cloud_normals);
  est.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures> ());

  // Compute curvature
  est.setRadiusSearch(matching_settings.searching_radius_for_normal_and_curvature); // Adjust the radius as needed
  est.compute(*cloud_curvatures);

  // Detect peaks based on positive curvature values and color them blue
  for (size_t i = 0; i < cloud_curvatures->size(); ++i) 
  {
    float curvature1 = cloud_curvatures->points[i].pc1; // Use the principal curvature 1
    float curvature2 = cloud_curvatures->points[i].pc2; // Use the principal curvature 2
    /* 
    curvature1 and curvature2 indicate the maximum and minimum eigenvalues of the principal curvature.
    (curvature1 * curvature2) is called Gaussian curvature and it is >0 if the surface is dome-shaped.
    if it is <0, the surface is hyperbloid shaped. if =0, it is cylinder shaped.
    the normal z value shows in the direction of the center of a curvature's radius, so in order to find
    the convex shaped peaks and avoid concave areas, we require points whose normal z value is positive.
    */
    if ((curvature1 * curvature2) > 0.0005 
        && curvature1 > 0
        && curvature2 > 0 
        && input_cloud.points[i].z > matching_settings.delete_lower_targets_threshold)
    { // Positive curvature indicates a peak
      pcl::PointXYZRGB blue_peak;
      blue_peak.x = input_cloud.points[i].x;
      blue_peak.y = input_cloud.points[i].y;
      blue_peak.z = input_cloud.points[i].z;
      blue_peak.r = 0; // Red component
      blue_peak.g = 0; // Green component
      blue_peak.b = 255; // Blue component
      peak_visualization_cloud.push_back(blue_peak);
    }
  }
  // Publish to ROS
  pcl::toROSMsg(peak_visualization_cloud, cloud_msg);
  cloud_msg.header.frame_id="regression_plane_frame";
  cloud_msg.header.stamp = this->now();
}

// ****** VOXEL MATCHING ******* //

std::vector<std::vector<int>> detect_graspable_points::voxel_matching(std::vector<std::vector<std::vector<int>>>& terrain_matrix, const std::vector<std::vector<std::vector<int>>>& gripper_mask, const MatchingSettings& matching_settings)
{
  // ****** preparations ******* //
  // Copy inputted terrain data
  std::vector<std::vector<std::vector<int>>> searching_voxel_array = terrain_matrix;

  // size_of_voxel_array is a 3 element std::vector filled with the sizes of terrain_matrix
  std::vector<int> size_of_voxel_array = {static_cast<int>(terrain_matrix.size()), static_cast<int>(terrain_matrix[0].size()), static_cast<int>(terrain_matrix[0][0].size())};
  std::vector<int> size_of_gripper_mask = {static_cast<int>(gripper_mask.size()), static_cast<int>(gripper_mask[0].size()), static_cast<int>(gripper_mask[0][0].size())};
  std::vector<int> half_size_of_gripper_mask = {static_cast<int>(size_of_gripper_mask[0] / 2), static_cast<int>(size_of_gripper_mask[1] / 2), static_cast<int>(size_of_gripper_mask[2] / 2)};

  // Save z subscript of solid voxels. first, find indices and values of nonzero elements in the terrain_matrix. then,
  // convert linear indices to subscripts
  int z_max = 0;
  int number_of_solid_after_voxelization=0;
  for (int i = 0; i < size_of_voxel_array[0]; ++i) {
    for (int j = 0; j < size_of_voxel_array[1]; ++j) {
      for (int k = 0; k < size_of_voxel_array[2]; ++k) {
        if (terrain_matrix[i][j][k] != 0) {
          number_of_solid_after_voxelization++;
          if (k > z_max) {
            z_max = k;
          }
        }
      }
    }
  }
  MY_PRINT(number_of_solid_after_voxelization);

  // Now, insert empty voxel layers in z-direction bottom, minus the auxiliary gripper mask layers
  std::vector<int> placeholderZeros(size_of_gripper_mask[2]-matching_settings.extra_sheet, 0);

  for (int i = 0; i < size_of_voxel_array[0]; ++i) {
    for (int j = 0; j < size_of_voxel_array[1]; ++j) {
      terrain_matrix[i][j].insert(terrain_matrix[i][j].begin(), placeholderZeros.begin(), placeholderZeros.end());
    }
  }

  // Crop edges of the searching voxel array
  searching_voxel_array = vox_clip(half_size_of_gripper_mask[0]+1, half_size_of_gripper_mask[1]+1, searching_voxel_array);

  // Find all "ones" (solid voxels) in the search voxel array and change indexes to subscripts of solid voxels
  std::vector<int> size_of_searching_voxel_array = {static_cast<int>(searching_voxel_array.size()), static_cast<int>(searching_voxel_array[0].size()), static_cast<int>(searching_voxel_array[0][0].size())};
  Subscripts subscripts_of_searching_solid_voxels;

  for (int i = 0; i < size_of_searching_voxel_array[0]; ++i) {
    for (int j = 0; j < size_of_searching_voxel_array[1]; ++j) {
      for (int k = 0; k < size_of_searching_voxel_array[2]; ++k) {
        if (searching_voxel_array[i][j][k] != 0) {
          subscripts_of_searching_solid_voxels.x.push_back(i);
          subscripts_of_searching_solid_voxels.y.push_back(j);
          subscripts_of_searching_solid_voxels.z.push_back(k);
        }
      }
    }
  }
  // Correct the positions of searching voxels
  subscripts_of_searching_solid_voxels.x = subtractInteger(subscripts_of_searching_solid_voxels.x, half_size_of_gripper_mask[0]);
  subscripts_of_searching_solid_voxels.y = subtractInteger(subscripts_of_searching_solid_voxels.y, half_size_of_gripper_mask[1]);
  int number_of_solid_voxels_in_searching_voxel_array = subscripts_of_searching_solid_voxels.x.size();

  // Prepare for loop
  std::vector<std::vector<int>> searching_solid_voxels_map(4, std::vector<int>(number_of_solid_voxels_in_searching_voxel_array));
  #if  DEBUG_MOD
  std::cout <<"Size of number_of_solid_voxels_in_searching_voxel_array:"<< number_of_solid_voxels_in_searching_voxel_array << std::endl;
  #endif
    
  // Great loop
  for (int index_of_voxel_being_compared = 0; index_of_voxel_being_compared < number_of_solid_voxels_in_searching_voxel_array; ++index_of_voxel_being_compared) 
  {
    std::vector<std::vector<std::vector<int>>> subset_of_voxel_array;
    // Extract subset in the same size of the gripper mask from the data voxel array
    subset_of_voxel_array = vox_extract(terrain_matrix,
                                        {subscripts_of_searching_solid_voxels.x[index_of_voxel_being_compared],
                                        subscripts_of_searching_solid_voxels.y[index_of_voxel_being_compared],
                                        subscripts_of_searching_solid_voxels.z[index_of_voxel_being_compared]},
                                        size_of_gripper_mask);

    // Initialize and count the number of voxels inside subset
    int number_of_matching_voxels = 0;
    for (const auto& row : subset_of_voxel_array) {
      for (const auto& col : row) 
      {
        number_of_matching_voxels += std::accumulate(col.begin(), col.end(), 0);
      }
    }
    // Compare the two arrays subset and gripper mask and check whether they match or not
    // returns the graspability score, which is an indicator for the suitability for grasping at this point
    float graspability = vox_evaluate(number_of_matching_voxels, subset_of_voxel_array, gripper_mask);

    // Fill in the point into the output data set
    searching_solid_voxels_map[0][index_of_voxel_being_compared] = subscripts_of_searching_solid_voxels.x[index_of_voxel_being_compared] + half_size_of_gripper_mask[0];
    searching_solid_voxels_map[1][index_of_voxel_being_compared] = subscripts_of_searching_solid_voxels.y[index_of_voxel_being_compared] + half_size_of_gripper_mask[1];
    searching_solid_voxels_map[2][index_of_voxel_being_compared] = subscripts_of_searching_solid_voxels.z[index_of_voxel_being_compared];      
    // The forth column of the output data set is the graspability score. It is reduced by a penalty ratio
    // if the number of solid terrain voxels inside the subset is below a thershold (Threshold of Solid Voxels, TSV)
    if (number_of_matching_voxels > matching_settings.threshold) {
      searching_solid_voxels_map[3][index_of_voxel_being_compared] = graspability;
    }
    // We only penalize those points which has a erroneous high graspability score
    else if (number_of_matching_voxels <= matching_settings.threshold && graspability >= 60) {
      searching_solid_voxels_map[3][index_of_voxel_being_compared] = graspability - (((matching_settings.threshold - number_of_matching_voxels)*1.0)/(matching_settings.threshold*1.0))*100;
    }
    else {
      searching_solid_voxels_map[3][index_of_voxel_being_compared] = graspability;
    }
  }  
  // End of the great loop
  std::vector<std::vector<int>> voxel_coordinates_of_graspable_points;
  // Crop the remaining 0 column
  searching_solid_voxels_map.erase(std::remove_if(searching_solid_voxels_map.begin(), searching_solid_voxels_map.end(),
                                  [](const std::vector<int>& matching) { return std::all_of(matching.begin(), matching.end(), [](int value) { return value == 0; }); }),
                                  searching_solid_voxels_map.end());
  //Correct the position of the voxel array of the terrain matrix
  //because we make the voxel array in the reverse direction in pcd_voxelize.m
  int max_z_position = z_max;
  
  for (auto& matching : searching_solid_voxels_map) 
  {
    matching[2] = -matching[2] + max_z_position;
  }
  voxel_coordinates_of_graspable_points = searching_solid_voxels_map;
  return voxel_coordinates_of_graspable_points;
}


// ****** RE-TRANSFORMATION ******* //

std::vector<std::vector<float>> detect_graspable_points::pcd_re_transform(std::vector<std::vector<int>> voxel_coordinates_of_graspable_points, float voxel_size, std::vector<float> offset_vector) 
{
  //pcd_re_transform returns the coordinates of the voxel array to the original input coordinate system.

  // Initialize output array.
  std::vector<std::vector<float>> graspable_points(0, std::vector<float>(0,0));
  if (voxel_coordinates_of_graspable_points.empty()) 
  {
    return graspable_points;
  }
  // Resize output array.
  graspable_points.resize(4, std::vector<float>(voxel_coordinates_of_graspable_points[0].size(), 0.0f));

  // Re-transformation using the parameters in the voxelization step
  for (long unsigned int i = 0; i < voxel_coordinates_of_graspable_points[0].size(); ++i) 
  {
    graspable_points[0][i] = (voxel_coordinates_of_graspable_points[0][i]*1.0 - voxel_size/4) * voxel_size + offset_vector[0];
    graspable_points[1][i] = (voxel_coordinates_of_graspable_points[1][i]*1.0 - voxel_size/4) * voxel_size + offset_vector[1];
    graspable_points[2][i] = (voxel_coordinates_of_graspable_points[2][i]*1.0 - voxel_size/4) * voxel_size + offset_vector[2];
    graspable_points[3][i] = voxel_coordinates_of_graspable_points[3][i];
  }
  return graspable_points;
}

// ****** VISUALIZATION (COLOR GRADIENT) ******* //

sensor_msgs::msg::PointCloud2 detect_graspable_points::visualizeRainbow(std::vector<std::vector<float>> array, const MatchingSettings& matching_settings) 
{
  // Create a marker message
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  for (long unsigned int n=0; n<array[0].size(); n++) 
  {
    pcl::PointXYZRGB point;
    point.x = array[0][n];
    point.y = array[1][n];
    point.z = array[2][n];
    // Convert the fourth value (0-100) to an RGB color
    point.b = 0; // Set blue to 0
    int color_value = static_cast<int>(array[3][n]);
    //5 different color from bright green to red for the scale of graspability
    if(color_value >= 90)
    {
      point.r = 8;
      point.g = 144;
    }
    else if(color_value >= 80 && color_value < 90) 
    {
      point.r = 99;
      point.g = 255;
    }
    else if(color_value >= 70 && color_value < 80) 
    {
      point.r = 214;
      point.g = 255;
    }
    else if(color_value >= 60 && color_value < 70) 
    {
      point.r = 255;
      point.g = 255;
    }
    else if(color_value >= 50 && color_value < 60) {
      point.r = 255;
      point.g = 193;
    }
    else if(color_value >= 40 && color_value < 50) 
    {
      point.r = 255;
      point.g = 154;
    }
    // Non-graspable is marked as red
    else 
    {
      point.r = 255;
      point.g = 0;
      //grade_5 = grade_5 + 1;
    }
    // if lower threshold is set, targets below will be set to white
    if(matching_settings.delete_lower_targets == "on" && point.z < matching_settings.delete_lower_targets_threshold) 
    {
      point.r = 255;
      point.g = 255;
      point.b = 255;
    }
    pcl_cloud.push_back(point);
  }
  // Create ROS Message for publishing
  pcl::toROSMsg(pcl_cloud, cloud_msg);
  cloud_msg.header.frame_id="regression_plane_frame";
  cloud_msg.header.stamp = this->now();
  return cloud_msg;
}

// ****** VISUALIZATION (INTERSECTION CONVEXITY & GRASPABILITY) ******* //

sensor_msgs::msg::PointCloud2 detect_graspable_points::combinedAnalysis(const std::vector<std::vector<float>> array, const pcl::PointCloud<pcl::PointXYZRGB> cloud2,
                                                                        float distance_threshold,
                                                                        const MatchingSettings& matching_settings)
{
  // this function combines the results of the curvature analysis (peaks) and the graspable points and returns a single point cloud
  // with points which belong to both categories
  // Initialize the variables
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::PointCloud<pcl::PointXYZRGB> cloud1;
  pcl::PointCloud<pcl::PointXYZRGB> close_points;
  // Take only those points with a higher graspability score of a certain value and if they are above the z-threshold
  // we give them blue color
  for (long unsigned int n=0; n<array[0].size(); n++) 
  {
    pcl::PointXYZRGB point;
    if (array[3][n] >= matching_settings.graspability_threshold && array[2][n] > matching_settings.delete_lower_targets_threshold)
    {
      point.x = array[0][n];
      point.y = array[1][n];
      point.z = array[2][n];
      point.r = 128; // Set blue to 0
      point.g = 0; // Set blue to 0
      point.b = 128; // Set blue to 0
      cloud1.push_back(point);
    }
  }
  // return the intersection of high graspability score points and convex peaks.
  // if both points are closer than a distance_threshold, we take it.
  for (const pcl::PointXYZRGB point1 : cloud1.points) 
  {
    for (const pcl::PointXYZRGB point2 : cloud2.points) 
    {
      float distance = pcl::euclideanDistance(point1, point2);
      if (distance < distance_threshold) 
      {
      close_points.push_back(point1);
      close_points.push_back(point2);
      }
    }
  }
  // Publish to ROS
  #if DEBUG_MOD
  std::cout << "published " << close_points.size () << " data points to ROS" << std::endl;
  #endif
  pcl::toROSMsg(close_points, cloud_msg);
  cloud_msg.header.frame_id="regression_plane_frame";
  cloud_msg.header.stamp = this->now();
  return cloud_msg;
}
