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

#define DEBUG_MOD true
#define TEST_MOD true

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
  combined_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (topic_prefix + "/graspable_points_before_cluster", 1); //graspable point
  transformed_point_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (topic_prefix + "/transformed_point", 1);
  point_visualization_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker> (topic_prefix + "/point_visualization_marker", 1);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker> (topic_prefix + "/normal_vector", 1);
  clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (topic_prefix + "/graspable_points", 1);
  
  //static broadcaster for now since the computation is quite long and the message are comming one by one not by a constant stream thus leading to sometime messge here for too long and crashing nodes
  tf_static_broadcast_= std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  //debugging publisher to know if the input map is wrong in the first place
  debugging_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (topic_prefix + "/received_map_checking", 1);

  //gripper mask creation for fast access and only create it once for all
  // ****** Gripper mask ******* //
  #if DEBUG_MOD
  auto start_mask = std::chrono::high_resolution_clock::now();
  #endif
  creategrippermask(gripper_mask_);
  #if DEBUG_MOD
  auto stop_mask = std::chrono::high_resolution_clock::now();
  auto duration_mask = std::chrono::duration_cast<std::chrono::microseconds>(stop_mask - start_mask);
  std::cout <<"Time for creategrippermask in µs : " << duration_mask.count() << std::endl;
  #endif
  std::cout<<"###-----FINISHED INITIALIZATION-----###"<<std::endl;
}

/*! ******************************
 ***        Destructor       *****
 *********************************/
detect_graspable_points::~detect_graspable_points(){}

//******** CALLBACK *******//

void detect_graspable_points::mapReceivedCallBack(const sensor_msgs::msg::PointCloud2 received_cloud_msg)
{
  camera_frame = received_cloud_msg.header.frame_id;

  // ************************** //
  std::cout <<"************************ START ************************" << std::endl;
  auto start_overall = std::chrono::high_resolution_clock::now();
  camera_frame = received_cloud_msg.header.frame_id; //get the input cloud frame id for later process

  // ****** Downsampling ******* //
  pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
  downsampling(received_cloud_msg, "downsampling_frame", downsampled_cloud);

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
    std::array<float,3> offset_vector_for_retransform;


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

    // ****** Voxelize ******* //
    #if DEBUG_MOD
    auto start_voxel = std::chrono::high_resolution_clock::now();
    #endif
    voxel_matrix = pcd_voxelize(interpolated_pcl);
    #if DEBUG_MOD
    auto stop_voxel = std::chrono::high_resolution_clock::now();
    auto duration_voxel = std::chrono::duration_cast<std::chrono::microseconds>(stop_voxel - start_voxel);
    std::cout <<"Time for pcd_voxelize in µs : " << duration_voxel.count() << std::endl;
    #endif

    //get minimum values for re-transform later
    offset_vector_for_retransform = getMinValues(interpolated_pcl);


    // ****** Voxel matching ******* //
    // create an empty 2d array ready to use. it will be 4 columns: x, y, z and graspability score
    std::vector<std::vector<int>> graspable_points(0, std::vector<int>(0,0));
    #if DEBUG_MOD
    auto start_matching = std::chrono::high_resolution_clock::now();
    #endif
    graspable_points = voxel_matching(voxel_matrix,offset_vector_for_retransform);
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
    graspable_points_after_retransform = pcd_re_transform(graspable_points, offset_vector_for_retransform);

    // ****** Visualization ******* //
    // ****** Color Gradient (Criterion I) ******* //
    // Turn the graspabiliy score to color gradient. Publish the pointcloud with respect to regression_plane_frame
    sensor_msgs::msg::PointCloud2 marker_of_graspable_points;
    marker_of_graspable_points = visualizeRainbow(graspable_points_after_retransform);
    graspability_map_pub_->publish(marker_of_graspable_points);

    // ****** Curvature Combined (Criterion II) ******* //
    sensor_msgs::msg::PointCloud2 msg_evaluated_points;
    //transforms the vector into a msg format only keeping point above threshold graspability that can be sent 
    msg_evaluated_points = to_msg_format(graspable_points_after_retransform);
    combined_pub_->publish(msg_evaluated_points);

    // ****** Clusterizing ******* //
    #if DEBUG_MOD
    auto start_cluster = std::chrono::high_resolution_clock::now();
    #endif
    pcl::PointCloud<pcl::PointXYZ> analysis_pcl;
    pcl::PointCloud<pcl::PointXYZ> clusters_centroid;
    sensor_msgs::msg::PointCloud2 msg_clusters_centroid;;
    pcl::fromROSMsg(msg_evaluated_points,analysis_pcl);
    clusters_centroid = clusterize(analysis_pcl);
    pcl::toROSMsg(clusters_centroid,msg_clusters_centroid);
    msg_clusters_centroid.header.frame_id = msg_evaluated_points.header.frame_id;
    msg_clusters_centroid.header.stamp= this->now();
    #if DEBUG_MOD
    auto stop_cluster = std::chrono::high_resolution_clock::now();
    auto duration_cluster = std::chrono::duration_cast<std::chrono::microseconds>(stop_cluster - start_cluster);
    std::cout <<"Time for clusterizing in µs : " << duration_cluster.count() << std::endl;
    #endif
    clusters_pub_->publish(msg_clusters_centroid);

    // ****** Overall time consumption ******* //
    auto stop_overall = std::chrono::high_resolution_clock::now();
    auto duration_overall = std::chrono::duration_cast<std::chrono::microseconds>(stop_overall - start_overall);
    std::cout <<"Total time in ms : " << duration_overall.count()/1000 << std::endl;
    std::cout <<"************************  END  ************************" << std::endl;
  }

}


// ****** SECONDARY FUNCTIONS ******* //

std::array<float,3> detect_graspable_points::getMinValues(const pcl::PointCloud<pcl::PointXYZ>& pointCloud)
{
  std::array<float,3> minValues{0,0,0};
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

void detect_graspable_points::save3DVectorToFile(const int (&vector3D)[gripper_mask_size][gripper_mask_size][gripper_mask_height], const std::string& filename) 
{
  std::ofstream outFile(filename);
  if (!outFile) 
  {
    std::cerr << "Failed to open file for writing: " << filename << std::endl;
    return;
  }
    for (int x=0; x<gripper_mask_size; x++) 
  {
    for (int y=0; y<gripper_mask_size; y++)
    {
      for (int z=0; z<gripper_mask_height; z++)
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

void detect_graspable_points::downsampling(const sensor_msgs::msg::PointCloud2 & cloud_msg, const std::string frame_id, pcl::PointCloud<pcl::PointXYZ> &filtered_points)
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
  sor.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
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

	if (centroid_vector_of_plane3f.dot(normal_vector_of_plane) > 0) // changes direction of std::vector to be from ground to sky if needed
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
  /*
  while (grid_size> VOXEL_SIZE*2){
    grid_size=grid_size/1.2; //not change it too fast to stay close to the target value, if changed manually like grid_size= VOXEL_SIZE*2 causes segfault in voxelization
  }
  */
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
std::vector<std::vector<std::vector<int>>> detect_graspable_points::pcd_voxelize(const pcl::PointCloud<pcl::PointXYZ>  input_pcd){
  
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
  xmatrix_size= trunc((max_x-min_x)/VOXEL_SIZE)+1;
  ymatrix_size= trunc((max_y-min_y)/VOXEL_SIZE)+1;
  zmatrix_size= trunc((max_z-min_z)/VOXEL_SIZE)+1;
  // change data type for adding it to class
  pcl::PCLPointCloud2 point_cloud;
  pcl::PCLPointCloud2::Ptr point_cloud_2format (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2(input_pcd,point_cloud);
  *point_cloud_2format=point_cloud;

  voxel_grid.setInputCloud(point_cloud_2format);
  voxel_grid.setLeafSize(VOXEL_SIZE,VOXEL_SIZE,VOXEL_SIZE);

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
    voxel_index_x= trunc((point.x-min_x)/VOXEL_SIZE + VOXEL_SIZE/4);
    voxel_index_y= trunc((point.y-min_y)/VOXEL_SIZE + VOXEL_SIZE/4);
    voxel_index_z= trunc((point.z-min_z)/VOXEL_SIZE + VOXEL_SIZE/4);
    voxelized_pcd[voxel_index_x][voxel_index_y][voxel_index_z]=1;
  }
  x.clear();
  y.clear();
  z.clear();
  return voxelized_pcd;
}

// ****** GRASPABILITY SCORE ******* //
float detect_graspable_points::vox_evaluate(const int number_of_points, const std::vector<std::vector<std::vector<int>>>& subset_of_voxel_array) {
  int number_of_proper_points = 0;
  const int x_size = subset_of_voxel_array.size();
  const int y_size = subset_of_voxel_array[0].size();
  const int z_size = subset_of_voxel_array[0][0].size();
  for (int x = 0; x < x_size; x++) {
    for (int y = 0; y < y_size; y++) {
      for (int z = 0; z < z_size; z++) {
        number_of_proper_points += subset_of_voxel_array[x][y][z] * gripper_mask_[x][y][z];
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
                                                                                std::array<int,3> size_extracting) 
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
void detect_graspable_points::creategrippermask(int (&gripper_mask)[gripper_mask_size][gripper_mask_size][gripper_mask_height])
{
  float opening_spine_depth = (std::round(OPENING_SPINE_DEPTH * ratio));
  float grippable_radius = 0;
  float unreachble_radius = 0;
  float distance_from_center_of_layer = 0;
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
  // Make the gripper_mask by setting the elements of 1
  if (opening_spine_depth ==1){opening_spine_depth =2;} //if the spinde depth is one then there is a division by 0
  for (int z_subscript = 1; z_subscript < gripper_mask_height+1; ++z_subscript)
  {
    // Calculate radius of inner cone and outer solid area
    grippable_radius =  (gripper_mask_top_solid_radius) + (gripper_mask_half_size - gripper_mask_top_solid_radius) * TO_FLOAT(z_subscript-1)/ (gripper_mask_clearance-1);
    unreachble_radius = (gripper_mask_half_size) - std::round((gripper_mask_half_size) - (opening_spine_radius + spine_depth)) * TO_FLOAT(z_subscript-1) / (opening_spine_depth-1);
    for (int y_subscript = 1; y_subscript < gripper_mask_size+1; ++y_subscript)
    {
      for (int x_subscript = 1; x_subscript < gripper_mask_size+1; ++x_subscript)
      {   // Caculate the distance from center of layer
        distance_from_center_of_layer = std::sqrt(std::pow((gripper_mask_half_size+1 - TO_FLOAT(x_subscript)), 2) + std::pow((gripper_mask_half_size+1 - TO_FLOAT(y_subscript)), 2));
        // Judges whether it is a solid(1) region or not
        if (((TO_FLOAT(z_subscript) <= gripper_mask_clearance) && (distance_from_center_of_layer < grippable_radius)) ||
            ((TO_FLOAT(z_subscript) <= gripper_mask_clearance) && (distance_from_center_of_layer > unreachble_radius*2 )) || //added a times 2 to prevent having walls in the gripper shape
            ((TO_FLOAT(z_subscript) > gripper_mask_clearance) && (z_subscript != gripper_mask_height) && (distance_from_center_of_layer > gripper_mask_bottom_void_radius)) || //added conditio to ave a bigger hole at the bottom of gripper
            ((TO_FLOAT(z_subscript) < std::round(gripper_mask_clearance*1.75)) && (TO_FLOAT(z_subscript) > gripper_mask_clearance)) ||
            ((z_subscript == gripper_mask_height) && (distance_from_center_of_layer > gripper_mask_bottom_void_radius)))
        {
          // Set the element as 1
          //(gripper_mask)[x_subscript-1][y_subscript-1][z_subscript-1] = 1;
          // Flip the gripper mask vertically
          (gripper_mask)[x_subscript-1][y_subscript-1][gripper_mask_height-z_subscript] = 1;
        }
      }
    }
  }
  //std::string filename = "/home/antonin/linked_ws/src/graspable_points_detection_ros2/pcd_data/GripperMask.csv";
  //save3DVectorToFile(gripper_mask, filename);
}

// ****** VOXEL MATCHING ******* //

std::vector<std::vector<int>> detect_graspable_points::voxel_matching(std::vector<std::vector<std::vector<int>>>& terrain_matrix,std::array<float,3> &offset_vector)
{
  // ****** preparations ******* //
  // Copy inputted terrain data
  std::vector<std::vector<std::vector<int>>> searching_voxel_array = terrain_matrix;

  // size_of_voxel_array is a 3 element std::vector filled with the sizes of terrain_matrix
  std::vector<int> size_of_voxel_array = {static_cast<int>(terrain_matrix.size()), static_cast<int>(terrain_matrix[0].size()), static_cast<int>(terrain_matrix[0][0].size())};
  std::array<int,3> size_of_gripper_mask = {gripper_mask_size, gripper_mask_size, gripper_mask_height};
  std::array<int,3> half_size_of_gripper_mask = {static_cast<int>(gripper_mask_half_size), static_cast<int>(gripper_mask_half_size), static_cast<int>(gripper_mask_height / 2)};

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
  std::vector<int> placeholderZeros(size_of_gripper_mask[2]-EXTRA_SHEET, 0);
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
  int start_k = std::trunc(std::abs(offset_vector[2])/VOXEL_SIZE + DELETE_LOWER_TARGETS_THRESHOLD/VOXEL_SIZE);
  for (int i = 0; i < size_of_searching_voxel_array[0]; ++i) {
    for (int j = 0; j < size_of_searching_voxel_array[1]; ++j) {
      for (int k = start_k; k < size_of_searching_voxel_array[2]; ++k) {
        if ( searching_voxel_array[i][j][k] != 0) {
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
    float graspability = vox_evaluate(number_of_matching_voxels, subset_of_voxel_array);

    // Fill in the point into the output data set
    searching_solid_voxels_map[0][index_of_voxel_being_compared] = subscripts_of_searching_solid_voxels.x[index_of_voxel_being_compared] + half_size_of_gripper_mask[0];
    searching_solid_voxels_map[1][index_of_voxel_being_compared] = subscripts_of_searching_solid_voxels.y[index_of_voxel_being_compared] + half_size_of_gripper_mask[1];
    searching_solid_voxels_map[2][index_of_voxel_being_compared] = subscripts_of_searching_solid_voxels.z[index_of_voxel_being_compared];      
    // The forth column of the output data set is the graspability score. It is reduced by a penalty ratio
    // if the number of solid terrain voxels inside the subset is below a thershold (Threshold of Solid Voxels, TSV)
    if (number_of_matching_voxels > THRESHOLD) {
      searching_solid_voxels_map[3][index_of_voxel_being_compared] = graspability;
    }
    // We only penalize those points which has a erroneous high graspability score
    else if (number_of_matching_voxels <= THRESHOLD && graspability >= 60) {
      searching_solid_voxels_map[3][index_of_voxel_being_compared] = graspability - (((THRESHOLD - number_of_matching_voxels)*1.0)/(THRESHOLD*1.0))*100;
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

std::vector<std::vector<float>> detect_graspable_points::pcd_re_transform(std::vector<std::vector<int>> voxel_coordinates_of_graspable_points, std::array<float,3> offset_vector) 
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
    graspable_points[0][i] = (voxel_coordinates_of_graspable_points[0][i]*1.0 - VOXEL_SIZE/4) * VOXEL_SIZE + offset_vector[0];
    graspable_points[1][i] = (voxel_coordinates_of_graspable_points[1][i]*1.0 - VOXEL_SIZE/4) * VOXEL_SIZE + offset_vector[1];
    graspable_points[2][i] = (voxel_coordinates_of_graspable_points[2][i]*1.0 - VOXEL_SIZE/4) * VOXEL_SIZE + offset_vector[2];
    graspable_points[3][i] = voxel_coordinates_of_graspable_points[3][i];
  }
  return graspable_points;
}

pcl::PointCloud<pcl::PointXYZ> detect_graspable_points::clusterize(const pcl::PointCloud<pcl::PointXYZ> &input_pcl)
{
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  std::vector<pcl::PointIndices> cluster_indices;
  ec.setInputCloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(input_pcl));
  ec.setSearchMethod(tree);
  ec.setClusterTolerance(PALM_DIAMETER*0.001); //the cluster tolerance in set as something that should not be larger than the palm size in m
  ec.setMinClusterSize(1);
  ec.setMaxClusterSize(25000);
  ec.extract(cluster_indices);

  pcl::PointCloud<pcl::PointXYZ> clusters_centroids;
  for (const auto& indices : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (int index : indices.indices) 
    {
      cluster->points.push_back(input_pcl.points[index]);
    }
    // Compute the centroid of the cluster
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    clusters_centroids.push_back(pcl::PointXYZ(centroid[0],centroid[1],centroid[2]));
    }
  #if DEBUG_MOD
  std::cout << "Clusters seen as graspable " << clusters_centroids.size () << std::endl;
  #endif
  return clusters_centroids;
}

// ****** VISUALIZATION (COLOR GRADIENT) ******* //

sensor_msgs::msg::PointCloud2 detect_graspable_points::visualizeRainbow(std::vector<std::vector<float>> array) 
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
    
    if(DELETE_LOWER_TARGETS && point.z < DELETE_LOWER_TARGETS_THRESHOLD) 
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

sensor_msgs::msg::PointCloud2 detect_graspable_points::to_msg_format(const std::vector<std::vector<float>> array)
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
    if (array[3][n] >= GRASPABILITY_THRESHOLD && array[2][n] > DELETE_LOWER_TARGETS_THRESHOLD)
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
  #if DEBUG_MOD
  std::cout << "Points seen as graspable " << cloud1.size () << std::endl;
  #endif

  pcl::toROSMsg(cloud1, cloud_msg);
  cloud_msg.header.frame_id="regression_plane_frame";
  cloud_msg.header.stamp = this->now();
  return cloud_msg;
}
