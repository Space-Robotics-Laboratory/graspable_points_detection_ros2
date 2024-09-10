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

    auto voxelized_pcl = pcd_voxelize(interpolated_pcl);

    #if DEBUG_MOD
    auto stop_voxel = std::chrono::high_resolution_clock::now();
    auto duration_voxel = std::chrono::duration_cast<std::chrono::microseconds>(stop_voxel - start_voxel);
    std::cout <<"Time for pcd_voxelize in µs : " << duration_voxel.count() << std::endl;
    #endif

    // ****** Voxel matching ******* //
    // create an empty 2d array ready to use. it will be 4 columns: x, y, z and graspability score
    #if DEBUG_MOD
    auto start_matching = std::chrono::high_resolution_clock::now();
    #endif

    auto evaluated_points = voxel_matching(voxelized_pcl);

    #if DEBUG_MOD
    std::cout<<"Size of graspable after voxel_matching"<<evaluated_points.size() <<std::endl;
    auto stop_matching = std::chrono::high_resolution_clock::now();
    auto duration_matching = std::chrono::duration_cast<std::chrono::microseconds>(stop_matching - start_matching);
    std::cout <<"Time for voxel_matching in µs : " << duration_matching.count() << std::endl;
    #endif

    // ****** Visualization ******* //
    // ****** Color Gradient ******* //
    // Turn the graspabiliy score to color gradient. Publish the pointcloud with respect to regression_plane_frame
    sensor_msgs::msg::PointCloud2 marker_of_graspable_points;
    marker_of_graspable_points = visualizeRainbow(evaluated_points);
    graspability_map_pub_->publish(marker_of_graspable_points);
    sensor_msgs::msg::PointCloud2 msg_evaluated_points;
    msg_evaluated_points = to_msg_format(evaluated_points);
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
    clusters_pub_->publish(msg_clusters_centroid);

    #if DEBUG_MOD
    auto stop_cluster = std::chrono::high_resolution_clock::now();
    auto duration_cluster = std::chrono::duration_cast<std::chrono::microseconds>(stop_cluster - start_cluster);
    std::cout <<"Time for clusterizing in µs : " << duration_cluster.count() << std::endl;
    #endif

    // ****** Overall time consumption ******* //
    auto stop_overall = std::chrono::high_resolution_clock::now();
    auto duration_overall = std::chrono::duration_cast<std::chrono::microseconds>(stop_overall - start_overall);
    std::cout <<"Total time in ms : " << duration_overall.count()/1000 << std::endl;
    std::cout <<"************************  END  ************************" << std::endl;
  }
}


// ****** SECONDARY FUNCTIONS ******* //

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
  #if FINNER_INTERPOLATION
  while (grid_size> VOXEL_SIZE*2){
    grid_size=grid_size/1.05; //not change it too fast to stay close to the target value, if changed manually like grid_size= VOXEL_SIZE*2 causes segfault in voxelization
  }
  #endif
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
pcl::PointCloud<pcl::PointXYZ> detect_graspable_points::pcd_voxelize(const pcl::PointCloud<pcl::PointXYZ>  &input_pcd){
  
  //voxel grid creates voxels of said size and then only keeps the centroid of voxels that have points in them
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
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
  
  return (pcl_after_filtering);
}

// ****** VOXEL EXTRACT ******* //
pcl::PointCloud<pcl::PointXYZ> detect_graspable_points::vox_extract(const pcl::PointCloud<pcl::PointXYZ>& input_pcl,const pcl::PointXYZ& position_reference_point) 
{
  Eigen::Vector4f min_point, max_point;
  pcl::PointCloud<pcl::PointXYZ> output_pcl;
  pcl::CropBox<pcl::PointXYZ> crop_box_filter;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(input_pcl));

  min_point[0] = position_reference_point.x - GRIPPER_MASK_HALF_SIZE*0.001;
  max_point[0] = position_reference_point.x + GRIPPER_MASK_HALF_SIZE*0.001;
  min_point[1] = position_reference_point.y - GRIPPER_MASK_HALF_SIZE*0.001;
  max_point[1] = position_reference_point.y + GRIPPER_MASK_HALF_SIZE*0.001;
  min_point[2] = position_reference_point.z - GRIPPER_MASK_HEIGHT*0.001;
  max_point[2] = position_reference_point.z;

  crop_box_filter.setMin(min_point);
  crop_box_filter.setMax(max_point);
  crop_box_filter.setInputCloud(cloud_ptr);
  crop_box_filter.filter(output_pcl);
  return (output_pcl);
}

// ****** CHECK GRASPABILITY ******* //
float detect_graspable_points::checkGraspability(const pcl::PointCloud<pcl::PointXYZ> &input_pcd,const pcl::PointXYZ &centroid){
  int counter_of_valid_points = 0;
  float grippable_radius = 0;
  float unreachble_radius = 0;
  float distance_from_center_of_layer;
  float z_subscript = 0;
  for (auto &point:input_pcd.points)
  {
    z_subscript = (centroid.z - point.z)*1000;
    grippable_radius =  (GRIPPER_MASK_TOP_SOLID_RADIUS) + (GRIPPER_MASK_HALF_SIZE - GRIPPER_MASK_TOP_SOLID_RADIUS) * z_subscript/ (GRIPPER_MASK_CLEARANCE);
    grippable_radius = grippable_radius / 1000;
    unreachble_radius = (GRIPPER_MASK_HALF_SIZE) - std::round((GRIPPER_MASK_HALF_SIZE) - (OPENING_SPINE_RADIUS + SPINE_DEPTH)) * z_subscript / (OPENING_SPINE_DEPTH);
    unreachble_radius = unreachble_radius/1000;
    distance_from_center_of_layer = pcl::euclideanDistance(point,pcl::PointXYZ(centroid.x,centroid.y,point.z));
    if((z_subscript>GRIPPER_MASK_HEIGHT)){
    }
    if ((z_subscript<=GRIPPER_MASK_HEIGHT)&& (distance_from_center_of_layer<=GRIPPER_MASK_HALF_SIZE*0.001) &&
        ((((z_subscript) <= GRIPPER_MASK_CLEARANCE) && (distance_from_center_of_layer < grippable_radius)) ||
        (( (z_subscript) <= GRIPPER_MASK_CLEARANCE) && (distance_from_center_of_layer > unreachble_radius*2 )) || //added a times 2 to prevent having walls in the gripper shape
        (( (z_subscript) >  GRIPPER_MASK_CLEARANCE) && (z_subscript != GRIPPER_MASK_HEIGHT) && (distance_from_center_of_layer > GRIPPER_MASK_BOTTOM_VOID_RADIUS*0.001)) || //added conditio to ave a bigger hole at the bottom of gripper
        (( (z_subscript) < std::round(GRIPPER_MASK_CLEARANCE*1.75)) && (TO_FLOAT(z_subscript) > GRIPPER_MASK_CLEARANCE)) ||
        (( (z_subscript) == GRIPPER_MASK_HEIGHT) && (distance_from_center_of_layer > GRIPPER_MASK_BOTTOM_VOID_RADIUS*0.001))))
    {
      counter_of_valid_points++;
    }
  }
  float graspability = (static_cast<float>(counter_of_valid_points) / input_pcd.size()) * 100.0;
  return graspability;
}

// ****** DELETELOWPOINTS ******* //
pcl::PointCloud<pcl::PointXYZ> detect_graspable_points::deleteLowPoints(const pcl::PointCloud<pcl::PointXYZ> &input_pcl)
{
  pcl::PointCloud<pcl::PointXYZ> output;
  float z_threshold = deviationFromPlane(input_pcl);
  global_z_thrshold=z_threshold;
  for (auto &point:input_pcl.points)
  {
    if (point.z > z_threshold)
    {
      output.push_back(point);
    }
  }
  return (output);
}

// ****** DEVIATIONFROMPLANE ******* //
float detect_graspable_points::deviationFromPlane(const pcl::PointCloud<pcl::PointXYZ> &input_cloud)const
{
  float sum_of_z_squared=0;
  for (auto &points:input_cloud.points)
  {
    sum_of_z_squared+=points.z*points.z;
  }
  float standard_deviation=std::sqrt(((sum_of_z_squared)/input_cloud.points.size()));
  return(standard_deviation);
}

// ****** VOXEL MATCHING ******* //
pcl::PointCloud<pcl::PointXYZI> detect_graspable_points::voxel_matching(pcl::PointCloud<pcl::PointXYZ>& terrain_matrix)
{
  pcl::PointCloud<pcl::PointXYZI> output;
  float max_z=global_z_thrshold;
  auto searching_voxel_array = deleteLowPoints(terrain_matrix);
  for (int i = 0; i<searching_voxel_array.size()-1; i++) 
  {
    pcl::PointXYZ point_center(searching_voxel_array.points[i].x,searching_voxel_array.points[i].y,searching_voxel_array.points[i].z);
    auto subset_of_pcl = vox_extract(terrain_matrix,point_center);
    int number_of_matching_voxels = subset_of_pcl.size();
    float graspability = checkGraspability(subset_of_pcl,point_center);
    if (number_of_matching_voxels > THRESHOLD) 
    {
      output.push_back(pcl::PointXYZI(point_center.x,point_center.y,point_center.z,graspability));
    }
    // We only penalize those points which has a erroneous high graspability score
    else if (number_of_matching_voxels <= THRESHOLD && graspability >= 60) 
    {
      graspability = graspability - (((THRESHOLD - number_of_matching_voxels)*1.0)/(THRESHOLD*1.0))*100;
      output.push_back(pcl::PointXYZI(point_center.x,point_center.y,point_center.z,graspability));
    }
    else 
    {
      output.push_back(pcl::PointXYZI(point_center.x,point_center.y,point_center.z,graspability));
    }
  }  
  return output;
}

// ****** CLUSTERIZE ******* //
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
sensor_msgs::msg::PointCloud2 detect_graspable_points::visualizeRainbow(pcl::PointCloud<pcl::PointXYZI> &input_pcl) 
{
  // Create a marker message
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  for (auto &point_loop:input_pcl) 
  {
    pcl::PointXYZRGB point;
    point.x = point_loop.x;
    point.z = point_loop.z;
    point.y = point_loop.y;
    // Convert the fourth value (0-100) to an RGB color
    point.b = 0; // Set blue to 0
    float color_value = point_loop.intensity;
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
    pcl_cloud.push_back(point);
  }
  // Create ROS Message for publishing
  pcl::toROSMsg(pcl_cloud, cloud_msg);
  cloud_msg.header.frame_id="regression_plane_frame";
  cloud_msg.header.stamp = this->now();
  return cloud_msg;
}

// ****** VISUALIZATION  ******* //
sensor_msgs::msg::PointCloud2 detect_graspable_points::to_msg_format(const pcl::PointCloud<pcl::PointXYZI> input_pcl)
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::PointCloud<pcl::PointXYZRGB> cloud1;
  pcl::PointCloud<pcl::PointXYZRGB> close_points;
  for (auto &point_loop:input_pcl) 
  {
    pcl::PointXYZRGB point;
    if (point_loop.intensity >= GRASPABILITY_THRESHOLD && point_loop.z > global_z_thrshold)
    {
      point.x = point_loop.x;
      point.y = point_loop.y;
      point.z = point_loop.z;
      point.r = 128; // Set blue to 0
      point.g = 0; // Set blue to 0
      point.b = 128; // Set blue to 0
      cloud1.push_back(point);
    }
  }
  pcl::toROSMsg(cloud1, cloud_msg);
  cloud_msg.header.frame_id="regression_plane_frame";
  cloud_msg.header.stamp = this->now();
  return cloud_msg;
}
