cmake_minimum_required(VERSION 3.5)
project(detect_graspable_points)

# Ensure C++17 is used
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(libInterpolate REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters)  # Adjust version as necessary

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${pcl_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Add the C++ shared library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
)

target_link_libraries(
  ${PROJECT_NAME} ${PCL_LIBRARIES} Eigen3::Eigen
)

# Specify libraries to link the shared library against
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_main
  src/${PROJECT_NAME}_main.cpp
)

# Link against libraries and dependencies
ament_target_dependencies(${PROJECT_NAME}_main
  rclcpp
  std_msgs
  tf2_ros
  pcl_conversions
  pcl_msgs
  sensor_msgs
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME}_main
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  Eigen3::Eigen
)

# Declare another C++ executable
add_executable(publish_pointcloud2
  src/publish_pointcloud2.cpp
)

# Link against libraries and dependencies
ament_target_dependencies(publish_pointcloud2
  rclcpp
  std_msgs
  tf2_ros
  pcl_conversions
  pcl_msgs
  sensor_msgs
  visualization_msgs
)

target_link_libraries(publish_pointcloud2
  ${PCL_LIBRARIES}
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_main
  publish_pointcloud2
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Generate and install package configuration
ament_package()
