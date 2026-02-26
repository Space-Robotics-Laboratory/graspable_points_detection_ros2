cmake_minimum_required(VERSION 3.8)
project(graspable_points_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
  add_compile_definitions(EIGEN_NO_DEBUG)
endif()


# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED COMPONENTS common io filters search segmentation)
find_package(Eigen3 REQUIRED)

ament_auto_add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "graspable_points_detection::GraspablePointsDetection"
)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# for testing
ament_auto_add_executable(point_cloud_publisher
  src/point_cloud_publisher.cpp
)
target_link_libraries(point_cloud_publisher ${PCL_LIBRARIES})

install(
  DIRECTORY config data launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
