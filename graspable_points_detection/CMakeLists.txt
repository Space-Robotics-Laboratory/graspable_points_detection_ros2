cmake_minimum_required(VERSION 3.8)
project(graspable_points_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
  add_compile_definitions(EIGEN_NO_DEBUG)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)
rclcpp_components_register_nodes(${PROJECT_NAME}
  "graspable_points_detection::GraspablePointsDetection"
)
ament_auto_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_main.cpp
)

install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
