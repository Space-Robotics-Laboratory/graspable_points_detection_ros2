#!/bin/bash

# Reformat by .clang-format
find ./src/graspable_points_detection_ros2 -path -o -regex '.*\.\(cpp\|hpp\)'\
  -exec clang-format -style=file:./src/graspable_points_detection_ros2/.clang-format/.clang-format -i {} \;

# Reformat by ament_uncrustify
find ./src/graspable_points_detection_ros2 -path -o -regex '.*\.\(cpp\|hpp\)'\
  -exec ament_uncrustify --reformat {} +
