// Copyright (c) 2024 Space Robotics Lab -- Tohoku University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file graspable_points_detection.hpp
 * @author Naomasa Takada, Jitao Zheng
 * @brief Describes the header of graspable target detection algorithm.
 */

#ifndef GRASPABLE_POINTS_DETECTION__GRASPABLE_POINTS_DETECTION_HPP_
#define GRASPABLE_POINTS_DETECTION__GRASPABLE_POINTS_DETECTION_HPP_

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "graspable_points_detection/gripper_params.hpp"
#include "graspable_points_detection/matching_params.hpp"
#include "graspable_points_detection/visibility_control.hpp"
#include <rclcpp/rclcpp.hpp>

namespace graspable_points_detection
{

class GraspablePointsDetection : public rclcpp::Node
{
public:
  GRASPABLE_POINTS_DETECTION_PUBLIC
  explicit GraspablePointsDetection(const rclcpp::NodeOptions & options);

  GRASPABLE_POINTS_DETECTION_PUBLIC
  virtual ~GraspablePointsDetection();

private:
  // Publishers

  // Subscriber
};

}  // namespace graspable_points_detection

#endif  // GRASPABLE_POINTS_DETECTION__GRASPABLE_POINTS_DETECTION_HPP_
