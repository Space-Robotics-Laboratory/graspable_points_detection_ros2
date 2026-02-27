// Copyright (c) 2026 Space Robotics Lab -- Tohoku University
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

#pragma once

#include <algorithm>
#include <cmath>

namespace graspable_points_detection
{

enum class GripperModel { HUBROBO, LIMBERO, SCARE };

// Choose a gripper model
constexpr GripperModel kGripperModel = GripperModel::LIMBERO;

struct GripperParams
{
  float palm_diameter;
  float palm_diameter_of_finger_joints;
  float finger_length;
  float spine_length;
  float spine_depth;
  float opening_angle;
  float closing_angle;
  float opening_spine_radius;
  float opening_spine_depth;
  float closing_height;
  float margin_of_top_solid_diameter;
  float inside_margin_of_bottom_void_diameter;
};

constexpr GripperParams kHubroboParams = {32.0f, 28.0f, 15.0f, 15.0f, 5.0f, 75.0f,
                                          30.0f, 37.0f, 5.0f,  16.0f, 4.0f, 2.0f};

constexpr GripperParams kLimberoParams = {64.0f,  70.0f, 38.0f, 30.0f, 10.0f, 80.0f,
                                          -10.0f, 77.0f, 5.0f,  33.0f, 4.0f,  2.0f};

constexpr GripperParams kScareParams = {71.0f, 92.0f,  40.0f, 41.0f, 5.0f, 85.0f,
                                        10.0f, 136.0f, 5.0f,  90.0f, 4.0f, 2.0f};

constexpr GripperParams defineGripperParams()
{
  switch (kGripperModel) {
    case GripperModel::HUBROBO:
      return kHubroboParams;
    case GripperModel::LIMBERO:
      return kLimberoParams;
    case GripperModel::SCARE:
      return kScareParams;
    default:
      return kLimberoParams;
  }
}

constexpr GripperParams kGripperParams = defineGripperParams();

// // 4. 依存パラメータの計算 (ACTIVE_GRIPPER を使ってコンパイル時に自動計算)
// constexpr float RATIO = 1.0f / (VOXEL_SIZE * 1000.0f);

// constexpr float PALM_DIAMETER_VOXELS = std::round(ACTIVE_GRIPPER.palm_diameter * RATIO);
// constexpr float PALM_DIAMETER_JOINTS_VOXELS =
//   std::round(ACTIVE_GRIPPER.palm_diameter_of_finger_joints * RATIO);
// constexpr float FINGER_LENGTH_VOXELS = std::round(ACTIVE_GRIPPER.finger_length * RATIO);
// // ... (他の constexpr 変数も ACTIVE_GRIPPER.xxx を使って同様に記述)

// // 5. 危険な関数マクロの廃止 -> テンプレート/インライン関数へ
// // C++ではマクロではなく、型安全な関数を使います
// template <typename Container>
// inline auto get_max(const Container & c)
// {
//   return *std::max_element(c.begin(), c.end());
// }

// template <typename Container>
// inline auto get_min(const Container & c)
// {
//   return *std::max_element(
//     c.begin(), c.end());  // ※注意：元のコードの挙動を再現する場合。正しくは min_element です。
// }

// // MY_PRINT はROS 2環境であれば RCLCPP_INFO 等を使うことを推奨しますが、
// // std::coutを残す場合はインライン関数にします。
// template <typename T>
// inline void my_print(const char * name, const T & value)
// {
//   std::cout << name << "=" << value << std::endl;
// }
// #define MY_PRINT(x) my_print(#x, x)  // どうしても変数名を出力したい場合のみ、ここだけマクロを許容

}  // namespace graspable_points_detection
