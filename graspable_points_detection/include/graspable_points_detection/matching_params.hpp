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

namespace graspable_points_detection
{

constexpr float kVoxelSize = 0.002f;  // [m]

// threshold of numbers of solid voxels within the subset (TSV)
constexpr int kThreshold = 100;

// choose wether or not you want to add more points during interpolation process in case of sparse map
constexpr bool kArtificiallyAddPoints = false;

// Lower threshold of targets [m]
constexpr float kDeleteLowerTargetsThreshold = 0.015f;

// size of extra sheet above the top layer of gripper mask (H_add)
constexpr int kExtraSheet = 3;

// Graspability threshold. Above which we can call it graspable with great certainty [%]
constexpr int kGraspabilityThreshold = 90;

}  // namespace graspable_points_detection
