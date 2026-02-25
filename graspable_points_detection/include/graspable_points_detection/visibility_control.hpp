// Copyright (c) 2025 Tohoku Univ. Space Robotics Lab.
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

#ifndef GRASPABLE_POINTS_DETECTION__VISIBILITY_CONTROL_HPP_
#define GRASPABLE_POINTS_DETECTION__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define GRASPABLE_POINTS_DETECTION_EXPORT __attribute__((dllexport))
#define GRASPABLE_POINTS_DETECTION_IMPORT __attribute__((dllimport))
#else
#define GRASPABLE_POINTS_DETECTION_EXPORT __declspec(dllexport)
#define GRASPABLE_POINTS_DETECTION_IMPORT __declspec(dllimport)
#endif
#ifdef GRASPABLE_POINTS_DETECTION_BUILDING_LIBRARY
#define GRASPABLE_POINTS_DETECTION_PUBLIC GRASPABLE_POINTS_DETECTION_EXPORT
#else
#define GRASPABLE_POINTS_DETECTION_PUBLIC GRASPABLE_POINTS_DETECTION_IMPORT
#endif
#define GRASPABLE_POINTS_DETECTION_PUBLIC_TYPE GRASPABLE_POINTS_DETECTION_PUBLIC
#define GRASPABLE_POINTS_DETECTION_LOCAL
#else
#define GRASPABLE_POINTS_DETECTION_EXPORT __attribute__((visibility("default")))
#define GRASPABLE_POINTS_DETECTION_IMPORT
#if __GNUC__ >= 4
#define GRASPABLE_POINTS_DETECTION_PUBLIC __attribute__((visibility("default")))
#define GRASPABLE_POINTS_DETECTION_LOCAL __attribute__((visibility("hidden")))
#else
#define GRASPABLE_POINTS_DETECTION_PUBLIC
#define GRASPABLE_POINTS_DETECTION_LOCAL
#endif
#define GRASPABLE_POINTS_DETECTION_PUBLIC_TYPE
#endif

#endif  // GRASPABLE_POINTS_DETECTION__VISIBILITY_CONTROL_HPP_
