// Copyright 2025 Gezp (https://github.com/gezp).
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

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace ssct_lidar_lidar_calib
{

// Finds the closest target/source timestamp pair within max_time_diff.
inline bool find_best_time_pair(
  const std::vector<double> & target_times, const std::vector<double> & source_times,
  double max_time_diff, size_t & target_index, size_t & source_index)
{
  if (target_times.empty() || source_times.empty()) {
    return false;
  }
  double best_diff = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < target_times.size(); ++i) {
    for (size_t j = 0; j < source_times.size(); ++j) {
      const double diff = std::fabs(target_times[i] - source_times[j]);
      if (diff < best_diff) {
        best_diff = diff;
        target_index = i;
        source_index = j;
      }
    }
  }
  return best_diff <= max_time_diff;
}

}  // namespace ssct_lidar_lidar_calib
