// Copyright 2026 Gezp (https://github.com/gezp).
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

#include <Eigen/Core>

#include <string>
#include <vector>

#include "ssct_common/sensor_data/imu_data.hpp"
#include "ssct_imu_intrinsic_calib/utils.hpp"

namespace ssct_imu_intrinsic_calib
{

struct StaticInterval
{
  int start;
  int end;  // inclusive
};

// Static intervals detected with one variance threshold.
struct PoseCandidate
{
  int threshold_multiplier{0};
  std::vector<StaticInterval> intervals;
};

// What the user is currently doing, derived from the last static interval.
enum class PoseTailState
{
  kNoStatic,   // no static interval touches the end of the recording
  kHolding,    // static, but the pose has not been held long enough yet
  kAccepted,   // new distinct pose captured, the user should move
  kDuplicate,  // static, but the orientation repeats an accepted pose
  kInvalid,    // static, but rejected for another reason
};

// Detects the static poses used by IMU-TK and reports the hold/move prompts.
class StaticPoseCollector
{
public:
  StaticPoseCollector(
    double static_window = 0.5, int interval_samples = 100, int min_intervals = 9);

  void collect(const std::vector<ssct_common::ImuData> & samples);
  void reset();
  std::string progress() const;
  std::string hint() const;

  bool valid() const {return valid_;}
  const std::string & message() const {return message_;}
  // One candidate per threshold multiplier, used by the estimator to pick the
  // set with the smallest accelerometer residual.
  const std::vector<PoseCandidate> & candidates() const {return candidates_;}
  // Largest distinct set, used for the debug plot and pose count.
  const std::vector<StaticInterval> & display_intervals() const
  {
    return display_intervals_;
  }

  int interval_samples() const {return interval_samples_;}
  int min_intervals() const {return min_intervals_;}

private:
  double static_window_;
  int interval_samples_;
  int min_intervals_;
  bool valid_{false};
  std::string message_;
  int sample_count_{0};
  double sample_period_{0.0};
  std::vector<PoseCandidate> candidates_;
  std::vector<StaticInterval> display_intervals_;
  // Start of the static interval touching the end of the recording, or -1.
  int static_tail_start_{-1};
  PoseTailState tail_state_{PoseTailState::kNoStatic};
  // True when the latest window looks like motion rather than noise.
  bool recent_moving_{false};
  // Time since the last accepted pose ended, used by the no-static timeout.
  double since_last_pose_seconds_{0.0};
};

}  // namespace ssct_imu_intrinsic_calib
