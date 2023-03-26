/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <list>
#include <memory>
#include <string>

#include "modules/map/pnc_map/route_segments.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_status.pb.h"
#include "modules/planning/tasks/deciders/decider.h"
/*
lane_change_decider，我觉得应该叫车辆换道状态检测器，因为该decider通篇都在通过UpdateStatus来对injector_->planning_context()->mutable_planning_status()
->mutable_change_lane()信息进行实时更新。当然，顺便对proc传进来的current_reference_line_info通过IsClearToChangeLane()做了是否具备换道条件检测，是的话就额外的将injector_->planning_context()->mutable_planning_status()
->mutable_change_lane()->set_is_clear_to_change_lane设置为true,表面车辆此时周围有可以换道的参考线。这一信息很重要，后面的path_bound_decider会依据此标志位来确定regular_path_bound。
*/
namespace apollo {
namespace planning {
// ???  decider的主要操作是对记录车此时的换道状态，并不决定车是否换道。
class LaneChangeDecider : public Decider {

 public:
  LaneChangeDecider(const TaskConfig& config,
                    const std::shared_ptr<DependencyInjector>& injector);

  /**
   * @brief A static function to check if the ChangeLanePath type of reference
   * line is safe or if current reference line is safe to deviate away and come
   * back
   */
  static bool IsClearToChangeLane(ReferenceLineInfo* reference_line_info);

  /**
   * @brief A static function to estimate if an obstacle in certain range in
   * front of ADV blocks too much space perception behind itself by beam
   * scanning
   * @param search_beam_length is the length of scanning beam
   * @param search_beam_radius_intensity is the resolution of scanning
   * @param search_range is the scanning range centering at ADV heading
   * @param is_block_angle_threshold is the threshold to tell how big a block
   *        angle range is perception blocking
   */
  static bool IsPerceptionBlocked(const ReferenceLineInfo& reference_line_info,
                                  const double search_beam_length,
                                  const double search_beam_radius_intensity,
                                  const double search_range,
                                  const double is_block_angle_threshold);

  static void UpdatePreparationDistance(
      const bool is_opt_succeed, const Frame* frame,
      const ReferenceLineInfo* const reference_line_info,
      PlanningContext* planning_context);

 private:
  common::Status Process(
      Frame* frame,
      ReferenceLineInfo* const current_reference_line_info) override;

  static bool HysteresisFilter(const double obstacle_distance,
                               const double safe_distance,
                               const double distance_buffer,
                               const bool is_obstacle_blocking);

  void UpdateStatus(ChangeLaneStatus::Status status_code,
                    const std::string& path_id);
  void UpdateStatus(double timestamp, ChangeLaneStatus::Status status_code,
                    const std::string& path_id);

  void PrioritizeChangeLane(
      const bool is_prioritize_change_lane,
      std::list<ReferenceLineInfo>* reference_line_info) const;

  void RemoveChangeLane(
      std::list<ReferenceLineInfo>* reference_line_info) const;

  std::string GetCurrentPathId(
      const std::list<ReferenceLineInfo>& reference_line_info) const;
};

}  // namespace planning
}  // namespace apollo
