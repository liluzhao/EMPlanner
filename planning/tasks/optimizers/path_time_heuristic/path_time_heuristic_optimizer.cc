/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @file path_time_heuristic_optimizer.cc
 **/

#include "modules/planning/tasks/optimizers/path_time_heuristic/path_time_heuristic_optimizer.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/gridded_path_time_graph.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
//taskconfig 是任务参数配置
PathTimeHeuristicOptimizer::PathTimeHeuristicOptimizer(const TaskConfig& config)
    : SpeedOptimizer(config) {
  //检查配置对象中有没有速度启发器的相关配置参数
  ACHECK(config.has_speed_heuristic_optimizer_config());
  //赋值  速度启发式优化器的参数
  speed_heuristic_optimizer_config_ = config.speed_heuristic_optimizer_config();
}
// speed_data 是一个ST点的vector数组 {(s0 t0 v0 a0 da0) (s1 t1 v1 a1 da1) ... }
//  speed_data是一个空指针 存放DP的搜索结果
/*

*/
bool PathTimeHeuristicOptimizer::SearchPathTimeGraph(//搜索路径时间图
    SpeedData* speed_data) const {
  //从reference_line_info上获得DP速度规划的配置参数 变道和非变道参数不同
  const auto& dp_st_speed_optimizer_config =
      reference_line_info_->IsChangeLanePath()
          ? speed_heuristic_optimizer_config_.lane_change_speed_config()
          : speed_heuristic_optimizer_config_.default_speed_config();//dp算法中变道和车道保持的参数配置不同。
  /*
    定义了网格化的st_graph
    reference_line_info_中ST图的数据
    DP的配置参数
    reference_line_info_路径决策的障碍物列表
    init_point 规划的起点
  */
  GriddedPathTimeGraph st_graph(
      reference_line_info_->st_graph_data(), dp_st_speed_optimizer_config,
      reference_line_info_->path_decision()->obstacles().Items(), init_point_);//dp参数初始化

  if (!st_graph.Search(speed_data).ok()) {
    AERROR << "failed to search graph with dynamic programming.";
    return false;
  }
  return true;
}

Status PathTimeHeuristicOptimizer::Process(
    const PathData& path_data, const common::TrajectoryPoint& init_point,
    SpeedData* const speed_data) {
  init_point_ = init_point;

  if (path_data.discretized_path().empty()) {
    const std::string msg = "Empty path data";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!SearchPathTimeGraph(speed_data)) {
    const std::string msg = absl::StrCat(
        Name(), ": Failed to search graph with dynamic programming.");
    AERROR << msg;
    RecordDebugInfo(*speed_data, reference_line_info_->mutable_st_graph_data()
                                     ->mutable_st_graph_debug());
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  RecordDebugInfo(
      *speed_data,
      reference_line_info_->mutable_st_graph_data()->mutable_st_graph_debug());
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
