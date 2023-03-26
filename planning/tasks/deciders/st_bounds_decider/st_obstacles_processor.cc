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

#include "modules/planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

#include <algorithm>
#include <unordered_set>

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/decision.pb.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;

namespace {
// ObsTEdge contains: (is_starting_t, t, s_min, s_max, obs_id).
using ObsTEdge = std::tuple<int, double, double, double, std::string>;
}  // namespace

void STObstaclesProcessor::Init(const double planning_distance,
                                const double planning_time,
                                const PathData& path_data,
                                PathDecision* const path_decision,
                                History* const history) {
  planning_time_ = planning_time;
  planning_distance_ = planning_distance;
  path_data_ = path_data;
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  adc_path_init_s_ = path_data_.discretized_path().front().s();
  path_decision_ = path_decision;
  history_ = history;

  obs_t_edges_.clear();
  obs_t_edges_idx_ = 0;

  obs_id_to_st_boundary_.clear();
  obs_id_to_decision_.clear();
  candidate_clear_zones_.clear();
  obs_id_to_alternative_st_boundary_.clear();
}

Status STObstaclesProcessor::MapObstaclesToSTBoundaries(
    PathDecision* const path_decision) {
  // Sanity checks.
  if (path_decision == nullptr) {
    const std::string msg = "path_decision is nullptr";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (planning_time_ < 0.0) {
    const std::string msg = "Negative planning time.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (planning_distance_ < 0.0) {
    const std::string msg = "Negative planning distance.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (path_data_.discretized_path().size() <= 1) {
    const std::string msg = "Number of path points is too few.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  obs_id_to_st_boundary_.clear();

  // Some preprocessing to save the adc_low_road_right segments.
  bool is_adc_low_road_right_beginning = true;
  for (const auto& path_pt_info : path_data_.path_point_decision_guide()) {
    double path_pt_s = 0.0;
    PathData::PathPointType path_pt_type;
    std::tie(path_pt_s, path_pt_type, std::ignore) = path_pt_info;
    // 如果路径点类型是前向车道外侧或者反向车道外侧，开始构造adc_low_road_right_segments_ ？？
    if (path_pt_type == PathData::PathPointType::OUT_ON_FORWARD_LANE ||
        path_pt_type == PathData::PathPointType::OUT_ON_REVERSE_LANE) {//驶出向前路
      if (is_adc_low_road_right_beginning) {
        adc_low_road_right_segments_.emplace_back(path_pt_s, path_pt_s);
        is_adc_low_road_right_beginning = false;
      } else {
        adc_low_road_right_segments_.back().second = path_pt_s;
      }
    } else if (path_pt_type == PathData::PathPointType::IN_LANE) {//在道路中
      if (!is_adc_low_road_right_beginning) {
        is_adc_low_road_right_beginning = true;
      }
    }
  }

  // Map obstacles into ST-graph.将map障碍物映射到st图中.
  // Go through every obstacle and plot them in ST-graph.
  std::unordered_set<std::string> non_ignore_obstacles;//非忽略障碍物
  std::tuple<std::string, STBoundary, Obstacle*> closest_stop_obstacle;// 最近的阻塞障碍物
  std::get<0>(closest_stop_obstacle) = "NULL";

  for (const auto* obs_item_ptr : path_decision->obstacles().Items()) {  //遍历所有的障碍物，构建障碍物的st-boundary
    // Sanity checks.
    Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());
    if (obs_ptr == nullptr) {
      const std::string msg = "Null obstacle pointer.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // Draw the obstacle's st-boundary.
    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;
    bool is_caution_obstacle = false;
    double obs_caution_end_t = 0.0;
    if (!ComputeObstacleSTBoundary(*obs_ptr, &lower_points, &upper_points, // 获得构建障碍物的ST图的STBoundary的lower_points upper_points
                                   &is_caution_obstacle, &obs_caution_end_t)) {
      // Obstacle doesn't appear on ST-Graph. 这个障碍物不需要显示在ST图上
      continue;
    }
    auto boundary = STBoundary::CreateInstanceAccurate(lower_points, upper_points);//构建障碍物的ST图的STBoundary
    boundary.set_id(obs_ptr->Id());//障碍物的ID给障碍物的STBoundary
    if (is_caution_obstacle) {
      boundary.set_obstacle_road_right_ending_t(obs_caution_end_t);
    }
    // Update the trimmed obstacle into alternative st-bound storage
    // for later uses. 静态障碍物的STboundary 进行修剪为两个点
    while (lower_points.size() > 2 &&
           lower_points.back().t() > obs_caution_end_t) {
      lower_points.pop_back();
    }
    while (upper_points.size() > 2 &&
           upper_points.back().t() > obs_caution_end_t) {
      upper_points.pop_back();
    }
    auto alternative_boundary =
        STBoundary::CreateInstanceAccurate(lower_points, upper_points);//精确后的alternative_boundary ？？
    alternative_boundary.set_id(obs_ptr->Id());
    obs_id_to_alternative_st_boundary_[obs_ptr->Id()] = alternative_boundary;//
    ADEBUG << "Obstacle " << obs_ptr->Id()
           << " has an alternative st-boundary with "
           << lower_points.size() + upper_points.size() << " points.";

    // Store all Keep-Clear zone together. 将所有Keep-Clear区域（临时停车区：路肩车道）存储在一起。
    if (obs_item_ptr->Id().find("KC") != std::string::npos) {
      candidate_clear_zones_.push_back(
          make_tuple(obs_ptr->Id(), boundary, obs_ptr));
      continue;
    }

    // Process all other obstacles than Keep-Clear zone.处理除保持清除区域外的所有其他障碍。
    if (obs_ptr->Trajectory().trajectory_point().empty()) {
      // Obstacle is static.
      if (std::get<0>(closest_stop_obstacle) == "NULL" ||
          std::get<1>(closest_stop_obstacle).bottom_left_point().s() >
              boundary.bottom_left_point().s()) {
        // If this static obstacle is closer for ADC to stop, record it.如果此静态障碍物离ADC较近，则记录该障碍物。
        closest_stop_obstacle =
            std::make_tuple(obs_ptr->Id(), boundary, obs_ptr);
      }
    } else {
      // Obstacle is dynamic.
      if (boundary.bottom_left_point().s() - adc_path_init_s_ <  //kSIgnoreThreshold 0.01 kTIgnoreThreshold 0.1
              kSIgnoreThreshold &&
          boundary.bottom_left_point().t() > kTIgnoreThreshold) {
        // Ignore obstacles that are behind. 忽略在自车后面的boundary 
        // TODO(jiacheng): don't ignore if ADC is in dangerous segments.
        continue;
      }
      obs_id_to_st_boundary_[obs_ptr->Id()] = boundary;//通过障碍物的ID找到
      obs_ptr->set_path_st_boundary(boundary); //从障碍物的boundary;DP使用；此处赋值
      non_ignore_obstacles.insert(obs_ptr->Id());//这个障碍物不可以忽略
      ADEBUG << "Adding " << obs_ptr->Id() << " into the ST-graph.";
    }
  }
  // For static obstacles, only retain the closest one (also considers
  // Keep-Clear zone here).
  // Note: We only need to check the overlapping between the closest obstacle 对于静态障碍物只考虑最近的那一个 （此处也考虑“保持畅通”区域）。
  //       and all the Keep-Clear zones. Because if there is another obstacle 我们只需要检查最近的障碍物和所有避开区域之间的重叠部分。
  //       overlapping with a Keep-Clear zone, which results in an even closer 因为如果有另一个障碍物与“保持畅通区域”重叠，
  //       stop fence, then that very Keep-Clear zone must also overlap with 就会形成更近的“阻止围栏”，那么这个“保持畅通区域”也必须与最近的障碍物重叠。(证明略)
  //       the closest obstacle. (Proof omitted here)
  if (std::get<0>(closest_stop_obstacle) != "NULL") {
    std::string closest_stop_obs_id;
    STBoundary closest_stop_obs_boundary;
    Obstacle* closest_stop_obs_ptr;
    std::tie(closest_stop_obs_id, closest_stop_obs_boundary,
             closest_stop_obs_ptr) = closest_stop_obstacle;
    ADEBUG << "Closest obstacle ID = " << closest_stop_obs_id;
    // Go through all Keep-Clear zones, and see if there is an even closer
    // stop fence due to them.
    if (!closest_stop_obs_ptr->IsVirtual()) {
      for (const auto& clear_zone : candidate_clear_zones_) {
        const auto& clear_zone_boundary = std::get<1>(clear_zone);
        if (closest_stop_obs_boundary.min_s() >= clear_zone_boundary.min_s() &&
            closest_stop_obs_boundary.min_s() <= clear_zone_boundary.max_s()) {
          std::tie(closest_stop_obs_id, closest_stop_obs_boundary,
                   closest_stop_obs_ptr) = clear_zone;
          ADEBUG << "Clear zone " << closest_stop_obs_id << " is closer.";
          break;
        }
      }
    }
    obs_id_to_st_boundary_[closest_stop_obs_id] = closest_stop_obs_boundary;
    closest_stop_obs_ptr->set_path_st_boundary(closest_stop_obs_boundary);
    non_ignore_obstacles.insert(closest_stop_obs_id);
    ADEBUG << "Adding " << closest_stop_obs_ptr->Id() << " into the ST-graph.";
    ADEBUG << "min_s = " << closest_stop_obs_boundary.min_s();
  }

  // Set IGNORE decision for those that are not in ST-graph:
  for (const auto* obs_item_ptr : path_decision->obstacles().Items()) {//设置忽略的decision
    Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());
    if (non_ignore_obstacles.count(obs_ptr->Id()) == 0) {
      ObjectDecisionType ignore_decision;
      ignore_decision.mutable_ignore();
      if (!obs_ptr->HasLongitudinalDecision()) {
        obs_ptr->AddLongitudinalDecision("st_obstacle_processor",
                                         ignore_decision);
      }
      if (!obs_ptr->HasLateralDecision()) {
        obs_ptr->AddLateralDecision("st_obstacle_processor", ignore_decision);
      }
    }
  }

  // Preprocess the obstacles for sweep-line algorithms.
  // Fetch every obstacle's beginning end ending t-edges only.就会形成更近的“阻止围栏”，那么这个“保持畅通区域”也必须与最近的障碍物重叠。(证明略)
  for (const auto& it : obs_id_to_st_boundary_) {//it  = st_boundary
    obs_t_edges_.emplace_back(true, it.second.min_t(),
                              it.second.bottom_left_point().s(),
                              it.second.upper_left_point().s(), it.first);//it.first = obs_id;it.second = st_boundary;
    obs_t_edges_.emplace_back(false, it.second.max_t(),
                              it.second.bottom_right_point().s(),
                              it.second.upper_right_point().s(), it.first);
  }
  // Sort the edges.// t不同的边，t小的在前面；t相同的边，是起始时间t的在前面
  std::sort(obs_t_edges_.begin(), obs_t_edges_.end(),
            [](const ObsTEdge& lhs, const ObsTEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  return Status::OK();
}

std::unordered_map<std::string, STBoundary>
STObstaclesProcessor::GetAllSTBoundaries() {
  return obs_id_to_st_boundary_;
}

bool STObstaclesProcessor::GetLimitingSpeedInfo(
    double t, std::pair<double, double>* const limiting_speed_info) {
  if (obs_id_to_decision_.empty()) {
    // If no obstacle, then no speed limits.
    return false;
  }

  double s_min = 0.0;
  double s_max = planning_distance_;// planning_distance_在该类Init函数被调用的时候传进来的是离散轨迹的长度，
  for (auto it : obs_id_to_decision_) {
    // 获取被遍历的第i个障碍物的id，决策，和st边界， obs_s_min/obs_s_max
    // 另外再获取障碍物ST上下边界的斜率即障碍物的速度obs_ds_lower/obs_ds_upper   
    auto obs_id = it.first;
    auto obs_decision = it.second;
    auto obs_st_boundary = obs_id_to_st_boundary_[obs_id];
    double obs_s_min = 0.0;
    double obs_s_max = 0.0;
    obs_st_boundary.GetBoundarySRange(t, &obs_s_max, &obs_s_min);
    double obs_ds_lower = 0.0;
    double obs_ds_upper = 0.0;
    obs_st_boundary.GetBoundarySlopes(t, &obs_ds_upper, &obs_ds_lower);
    // 如果对这个障碍物的决策有yield或stop时，
    // 且if障碍物占据的下界obs_s_min<=s_max就是障碍物的ST边界和自车的S可以到达的上下界存在重叠时，
    // 那么ADC最多只能到达obs_s_min，而且自车的车速不能比障碍物下边界斜率快
    // 那么将smax置为obs_s_min同时将速度限制limiting_speed_info的第二项设置为obs_ds_lower下边界斜率，就是不能比障碍物速度快
    if (obs_decision.has_yield() || obs_decision.has_stop()) {
      if (obs_s_min <= s_max) {
        s_max = obs_s_min;
        limiting_speed_info->second = obs_ds_lower;
      }
    } else if (it.second.has_overtake()) {
    // 如果对这个障碍物的决策有overtake时，且障碍物占据上界obs_s_max>=s_min也是ADC可能到达的s和障碍物ST边界有重叠，
    // 那么既然要超车，自车可以到达的smin最小为obs_s_max，同时自车的速度不能低于被超车的障碍物，
    // 将limiting_speed_info的第一项填入obs_ds_upperST上边界斜率。
      if (obs_s_max >= s_min) {
        s_min = obs_s_max;
        limiting_speed_info->first = obs_ds_upper;
      }
    }
  }
  return s_min <= s_max;
}

/* * 给定一个时间t，得到上下s边界。如果边界是根据先前的决定定义的，则只使用一个边界填充“available_s_bounds”。
   * 否则，用所有候选项填写“available_s_bound”，用相应的可能障碍决策填写“available _obs_decisions”。
   * 求得自车所有可能s边界available_s_bounds及其对应得决策列表available_obs_decisions，
   * 其实就是看自车可以挤到块空隙里去，不同得空隙对应不同得上下边界以及决策。
   * available_s_bounds，available_obs_decisions正是用来储存这些可能供自车进入得ST空隙以及相对应得可能决策
*/
bool STObstaclesProcessor::GetSBoundsFromDecisions(
    double t, std::vector<std::pair<double, double>>* const available_s_bounds,
    std::vector<std::vector<std::pair<std::string, ObjectDecisionType>>>* const
        available_obs_decisions) {
  // Sanity checks.
  available_s_bounds->clear();
  available_obs_decisions->clear();
  ADEBUG << "There are " << obs_t_edges_.size() << " t-edges.";
  std::vector<ObsTEdge> new_t_edges;
  // 这个while循环从obs_t_edges_，将输入时刻t之前的所有时刻的所有障碍物的占据的s上下界的竖线塞入new_t_edges
  // 一条竖线指的是障碍物的t时刻占据了ST图的s_obs_min,s_obs_max,这两点连起来的线
  while (obs_t_edges_idx_ < static_cast<int>(obs_t_edges_.size()) &&
         std::get<1>(obs_t_edges_[obs_t_edges_idx_]) <= t) {//进入ST图的障碍物的时间小于t才考虑做出决策 t = 0 0.1 0.2 ... 7.0
      // 如果不是开始时间且到达时间t，跳出循环
    if (std::get<0>(obs_t_edges_[obs_t_edges_idx_]) == 0 &&
        std::get<1>(obs_t_edges_[obs_t_edges_idx_]) == t) {
      break;
    }
    ADEBUG << "Seeing a new t-edge at t = "
           << std::get<1>(obs_t_edges_[obs_t_edges_idx_]);
    new_t_edges.push_back(obs_t_edges_[obs_t_edges_idx_]);//塞入new_t_edges
    ++obs_t_edges_idx_;
  }
  // 对于t之前消失的st边界，请删除它们。
  // 大致意思就是说在这个区间段内它的起始边界不在，起始边界在上一个时间区间段，对于这种障碍物不属于这个时间区间段内去管理
  // For st-boundaries that disappeared before t, remove them.
  for (const auto& obs_t_edge : new_t_edges) {
    if (std::get<0>(obs_t_edge) == 0) {
      ADEBUG << "Obstacle id: " << std::get<4>(obs_t_edge)
             << " is leaving st-graph.";
      if (obs_id_to_decision_.count(std::get<4>(obs_t_edge)) != 0) {//这个ID的障碍物已经有决策 删除
        obs_id_to_decision_.erase(std::get<4>(obs_t_edge));
      }
    }
  }
 // 对于被超越的障碍物，如果我们在其最右侧的道路结束时间之后（有余量），请将其移除。
  //  obs_id_to_decision_ ->需要做决策的障碍物ID
  // 这里是创建了一个string的vector专门用来存放要被移除的障碍物的id
  // 遍历obs_id_to_decision_障碍物id和决策映射表，如果要被overtake的障碍物其被注意时间只持续到t时刻前0.5s(kOvertakeObsCautionTime)，
  // 则将其id塞入obs_id_to_remove，是准备被从id决策映射表obs_id_to_decision_里remove的

  // For overtaken obstacles, remove them if we are after
  // their high right-of-road ending time (with a margin).
  std::vector<std::string> obs_id_to_remove;
  for (const auto& obs_id_to_decision_pair : obs_id_to_decision_) {
    auto obs_id = obs_id_to_decision_pair.first;
    auto obs_decision = obs_id_to_decision_pair.second;
    auto obs_st_boundary = obs_id_to_st_boundary_[obs_id];
    if (obs_decision.has_overtake() &&
        obs_st_boundary.min_t() <= t - kOvertakenObsCautionTime &&
        obs_st_boundary.obstacle_road_right_ending_t() <=
            t - kOvertakenObsCautionTime) {//
      obs_id_to_remove.push_back(obs_id_to_decision_pair.first);
    }
  }
  for (const auto& obs_id : obs_id_to_remove) {
    obs_id_to_decision_.erase(obs_id);
    // Change the displayed st-boundary to the alternative one:
    if (obs_id_to_alternative_st_boundary_.count(obs_id) > 0) {
      Obstacle* obs_ptr = path_decision_->Find(obs_id);
      obs_id_to_st_boundary_[obs_id] =
          obs_id_to_alternative_st_boundary_[obs_id];
      obs_id_to_st_boundary_[obs_id].SetBoundaryType(
          STBoundary::BoundaryType::OVERTAKE);
      obs_ptr->set_path_st_boundary(obs_id_to_alternative_st_boundary_[obs_id]);
    }
  }

  // Based on existing decisions, get the s-boundary.
  double s_min = 0.0;
  double s_max = planning_distance_;
   /*
    这个for循环遍历id和决策映射表，获取t时刻的障碍物上下界obs_max,obs_min,
    如果障碍物之前已经有yield或stop决策了,那么ADCt时刻的上界不能超obs_min；如果对障碍物有overtake决策，
    那么ADC在t时刻的下界不能低于obs_max
  */
  for (auto it : obs_id_to_decision_) {
    auto obs_id = it.first;
    auto obs_decision = it.second;
    auto obs_st_boundary = obs_id_to_st_boundary_[obs_id];
    double obs_s_min = 0.0;
    double obs_s_max = 0.0;
    obs_st_boundary.GetBoundarySRange(t, &obs_s_max, &obs_s_min);
    if (obs_decision.has_yield() || obs_decision.has_stop()) { //障碍物之前已经有yield或stop决策了
      s_max = std::fmin(s_max, obs_s_min);
    } else if (it.second.has_overtake()) {
      s_min = std::fmax(s_min, obs_s_max);
    }
  }
  if (s_min > s_max) {
    return false;
  }
  ADEBUG << "S-boundary based on existing decisions = (" << s_min << ", "
         << s_max << ")";

  // For newly entering st_boundaries, determine possible new-boundaries.
  // For apparent ones, make decisions directly.
  // 对于新输入的st_boundaries，确定可能的新边界。
  // 对于明显的问题，直接做出决定。
  // ambiguous_t_edges这个vector用来储存模糊的障碍物ST竖线
  std::vector<ObsTEdge> ambiguous_t_edges;
  for (auto obs_t_edge : new_t_edges) {
    ADEBUG << "For obstacle id: " << std::get<4>(obs_t_edge)
           << ", its s-range = [" << std::get<2>(obs_t_edge) << ", "
           << std::get<3>(obs_t_edge) << "]";
    // if (std::get<0>(obs_t_edge) == 1) 这个if代表这条竖线是障碍物的起始竖线，也就是障碍物ST边界的最左边，
    // 表明这个障碍物是刚刚进入ST图，之前不在ST图上那么要对其添加决策， 之前有决策的是否需要根据环境变化更改决策？？
    if (std::get<0>(obs_t_edge) == 1) {
      // 避让情况
      if (std::get<2>(obs_t_edge) >= s_max) {
        ADEBUG << "  Apparently, it should be yielded.";
        obs_id_to_decision_[std::get<4>(obs_t_edge)] =
            DetermineObstacleDecision(std::get<2>(obs_t_edge),
                                      std::get<3>(obs_t_edge), s_max);
        obs_id_to_st_boundary_[std::get<4>(obs_t_edge)].SetBoundaryType(
            STBoundary::BoundaryType::YIELD);
      } else if (std::get<3>(obs_t_edge) <= s_min) {
        // 超车情况
        ADEBUG << "Apparently, it should be overtaken.";
        obs_id_to_decision_[std::get<4>(obs_t_edge)] =
            DetermineObstacleDecision(std::get<2>(obs_t_edge),
                                      std::get<3>(obs_t_edge), s_min);
        obs_id_to_st_boundary_[std::get<4>(obs_t_edge)].SetBoundaryType(
            STBoundary::BoundaryType::OVERTAKE);
      // if (std::get<0>(obs_t_edge) == 1) 这个if代表这条竖线是障碍物的起始竖线，
      // 也就是障碍物ST边界的最左边，表明这个障碍物是刚刚进入ST图，之前不在ST图上那么要对其添加决策，
      } else {
        // 比较模糊的障碍物情况，存入ambiguous_t_edges
        ADEBUG << "  It should be further analyzed.";//应进一步分析。
        ambiguous_t_edges.push_back(obs_t_edge);
      }
    }
  }
  // For ambiguous ones, enumerate all decisions and corresponding bounds.
  // 对于不明确的，列举所有决策和相应的界限。
  // findgaps这个函数就是根据新加入ST图且未明确决策的障碍物(其他的能直接判断的都直接给yield和overtake了)，
  // 根据这些障碍物的竖线ST边界,去找到多个障碍物之间自车可以通行的间隙。
  // 关于这个函数后面有介绍
  auto s_gaps = FindSGaps(ambiguous_t_edges, s_min, s_max);
  if (s_gaps.empty()) {
    return false;
  }
  // 遍历所有的空隙，将可能的空闲先塞入可利用的s边界中available_s_bounds
  for (auto s_gap : s_gaps) {
    // 这个for循环结束后，GetSBoundsFromDecisions()函数的入参指针available_s_bounds里塞了多个可行驶的空隙的S边界，
    available_s_bounds->push_back(s_gap);
    // 又定义了个vector来储存障碍物id及其对应的决策类型，存储之前不明确行为的障碍物决策，并没有重复定义
    std::vector<std::pair<std::string, ObjectDecisionType>> obs_decisions;
    // 遍历所有的模糊的edges也就是障碍物ST边界竖线未决策过的，然后看可行驶的空隙的上下s边界的中点比较和obs_s_min, obs_s_max的关系。
    for (auto obs_t_edge : ambiguous_t_edges) {
      std::string obs_id = std::get<4>(obs_t_edge);
      double obs_s_min = std::get<2>(obs_t_edge);
      double obs_s_max = std::get<3>(obs_t_edge);
       // 如果可行驶的空隙的上下s边界的中点在obs_s_min之下就yield
      // 如果可行驶的空隙的上下s边界的中点在obs_s_max之上就overtake
      obs_decisions.emplace_back(
          obs_id,
          DetermineObstacleDecision(obs_s_min, obs_s_max,
                                    (s_gap.first + s_gap.second) / 2.0));
    }
    // GetSBoundsFromDecisions()函数的入参指针available_obs_decisions里塞了多个空隙的S边界对应的决策，
    // available_obs_decisions里的第i套可用决策对应available_s_bounds里第i个空隙
    available_obs_decisions->push_back(obs_decisions);
  }

  return true;
}

void STObstaclesProcessor::SetObstacleDecision(
    const std::string& obs_id, const ObjectDecisionType& obs_decision) {
  obs_id_to_decision_[obs_id] = obs_decision;
  ObjectStatus object_status;
  object_status.mutable_motion_type()->mutable_dynamic();
  if (obs_decision.has_yield() || obs_decision.has_stop()) {
    obs_id_to_st_boundary_[obs_id].SetBoundaryType(
        STBoundary::BoundaryType::YIELD);
    object_status.mutable_decision_type()->mutable_yield();
  } else if (obs_decision.has_overtake()) {
    obs_id_to_st_boundary_[obs_id].SetBoundaryType(
        STBoundary::BoundaryType::OVERTAKE);
    object_status.mutable_decision_type()->mutable_overtake();
  }
  history_->mutable_history_status()->SetObjectStatus(obs_id, object_status);
}

void STObstaclesProcessor::SetObstacleDecision(
    const std::vector<std::pair<std::string, ObjectDecisionType>>&
        obstacle_decisions) {
  for (auto obs_decision : obstacle_decisions) {
    SetObstacleDecision(obs_decision.first, obs_decision.second);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Private helper functions.

bool STObstaclesProcessor::ComputeObstacleSTBoundary(
    const Obstacle& obstacle, std::vector<STPoint>* const lower_points,
    std::vector<STPoint>* const upper_points, bool* const is_caution_obstacle,
    double* const obs_caution_end_t) {
  lower_points->clear();
  upper_points->clear();
  *is_caution_obstacle = false;//需要特别小心的障碍物？
  const auto& adc_path_points = path_data_.discretized_path();//横向规划的轨迹点规划的
  const auto& obs_trajectory = obstacle.Trajectory();//预测轨迹点你8s 0.1s

  if (obs_trajectory.trajectory_point().empty()) {
    // Processing a static obstacle.
    // Sanity checks.
    // 处理静态障碍物
    // 检查障碍物是否为静止障碍物
    if (!obstacle.IsStatic()) {
      AWARN << "Non-static obstacle[" << obstacle.Id()
            << "] has NO prediction trajectory."
            << obstacle.Perception().ShortDebugString();
    }
    // Get the overlapping s between ADC path and obstacle's perception box.
    // 得到ADC路径与障碍物感知框的重叠。
    // 初始化obstacle的box边界
    const Box2d& obs_box = obstacle.PerceptionBoundingBox();
    std::pair<double, double> overlapping_s;
    if (GetOverlappingS(adc_path_points, obs_box, kADCSafetyLBuffer,
                        &overlapping_s)) {
      lower_points->emplace_back(overlapping_s.first, 0.0);//静态障碍物的lower_points upper_points只有四个点
      lower_points->emplace_back(overlapping_s.first, planning_time_);//
      upper_points->emplace_back(overlapping_s.second, 0.0);
      upper_points->emplace_back(overlapping_s.second, planning_time_);//planning_time_ = 8s
    }
    *is_caution_obstacle = true;
    *obs_caution_end_t = planning_time_;
  } else {
    // 处理动态障碍物
    // 遍历所有时间步上出现的障碍物，逐一求出重叠的s-max和s-min。
    // 默认是障碍物预测轨迹上的第一个点 
    // Processing a dynamic obstacle.
    // Go through every occurrence of the obstacle at all timesteps, and
    // figure out the overlapping s-max and s-min one by one.
    bool is_obs_first_traj_pt = true;
    for (const auto& obs_traj_pt : obs_trajectory.trajectory_point()) {//预测轨迹点你8s 0.1s 80个
      // TODO(jiacheng): 目前，如果障碍物与ADC在不相交的部分重叠(很少发生)，我们将它们合并为一个。
      // 在将来，可以更详细地考虑这一点，而不是近似地考虑。
      // TODO(jiacheng): Currently, if the obstacle overlaps with ADC at
      // disjoint segments (happens very rarely), we merge them into one.
      // In the future, this could be considered in greater details rather
      // than being approximated.
      const Box2d& obs_box = obstacle.GetBoundingBox(obs_traj_pt);//轨迹点变成box
      ADEBUG << obs_box.DebugString();
      std::pair<double, double> overlapping_s;
      if (GetOverlappingS(adc_path_points, obs_box, kADCSafetyLBuffer,
                          &overlapping_s)) {
        ADEBUG << "Obstacle instance is overlapping with ADC path.";
        lower_points->emplace_back(overlapping_s.first,
                                   obs_traj_pt.relative_time());//overlapping_s是障碍物box和自车横向规划的轨迹重叠 80个
        upper_points->emplace_back(overlapping_s.second,
                                   obs_traj_pt.relative_time());
        if (is_obs_first_traj_pt) {
          if (IsSWithinADCLowRoadRightSegment(overlapping_s.first) ||
              IsSWithinADCLowRoadRightSegment(overlapping_s.second)) {
            *is_caution_obstacle = true;
          }
        }
        if ((*is_caution_obstacle)) {
          if (IsSWithinADCLowRoadRightSegment(overlapping_s.first) ||
              IsSWithinADCLowRoadRightSegment(overlapping_s.second)) {
            *obs_caution_end_t = obs_traj_pt.relative_time();
          }
        }
      }
      is_obs_first_traj_pt = false;
    }
    if (lower_points->size() == 1) {
      lower_points->emplace_back(lower_points->front().s(),
                                 lower_points->front().t() + 0.1);
      upper_points->emplace_back(upper_points->front().s(),
                                 upper_points->front().t() + 0.1);
    }
  }
  return (!lower_points->empty() && !upper_points->empty());
}

bool STObstaclesProcessor::GetOverlappingS(
    const std::vector<PathPoint>& adc_path_points,
    const Box2d& obstacle_instance, const double adc_l_buffer,
    std::pair<double, double>* const overlapping_s) {
  // Locate the possible range to search in details.
  int pt_before_idx = GetSBoundingPathPointIndex(
      adc_path_points, obstacle_instance, vehicle_param_.front_edge_to_center(),
      true, 0, static_cast<int>(adc_path_points.size()) - 2);
  ADEBUG << "The index before is " << pt_before_idx;
  int pt_after_idx = GetSBoundingPathPointIndex(
      adc_path_points, obstacle_instance, vehicle_param_.back_edge_to_center(),
      false, 0, static_cast<int>(adc_path_points.size()) - 2);
  ADEBUG << "The index after is " << pt_after_idx;
  if (pt_before_idx == static_cast<int>(adc_path_points.size()) - 2) {
    return false;
  }
  if (pt_after_idx == 0) {
    return false;
  }

  if (pt_before_idx == -1) {
    pt_before_idx = 0;
  }
  if (pt_after_idx == -1) {
    pt_after_idx = static_cast<int>(adc_path_points.size()) - 2;
  }
  if (pt_before_idx >= pt_after_idx) {
    return false;
  }

  // Detailed searching.
  bool has_overlapping = false;
  for (int i = pt_before_idx; i <= pt_after_idx; ++i) {
    ADEBUG << "At ADC path index = " << i << " :";
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {
      overlapping_s->first = adc_path_points[std::max(i - 1, 0)].s();
      has_overlapping = true;
      ADEBUG << "There is overlapping.";
      break;
    }
  }
  if (!has_overlapping) {
    return false;
  }
  for (int i = pt_after_idx; i >= pt_before_idx; --i) {
    ADEBUG << "At ADC path index = " << i << " :";
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {
      overlapping_s->second = adc_path_points[i + 1].s();
      ADEBUG << "There is overlapping.";
      break;
    }
  }
  return true;
}

int STObstaclesProcessor::GetSBoundingPathPointIndex(
    const std::vector<PathPoint>& adc_path_points,
    const Box2d& obstacle_instance, const double s_thresh, const bool is_before,
    const int start_idx, const int end_idx) {
  if (start_idx == end_idx) {
    if (IsPathPointAwayFromObstacle(adc_path_points[start_idx],
                                    adc_path_points[start_idx + 1],
                                    obstacle_instance, s_thresh, is_before)) {
      return start_idx;
    } else {
      return -1;
    }
  }

  if (is_before) {
    int mid_idx = (start_idx + end_idx - 1) / 2 + 1;
    if (IsPathPointAwayFromObstacle(adc_path_points[mid_idx],
                                    adc_path_points[mid_idx + 1],
                                    obstacle_instance, s_thresh, is_before)) {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, mid_idx, end_idx);
    } else {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, start_idx,
                                        mid_idx - 1);
    }
  } else {
    int mid_idx = (start_idx + end_idx) / 2;
    if (IsPathPointAwayFromObstacle(adc_path_points[mid_idx],
                                    adc_path_points[mid_idx + 1],
                                    obstacle_instance, s_thresh, is_before)) {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, start_idx,
                                        mid_idx);
    } else {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, mid_idx + 1,
                                        end_idx);
    }
  }
}

bool STObstaclesProcessor::IsPathPointAwayFromObstacle(
    const PathPoint& path_point, const PathPoint& direction_point,
    const Box2d& obs_box, const double s_thresh, const bool is_before) {
  Vec2d path_pt(path_point.x(), path_point.y());
  Vec2d dir_pt(direction_point.x(), direction_point.y());
  LineSegment2d path_dir_lineseg(path_pt, dir_pt);
  LineSegment2d normal_line_seg(path_pt, path_dir_lineseg.rotate(M_PI_2));

  auto corner_points = obs_box.GetAllCorners();
  for (const auto& corner_pt : corner_points) {
    Vec2d normal_line_ft_pt;
    normal_line_seg.GetPerpendicularFoot(corner_pt, &normal_line_ft_pt);
    Vec2d path_dir_unit_vec = path_dir_lineseg.unit_direction();
    Vec2d perpendicular_vec = corner_pt - normal_line_ft_pt;
    double corner_pt_s_dist = path_dir_unit_vec.InnerProd(perpendicular_vec);
    if (is_before && corner_pt_s_dist < s_thresh) {
      return false;
    }
    if (!is_before && corner_pt_s_dist > -s_thresh) {
      return false;
    }
  }
  return true;
}

bool STObstaclesProcessor::IsADCOverlappingWithObstacle(
    const PathPoint& adc_path_point, const Box2d& obs_box,
    const double l_buffer) const {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(adc_path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + adc_path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + adc_path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, adc_path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  ADEBUG << "    ADC box is: " << adc_box.DebugString();
  ADEBUG << "    Obs box is: " << obs_box.DebugString();

  // Check whether ADC bounding box overlaps with obstacle bounding box.
  return obs_box.HasOverlap(adc_box);
}

std::vector<std::pair<double, double>> STObstaclesProcessor::FindSGaps(
    const std::vector<ObsTEdge>& obstacle_t_edges, double s_min, double s_max) {
  std::vector<std::pair<double, int>> obs_s_edges;
  for (auto obs_t_edge : obstacle_t_edges) {
    obs_s_edges.emplace_back(std::get<2>(obs_t_edge), 1);//障碍物STboundary的下线
    obs_s_edges.emplace_back(std::get<3>(obs_t_edge), 0);
  }
  // obs_s_edges.emplace_back(std::numeric_limits<double>::lowest(), 1);
  obs_s_edges.emplace_back(s_min, 0);//adc在t时刻的S的范围
  obs_s_edges.emplace_back(s_max, 1);
  // obs_s_edges.emplace_back(std::numeric_limits<double>::max(), 0);
  std::sort(
      obs_s_edges.begin(), obs_s_edges.end(),
      [](const std::pair<double, int>& lhs, const std::pair<double, int>& rhs) {
        if (lhs.first != rhs.first) {
          return lhs.first < rhs.first;
        } else {
          return lhs.second > rhs.second;
        }
      });

  std::vector<std::pair<double, double>> s_gaps;
  int num_st_obs = 1;
  double prev_open_s = 0.0;
  for (auto obs_s_edge : obs_s_edges) {
    if (obs_s_edge.second == 1) {
      num_st_obs++;
      if (num_st_obs == 1) {
        s_gaps.emplace_back(prev_open_s, obs_s_edge.first);
      }
    } else {
      num_st_obs--;
      if (num_st_obs == 0) {
        prev_open_s = obs_s_edge.first;
      }
    }
  }

  return s_gaps;
}

ObjectDecisionType STObstaclesProcessor::DetermineObstacleDecision(
    const double obs_s_min, const double obs_s_max, const double s) const {
  ObjectDecisionType decision;
  if (s <= obs_s_min) {
    decision.mutable_yield()->set_distance_s(0.0);
  } else if (s >= obs_s_max) {
    decision.mutable_overtake()->set_distance_s(0.0);
  }
  return decision;
}

bool STObstaclesProcessor::IsSWithinADCLowRoadRightSegment(
    const double s) const {
  for (const auto& seg : adc_low_road_right_segments_) {
    if (s >= seg.first && s <= seg.second) {
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
