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
 * @file
 **/

#pragma once

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {
/*
1.用自车状态，参考线信息，规划起始点，routing的结果去构造参考线信息类对象,同时也会储存这些信息到ReferenceLineInfo类对象中；
2.类成员path_decision_里存放障碍物对象列表以及对障碍物的横纵向决策信息；
3.判断是否到达终点以及距终点的距离，终点通常设置为一障碍物对象，其id为"DEST"；
4.设置及存储离散轨迹；
5.设置及存储参考线的cost？是什么cost?类成员cost_；
6.设置/返回参考线的巡航速度，停止点等；
7.获取相邻车道，左前/左后/右前/右后；
8.根据给定的s,找到参考线上给定s对应的车道；
9.判断当前参考线的起点位于上一次参考线上？
10.结合速度规划和路径规划，共同结合生成一条离散轨迹ptr_discretized_trajectory
11.将规划轨迹调整到从当前位置开始，裁剪掉轨迹上规划起始点之前的轨迹；
12.返回自车的SL边界，包含自车边界start_s,end_s,start_l,end_l信息；
13.存储及设置参考线是否可驾驶？
14.检查当前参考线是否是一个变道参考线，ADC当前位置就不在这条参考线上说明是？
15.检查当前的参考线是否是自车当前位置相邻参考线；
16.导出参与建议？engage_advice是planning发布轨迹ADCTrajectory中包含的信息，推测是判断规划是否应该介入？
17.导出决策信息到decision_result，planning_context中？这两都是待发布轨迹的成员？
18.设置/获取自车在指定纵坐标s对应路口的路权？是否有保护？
19.获取路径的转弯类型，指定纵向位置s处的转弯类型
20.获取指定纵向位置S处的PNC路口的overlap
21.设置车辆的信号灯信息
22.判断输入的障碍物是否可被忽略
23.储存阻塞参考线的障碍物id
*/
/**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */
class ReferenceLineInfo {
 public:
 //左前 左后 右前 右后 
  enum class LaneType { LeftForward, LeftReverse, RightForward, RightReverse };
  ReferenceLineInfo() = default;

  ReferenceLineInfo(const common::VehicleState& vehicle_state,
                    const common::TrajectoryPoint& adc_planning_point,
                    const ReferenceLine& reference_line,
                    const hdmap::RouteSegments& segments);

  bool Init(const std::vector<const Obstacle*>& obstacles);
  //障碍物加入在path_decision中
  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  //在path_decision中增加一个障碍物
  Obstacle* AddObstacle(const Obstacle* obstacle);
   //返回自车状态
  const common::VehicleState& vehicle_state() const { return vehicle_state_; }
  //返回成员路径决策类对象path_decision  
  PathDecision* path_decision();
  const PathDecision& path_decision() const;
  //返回类成员参考线
  const ReferenceLine& reference_line() const;
  ReferenceLine* mutable_reference_line();
  //计算离目标点纵向距离
  double SDistanceToDestination() const;
  //是否到达了目标点 距离小于5米 判断到了
  bool ReachedDestination() const;
  //设置类成员 离散轨迹类对象 discretized_trajectory
  void SetTrajectory(const DiscretizedTrajectory& trajectory);
  const DiscretizedTrajectory& trajectory() const;
  //返回cost
  double Cost() const { return cost_; }
  void AddCost(double cost) { cost_ += cost; }
  void SetCost(double cost) { cost_ = cost; }
  double PriorityCost() const { return priority_cost_; }//优先代价
  void SetPriorityCost(double cost) { priority_cost_ = cost; }
  // For lattice planner'speed planning target
  void SetLatticeStopPoint(const StopPoint& stop_point);//设置停止点
  void SetLatticeCruiseSpeed(double speed);//设置巡航速度
  const PlanningTarget& planning_target() const { return planning_target_; }

  void SetCruiseSpeed(double speed) { cruise_speed_ = speed; }
  double GetCruiseSpeed() const;
  //给定的S找到对应的LaneInfo
  hdmap::LaneInfoConstPtr LocateLaneInfo(const double s) const;
  //获得相邻车道的信息  ID lane_type
  bool GetNeighborLaneInfo(const ReferenceLineInfo::LaneType lane_type,
                           const double s, hdmap::Id* ptr_lane_id,
                           double* ptr_lane_width) const;

  /**
   * @brief check if current reference line is started from another reference
   *line info line. The method is to check if the start point of current
   *reference line is on previous reference line info.
   * @return returns true if current reference line starts on previous reference
   *line, otherwise false.
   **/
  //判断当前参考线的起点在上一个参考线的位置
  bool IsStartFrom(const ReferenceLineInfo& previous_reference_line_info) const;
  //调试
  planning_internal::Debug* mutable_debug() { return &debug_; }
  const planning_internal::Debug& debug() const { return debug_; }
  LatencyStats* mutable_latency_stats() { return &latency_stats_; }
  const LatencyStats& latency_stats() const { return latency_stats_; }

  const PathData& path_data() const;
  const PathData& fallback_path_data() const;
  //返回类 规划速度
  const SpeedData& speed_data() const;
  PathData* mutable_path_data();
  PathData* mutable_fallback_path_data();
  SpeedData* mutable_speed_data();

  const RSSInfo& rss_info() const;
  RSSInfo* mutable_rss_info();
  // aggregate final result together by some configuration 轨迹缝合
  bool CombinePathAndSpeedProfile(
      const double relative_time, const double start_s,
      DiscretizedTrajectory* discretized_trajectory);

  // adjust trajectory if it starts from cur_vehicle postion rather planning
  // init point from upstream
  bool AdjustTrajectoryWhichStartsFromCurrentPos(
      const common::TrajectoryPoint& planning_start_point,
      const std::vector<common::TrajectoryPoint>& trajectory,
      DiscretizedTrajectory* adjusted_trajectory);
  //自车SL边界 start_s start_l end_s end_L
  const SLBoundary& AdcSlBoundary() const;
  std::string PathSpeedDebugString() const;

  /**
   * Check if the current reference line is a change lane reference line, i.e.,
   * ADC's current position is not on this reference line. 自车不在该参考线 就是换道
   */
  bool IsChangeLanePath() const;

  /**
   * Check if the current reference line is the neighbor of the vehicle
   * current position 是否自车的相领车道线
   */
  bool IsNeighborLanePath() const;

  /**
   * Set if the vehicle can drive following this reference line
   * A planner need to set this value to true if the reference line is OK
   */
  void SetDrivable(bool drivable); //是否可驾驶
  bool IsDrivable() const;

  void ExportEngageAdvice(common::EngageAdvice* engage_advice,
                          PlanningContext* planning_context) const;

  const hdmap::RouteSegments& Lanes() const;//route的车道类对象
  std::list<hdmap::Id> TargetLaneId() const;

  void ExportDecision(DecisionResult* decision_result,
                      PlanningContext* planning_context) const;
  //设置路口  是否保护路口 
  void SetJunctionRightOfWay(const double junction_s,
                             const bool is_protected) const;

  ADCTrajectory::RightOfWayStatus GetRightOfWayStatus() const;
  //道路转向类型
  hdmap::Lane::LaneTurn GetPathTurnType(const double s) const;

  bool GetIntersectionRightofWayStatus(
      const hdmap::PathOverlap& pnc_junction_overlap) const;
  //偏置距离
  double OffsetToOtherReferenceLine() const {
    return offset_to_other_reference_line_;
  }
  void SetOffsetToOtherReferenceLine(const double offset) {
    offset_to_other_reference_line_ = offset;
  }
  //候选路径的边界SL
  const std::vector<PathBoundary>& GetCandidatePathBoundaries() const;
  //候选路径
  void SetCandidatePathBoundaries(
      std::vector<PathBoundary>&& candidate_path_boundaries);

  const std::vector<PathData>& GetCandidatePathData() const;

  void SetCandidatePathData(std::vector<PathData>&& candidate_path_data);

  Obstacle* GetBlockingObstacle() const { return blocking_obstacle_; }
  void SetBlockingObstacle(const std::string& blocking_obstacle_id);
   //借到
  bool is_path_lane_borrow() const { return is_path_lane_borrow_; }
  void set_is_path_lane_borrow(const bool is_path_lane_borrow) {
    is_path_lane_borrow_ = is_path_lane_borrow;
  }
  //自车是否在该车道线上
  void set_is_on_reference_line() { is_on_reference_line_ = true; }
  //车道线的优先级
  uint32_t GetPriority() const { return reference_line_.GetPriority(); }

  void SetPriority(uint32_t priority) { reference_line_.SetPriority(priority); }

  void set_trajectory_type(
      const ADCTrajectory::TrajectoryType trajectory_type) {
    trajectory_type_ = trajectory_type;
  }

  ADCTrajectory::TrajectoryType trajectory_type() const {
    return trajectory_type_;
  }
  //返回ST图
  StGraphData* mutable_st_graph_data() { return &st_graph_data_; }

  const StGraphData& st_graph_data() { return st_graph_data_; }

  // different types of overlaps that can be handled by different scenarios.
  //不同的道路重叠类型
  enum OverlapType {
    CLEAR_AREA = 1,
    CROSSWALK = 2,
    OBSTACLE = 3,
    PNC_JUNCTION = 4,
    SIGNAL = 5,
    STOP_SIGN = 6,
    YIELD_SIGN = 7,
  };

  const std::vector<std::pair<OverlapType, hdmap::PathOverlap>>&
  FirstEncounteredOverlaps() const {
    return first_encounter_overlaps_;
  }
  //纵向S处的PNC路口
  int GetPnCJunction(const double s,
                     hdmap::PathOverlap* pnc_junction_overlap) const;
  //停止点
  std::vector<common::SLPoint> GetAllStopDecisionSLPoint() const;
  //转向信号灯
  void SetTurnSignal(const common::VehicleSignal::TurnSignal& turn_signal);
  void SetEmergencyLight();

  void set_path_reusable(const bool path_reusable) {
    path_reusable_ = path_reusable;
  }

  bool path_reusable() const { return path_reusable_; }

 private:
  void InitFirstOverlaps();

  bool CheckChangeLane() const; //是否位于换道

  void SetTurnSignalBasedOnLaneTurnType(
      common::VehicleSignal* vehicle_signal) const;//设置转向灯

  void ExportVehicleSignal(common::VehicleSignal* vehicle_signal) const;

  bool IsIrrelevantObstacle(const Obstacle& obstacle);

  void MakeDecision(DecisionResult* decision_result,
                    PlanningContext* planning_context) const;

  int MakeMainStopDecision(DecisionResult* decision_result) const;

  void MakeMainMissionCompleteDecision(DecisionResult* decision_result,
                                       PlanningContext* planning_context) const;

  void MakeEStopDecision(DecisionResult* decision_result) const;

  void SetObjectDecisions(ObjectDecisions* object_decisions) const;

  bool AddObstacleHelper(const std::shared_ptr<Obstacle>& obstacle);

  bool GetFirstOverlap(const std::vector<hdmap::PathOverlap>& path_overlaps,
                       hdmap::PathOverlap* path_overlap);

 private:
  static std::unordered_map<std::string, bool> junction_right_of_way_map_;
  const common::VehicleState vehicle_state_;
  const common::TrajectoryPoint adc_planning_point_;
  ReferenceLine reference_line_;

  /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
  double cost_ = 0.0;

  bool is_drivable_ = true;

  PathDecision path_decision_;

  Obstacle* blocking_obstacle_;

  std::vector<PathBoundary> candidate_path_boundaries_;
  std::vector<PathData> candidate_path_data_;

  PathData path_data_;
  PathData fallback_path_data_;
  SpeedData speed_data_;

  DiscretizedTrajectory discretized_trajectory_;

  RSSInfo rss_info_;

  /**
   * @brief SL boundary of stitching point (starting point of plan trajectory)
   * relative to the reference line
   */
  SLBoundary adc_sl_boundary_;

  planning_internal::Debug debug_;
  LatencyStats latency_stats_;

  hdmap::RouteSegments lanes_;

  bool is_on_reference_line_ = false;

  bool is_path_lane_borrow_ = false;

  ADCTrajectory::RightOfWayStatus status_ = ADCTrajectory::UNPROTECTED;

  double offset_to_other_reference_line_ = 0.0;

  double priority_cost_ = 0.0;

  PlanningTarget planning_target_;

  ADCTrajectory::TrajectoryType trajectory_type_ = ADCTrajectory::UNKNOWN;

  /**
   * Overlaps encountered in the first time along the reference line in front of
   * the vehicle
   */
  std::vector<std::pair<OverlapType, hdmap::PathOverlap>>
      first_encounter_overlaps_;

  /**
   * @brief Data generated by speed_bounds_decider for constructing st_graph for
   * different st optimizer
   */
  StGraphData st_graph_data_;

  common::VehicleSignal vehicle_signal_;

  double cruise_speed_ = 0.0;

  bool path_reusable_ = false;

  DISALLOW_COPY_AND_ASSIGN(ReferenceLineInfo);
};

}  // namespace planning
}  // namespace apollo
