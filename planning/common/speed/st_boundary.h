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
 *   @file
 **/

#pragma once

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {
/*
STBoundary的数据结构就是ST点对集，对应每个时间t根据动态障碍物预测轨迹及结构道路信息，纵向距离限制在，smin,smax间
ST点对集的数据结构如下：
{
((Smin0,T0),(Smax0,T0))，
((Smin1,t1),(Smax1,T1))，
((Smin1,T2),(max1,T1))，
...}
t0时刻下边界点(smin,t0)，t1时刻上边界点(smax0,t0)，
STPoint ST点的数据结构 (s,t)
STBoundary类其实就是起到存储当前时间附近一段纵向位置上下边界点集的作用，以及对这个上下边界进行的一些查询，删除冗余点，插值等操作。
*/
class STBoundary : public common::math::Polygon2d {
 public:
  /** Constructors:
   *   STBoundary must be initialized with a vector of ST-point pairs.
   *   Each pair refers to a time t, with (lower_s, upper_s).
   */
//继承Polygon2d类，Polygon2d类定义参见modules\common\math\polygon2d.h
//Polygon2d类就是Vec2d类型的vector，简单说就是一系列的(s,t)点对
//这个vector每个元素包含两个ST点

/** 构造函数:
   *   STBoundary类必须被用一系列的(t,s)点对vector来初始化
   *   每一个时间t对应着(lower_s, upper_s)，上界和下界
   */
  STBoundary() = default;
  explicit STBoundary(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs,
      bool is_accurate_boundary = false);
  explicit STBoundary(const common::math::Box2d& box) = delete;
  explicit STBoundary(std::vector<common::math::Vec2d> points) = delete;

  /** @brief Wrapper of the constructor (old).
   */
  //输入参数就是ST图的下界STPoint类对象点vector，上界STPoint类对象点vector
  static STBoundary CreateInstance(const std::vector<STPoint>& lower_points,
                                   const std::vector<STPoint>& upper_points);

  /** @brief Wrapper of the constructor. It doesn't use RemoveRedundantPoints
   * and generates an accurate ST-boundary.
   */
  static STBoundary CreateInstanceAccurate(
      const std::vector<STPoint>& lower_points,
      const std::vector<STPoint>& upper_points);

  /** @brief Default destructor.
   */
  ~STBoundary() = default;

  bool IsEmpty() const { return lower_points_.empty(); }
  //获取未阻塞的s范围，代入当前时间，此刻s上界，此刻s下界？
  bool GetUnblockSRange(const double curr_time, double* s_upper,
                        double* s_lower) const;
  //获取边界的s范围，代入当前时间，此刻s上界，此刻s下界？
  bool GetBoundarySRange(const double curr_time, double* s_upper,
                         double* s_lower) const;
  //获取边界的变化率，代入当前时间，此刻s上界变化，此刻s下界变化？
  bool GetBoundarySlopes(const double curr_time, double* ds_upper,
                         double* ds_lower) const;

  // if you need to add boundary type, make sure you modify
  // GetUnblockSRange accordingly.
   // 如果你要增加边界类型，请同时修改GetUnblockSRange函数
  // GetUnblockSRange accordingly.
  //枚举的边界类型变量BoundaryType 
  //UNKNOWN未知，STOP停车，FOLLOW跟车，YIELD避让，OVERTAKE超车
  //KEEP_CLEAR不能占用的路肩
  enum class BoundaryType {
    UNKNOWN,
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
    KEEP_CLEAR,
  };

  static std::string TypeName(BoundaryType type);

  BoundaryType boundary_type() const;
  const std::string& id() const;
  double characteristic_length() const;

  void set_id(const std::string& id);
    //设置数据成员边界类型boundary_type_
  void SetBoundaryType(const BoundaryType& boundary_type);
  void SetCharacteristicLength(const double characteristic_length);  //设置数据成员特征长度characteristic_length_

  double min_s() const;
  double min_t() const;
  double max_s() const;
  double max_t() const;

  std::vector<STPoint> upper_points() const { return upper_points_; }  //获取ST边界的上界upper_points_
  std::vector<STPoint> lower_points() const { return lower_points_; }  //返回ST边界中最小/大的s或t

  // Used by st-optimizer.
  bool IsPointInBoundary(const STPoint& st_point) const;//判断ST点是否在边界范围内
  STBoundary ExpandByS(const double s) const;  //现有的ST边界上下边界往外扩给定的s
  STBoundary ExpandByT(const double t) const;

  // Unused function so far.
  STBoundary CutOffByT(const double t) const;

  // Used by Lattice planners.
  STPoint upper_left_point() const;  //返回上下边界点集里的首尾点
  STPoint upper_right_point() const;
  STPoint bottom_left_point() const;
  STPoint bottom_right_point() const;

  void set_upper_left_point(STPoint st_point);  //设置上下边界点集里的首尾点 
  void set_upper_right_point(STPoint st_point);
  void set_bottom_left_point(STPoint st_point);
  void set_bottom_right_point(STPoint st_point);
  //设置返回障碍物道路右边终点？这个成员是用来干什么的？
  void set_obstacle_road_right_ending_t(double road_right_ending_t) {
    obstacle_road_right_ending_t_ = road_right_ending_t;
  }
  double obstacle_road_right_ending_t() const {
    return obstacle_road_right_ending_t_;
  }

 private:
  /** @brief The sanity check function for a vector of ST-point pairs.
   */
  bool IsValid(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const;  //判断ST边界是否有效，主要看上下边界点时间是否对齐

  /** @brief Returns true if point is within max_dist distance to seg.
   */
  bool IsPointNear(const common::math::LineSegment2d& seg,
                   const common::math::Vec2d& point, const double max_dist);

  /** @brief Sometimes a sequence of upper and lower points lie almost on
   * two straightlines. In this case, the intermediate points are removed,
   * with only the end-points retained.
   */
  // TODO(all): When slope is high, this may introduce significant errors.
  // Also, when accumulated for multiple t, the error can get significant.
  // This function should be reconsidered, because it may be dangerous.
    //移除ST上下边界点中的冗余点，每相邻的3个点间，若中间点到前后连线距离小于0.1就删除
  void RemoveRedundantPoints(
      std::vector<std::pair<STPoint, STPoint>>* point_pairs);
  FRIEND_TEST(StBoundaryTest, remove_redundant_points);

  /** @brief Given time t, find a segment denoted by left and right idx, that
   * contains the time t.
   * - If t is less than all or larger than all, return false.
   */
    //查询给定时间t在ST边界对象上下边界点对应的时间的区间的下表left,right
  bool GetIndexRange(const std::vector<STPoint>& points, const double t,
                     size_t* left, size_t* right) const;
  FRIEND_TEST(StBoundaryTest, get_index_range);

 private:
  BoundaryType boundary_type_ = BoundaryType::UNKNOWN;

  std::vector<STPoint> upper_points_;//palnning->tasks->decisers->st_bounds_deciser->st_obstacles_processor 赋值 80个点
  std::vector<STPoint> lower_points_;//障碍物80预测点变成box，和横向规划的轨迹重叠，获得lower_s 和 upper_s t 可能只有40个重叠

  std::string id_;
  double characteristic_length_ = 1.0;
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();

  STPoint bottom_left_point_;//左边的下点
  STPoint bottom_right_point_;
  STPoint upper_left_point_;//左边的上点
  STPoint upper_right_point_;

  double obstacle_road_right_ending_t_;
};

}  // namespace planning
}  // namespace apollo
