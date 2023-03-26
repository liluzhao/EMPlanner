/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include "modules/common/math/box2d.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
/*
从代码来看DecisionData类主要是实现：
储存所有的预测障碍物，按动态/静态/所有障碍物分类储存成列表，并且建立id和障碍物对象的映射map，然后可以通过id/类型来查询障碍物；
结合道路参考线信息，可以创建虚拟障碍物对象，并储存到相应的数据成员障碍物列表中。
*/
namespace apollo {
namespace planning {
//虚拟障碍物的类型
enum class VirtualObjectType {//enum class 限定作用域
  DESTINATION = 0,//目的地
  CROSSWALK = 1,//人行横道
  TRAFFIC_LIGHT = 2,//交通灯
  CLEAR_ZONE = 3,//停车区
  REROUTE = 4,
  DECISION_JUMP = 5,
  PRIORITY = 6 //优先行驶
};

struct EnumClassHash {
  template <typename T>
  size_t operator()(T t) const {
    return static_cast<size_t>(t);
  }
};

class DecisionData {
 public:
  DecisionData(const prediction::PredictionObstacles& prediction_obstacles,
               const ReferenceLine& reference_line);
  ~DecisionData() = default;

 public:
  Obstacle* GetObstacleById(const std::string& id);
  std::vector<Obstacle*> GetObstacleByType(const VirtualObjectType& type);
  std::unordered_set<std::string> GetObstacleIdByType(
      const VirtualObjectType& type);
  const std::vector<Obstacle*>& GetStaticObstacle() const;
  const std::vector<Obstacle*>& GetDynamicObstacle() const;
  const std::vector<Obstacle*>& GetVirtualObstacle() const;
  const std::vector<Obstacle*>& GetPracticalObstacle() const;
  const std::vector<Obstacle*>& GetAllObstacle() const;

 public:
  bool CreateVirtualObstacle(const ReferencePoint& point,
                             const VirtualObjectType& type,
                             std::string* const id);
  bool CreateVirtualObstacle(const double point_s,
                             const VirtualObjectType& type,
                             std::string* const id);

 private:
  bool IsValidTrajectory(const prediction::Trajectory& trajectory);
  bool IsValidTrajectoryPoint(const common::TrajectoryPoint& point);
  bool CreateVirtualObstacle(const common::math::Box2d& obstacle_box,
                             const VirtualObjectType& type,
                             std::string* const id);

 private:
  std::vector<Obstacle*> static_obstacle_;
  std::vector<Obstacle*> dynamic_obstacle_;
  std::vector<Obstacle*> virtual_obstacle_;
  std::vector<Obstacle*> practical_obstacle_;
  std::vector<Obstacle*> all_obstacle_;

 private:
  const ReferenceLine& reference_line_;
  std::list<std::unique_ptr<Obstacle>> obstacles_;
  std::unordered_map<std::string, Obstacle*> obstacle_map_;
  std::unordered_map<VirtualObjectType, std::unordered_set<std::string>,
                     EnumClassHash>
      virtual_obstacle_id_map_;
  std::mutex mutex_;
  std::mutex transaction_mutex_;
};

}  // namespace planning
}  // namespace apollo
