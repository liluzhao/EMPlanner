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

#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning {
/*
SpeedData类继承速度点类vector，本质上也是一个数组容器，用来存放一系列的速度点，并实现以下功能：
将这些速度点按时间升序排序；
在容器末尾增加一个速度点；
从速度点序列按时间t插值得到速度点；
从速度点序列按轨迹长度s插值得到速度点；
获取速度点序列总时间、轨迹总长度；
返回多个速度点信息作为debug调试字符串。
速度点SpeedPoint包括v,a,s,t,da等信息
*/
class SpeedData : public std::vector<common::SpeedPoint> {
 public:
  SpeedData() = default;

  virtual ~SpeedData() = default;
  //带参数的speed_points 数组初始化speeddata类
  explicit SpeedData(std::vector<common::SpeedPoint> speed_points);
  //speeddata类对象插入一个速度点
  void AppendSpeedPoint(const double s, const double time, const double v,
                        const double a, const double da);
  //插值 通过 t或者S 去插值
  bool EvaluateByTime(const double time,
                      common::SpeedPoint* const speed_point) const;

  // Assuming spatial traversed distance is monotonous, which is the case for
  // current usage on city driving scenario
  bool EvaluateByS(const double s, common::SpeedPoint* const speed_point) const;

  double TotalTime() const;

  // Assuming spatial traversed distance is monotonous
  double TotalLength() const;

  virtual std::string DebugString() const;
};

}  // namespace planning
}  // namespace apollo
