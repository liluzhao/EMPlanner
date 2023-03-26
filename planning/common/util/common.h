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

#pragma once

#include <string>
#include <vector>

#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"

namespace apollo {
namespace planning {
namespace util {
/*
从代码来看,common.cc主要是实现：
第一个函数
根据停止线的Frenet系的纵坐标s构建PathDecision路径决策类对象(停车决策)，
并设置相关的停车原因，停车点xyz坐标，停车处heading角等信息。
第二个函数(重载第一个函数)
根据障碍物所处的车道id以及在障碍物在车道中的Frenet系的纵坐标s构建PathDecision路径决策类对象(停车决策)，
并设置相关的停车原因，停车点xyz坐标，停车处heading角等信息。主要就是实现针对停止线/车道中的障碍物设定路径决策对象。
*/
int BuildStopDecision(const std::string& stop_wall_id, const double stop_line_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* const frame,
                      ReferenceLineInfo* const reference_line_info);

int BuildStopDecision(const std::string& stop_wall_id,
                      const std::string& lane_id, const double lane_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* const frame,
                      ReferenceLineInfo* const reference_line_info);
}  // namespace util
}  // namespace planning
}  // namespace apollo
