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
 * @file path_data.cc
 **/

#include "modules/planning/common/path/path_data.h"

#include <algorithm>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "cyber/common/log.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::PointENU;
using apollo::common::SLPoint;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::util::PointFactory;

bool PathData::SetDiscretizedPath(DiscretizedPath path) {
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set discretized path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  discretized_path_ = std::move(path);
  if (!XYToSL(discretized_path_, &frenet_path_)) {
    AERROR << "Fail to transfer discretized path to frenet path.";
    return false;
  }
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  return true;
}

bool PathData::SetFrenetPath(FrenetFramePath frenet_path) {
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set frenet path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  frenet_path_ = std::move(frenet_path);
  if (!SLToXY(frenet_path_, &discretized_path_)) {
    AERROR << "Fail to transfer frenet path to discretized path.";
    return false;
  }
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  return true;
}

bool PathData::SetPathPointDecisionGuide(
    std::vector<std::tuple<double, PathPointType, double>>
        path_point_decision_guide) {
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set path_point_decision_guide when reference line is "
              "nullptr. ";
    return false;
  }
  if (frenet_path_.empty() || discretized_path_.empty()) {
    AERROR << "Should NOT set path_point_decision_guide when frenet_path or "
              "world frame trajectory is empty. ";
    return false;
  }
  path_point_decision_guide_ = std::move(path_point_decision_guide);
  return true;
}

const DiscretizedPath &PathData::discretized_path() const {
  return discretized_path_;
}

const FrenetFramePath &PathData::frenet_frame_path() const {
  return frenet_path_;
}

const std::vector<std::tuple<double, PathData::PathPointType, double>>
    &PathData::path_point_decision_guide() const {
  return path_point_decision_guide_;
}

bool PathData::Empty() const {
  return discretized_path_.empty() && frenet_path_.empty();
}

void PathData::SetReferenceLine(const ReferenceLine *reference_line) {
  Clear();
  reference_line_ = reference_line;
}

common::PathPoint PathData::GetPathPointWithPathS(const double s) const {
  return discretized_path_.Evaluate(s);
}
//获取路径点用参考纵向位置ref_s? 函数结果放入指针path_point中
//其实这个函数的作用就是给定纵向位置ref_s去frenet系上找到最近点，然后根据最近点索引
//去离散路径上discretized_path_求出对应的离散路径点，貌似功能有点重复？
bool PathData::GetPathPointWithRefS(const double ref_s,
                                    common::PathPoint *const path_point) const {
  ACHECK(reference_line_);
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  if (ref_s < 0) {
    AERROR << "ref_s[" << ref_s << "] should be > 0";
    return false;
  }
  if (ref_s > frenet_path_.back().s()) {
    AERROR << "ref_s is larger than the length of frenet_path_ length ["
           << frenet_path_.back().s() << "].";
    return false;
  }
//实际上frenet_path_和discretized_path_都是继承路径点或frenet系路径点vector类数据结构
    //对于vector序列容器，.at(0)表示获取vector的第一个元素，相比[]会自动检查索引是否越界
    //如果参考点s距离frenet系路径的第i个点的s间的距离小于小正数kDistanceEpsilon
    //那么路径点就从离散路径的第i个进行拷贝并直接返回

  int32_t index = 0;
  const double kDistanceEpsilon = 1e-3;
  for (uint32_t i = 0; i + 1 < frenet_path_.size(); ++i) {
    if (fabs(ref_s - frenet_path_.at(i).s()) < kDistanceEpsilon) {
      path_point->CopyFrom(discretized_path_.at(i));
      return true;
    }
    if (frenet_path_.at(i).s() < ref_s && ref_s <= frenet_path_.at(i + 1).s()) {
      index = i;
      break;
    }
  }
  //由上面可知index就是当ref_s位于frenet系路径第i,i+1个点的路径之间，index取i
  //即index为ref_s在frenet系路径中的纵向最近点索引
  //定义r为给定(纵向位置ref_s减去frenet系上最近点s)/(第i+1,i个点之间的纵向距离)
  //其实r就是ref_s在i,i+1之间的距离比例，如r=0.5，则ref_s对应第i,i+1点的中点
  double r = (ref_s - frenet_path_.at(index).s()) /
             (frenet_path_.at(index + 1).s() - frenet_path_.at(index).s());

  const double discretized_path_s = discretized_path_.at(index).s() +
                                    r * (discretized_path_.at(index + 1).s() -
                                         discretized_path_.at(index).s());
  path_point->CopyFrom(discretized_path_.Evaluate(discretized_path_s));

  return true;
}

void PathData::Clear() {
  discretized_path_.clear();
  frenet_path_.clear();
  path_point_decision_guide_.clear();
  path_reference_.clear();
  reference_line_ = nullptr;
}

std::string PathData::DebugString() const {
  const auto limit =
      std::min(discretized_path_.size(),
               static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));

  return absl::StrCat(
      "[\n",
      absl::StrJoin(discretized_path_.begin(),
                    discretized_path_.begin() + limit, ",\n",
                    apollo::common::util::DebugStringFormatter()),
      "]\n");
}
//转化frenet路径到笛卡尔坐标系路径，通过道路参考线(SL坐标转化为xy坐标)
bool PathData::SLToXY(const FrenetFramePath &frenet_path,
                      DiscretizedPath *const discretized_path) {
  std::vector<common::PathPoint> path_points;
  for (const common::FrenetFramePoint &frenet_point : frenet_path) {
    const common::SLPoint sl_point =
        PointFactory::ToSLPoint(frenet_point.s(), frenet_point.l());
    common::math::Vec2d cartesian_point;
    if (!reference_line_->SLToXY(sl_point, &cartesian_point)) {
      AERROR << "Fail to convert sl point to xy point";
      return false;
    }
    const ReferencePoint ref_point =
        reference_line_->GetReferencePoint(frenet_point.s());
    const double theta = CartesianFrenetConverter::CalculateTheta(
        ref_point.heading(), ref_point.kappa(), frenet_point.l(),
        frenet_point.dl());
    ADEBUG << "frenet_point: " << frenet_point.ShortDebugString();
    const double kappa = CartesianFrenetConverter::CalculateKappa(
        ref_point.kappa(), ref_point.dkappa(), frenet_point.l(),
        frenet_point.dl(), frenet_point.ddl());

    double s = 0.0;
    double dkappa = 0.0;
    if (!path_points.empty()) {
      common::math::Vec2d last = PointFactory::ToVec2d(path_points.back());
      const double distance = (last - cartesian_point).Length();
      s = path_points.back().s() + distance;
      dkappa = (kappa - path_points.back().kappa()) / distance;
    }
    path_points.push_back(PointFactory::ToPathPoint(cartesian_point.x(),
                                                    cartesian_point.y(), 0.0, s,
                                                    theta, kappa, dkappa));
  }
  *discretized_path = DiscretizedPath(std::move(path_points));

  return true;
}

bool PathData::XYToSL(const DiscretizedPath &discretized_path,
                      FrenetFramePath *const frenet_path) {
  ACHECK(reference_line_);
  std::vector<common::FrenetFramePoint> frenet_frame_points;
  const double max_len = reference_line_->Length();
  for (const auto &path_point : discretized_path) {
    common::FrenetFramePoint frenet_point =
        reference_line_->GetFrenetPoint(path_point);
    if (!frenet_point.has_s()) {
      SLPoint sl_point;
      if (!reference_line_->XYToSL(path_point, &sl_point)) {
        AERROR << "Fail to transfer cartesian point to frenet point.";
        return false;
      }
      common::FrenetFramePoint frenet_point;
      // NOTICE: does not set dl and ddl here. Add if needed.
      frenet_point.set_s(std::max(0.0, std::min(sl_point.s(), max_len)));
      frenet_point.set_l(sl_point.l());
      frenet_frame_points.push_back(std::move(frenet_point));
      continue;
    }
    frenet_point.set_s(std::max(0.0, std::min(frenet_point.s(), max_len)));
    frenet_frame_points.push_back(std::move(frenet_point));
  }
  *frenet_path = FrenetFramePath(std::move(frenet_frame_points));
  return true;
}

bool PathData::LeftTrimWithRefS(const common::FrenetFramePoint &frenet_point) {
  ACHECK(reference_line_);
  std::vector<common::FrenetFramePoint> frenet_frame_points;
  frenet_frame_points.emplace_back(frenet_point);

  for (const common::FrenetFramePoint fp : frenet_path_) {
    if (std::fabs(fp.s() - frenet_point.s()) < 1e-6) {
      continue;
    }
    if (fp.s() > frenet_point.s()) {
      frenet_frame_points.push_back(fp);
    }
  }
  SetFrenetPath(FrenetFramePath(std::move(frenet_frame_points)));
  return true;
}

bool PathData::UpdateFrenetFramePath(const ReferenceLine *reference_line) {
  reference_line_ = reference_line;
  return SetDiscretizedPath(discretized_path_);
}

void PathData::set_path_label(const std::string &label) { path_label_ = label; }

const std::string &PathData::path_label() const { return path_label_; }

const std::vector<PathPoint> &PathData::path_reference() const {
  return path_reference_;
}

void PathData::set_path_reference(
    const std::vector<PathPoint> &path_reference) {
  path_reference_ = std::move(path_reference);
}

}  // namespace planning
}  // namespace apollo
