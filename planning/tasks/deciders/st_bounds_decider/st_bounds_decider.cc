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

#include "modules/planning/tasks/deciders/st_bounds_decider/st_bounds_decider.h"

#include <limits>
#include <memory>

#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::planning_internal::StGraphBoundaryDebug;
using apollo::planning_internal::STGraphDebug;

namespace {
// STBoundPoint contains (t, s_min, s_max)
using STBoundPoint = std::tuple<double, double, double>;
// STBound is a vector of STBoundPoints
using STBound = std::vector<STBoundPoint>;
// ObsDecSet is a set of decision for new obstacles.
using ObsDecSet = std::vector<std::pair<std::string, ObjectDecisionType>>;
}  // namespace

STBoundsDecider::STBoundsDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  ACHECK(config.has_st_bounds_decider_config());
  st_bounds_config_ = config.st_bounds_decider_config();
}
/*
//入参1:Frame类对象，储存了一个规划周期的所有数据，如规划起始点，规划轨迹，参考线，车辆状态，routing路由信息等；
//入参2：一个参考线信息类对象reference_line_info，这上面信息很多，障碍物列表，对个障碍物决策，以及个障碍物以及自车在ST图上的投影等。
  对于纵向决策与规划而言，需要依据障碍物的预测轨迹、类型等，构建其st_boundary映射至planned_path上，并对其进行决策打标签(yield, overtake…);
  轨迹规划模块再依据决策信息，添加对应求解目标、约束条件，求解处"最优"轨迹。
  STBoundsDecider 主要是对动态以及最近的一个静态且阻塞当前引导路径的障碍物进行st图构建,对不影响纵向规划的障碍物设置IGNORE属性,
  并按照设定轨迹给出每一个障碍物boundary的最优决策(overtake/yield),最终决策出最优的
*/
Status STBoundsDecider::Process(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  // Initialize the related helper classes.
  InitSTBoundsDecider(*frame, reference_line_info);//初始化相关的辅助类，主要是初始化STObstaclesProcessor类，st_guide_line_，st_driving_limits_
  // Sweep the t-axis, and determine the s-boundaries step by step.
  STBound regular_st_bound;
  STBound regular_vt_bound;
  std::vector<std::pair<double, double>> st_guide_line;//先定义3个空的变量regular_st_bound,regular_vt_bound,st_guide_line为常规ST边界，常规VT边界，ST引导线。
  Status ret = GenerateRegularSTBound(&regular_st_bound, &regular_vt_bound,
                                      &st_guide_line);
   //把ST图的最大时间，最大长度传进去STObstaclesProcessor，以及将各障碍物投影到ST图上
  /*
    遍历整个时间轴，一步一步确定s边界
    regular_st_bound,regular_vt_bound,st_guide_line。
    同时求出整个规划时域0-7.0s上v,s的边界和ST引导线(大概是以默认巡航速度行驶的ST进行修正得到)
    regular_st_bound,regular_vt_bound都是STBound类型
    STBound 定义了一个ST或VT边界类型，其实就是上面STBoundPoint 的一个vector数组，就是一系列时刻对应的s或v上下边界
    */                                   
  if (!ret.ok()) {//如果上面调用GenerateRegularSTBound()生成常规s,v边界失败的话，就打印debug信息，返回故障状态码
    ADEBUG << "Cannot generate a regular ST-boundary.";
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }
  if (regular_st_bound.empty()) {
    const std::string msg = "Generated regular ST-boundary is empty.";//如果上面求解出的常规ST边界是空的，则报错
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  //创建一个st图数据对象指针指向类成员参考线信息对象里的st图数据
  //然后将上面获得的st边界vt边界设置到st图数据对象的行驶边界
  StGraphData* st_graph_data = reference_line_info_->mutable_st_graph_data();
  st_graph_data->SetSTDrivableBoundary(regular_st_bound, regular_vt_bound);//设置可行驶区域 dp使用
  // Record the ST-Graph for good visualization and easy debugging.
  auto all_st_boundaries = st_obstacles_processor_.GetAllSTBoundaries();
  std::vector<STBoundary> st_boundaries;
  for (const auto& st_boundary : all_st_boundaries) {
    st_boundaries.push_back(st_boundary.second);
  }
  ADEBUG << "Total ST boundaries = " << st_boundaries.size();
  STGraphDebug* st_graph_debug = reference_line_info->mutable_debug()
                                     ->mutable_planning_data()
                                     ->add_st_graph();
  RecordSTGraphDebug(st_boundaries, regular_st_bound, st_guide_line,
                     st_graph_debug);
  //自车常规0-7s S上下边界，引导线数据全部都储存到该函数输入参数reference_line_info里的debug里的planning_data里的st_graph里，用于debug                  
  return Status::OK();
}

void STBoundsDecider::InitSTBoundsDecider(
    const Frame& frame, ReferenceLineInfo* const reference_line_info) {
  const PathData& path_data = reference_line_info->path_data();
  PathDecision* path_decision = reference_line_info->path_decision();//路径决策是工具类
  // Map all related obstacles onto ST-Graph.
  //这里其实就是用这个st_obstacles_processor_将输入参数path_decision里的障碍物列表的所有障碍物投影到这个ST图上，
  //  其实就是得到所有相关障碍物的ST边界，然后同时记录这一过程的耗时。看来这一块应该耗时不少 
  auto time1 = std::chrono::system_clock::now();
  st_obstacles_processor_.Init(path_data.discretized_path().Length(),
                               st_bounds_config_.total_time(), path_data,
                               path_decision, injector_->history());
  
  st_obstacles_processor_.MapObstaclesToSTBoundaries(path_decision);//遍历所有的障碍物，构建障碍物的ST图的STBoundary。
  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;
  ADEBUG << "Time for ST Obstacles Processing = " << diff.count() * 1000
         << " msec.";

  // Initialize Guide-Line and Driving-Limits.//初始化引导线和驾驶限制，这里其实是一个常量默认期望速度15m/s，约54kph
  static constexpr double desired_speed = 15.0;
  // If the path_data optimization is guided from a reference path of a
  // reference trajectory, use its reference speed profile to select the st
  // bounds in LaneFollow Hybrid Mode
  //如果路径数据优化被一个参考路径引导，用它参考的速度规划来选择这个ST的边界在车道跟随的混合模式中
  //如果路径数据是朝着参考轨迹优化的，这里is_optimized_towards_trajectory_reference默认为false，直接看else
  //引导线就是直接以期望速度15m/s匀速运动？st_guide_line_就是以期望速度匀速行驶的S插值功能的一个类
  if (path_data.is_optimized_towards_trajectory_reference()) {
    st_guide_line_.Init(desired_speed,
                        injector_->learning_based_data()
                            ->learning_data_adc_future_trajectory_points());
  } else {
    st_guide_line_.Init(desired_speed);//引导线在ST图中不可超越
  }
  //定义了最大加速度2.5，最大减速度5.0，最大速度1.5倍期望速度
  //STDrivingLimits这个类其实就是根据车辆最大加减速的限制计算处相应时间可以到达的s这个类就是实现这样一个功能，这里就是对这样一个类对象进行初始化，同时输入规划起始点的速度
  static constexpr double max_acc = 2.5;
  static constexpr double max_dec = 5.0;
  static constexpr double max_v = desired_speed * 1.5;
  st_driving_limits_.Init(max_acc, max_dec, max_v,
                          frame.PlanningStartPoint().v());
}

//这个函数就是产生FALLBACK的ST边界，就是规划求解失败后的ST边界
//入参就是st的边界和vt的边界
//这个函数压根没被用到？直接跳过！
Status STBoundsDecider::GenerateFallbackSTBound(STBound* const st_bound,
                                                STBound* const vt_bound) {
  // Initialize st-boundary.
  for (double curr_t = 0.0; curr_t <= st_bounds_config_.total_time();
       curr_t += kSTBoundsDeciderResolution) {
    st_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
    vt_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
  }

  // Sweep-line to get detailed ST-boundary.
  for (size_t i = 0; i < st_bound->size(); ++i) {
    double t, s_lower, s_upper, lower_obs_v, upper_obs_v;
    std::tie(t, s_lower, s_upper) = st_bound->at(i);
    std::tie(t, lower_obs_v, upper_obs_v) = vt_bound->at(i);
    ADEBUG << "Processing st-boundary at t = " << t;

    // Get Boundary due to driving limits
    auto driving_limits_bound = st_driving_limits_.GetVehicleDynamicsLimits(t);
    s_lower = std::fmax(s_lower, driving_limits_bound.first);
    s_upper = std::fmin(s_upper, driving_limits_bound.second);
    ADEBUG << "Bounds for s due to driving limits are "
           << "s_upper = " << s_upper << ", s_lower = " << s_lower;

    // Get Boundary due to obstacles
    std::vector<std::pair<double, double>> available_s_bounds;
    std::vector<ObsDecSet> available_obs_decisions;
    if (!st_obstacles_processor_.GetSBoundsFromDecisions(
            t, &available_s_bounds, &available_obs_decisions)) {
      const std::string msg =
          "Failed to find a proper boundary due to obstacles.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    std::vector<std::pair<STBoundPoint, ObsDecSet>> available_choices;
    ADEBUG << "Available choices are:";
    for (int j = 0; j < static_cast<int>(available_s_bounds.size()); ++j) {
      ADEBUG << "  (" << available_s_bounds[j].first << ", "
             << available_s_bounds[j].second << ")";
      available_choices.emplace_back(
          std::make_tuple(0.0, available_s_bounds[j].first,
                          available_s_bounds[j].second),
          available_obs_decisions[j]);
    }
    RemoveInvalidDecisions(driving_limits_bound, &available_choices);

    // Always go for the most conservative option.
    if (!available_choices.empty()) {
      // Select the most conservative decision. 谨慎决策
      auto top_choice_s_range = available_choices.front().first;
      auto top_choice_decision = available_choices.front().second;
      for (size_t j = 1; j < available_choices.size(); ++j) {
        if (std::get<1>(available_choices[j].first) <
            std::get<1>(top_choice_s_range)) {
          top_choice_s_range = available_choices[j].first;
          top_choice_decision = available_choices[j].second;
        }
      }

      // Set decision for obstacles without decisions.
      bool is_limited_by_upper_obs = false;
      bool is_limited_by_lower_obs = false;
      if (s_lower < std::get<1>(top_choice_s_range)) {
        s_lower = std::get<1>(top_choice_s_range);
        is_limited_by_lower_obs = true;
      }
      if (s_upper > std::get<2>(top_choice_s_range)) {
        s_upper = std::get<2>(top_choice_s_range);
        is_limited_by_upper_obs = true;
      }
      st_obstacles_processor_.SetObstacleDecision(top_choice_decision);

      // Update st-guide-line, st-driving-limit info, and v-limits.
      std::pair<double, double> limiting_speed_info;
      if (st_obstacles_processor_.GetLimitingSpeedInfo(t,
                                                       &limiting_speed_info)) {
        st_driving_limits_.UpdateBlockingInfo(
            t, s_lower, limiting_speed_info.first, s_upper,
            limiting_speed_info.second);
        st_guide_line_.UpdateBlockingInfo(t, s_lower, true);
        st_guide_line_.UpdateBlockingInfo(t, s_upper, false);
        if (is_limited_by_lower_obs) {
          lower_obs_v = limiting_speed_info.first;
        }
        if (is_limited_by_upper_obs) {
          upper_obs_v = limiting_speed_info.second;
        }
      }
    } else {
      const std::string msg = "No valid st-boundary exists.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // Update into st_bound
    st_bound->at(i) = std::make_tuple(t, s_lower, s_upper);
    vt_bound->at(i) = std::make_tuple(t, lower_obs_v, upper_obs_v);
  }

  return Status::OK();
}

//产生常规的ST边界
//入参1：一系列的ST边界点，一个ST边界点就是(t,s下界，s上界)，这个函数会产生这个ST边界。
//入参2：一些列的vt边界点，这个函数会产生这个vt边界。
//入参3:st引导线，是一个pair的vector,这个是用来存放t和对应的引导线s的
Status STBoundsDecider::GenerateRegularSTBound(
    STBound* const st_bound, STBound* const vt_bound,
    std::vector<std::pair<double, double>>* const st_guide_line) {

  for (double curr_t = 0.0; curr_t <= st_bounds_config_.total_time();//total_time 被定义为7.0s,
       curr_t += kSTBoundsDeciderResolution) {//0.1s kSTBoundsDeciderResolution采样一次 0 0.1 0.2 0.3 0.4 ... 7.0
    st_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
    vt_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
  }
  // Sweep-line to get detailed ST-boundary.
  //此时st_bound里应该塞了71个点(共7.0s,0.1s一个点)
  //通过std::tie取出tuple(5维向量中)的数到t,s_lower,s_upper,lower_obs_v,upper_obs_v
  for (size_t i = 0; i < st_bound->size(); ++i) {//遍历
    double t, s_lower, s_upper, lower_obs_v, upper_obs_v;
    std::tie(t, s_lower, s_upper) = st_bound->at(i);
    std::tie(t, lower_obs_v, upper_obs_v) = vt_bound->at(i);//为什么会有障碍物的lower_obs_v,upper_obs_v
    ADEBUG << "Processing st-boundary at t = " << t;
    // Get Boundary due to driving limits
    //按照init中定义的最大加速度2.5，最大减速度5.0，最大速度1.5倍的期望速度
    auto driving_limits_bound = st_driving_limits_.GetVehicleDynamicsLimits(t);//根据车辆的最大加减速度限制获取St上下边界也就是t时刻s_lower,s_upper
    s_lower = std::fmax(s_lower, driving_limits_bound.first);
    s_upper = std::fmin(s_upper, driving_limits_bound.second);
    ADEBUG << "Bounds for s due to driving limits are "
           << "s_upper = " << s_upper << ", s_lower = " << s_lower;
    // Get Boundary due to obstacles
    /*
      定义了available_s_bounds是一个二维向量的vector数组，储存t时刻所有可能的s上下界(可能夹在多个障碍物ST边界之间，因此存在多个可能)
      available_obs_decisions是一个储存障碍物列表及其决策的vector数组，里面每一个元素都是一个可能得决策列表，用来储存t时刻对所有相关障碍物的决策。如果取不同障碍物之间得区域可能有不同得决策，因此有多套决策列表和上面得available_s_bounds对应，一个bounds对应着一个决策列表。
      之前init初始化过的st_obstacle_processor_，障碍物的ST图投影即ST边界全部得到了，调用st_obstacle_processor_的成员函数GetSBoundsFromDecisions(),求得自车所有可能s边界available_s_bounds及其对应得决策列表available_obs_decisions
      GetSBoundsFromDecisions()得结果就存在这两个变量里，如果函数运行失败报错
      st_obstacle_processor_的成员函数GetSBoundsFromDecisions()需要好好看一看其中原理。
      GetSBoundsFromDecisions()其作用就是根据障碍物t时刻在St图上投影的ST边界求出自车可行使的区域available_s_bounds(可能ST图上夹在几个障碍物之间的多个空隙都可以行驶)以及每个可行驶区域对应的一套障碍物决策available_obs_decisions
    */
   
   /* vector available_s_bounds，用来储存可用的选择，其实就是把上面可行驶的空隙S边界和对应的障碍物决策一起重新组织了下结构变成一个vector 自车行驶区域和决策的可利用的选择
      RemoveInvalidDecisions()这个函数又将可利用的选择里不符合运动学，也就是最大加减速度限制，不符合的choice剔除掉
      按照init中定义的最大加速度2.5，最大减速度5.0，最大速度1.5倍期望速度
   */
    std::vector<std::pair<double, double>> available_s_bounds;
    std::vector<ObsDecSet> available_obs_decisions;
    if (!st_obstacles_processor_.GetSBoundsFromDecisions(
            t, &available_s_bounds, &available_obs_decisions)) {//根据ST图的t得到available_s_bounds，available_obs_decisions；
           
      const std::string msg =
          "Failed to find a proper boundary due to obstacles.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    std::vector<std::pair<STBoundPoint, ObsDecSet>> available_choices;
    ADEBUG << "Available choices are:";
    for (int j = 0; j < static_cast<int>(available_s_bounds.size()); ++j) {
      ADEBUG << "  (" << available_s_bounds[j].first << ", "
             << available_s_bounds[j].second << ")";
      available_choices.emplace_back(
          std::make_tuple(0.0, available_s_bounds[j].first,
                          available_s_bounds[j].second),
          available_obs_decisions[j]);
    }
    RemoveInvalidDecisions(driving_limits_bound, &available_choices);
    /*
      如果剔除不符合运动学限制的choice后，剩下的choice获取
      st_guide_line_在上面L115行已经被默认巡航参考速度15.0m/s初始化过了，这个就是一条ST引导线，就是以15m/s匀速行驶的一条引导线
      guide_line_s是引导线插值出t时刻的引导线s,然后函数的入参指针st_guide_line里塞入t和引导线S
      然后调用RankDecisions()对剩下的choice进行排序：
      两个相邻的空隙，若有一个上下界差小于3m，那就把上下界差大的放左边；
      若两相邻空隙上下界之差都大于3m，谁包含引导线s谁放左边
      原则就是上下界差<3m的情况下谁大谁在前面，上下界差>3m的情况下谁包含引导线谁在前面
      然后选取排在最前面也就是最优的choice
      s_lower,s_upper已在之前用运动学初值算了一个值，最大加减速度对应的s.
      如果s_lower比top_choice的s_min还要小，就将s_lower取为top_choice的s_min还要，说明其被障碍物限制了下限不能太慢；并设定标志位
      如果s_upper比top_choice的s_max还要小，就将s_max取为top_choice的s_max还要，说明其被障碍物限制了上限不能太快；并设定标志位
      这里s_lower,s_upper在继运动学更新一次后，又被障碍物决策更新了一次；
    */
    if (!available_choices.empty()) {
      ADEBUG << "One decision needs to be made among "
             << available_choices.size() << " choices.";
      double guide_line_s = st_guide_line_.GetGuideSFromT(t);// st_guide_line_ ??st_guide_line_在上面L115行已经被默认巡航参考速度15.0m/s初始化过了，这个就是一条ST引导线，
      st_guide_line->emplace_back(t, guide_line_s);
      RankDecisions(guide_line_s, driving_limits_bound, &available_choices);
      // Select the top decision.
      auto top_choice_s_range = available_choices.front().first;
      bool is_limited_by_upper_obs = false;
      bool is_limited_by_lower_obs = false;
      if (s_lower < std::get<1>(top_choice_s_range)) {
        s_lower = std::get<1>(top_choice_s_range);
        is_limited_by_lower_obs = true;
      }
      if (s_upper > std::get<2>(top_choice_s_range)) {
        s_upper = std::get<2>(top_choice_s_range);
        is_limited_by_upper_obs = true;
      }

      // Set decision for obstacles without decisions.
      //这里取出了最优选择top_choice里的决策top_choice_decision
      //然后将这些决策设置到刚刚那些ambiguous模糊未决策的障碍物上
      auto top_choice_decision = available_choices.front().second;
      st_obstacles_processor_.SetObstacleDecision(top_choice_decision);

      // Update st-guide-line, st-driving-limit info, and v-limits.
      /*
      这里更新st引导线，st驾驶限制信息和速度限制
      GetLimitingSpeedInfo(...)函数在后面的小节会单独介绍，这里只了解其大概作用其主要就是根据障碍物的yield,stop,overtake决策算出自车的在每个时刻的速度限制上下限。
      如果yield,stop的就不能比它快，overtake的就不能比它慢。得到的自车速度限制放在limiting_speed_info里。
      然后st_driving_limits_调用UpdateBlockingInfo()函数更新阻塞信息？把当前的s的上下边界，速度的上下边界及对应的时间更新到st_driving_limits的类成员里。
      然后又st_guide_line_引导线也调用UpdateBlockingInfo()函数更新t时刻的引导线,其实就是用求出的S上下界去限制t时刻引导线s不能超出这个范围。
      如果s被下方的的障碍物限制住了，那么自车速度下限lower_obs_v不能小于下方障碍物的上斜边ST斜率速度 limiting_speed_info.first
      如果s被上方的的障碍物限制住了，那么自车速度上限upper_obs_v不能小于上方障碍物的下斜边ST斜率速度 limiting_speed_info.second
      如果GetLimitingSpeedInfo(...)函数执行失败则报错。
      然后更新t时刻的st_bound和vt_bound
      */
      std::pair<double, double> limiting_speed_info;
      if (st_obstacles_processor_.GetLimitingSpeedInfo(t,
                                                       &limiting_speed_info)) {
        st_driving_limits_.UpdateBlockingInfo(
            t, s_lower, limiting_speed_info.first, s_upper,
            limiting_speed_info.second);
        st_guide_line_.UpdateBlockingInfo(t, s_lower, true);
        st_guide_line_.UpdateBlockingInfo(t, s_upper, false);
        if (is_limited_by_lower_obs) {
          lower_obs_v = limiting_speed_info.first;
        }
        if (is_limited_by_upper_obs) {
          upper_obs_v = limiting_speed_info.second;
        }
      }
    } else {
      const std::string msg = "No valid st-boundary exists.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // Update into st_bound
    st_bound->at(i) = std::make_tuple(t, s_lower, s_upper);
    vt_bound->at(i) = std::make_tuple(t, lower_obs_v, upper_obs_v);
  }

  return Status::OK();
}
/*
  移除无效的决策函数
  在多套可以选用的决策里剔除无效的决策那几套。
  比如选择①②③中的任一个区域(代码中叫gap)对这两个障碍物的决策都不同，选择一个区域就对应着一套对所有障碍物的决策，这个函数就是要剔除里面无效的那几套决策。
  剔除的原则就是如果一个gap对应的上下界超出了最大加减速度限制的话就是无效的(gap的s下界超出了最大加速度能到达的s 或 gap的上界甚至低于最大减速度到达的s)
*/
void STBoundsDecider::RemoveInvalidDecisions(
    std::pair<double, double> driving_limit,
    std::vector<std::pair<STBoundPoint, ObsDecSet>>* available_choices) {
  // Remove those choices that don't even fall within driving-limits.
  size_t i = 0;
  while (i < available_choices->size()) {
    double s_lower = 0.0;
    double s_upper = 0.0;
    std::tie(std::ignore, s_lower, s_upper) = available_choices->at(i).first;
    if (s_lower > driving_limit.second || s_upper < driving_limit.first) {
      // Invalid bound, should be removed.
      if (i != available_choices->size() - 1) {
        swap(available_choices->at(i),
             available_choices->at(available_choices->size() - 1));
      }
      available_choices->pop_back();
    } else {
      // Valid bound, proceed to the next one.
      ++i;
    }
  }
}
/*
  这个函数对所有可能的一套套决策按套进行排序,光看代码可能比较难理解其原理及最终输出效果，可以自己造个简单的输入，包含2个障碍物，3个gap，还有运动学上下限，再给出3套可选的决策，在草稿纸上按代码推算一遍就能明白了，这里不过多赘述，只说这个函数的大概的作用，就是将可能的一套套决策按照如下原则进行排序，遍历所有gap：
  如果相邻的两个gap s上下界之差有一个gap是小于3m的，那就将其中空间更大的一个排在前面；
  如果相邻的两个gap s上下界之差都大于等于3m，就将包含引导线的那个gap排在前面
  大体原则就是空间尽量大，尽量包含引导线的排在前面，排在最前面的就是top_choice，就是最后要选用的那套决策和gap。
*/
void STBoundsDecider::RankDecisions(
    double s_guide_line, std::pair<double, double> driving_limit,
    std::vector<std::pair<STBoundPoint, ObsDecSet>>* available_choices) {
  // Perform sorting of the existing decisions.
  bool has_swaps = true;
  while (has_swaps) {
    has_swaps = false;
    for (int i = 0; i < static_cast<int>(available_choices->size()) - 1; ++i) {
      double A_s_lower = 0.0;
      double A_s_upper = 0.0;
      std::tie(std::ignore, A_s_lower, A_s_upper) =
          available_choices->at(i).first;
      double B_s_lower = 0.0;
      double B_s_upper = 0.0;
      std::tie(std::ignore, B_s_lower, B_s_upper) =
          available_choices->at(i + 1).first;

      ADEBUG << "    Range ranking: A has s_upper = " << A_s_upper
             << ", s_lower = " << A_s_lower;
      ADEBUG << "    Range ranking: B has s_upper = " << B_s_upper
             << ", s_lower = " << B_s_lower;

      // If not both are larger than passable-threshold, should select
      // the one with larger room.
      double A_room = std::fmin(driving_limit.second, A_s_upper) -
                      std::fmax(driving_limit.first, A_s_lower);
      double B_room = std::fmin(driving_limit.second, B_s_upper) -
                      std::fmax(driving_limit.first, B_s_lower);// S的间隔更大
      if (A_room < kSTPassableThreshold || B_room < kSTPassableThreshold) {
        if (A_room < B_room) {
          swap(available_choices->at(i + 1), available_choices->at(i));
          has_swaps = true;
          ADEBUG << "Swapping to favor larger room.";
        }
        continue;
      }

      // Should select the one with overlap to guide-line  和guide-line 重叠
      bool A_contains_guideline =
          A_s_upper >= s_guide_line && A_s_lower <= s_guide_line;
      bool B_contains_guideline =
          B_s_upper >= s_guide_line && B_s_lower <= s_guide_line;
      if (A_contains_guideline != B_contains_guideline) {
        if (!A_contains_guideline) {
          swap(available_choices->at(i + 1), available_choices->at(i));
          has_swaps = true;
          ADEBUG << "Swapping to favor overlapping with guide-line.";
        }
        continue;
      }
    }
  }
}

void STBoundsDecider::RecordSTGraphDebug(
    const std::vector<STBoundary>& st_graph_data, const STBound& st_bound,
    const std::vector<std::pair<double, double>>& st_guide_line,
    planning_internal::STGraphDebug* const st_graph_debug) {
  if (!FLAGS_enable_record_debug || !st_graph_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  // Plot ST-obstacle boundaries.
  for (const auto& boundary : st_graph_data) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary.id());
    if (boundary.boundary_type() == STBoundary::BoundaryType::YIELD) {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = YIELD";
    } else if (boundary.boundary_type() == STBoundary::BoundaryType::OVERTAKE) {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = OVERTAKE";
    } else {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = UNKNOWN";
    }

    for (const auto& point : boundary.points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  // Plot the chosen ST boundary.
  auto boundary_debug = st_graph_debug->add_boundary();
  boundary_debug->set_name("Generated ST-Boundary");
  boundary_debug->set_type(
      StGraphBoundaryDebug::ST_BOUNDARY_TYPE_DRIVABLE_REGION);
  for (const auto& st_bound_pt : st_bound) {
    auto point_debug = boundary_debug->add_point();
    double t = 0.0;
    double s_lower = 0.0;
    std::tie(t, s_lower, std::ignore) = st_bound_pt;
    point_debug->set_t(t);
    point_debug->set_s(s_lower);
    ADEBUG << "(" << t << ", " << s_lower << ")";
  }
  for (int i = static_cast<int>(st_bound.size()) - 1; i >= 0; --i) {
    auto point_debug = boundary_debug->add_point();
    double t = 0.0;
    double s_upper = 0.0;
    std::tie(t, std::ignore, s_upper) = st_bound[i];
    point_debug->set_t(t);
    point_debug->set_s(s_upper);
    ADEBUG << "(" << t << ", " << s_upper << ")";
  }

  // Plot the used st_guide_line when generating the st_bounds
  for (const auto& st_points : st_guide_line) {
    auto* speed_point = st_graph_debug->add_speed_profile();
    speed_point->set_t(st_points.first);
    speed_point->set_s(st_points.second);
  }
}

}  // namespace planning
}  // namespace apollo
