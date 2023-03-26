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

#include <memory>
#include <string>

#include "modules/common/status/status.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

class Task {
 public:
  explicit Task(const TaskConfig& config);

  Task(const TaskConfig& config,  //TaskConfig在文件planning_config.proto里面 或者task_config里面 28种任务 枚举
       const std::shared_ptr<DependencyInjector>& injector);

  virtual ~Task() = default;

  const std::string& Name() const;//const 在成员函数后 常函数 该函数不能改变成员变量

  const TaskConfig& Config() const { return config_; }//const在函数前 返回值不可以改变 如果返回的是指针 指针指向的值不可以改变

  virtual common::Status Execute(Frame* frame,
                                 ReferenceLineInfo* reference_line_info);

  virtual common::Status Execute(Frame* frame);

 protected:
  Frame* frame_ = nullptr;
  ReferenceLineInfo* reference_line_info_ = nullptr;

  TaskConfig config_;
  std::string name_;

  std::shared_ptr<DependencyInjector> injector_;//依赖注入
};

}  // namespace planning
}  // namespace apollo
