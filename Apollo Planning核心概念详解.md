# Apollo Planning 核心概念详解

## 文档说明

本文档详细解释Apollo Planning模块中的核心概念：Scenario（场景）、Stage（阶段）、Task（任务）等，帮助开发者快速理解Apollo的架构设计。

**适用读者**：
- Apollo Planning模块开发者
- 需要理解Apollo架构的工程师
- 准备进行二次开发的研究人员

---

## 目录

1. [三层架构概览](#1-三层架构概览)
2. [Scenario场景详解](#2-scenario场景详解)
3. [Stage阶段详解](#3-stage阶段详解)
4. [Task任务详解](#4-task任务详解)
5. [执行流程详解](#5-执行流程详解)
6. [配置文件体系](#6-配置文件体系)
7. [实战示例](#7-实战示例)
8. [扩展开发指南](#8-扩展开发指南)

---

## 1. 三层架构概览

### 1.1 形象比喻

把Apollo Planning想象成一个**电影制作系统**：

```
📽️ Scenario (场景) = 电影剧本
   └─ 例如："追车戏"、"停车戏"、"路口转弯戏"
   
🎬 Stage (阶段) = 剧本的幕/章节
   └─ 例如："接近"、"准备"、"执行"、"完成"
   
🎭 Task (任务) = 演员的具体动作
   └─ 例如："打方向盘"、"踩刹车"、"加速"
```

### 1.2 架构层级图

```
┌─────────────────────────────────────────────────┐
│         PlanningComponent (入口)                 │
│         接收消息触发 (10Hz)                       │
└─────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────┐
│           PublicRoadPlanner                      │
│           (公共道路规划器)                        │
└─────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────┐
│         ScenarioManager                          │
│         根据环境选择场景                          │
└─────────────────────────────────────────────────┘
                      ↓
╔═════════════════════════════════════════════════╗
║  Scenario Layer (场景层) - 11种场景             ║
║  ┌───────────────────────────────────────────┐  ║
║  │  LaneFollowScenario (车道跟随 - 默认)    │  ║
║  │  PullOverScenario (靠边停车)             │  ║
║  │  StopSignScenario (停止标志路口)         │  ║
║  │  TrafficLightScenario (交通灯路口)       │  ║
║  │  ... (其他7种场景)                       │  ║
║  └───────────────────────────────────────────┘  ║
╚═════════════════════════════════════════════════╝
                      ↓
╔═════════════════════════════════════════════════╗
║  Stage Layer (阶段层) - 每个场景2-5个阶段       ║
║  ┌───────────────────────────────────────────┐  ║
║  │  Stage 1: PRE_STOP (接近)                │  ║
║  │  Stage 2: STOP (停止)                    │  ║
║  │  Stage 3: CREEP (蠕行)                   │  ║
║  │  Stage 4: CRUISE (通过)                  │  ║
║  └───────────────────────────────────────────┘  ║
╚═════════════════════════════════════════════════╝
                      ↓
╔═════════════════════════════════════════════════╗
║  Task Layer (任务层) - 每个阶段5-15个任务       ║
║  ┌───────────────────────────────────────────┐  ║
║  │  LaneFollowPath (路径规划)               │  ║
║  │  SpeedBoundsDecider (速度边界)           │  ║
║  │  PiecewiseJerkSpeed (速度优化)           │  ║
║  │  PathDecider (路径决策)                  │  ║
║  │  ... (其他任务)                          │  ║
║  └───────────────────────────────────────────┘  ║
╚═════════════════════════════════════════════════╝
                      ↓
              PathData + SpeedData
                      ↓
              ADCTrajectory (输出)
```

### 1.3 关键特点

| 层级 | 特点 | 切换频率 | 配置方式 |
|------|------|---------|---------|
| **Scenario** | 根据环境自动切换 | 秒级 | planning_config.pb.txt |
| **Stage** | 顺序执行，自动转换 | 秒-分钟级 | pipeline.pb.txt |
| **Task** | 每周期都执行 | 10Hz | task/conf/*.pb.txt |

---

## 2. Scenario（场景）详解

### 2.1 什么是Scenario

**定义**：Scenario是对特定驾驶情境的完整抽象，从进入该情境到退出，包含了完整的处理逻辑。

**关键特征**：
- 每个Scenario有自己的**进入条件**（IsTransferable）
- 每个Scenario包含**多个Stage**（执行步骤）
- Scenario之间可以**动态切换**

### 2.2 Scenario类定义

```cpp
class Scenario {
public:
  // 判断是否可以从other_scenario切换到本场景
  virtual bool IsTransferable(const Scenario* other_scenario,
                              const Frame& frame) {
    return false;  // 子类重写
  }

  // 场景处理（执行当前Stage）
  virtual ScenarioResult Process(
      const common::TrajectoryPoint& planning_init_point, 
      Frame* frame);

  // 场景进入时调用
  virtual bool Enter(Frame* frame) { return true; }

  // 场景退出时调用
  virtual bool Exit(Frame* frame) { return true; }

  // 获取场景上下文（存储场景特有数据）
  virtual ScenarioContext* GetContext() = 0;

  // 获取场景名称
  const std::string& Name() const { return name_; }

protected:
  std::shared_ptr<Stage> current_stage_;  // 当前执行的阶段
  std::unordered_map<std::string, const StagePipeline*> stage_pipeline_map_;
};
```

### 2.3 Apollo支持的11种场景

#### 2.3.1 LaneFollowScenario（车道跟随）

**触发条件**：默认场景，所有其他场景都不满足时使用

**功能**：
- 沿车道行驶
- 车道内避障
- 借道绕行
- 变道行驶

**包含的Stage**：
```
只有1个Stage: LANE_FOLLOW_STAGE
```

**使用频率**：⭐⭐⭐⭐⭐（最常用）

**配置位置**：
```
modules/planning/scenarios/lane_follow/conf/pipeline.pb.txt
```

#### 2.3.2 PullOverScenario（靠边停车）

**触发条件**：
```cpp
bool IsTransferable(const Scenario* other_scenario, const Frame& frame) {
  // 距离目的地 < 50米
  if (frame.DistanceToDestination() < 50.0) {
    return true;
  }
  return false;
}
```

**功能**：
- 寻找合适的停车位置
- 规划靠边停车轨迹
- 平稳停车

**包含的Stage**：
```
Stage 1: APPROACH (接近停车点)
Stage 2: RETRY_APPROACH_PARKING (重试接近)
Stage 3: RETRY_PARKING (重试停车)
```

**使用频率**：⭐⭐⭐

#### 2.3.3 TrafficLightProtectedScenario（交通灯路口）

**触发条件**：
```cpp
bool IsTransferable(const Scenario* other_scenario, const Frame& frame) {
  // 前方100米内有交通灯
  if (HasTrafficLightAhead(frame, 100.0)) {
    return true;
  }
  return false;
}
```

**功能**：
- 识别交通灯状态
- 红灯停车
- 绿灯通过

**包含的Stage**：
```
Stage 1: APPROACH (接近路口)
Stage 2: INTERSECTION_CRUISE (通过路口)
```

**使用频率**：⭐⭐⭐⭐

#### 2.3.4 StopSignUnprotectedScenario（停止标志）

**触发条件**：检测到STOP标志

**功能**：
- 在停止线停车
- 观察路口情况
- 安全通过

**包含的Stage**：
```
Stage 1: PRE_STOP (接近停止线)
Stage 2: STOP (停止)
Stage 3: CREEP (蠕行观察)
Stage 4: INTERSECTION_CRUISE (通过路口)
```

**特殊逻辑**：
- 必须停止3秒以上
- 观察四向来车
- 按到达顺序通过（四向停止标志）

#### 2.3.5 ParkAndGoScenario（泊车出库）

**触发条件**：
- 车辆静止
- 收到出库命令

**功能**：
- 从停车位驶出
- 使用OpenSpace算法

**包含的Stage**：
```
Stage 1: CHECK (检查环境)
Stage 2: ADJUST (调整姿态)
Stage 3: PRE_CRUISE (准备行驶)
Stage 4: CRUISE (正常行驶)
```

#### 2.3.6 其他场景

| 场景 | 触发条件 | 主要用途 |
|------|---------|---------|
| **EmergencyPullOver** | 收到紧急停车命令 | 紧急靠边停车 |
| **ValetParking** | 收到泊车命令 | 自动泊车入库 |
| **BareIntersectionUnprotected** | 无信号路口 | 通过无灯路口 |
| **TrafficLightUnprotectedLeftTurn** | 无保护左转 | 路口左转 |
| **TrafficLightUnprotectedRightTurn** | 无保护右转 | 路口右转 |
| **YieldSign** | 遇到让行标志 | 让行处理 |

### 2.4 Scenario切换机制

```cpp
// ScenarioManager::Update()
Status ScenarioManager::Update(const Frame& frame) {
  // 遍历所有场景（按优先级从高到低）
  for (const auto& scenario_config : scenario_configs_) {
    auto scenario = CreateScenario(scenario_config);
    
    // 检查是否可以切换到此场景
    if (scenario->IsTransferable(current_scenario_.get(), frame)) {
      // 退出当前场景
      if (current_scenario_) {
        current_scenario_->Exit(frame);
      }
      
      // 切换到新场景
      current_scenario_ = scenario;
      current_scenario_->Enter(frame);
      
      AINFO << "Scenario changed to: " << scenario->Name();
      break;  // 找到第一个可转入的场景，停止遍历
    }
  }
  
  return Status::OK();
}
```

**场景切换示例**：

```
时间线：
T0: LaneFollow (正常行驶)
    └─ 检测到前方50米有停止标志
    
T1: 场景切换判断
    ├─ ValetParking::IsTransferable() → false (无泊车命令)
    ├─ PullOver::IsTransferable() → false (未到终点)
    ├─ StopSign::IsTransferable() → true ✓ (检测到STOP标志)
    └─ 切换到StopSignScenario
    
T2: StopSignScenario
    └─ Stage 1: PRE_STOP (开始减速)
    
T3: 到达停止线
    └─ Stage切换: PRE_STOP → STOP
    
T4: 停止3秒后
    └─ Stage切换: STOP → CREEP
    
T5: 路口安全
    └─ Stage切换: CREEP → INTERSECTION_CRUISE
    
T6: 通过路口
    └─ Scenario切换: StopSign → LaneFollow
```

---

## 3. Stage（阶段）详解

### 3.1 什么是Stage

**定义**：Stage是Scenario的执行阶段，一个场景被分解为多个顺序执行的阶段。

**关键特征**：
- 每个Stage包含一个**Task列表**
- Stage按顺序执行，完成后**自动转到下一Stage**
- Stage可以设置**转换条件**

### 3.2 Stage类定义

```cpp
class Stage {
public:
  // 阶段处理（纯虚函数，子类必须实现）
  virtual StageResult Process(
      const common::TrajectoryPoint& planning_init_point, 
      Frame* frame) = 0;

  // 获取下一个阶段名称
  const std::string& NextStage() const { return next_stage_; }

protected:
  // 在参考线上执行任务列表
  StageResult ExecuteTaskOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, 
      Frame* frame);

  std::vector<std::shared_ptr<Task>> task_list_;  // 任务列表
  std::shared_ptr<Task> fallback_task_;           // 失败时的回退任务
  std::string next_stage_;                        // 下一阶段名称
  void* context_;                                 // 场景上下文
};
```

### 3.3 Stage状态

```cpp
enum class StageStatusType {
  ERROR = 1,      // 执行出错
  READY = 2,      // 准备就绪
  RUNNING = 3,    // 正在执行
  FINISHED = 4,   // 执行完成（切换到下一Stage）
};
```

### 3.4 Stage配置

```protobuf
// pipeline.pb.txt
stage: {
  name: "LANE_FOLLOW_STAGE"          // 阶段名称
  type: "LaneFollowStage"            // 阶段类型
  enabled: true                       // 是否启用
  
  // 任务列表（按顺序执行）
  task {
    name: "LANE_FOLLOW_PATH"
    type: "LaneFollowPath"
  }
  task {
    name: "SPEED_BOUNDS_DECIDER"
    type: "SpeedBoundsDecider"
  }
  task {
    name: "PIECEWISE_JERK_SPEED"
    type: "PiecewiseJerkSpeedOptimizer"
  }
  
  // 回退任务
  fallback_task {
    name: "FAST_STOP_TRAJECTORY_FALLBACK"
    type: "FastStopTrajectoryFallback"
  }
}
```

### 3.5 典型Stage示例

#### StopSign场景的4个Stage

**Stage 1: PRE_STOP（接近停止线）**
```cpp
StageResult StagePreStop::Process(const TrajectoryPoint& planning_start_point,
                                  Frame* frame) {
  // 1. 执行任务列表
  auto result = ExecuteTaskOnReferenceLine(planning_start_point, frame);
  
  // 2. 检查是否到达停止线
  const double distance_to_stop_line = GetDistanceToStopLine();
  const double current_speed = frame->vehicle_state().linear_velocity();
  
  if (distance_to_stop_line < 0.5 && current_speed < 0.1) {
    // 已到达停止线并停止
    next_stage_ = "STOP";
    return StageResult(StageStatusType::FINISHED);
  }
  
  // 继续当前Stage
  return StageResult(StageStatusType::RUNNING);
}
```

**Stage 2: STOP（停止等待）**
```cpp
StageResult StageStop::Process(...) {
  // 保持停止，记录停止时间
  double stop_duration = Clock::NowInSeconds() - context->stop_start_time;
  
  // 停止至少3秒（法规要求）
  if (stop_duration > 3.0 && IsIntersectionClear()) {
    next_stage_ = "CREEP";
    return StageResult(StageStatusType::FINISHED);
  }
  
  return StageResult(StageStatusType::RUNNING);
}
```

**Stage 3: CREEP（蠕行观察）**
```cpp
StageResult StageCreep::Process(...) {
  // 以很慢的速度(2m/s)前进，观察路口
  
  if (HasEnteredIntersection()) {
    // 已进入路口，可以正常通过
    next_stage_ = "INTERSECTION_CRUISE";
    return StageResult(StageStatusType::FINISHED);
  }
  
  return StageResult(StageStatusType::RUNNING);
}
```

**Stage 4: INTERSECTION_CRUISE（通过路口）**
```cpp
StageResult StageIntersectionCruise::Process(...) {
  // 正常速度通过路口
  
  if (HasPassedIntersection()) {
    // 已通过路口，场景完成
    return FinishScenario();
  }
  
  return StageResult(StageStatusType::RUNNING);
}
```

**Stage转换流程图**：

```
PRE_STOP ──到达停止线──→ STOP ──停止3秒──→ CREEP ──进入路口──→ INTERSECTION_CRUISE ──通过路口──→ [场景结束]
   ↑                        ↑                     ↑                      ↑
 RUNNING                 RUNNING              RUNNING                RUNNING
   ↓                        ↓                     ↓                      ↓
[继续当前Stage]        [继续停止]          [继续蠕行]             [继续通过]
```

---

## 4. Task（任务）详解

### 4.1 什么是Task

**定义**：Task是最小的执行单元，负责具体的规划算法实现。

**关键特征**：
- 每个Task执行一个**特定的规划算法**
- Task在每个规划周期都会执行（如果在当前Stage的task_list中）
- Task之间有**依赖关系**（通过顺序保证）

### 4.2 Task类定义

```cpp
class Task {
public:
  virtual ~Task() = default;

  // 初始化Task
  virtual bool Init(const std::string& config_dir, 
                    const std::string& name,
                    const std::shared_ptr<DependencyInjector>& injector);

  // 执行Task（纯虚函数）
  virtual common::Status Execute(Frame* frame,
                                 ReferenceLineInfo* reference_line_info);

  // 获取Task名称
  const std::string& Name() const;

protected:
  Frame* frame_;                                 // 当前帧
  ReferenceLineInfo* reference_line_info_;       // 参考线信息
  std::shared_ptr<DependencyInjector> injector_; // 依赖注入器
};
```

### 4.3 Task分类

#### 4.3.1 PathGeneration（路径生成类）

**基类**：
```cpp
class PathGeneration : public Task {
protected:
  virtual Status Process(Frame* frame, 
                        ReferenceLineInfo* reference_line_info) = 0;
};
```

**子类**：
- **LaneFollowPath**：车道跟随路径
- **LaneChangePath**：变道路径
- **LaneBorrowPath**：借道路径  
- **FallbackPath**：回退路径（失败时使用）

**典型执行流程**：
```cpp
Status LaneFollowPath::Process(Frame* frame, ReferenceLineInfo* ref_info) {
  // 1. 计算路径边界
  DecidePathBounds(&path_boundaries);
  
  // 2. 优化路径
  OptimizePath(path_boundaries, &candidate_paths);
  
  // 3. 评估选择
  AssessPath(&candidate_paths, &final_path);
  
  // 4. 保存结果
  ref_info->SetPathData(final_path);
  
  return Status::OK();
}
```

#### 4.3.2 SpeedOptimizer（速度优化类）

**基类**：
```cpp
class SpeedOptimizer : public Task {
protected:
  virtual Status Process(const PathData& path_data,
                        const TrajectoryPoint& init_point,
                        SpeedData* speed_data) = 0;
};
```

**子类**：
- **PiecewiseJerkSpeedOptimizer**：QP速度优化
- **PiecewiseJerkSpeedNonlinearOptimizer**：NLP速度优化
- **PathTimeHeuristicOptimizer**：动态规划速度优化

**典型执行流程**：
```cpp
Status PiecewiseJerkSpeedOptimizer::Process(const PathData& path_data,
                                            const TrajectoryPoint& init_point,
                                            SpeedData* speed_data) {
  // 1. 获取ST图数据
  const StGraphData& st_graph_data = ref_info->st_graph_data();
  
  // 2. 构建QP问题
  SetUpStatesAndBounds(path_data, st_graph_data);
  
  // 3. 求解QP
  OptimizeByQP(speed_data, &distance, &velocity, &acceleration);
  
  // 4. 构造SpeedData
  BuildSpeedData(distance, velocity, acceleration, speed_data);
  
  return Status::OK();
}
```

#### 4.3.3 Decider（决策类）

**基类**：
```cpp
class Decider : public Task {
protected:
  virtual Status Process(Frame* frame,
                        ReferenceLineInfo* reference_line_info) = 0;
};
```

**子类**：
- **PathDecider**：路径决策（选择最优路径）
- **SpeedDecider**：速度决策（确定速度策略）
- **SpeedBoundsDecider**：速度边界决策（构建ST图）
- **RuleBasedStopDecider**：基于规则的停车决策

**典型执行流程**：
```cpp
Status SpeedBoundsDecider::Process(Frame* frame, 
                                   ReferenceLineInfo* ref_info) {
  // 1. 映射障碍物到ST图
  boundary_mapper.ComputeSTBoundary(&path_decision);
  
  // 2. 计算速度限制
  speed_limit_decider.GetSpeedLimits(&speed_limit);
  
  // 3. 加载ST图数据
  st_graph_data->LoadData(boundaries, init_point, speed_limit, ...);
  
  return Status::OK();
}
```

### 4.4 Task执行顺序的重要性

Task的执行顺序非常重要，因为存在**数据依赖**：

```
LaneFollow Stage的Task执行顺序：

1. LaneFollowPath          → 生成 PathData
   ↓ (PathData)
2. PathDecider             → 使用PathData，生成路径决策
   ↓ (PathData + Decisions)
3. SpeedBoundsDecider      → 使用PathData，生成ST图
   ↓ (PathData + StGraphData)
4. PiecewiseJerkSpeed      → 使用StGraphData，生成SpeedData
   ↓ (PathData + SpeedData)
5. CombinePathAndSpeed     → 生成最终Trajectory
```

**依赖关系图**：

```
       PathTask
          ↓
    [PathData]
      ↙     ↘
PathDecider  SpeedBoundsDecider
              ↓
         [StGraphData]
              ↓
         SpeedOptimizer
              ↓
         [SpeedData]
```

---

## 5. 执行流程详解

### 5.1 完整执行流程

```
每个规划周期（100ms）:

Step 1: 消息触发
  └─ PredictionObstacles消息到达
  └─ PlanningComponent::Proc() 被调用

Step 2: 数据准备
  └─ 获取Chassis、Localization等数据
  └─ 构建Frame对象

Step 3: TrafficRules处理
  ├─ Crosswalk (人行道规则)
  ├─ TrafficLight (交通灯规则)
  ├─ StopSign (停止标志规则)
  ├─ Destination (目的地规则)
  └─ ... (其他交通规则)

Step 4: 场景选择
  └─ ScenarioManager::Update()
      ├─ 遍历场景配置列表
      ├─ 调用IsTransferable()判断
      └─ 选择第一个可转入的场景

Step 5: 场景执行
  └─ CurrentScenario::Process()
      └─ CurrentStage::Process()
          └─ ExecuteTaskOnReferenceLine()
              
              循环：对每条参考线 {
                循环：对每个Task {
                  1. Task::Execute()
                  2. 检查执行结果
                  3. 如果失败 → 执行fallback_task
                }
              }

Step 6: 轨迹组合
  └─ 选择最优参考线
  └─ CombinePathAndSpeed()
  └─ 生成DiscretizedTrajectory

Step 7: 输出发布
  └─ 转换为ADCTrajectory
  └─ 发布给Control模块
```

### 5.2 Stage::ExecuteTaskOnReferenceLine() 详解

这是Task执行的核心函数：

```cpp
StageResult Stage::ExecuteTaskOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  
  bool has_drivable_reference_line = false;
  
  // 对每条参考线执行任务
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    
    // 顺序执行Task列表
    for (auto& task : task_list_) {
      
      // 记录开始时间
      const double start_time = Clock::NowInSeconds();
      
      // ========== 执行Task ==========
      auto ret = task->Execute(frame, &reference_line_info);
      
      // 记录结束时间
      const double end_time = Clock::NowInSeconds();
      const double time_diff_ms = (end_time - start_time) * 1000.0;
      
      ADEBUG << "Task [" << task->Name() << "] finished in " 
             << time_diff_ms << " ms";
      
      // 检查Task执行结果
      if (!ret.ok()) {
        AERROR << "Task [" << task->Name() << "] failed: " 
               << ret.error_message();
        
        // 执行fallback任务
        if (fallback_task_) {
          AINFO << "Execute fallback task: " << fallback_task_->Name();
          fallback_task_->Execute(frame, &reference_line_info);
        }
        
        return StageResult(StageStatusType::ERROR, ret);
      }
    }
    
    // 检查参考线是否可行驶
    if (reference_line_info.IsDrivable()) {
      has_drivable_reference_line = true;
    }
  }
  
  // 如果没有可行驶的参考线，返回错误
  if (!has_drivable_reference_line) {
    return StageResult(StageStatusType::ERROR);
  }
  
  return StageResult(StageStatusType::RUNNING);
}
```

### 5.3 任务执行详细流程

以**LaneFollowPath**为例：

```cpp
// Task::Execute() - 基类实现
Status Task::Execute(Frame* frame, ReferenceLineInfo* reference_line_info) {
  // 保存指针供子类使用
  frame_ = frame;
  reference_line_info_ = reference_line_info;
  
  // 调用子类的具体实现
  return Process(frame, reference_line_info);
}

// LaneFollowPath::Process() - 子类实现
Status LaneFollowPath::Process(Frame* frame, ReferenceLineInfo* ref_info) {
  // 1. 检查是否需要规划
  if (!ref_info->path_data().Empty() || ref_info->path_reusable()) {
    ADEBUG << "Skip path planning";
    return Status::OK();
  }
  
  // 2. 计算路径边界
  std::vector<PathBoundary> path_boundaries;
  if (!DecidePathBounds(&path_boundaries)) {
    AERROR << "Failed to decide path bounds";
    return Status::OK();  // 注意：返回OK但不设置路径
  }
  
  // 3. 优化路径
  std::vector<PathData> candidate_path_data;
  if (!OptimizePath(path_boundaries, &candidate_path_data)) {
    AERROR << "Failed to optimize path";
    return Status::OK();
  }
  
  // 4. 评估选择
  PathData final_path;
  if (!AssessPath(&candidate_path_data, &final_path)) {
    AERROR << "Failed to assess path";
    return Status::OK();
  }
  
  // 5. 保存结果
  ref_info->SetPathData(final_path);
  
  return Status::OK();
}
```

---

## 6. 配置文件体系

### 6.1 配置文件层级

```
modules/planning/planning_component/conf/
└─ planning_config.pb.txt                    [总配置]
    └─ 定义启用的Scenarios

modules/planning/scenarios/[scenario_name]/conf/
└─ pipeline.pb.txt                           [场景配置]
    └─ 定义Stages和每个Stage包含的Tasks

modules/planning/scenarios/[scenario_name]/proto/
└─ [scenario_name].proto                     [场景参数]
    └─ 场景特有的配置参数

modules/planning/tasks/[task_name]/conf/
└─ default_conf.pb.txt                       [任务配置]
    └─ Task的具体参数
```

### 6.2 配置示例

#### 6.2.1 总配置

```protobuf
// modules/planning/planning_component/conf/planning_config.pb.txt
standard_planning_config {
  planner_type: PUBLIC_ROAD
  planner_public_road_config {
    // 场景列表（按优先级从高到低）
    scenario {
      name: "VALET_PARKING"
      type: "ValetParkingScenario"
    }
    scenario {
      name: "PULL_OVER"
      type: "PullOverScenario"
    }
    scenario {
      name: "EMERGENCY_PULL_OVER"
      type: "EmergencyPullOverScenario"
    }
    scenario {
      name: "STOP_SIGN_UNPROTECTED"
      type: "StopSignUnprotectedScenario"
    }
    scenario {
      name: "TRAFFIC_LIGHT_PROTECTED"
      type: "TrafficLightProtectedScenario"
    }
    scenario {
      name: "LANE_FOLLOW"
      type: "LaneFollowScenario"
    }
  }
}
```

**注意**：场景顺序很重要！优先级从上到下递减。

#### 6.2.2 场景配置（Scenario Pipeline）

```protobuf
// modules/planning/scenarios/lane_follow/conf/pipeline.pb.txt
stage: {
  name: "LANE_FOLLOW_STAGE"
  type: "LaneFollowStage"
  enabled: true
  
  # 路径规划任务组
  task {
    name: "LANE_CHANGE_PATH"
    type: "LaneChangePath"
  }
  task {
    name: "LANE_FOLLOW_PATH"
    type: "LaneFollowPath"
  }
  task {
    name: "LANE_BORROW_PATH"
    type: "LaneBorrowPath"
  }
  task {
    name: "FALLBACK_PATH"
    type: "FallbackPath"
  }
  task {
    name: "PATH_DECIDER"
    type: "PathDecider"
  }
  
  # 速度规划任务组
  task {
    name: "RULE_BASED_STOP_DECIDER"
    type: "RuleBasedStopDecider"
  }
  task {
    name: "SPEED_BOUNDS_PRIORI_DECIDER"
    type: "SpeedBoundsDecider"
  }
  task {
    name: "SPEED_HEURISTIC_OPTIMIZER"
    type: "PathTimeHeuristicOptimizer"
  }
  task {
    name: "SPEED_DECIDER"
    type: "SpeedDecider"
  }
  task {
    name: "SPEED_BOUNDS_FINAL_DECIDER"
    type: "SpeedBoundsDecider"
  }
  task {
    name: "PIECEWISE_JERK_SPEED"
    type: "PiecewiseJerkSpeedOptimizer"
  }
  
  # 回退任务
  fallback_task {
    name: "FAST_STOP_TRAJECTORY_FALLBACK"
    type: "FastStopTrajectoryFallback"
  }
}
```

#### 6.2.3 任务配置

```protobuf
// modules/planning/tasks/lane_follow_path/conf/default_conf.pb.txt
path_optimizer_config {
  l_weight: 1.0
  dl_weight: 100.0
  ddl_weight: 1000.0
  dddl_weight: 10000.0
  lateral_derivative_bound_default: 0.5
  max_iteration: 4000
}
```

---

## 7. 实战示例

### 7.1 场景：正常行驶遇到前方停止车辆

**完整执行流程**：

```
周期 N (t=0.0s):
=================
Scenario: LaneFollowScenario
Stage: LANE_FOLLOW_STAGE

Task执行顺序：
1. LaneChangePath::Execute()
   └─ 检查变道条件 → 不满足，跳过

2. LaneFollowPath::Execute()
   ├─ DecidePathBounds()
   │   ├─ 车道边界: l ∈ [-1.75, 1.75]
   │   ├─ 检测到前方停止车辆 at s=50m
   │   └─ 调整边界: s∈[45,55] → l ∈ [-1.75, 0.5] (左侧绕行)
   ├─ OptimizePath()
   │   └─ QP优化 → 生成平滑绕行路径
   ├─ AssessPath()
   │   └─ 路径有效
   └─ SetPathData() → PathData存入reference_line_info

3. LaneBorrowPath::Execute()
   └─ 不需要借道，跳过

4. FallbackPath::Execute()
   └─ 主路径有效，跳过

5. PathDecider::Execute()
   └─ 确认使用LaneFollowPath生成的路径

6. RuleBasedStopDecider::Execute()
   └─ 无需停车

7. SpeedBoundsDecider::Execute()
   ├─ 将停止车辆映射到ST图 → STOP边界 at s=50m
   ├─ 计算速度限制
   └─ 生成StGraphData

8. PathTimeHeuristicOptimizer::Execute()
   └─ DP速度优化（可选）

9. SpeedDecider::Execute()
   └─ 速度决策

10. SpeedBoundsDecider::Execute()
    └─ 最终速度边界确认

11. PiecewiseJerkSpeedOptimizer::Execute()
    ├─ QP速度优化
    └─ 生成平滑减速曲线 → SpeedData

结果:
├─ PathData: 绕行路径
└─ SpeedData: 减速曲线

输出:
└─ ADCTrajectory: 平滑的绕行+减速轨迹
```

### 7.2 场景：接近目的地靠边停车

```
T0: LaneFollow场景，正常行驶

T1: 距离目的地45米
  └─ PullOver::IsTransferable() → true
  └─ 场景切换: LaneFollow → PullOver

T2: PullOverScenario::Enter()
  └─ 初始化PullOver上下文
  └─ 寻找合适停车位置

T3: PullOverScenario, Stage: APPROACH
  └─ Tasks:
      ├─ PullOverPath → 规划靠边路径
      ├─ PathDecider → 确认路径
      ├─ SpeedOptimizer → 规划减速停车
      └─ 生成靠边轨迹

T4: 到达停车位置
  └─ Stage切换: APPROACH → FINISHED
  └─ Scenario状态: DONE
  └─ PullOverScenario::Exit()

T5: 场景切换回LaneFollow（或停止）
```

---

## 8. 扩展开发指南

### 8.1 添加自定义Task

**步骤1：创建Task类**

```cpp
// my_custom_task.h
#pragma once
#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {

class MyCustomTask : public Task {
public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
  
  common::Status Execute(Frame* frame,
                        ReferenceLineInfo* reference_line_info) override;
};

// 注册为插件
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::MyCustomTask, Task)

}  // namespace planning
}  // namespace apollo
```

**步骤2：实现Task**

```cpp
// my_custom_task.cc
#include "my_custom_task.h"

namespace apollo {
namespace planning {

bool MyCustomTask::Init(const std::string& config_dir, 
                        const std::string& name,
                        const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // 加载配置
  return Task::LoadConfig<MyCustomTaskConfig>(&config_);
}

Status MyCustomTask::Execute(Frame* frame,
                             ReferenceLineInfo* reference_line_info) {
  // 你的算法实现
  AINFO << "Executing MyCustomTask";
  
  // 示例：访问当前路径
  const PathData& path_data = reference_line_info->path_data();
  
  // 示例：添加决策
  auto* path_decision = reference_line_info->path_decision();
  
  // 示例：访问障碍物
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    ADEBUG << "Obstacle: " << obstacle->Id();
  }
  
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
```

**步骤3：创建配置文件**

```protobuf
// conf/default_conf.pb.txt
my_parameter: 1.0
another_parameter: true
```

**步骤4：添加到Stage**

```protobuf
// 在对应场景的pipeline.pb.txt中添加
task {
  name: "MY_CUSTOM_TASK"
  type: "MyCustomTask"
}
```

### 8.2 添加自定义Stage

```cpp
// my_custom_stage.h
class MyCustomStage : public Stage {
public:
  StageResult Process(const TrajectoryPoint& planning_init_point,
                     Frame* frame) override;
};

// my_custom_stage.cc
StageResult MyCustomStage::Process(const TrajectoryPoint& planning_init_point,
                                   Frame* frame) {
  // 1. 执行任务列表
  auto result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  
  // 2. 检查Stage转换条件
  if (SomeConditionMet()) {
    next_stage_ = "NEXT_STAGE_NAME";
    return StageResult(StageStatusType::FINISHED);
  }
  
  // 3. 继续当前Stage
  return StageResult(StageStatusType::RUNNING);
}

// 注册
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(MyCustomStage, Stage)
```

### 8.3 添加自定义Scenario

```cpp
// my_custom_scenario.h
class MyCustomScenario : public Scenario {
public:
  // 判断是否可以切换到此场景
  bool IsTransferable(const Scenario* other_scenario,
                      const Frame& frame) override;
  
  // 获取场景上下文
  ScenarioContext* GetContext() override { return &context_; }

private:
  MyCustomScenarioContext context_;
};

// my_custom_scenario.cc
bool MyCustomScenario::IsTransferable(const Scenario* other_scenario,
                                      const Frame& frame) {
  // 定义触发条件
  if (frame.某个条件()) {
    return true;
  }
  return false;
}

// 注册
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(MyCustomScenario, Scenario)
```

---

## 9. TrafficRules（交通规则）

### 9.1 什么是TrafficRules

TrafficRules是在**所有Scenario执行之前**运行的规则检查，对所有场景都生效。

**执行位置**：
```cpp
// PublicRoadPlanner::Plan()
Status PublicRoadPlanner::Plan(...) {
  // 1. 先执行TrafficRules
  for (auto& traffic_rule : traffic_rules_) {
    traffic_rule->ApplyRule(frame, reference_line_info);
  }
  
  // 2. 再执行Scenario
  scenario_manager_->Update(frame);
  current_scenario_->Process(planning_init_point, frame);
  
  return Status::OK();
}
```

### 9.2 支持的TrafficRules

| 规则 | 功能 | 效果 |
|------|------|------|
| **Crosswalk** | 人行道 | 行人附近减速/停车 |
| **Destination** | 目的地 | 接近终点时设置停止墙 |
| **KeepClear** | 禁停区 | 禁停区域不能停车 |
| **ReferenceLineEnd** | 参考线终点 | 参考线末端设置停止墙 |
| **Rerouting** | 重路由 | 阻塞时请求重新路由 |
| **StopSign** | 停止标志 | 停止标志前停车 |
| **TrafficLight** | 交通灯 | 红灯停车、绿灯通过 |
| **YieldSign** | 让行标志 | 冲突时让行 |
| **BacksideVehicle** | 后向车辆 | 决定是否忽略后车 |

### 9.3 TrafficRule执行示例

```cpp
// TrafficLight规则
Status TrafficLight::ApplyRule(Frame* frame, 
                               ReferenceLineInfo* reference_line_info) {
  // 1. 检查前方是否有交通灯
  const auto& traffic_lights = frame->GetTrafficLights();
  
  for (const auto& traffic_light : traffic_lights) {
    // 2. 检查信号灯状态
    if (traffic_light.color() == RED || traffic_light.color() == YELLOW) {
      // 3. 计算停止位置
      double stop_line_s = GetStopLineS(traffic_light);
      
      // 4. 创建虚拟停止墙障碍物
      const std::string stop_wall_id = "TL_" + traffic_light.id();
      auto* stop_obstacle = frame->CreateVirtualObstacle(stop_wall_id, stop_line_s);
      
      // 5. 添加停车决策
      auto* path_decision = reference_line_info->path_decision();
      path_decision->AddLongitudinalDecision(
          stop_wall_id, 
          ObjectDecisionType::STOP, 
          stop_line_s - safe_distance);
    }
  }
  
  return Status::OK();
}
```

---

## 10. 关键数据结构

### 10.1 Frame（规划帧）

```cpp
class Frame {
public:
  // 序号
  uint32_t SequenceNum() const;
  
  // 时间戳
  double timestamp() const;
  
  // 车辆状态
  const VehicleState& vehicle_state() const;
  
  // 规划起点
  const TrajectoryPoint& PlanningStartPoint() const;
  
  // 障碍物列表
  const std::vector<const Obstacle*>& obstacles() const;
  
  // 参考线信息（可能有多条）
  std::vector<ReferenceLineInfo>& mutable_reference_line_info();
  
  // 选择最优参考线
  const ReferenceLineInfo* FindDriveReferenceLineInfo();
  
  // 规划上下文
  PlanningContext* mutable_planning_context();
};
```

### 10.2 ReferenceLineInfo（参考线信息）

```cpp
class ReferenceLineInfo {
public:
  // === 基础数据 ===
  const ReferenceLine& reference_line() const;  // 参考线
  const VehicleState& vehicle_state() const;    // 车辆状态
  
  // === 规划结果 ===
  PathData& mutable_path_data();          // 路径数据
  SpeedData& mutable_speed_data();        // 速度数据
  
  // === 决策信息 ===
  PathDecision* path_decision();          // 路径决策
  
  // === 中间数据 ===
  StGraphData* mutable_st_graph_data();   // ST图数据
  
  // === 状态查询 ===
  bool IsDrivable() const;                // 是否可行驶
  bool IsChangeLanePath() const;          // 是否变道路径
  double Cost() const;                    // 代价
  
  // === 巡航速度 ===
  double GetCruiseSpeed() const;          // 获取巡航速度
};
```

### 10.3 DependencyInjector（依赖注入器）

```cpp
class DependencyInjector {
public:
  // 获取车辆信息
  EgoInfo* ego_info();
  
  // 获取规划上下文
  PlanningContext* planning_context();
  
  // 获取历史轨迹
  FrameHistory* frame_history();
  
  // 获取学习数据
  LearningBasedData* learning_based_data();
};
```

**用途**：
- 在Task/Stage/Scenario之间共享数据
- 避免全局变量
- 便于单元测试

---

## 11. 日志和调试

### 11.1 关键日志点

```cpp
// Scenario切换
AINFO << "Scenario changed from [" << old_scenario->Name() 
      << "] to [" << new_scenario->Name() << "]";

// Stage切换  
AINFO << "Stage changed from [" << old_stage 
      << "] to [" << new_stage << "]";

// Task执行
ADEBUG << "Task [" << task->Name() << "] finished in " 
       << time_ms << " ms";

// Task失败
AERROR << "Task [" << task->Name() << "] failed: " 
       << error_message;
```

### 11.2 调试技巧

**查看当前Scenario**：
```bash
grep "current scenario" /apollo/data/log/planning.INFO | tail -1
```

**查看当前Stage**：
```bash
grep "current stage" /apollo/data/log/planning.INFO | tail -1
```

**查看Task执行时间**：
```bash
grep "Task.*finished in" /apollo/data/log/planning.INFO | tail -20
```

**查看失败的Task**：
```bash
grep "Task.*failed" /apollo/data/log/planning.ERROR
```

### 11.3 DreamView可视化

在DreamView中可以实时查看：
- **Current Scenario**: 当前场景名称
- **Current Stage**: 当前阶段名称
- **Planning Status**: 规划状态
- **Path**: 规划的路径（可视化）
- **Speed Profile**: 速度曲线

---

## 12. 常见问题解答

### Q1: Scenario和Planner有什么区别？

**答**：
- **Planner**是更高层的概念，决定**使用哪种规划策略**
  - PublicRoadPlanner（基于高精地图）
  - NaviPlanner（相对地图导航）
  - LatticePlanner（网格算法）

- **Scenario**是Planner内的具体场景
  - 一个Planner可以包含多个Scenario
  - PublicRoadPlanner包含11种Scenario

**关系**：
```
PublicRoadPlanner
  ├─ LaneFollowScenario
  ├─ PullOverScenario
  ├─ StopSignScenario
  └─ ...
```

### Q2: 为什么需要Stage？

**答**：Stage将复杂场景分解为多个阶段，便于：

1. **逻辑清晰**：每个阶段职责单一
2. **状态管理**：便于跟踪执行进度
3. **错误处理**：某阶段失败可以重试或回退
4. **代码复用**：不同阶段可以复用相同Task

**示例**：StopSign场景如果没有Stage：

```
❌ 没有Stage（混乱）:
   └─ 需要在一个大函数里处理所有逻辑
      ├─ 接近、停止、观察、通过全部混在一起
      └─ 难以维护和调试

✅ 有Stage（清晰）:
   ├─ Stage 1: PRE_STOP → 专注于接近和停止
   ├─ Stage 2: STOP → 专注于保持停止和观察
   ├─ Stage 3: CREEP → 专注于蠕行
   └─ Stage 4: CRUISE → 专注于通过
```

### Q3: Task的执行顺序可以改变吗？

**答**：不建议随意改变，因为Task之间有**数据依赖**：

```
正确顺序：
  LaneFollowPath (生成PathData)
    ↓
  SpeedBoundsDecider (使用PathData生成StGraphData)
    ↓
  PiecewiseJerkSpeed (使用StGraphData生成SpeedData)

错误顺序：
  PiecewiseJerkSpeed (需要StGraphData，但还未生成)
    ↓
  SpeedBoundsDecider (现在才生成StGraphData)
    ❌ 前面的Task会失败！
```

**但是**：在某些情况下可以调整无依赖的Task顺序。

### Q4: 一个Task失败了会怎样？

**答**：有多层保护机制：

```
Task失败
  ↓
1. 执行fallback_task（如果配置了）
   └─ FallbackPath / FastStopTrajectoryFallback
  ↓
2. 如果fallback也失败
   └─ Stage返回ERROR状态
  ↓
3. Scenario检测到Stage ERROR
   └─ 可能切换到其他Scenario
  ↓
4. 最终保护
   └─ 生成紧急停车轨迹
```

### Q5: 如何禁用某个Task？

**方法1：在pipeline.pb.txt中注释掉**
```protobuf
# task {
#   name: "SPEED_HEURISTIC_OPTIMIZER"
#   type: "PathTimeHeuristicOptimizer"
# }
```

**方法2：在代码中跳过**
```cpp
Status MyTask::Execute(Frame* frame, ReferenceLineInfo* ref_info) {
  if (FLAGS_disable_my_task) {
    return Status::OK();  // 直接返回成功
  }
  // 正常执行...
}
```

### Q6: 多条参考线是怎么处理的？

**答**：每条参考线**独立执行所有Task**：

```cpp
// Stage::ExecuteTaskOnReferenceLine()
for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
  // 对参考线1执行所有Task
  for (auto& task : task_list_) {
    task->Execute(frame, &reference_line_info);
  }
}

// 最后选择最优参考线
auto* best_ref_info = frame->FindDriveReferenceLineInfo();
```

**示例**：
```
参考线1（直行）:
  └─ 执行所有Task → 生成轨迹1 → 代价 = 100

参考线2（左转）:
  └─ 执行所有Task → 生成轨迹2 → 代价 = 150

参考线3（右转）:
  └─ 执行所有Task → 生成轨迹3 → 代价 = 120

选择: 参考线1（代价最小）
```

---

## 13. 完整执行时序图

```
时间轴 (每100ms一个周期):

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
周期 N:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[PlanningComponent::Proc] ← PredictionObstacles消息触发
    ↓
[OnLanePlanning::RunOnce]
    ↓
[PublicRoadPlanner::Plan]
    ↓
    ├─ [TrafficRules处理] (5-10ms)
    │   ├─ Crosswalk::ApplyRule()
    │   ├─ TrafficLight::ApplyRule()  → 在停止线添加虚拟障碍物
    │   ├─ StopSign::ApplyRule()
    │   └─ Destination::ApplyRule()
    │
    ├─ [ScenarioManager::Update] (1ms)
    │   ├─ 遍历所有Scenario
    │   ├─ 调用IsTransferable()
    │   └─ 选择Scenario → LaneFollowScenario
    │
    └─ [LaneFollowScenario::Process] (80-90ms)
        └─ [LaneFollowStage::Process]
            └─ [ExecuteTaskOnReferenceLine]
                
                对参考线1执行:
                ├─ [LaneFollowPath] (10ms)
                │   └─ PathData生成
                ├─ [PathDecider] (1ms)
                ├─ [SpeedBoundsDecider] (5ms)
                │   └─ StGraphData生成
                ├─ [PiecewiseJerkSpeed] (15ms)
                │   └─ SpeedData生成
                └─ ... (其他Task)
                
                对参考线2执行:
                └─ ... (同样的Task)
                
                选择最优参考线
    ↓
[CombinePathAndSpeed] (5ms)
    └─ PathData + SpeedData → Trajectory
    ↓
[发布ADCTrajectory]

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
总耗时: ~100ms (满足10Hz要求)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## 14. 核心概念对比表

### 14.1 概念对比

| 概念 | 层级 | 数量 | 生命周期 | 主要职责 | 配置位置 |
|------|------|------|---------|---------|---------|
| **Planner** | L1 | 4种 | Planning模块生命周期 | 规划策略选择 | planning_config.pb.txt |
| **Scenario** | L2 | 11种 | 秒-分钟级 | 处理特定驾驶场景 | scenario/conf/pipeline.pb.txt |
| **Stage** | L3 | 2-5个 | 秒-分钟级 | 场景内的执行阶段 | 定义在pipeline.pb.txt |
| **Task** | L4 | 5-15个 | 每周期执行 | 具体算法实现 | task/conf/default_conf.pb.txt |
| **TrafficRule** | 特殊 | 9种 | 每周期执行 | 交通规则检查 | 在所有Scenario前执行 |

### 14.2 代码位置对比

| 概念 | 基类位置 | 实现位置 | 配置位置 |
|------|---------|---------|---------|
| **Scenario** | planning_interface_base/scenario_base/ | scenarios/* | planning_config.pb.txt |
| **Stage** | planning_interface_base/scenario_base/ | scenarios/*/stage_*.cc | scenario/conf/pipeline.pb.txt |
| **Task** | planning_interface_base/task_base/ | tasks/* | task/conf/*.pb.txt |
| **TrafficRule** | planning_base/traffic_rules/ | traffic_rules/* | planning_config.pb.txt |

---

## 15. 快速查找指南

### 15.1 我想知道某个Task的功能

```bash
# 方法1: 查看README
cat modules/planning/tasks/[task_name]/README_cn.md

# 方法2: 查看代码注释
head -50 modules/planning/tasks/[task_name]/[task_name].h

# 方法3: 查看配置参数（推断功能）
cat modules/planning/tasks/[task_name]/conf/default_conf.pb.txt
```

### 15.2 我想知道某个Scenario包含哪些Stage

```bash
cat modules/planning/scenarios/[scenario_name]/conf/pipeline.pb.txt
```

### 15.3 我想知道某个Stage包含哪些Task

```bash
# 在pipeline.pb.txt中查找对应的stage配置
grep -A 50 "name: \"STAGE_NAME\"" modules/planning/scenarios/[scenario_name]/conf/pipeline.pb.txt
```

### 15.4 我想修改Task执行顺序

```bash
# 编辑对应的pipeline.pb.txt
vim modules/planning/scenarios/lane_follow/conf/pipeline.pb.txt

# 调整task的顺序（注意依赖关系）
```

---

## 16. 完整示例：分析LaneFollow场景

### 16.1 查看配置

```bash
# 1. 查看Scenario配置
cat modules/planning/scenarios/lane_follow/conf/pipeline.pb.txt
```

**输出**：
```protobuf
stage: {
  name: "LANE_FOLLOW_STAGE"
  type: "LaneFollowStage"
  enabled: true
  
  # 共11个Task
  task { name: "LANE_CHANGE_PATH", type: "LaneChangePath" }
  task { name: "LANE_FOLLOW_PATH", type: "LaneFollowPath" }
  # ... 其他9个Task
}
```

### 16.2 代码分析

**Scenario代码**：
```cpp
// modules/planning/scenarios/lane_follow/lane_follow_scenario.cc
bool LaneFollowScenario::IsTransferable(const Scenario* other_scenario,
                                        const Frame& frame) {
  // LaneFollow是默认场景，始终可转入
  return true;
}
```

**Stage代码**：
```cpp
// modules/planning/scenarios/lane_follow/lane_follow_stage.cc
StageResult LaneFollowStage::Process(const TrajectoryPoint& planning_start_point,
                                     Frame* frame) {
  // 执行所有Task
  return ExecuteTaskOnReferenceLine(planning_start_point, frame);
}
```

**Task代码**：
```cpp
// modules/planning/tasks/lane_follow_path/lane_follow_path.cc
Status LaneFollowPath::Process(Frame* frame, ReferenceLineInfo* ref_info) {
  // 1. 决策边界
  DecidePathBounds(&path_boundaries);
  
  // 2. 优化路径
  OptimizePath(path_boundaries, &candidate_paths);
  
  // 3. 评估选择
  AssessPath(&candidate_paths, &final_path);
  
  return Status::OK();
}
```

### 16.3 运行流程

```
1. Planning触发
   └─ Proc()被调用

2. Scenario选择
   └─ LaneFollowScenario (默认场景)

3. Stage执行
   └─ LANE_FOLLOW_STAGE

4. Task顺序执行（共11个）:
   
   Task 1: LANE_CHANGE_PATH
     └─ 检查是否需要变道
     └─ 不需要 → 跳过
   
   Task 2: LANE_FOLLOW_PATH
     └─ DecidePathBounds() → PathBoundary
     └─ OptimizePath() → 优化生成路径
     └─ SetPathData() → 保存PathData ✓
   
   Task 3: LANE_BORROW_PATH
     └─ 检查是否需要借道
     └─ 不需要 → 跳过
   
   Task 4: FALLBACK_PATH
     └─ PathData有效 → 跳过
   
   Task 5: PATH_DECIDER
     └─ 确认使用PathData ✓
   
   Task 6: RULE_BASED_STOP_DECIDER
     └─ 检查停车规则 ✓
   
   Task 7: SPEED_BOUNDS_PRIORI_DECIDER
     └─ 构建ST图
     └─ 生成StGraphData ✓
   
   Task 8: SPEED_HEURISTIC_OPTIMIZER
     └─ DP速度优化（可选）
   
   Task 9: SPEED_DECIDER
     └─ 速度决策 ✓
   
   Task 10: SPEED_BOUNDS_FINAL_DECIDER
     └─ 最终速度边界 ✓
   
   Task 11: PIECEWISE_JERK_SPEED
     └─ QP速度优化
     └─ 生成SpeedData ✓

5. 结果汇总
   └─ PathData + SpeedData → Trajectory

6. 输出
   └─ ADCTrajectory发布
```

---

## 17. 设计模式和最佳实践

### 17.1 插件化架构

Apollo Planning采用**插件模式**，所有Scenario、Stage、Task都是插件：

**优势**：
- ✅ **模块解耦**：每个插件独立开发
- ✅ **热插拔**：通过配置文件启用/禁用
- ✅ **易扩展**：添加新功能无需修改核心代码
- ✅ **可测试**：每个插件可独立单元测试

**实现**：
```cpp
// 插件注册宏
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(ClassName, BaseClass)

// 插件加载
auto instance = PluginManager::Instance()->CreateInstance<BaseClass>(class_name);
```

### 17.2 责任链模式

Task的执行采用**责任链模式**：

```
Request (规划请求)
  ↓
Task1 → 处理 → 传递给Task2
  ↓
Task2 → 处理 → 传递给Task3
  ↓
Task3 → 处理 → 传递给Task4
  ↓
...
  ↓
Result (规划结果)
```

### 17.3 状态机模式

Scenario和Stage的切换采用**状态机模式**：

```
State Machine:
  ┌─────────┐  condition1  ┌─────────┐
  │ State A │ ──────────→ │ State B │
  └─────────┘             └─────────┘
      ↑                        │
      │ condition3             │ condition2
      │                        ↓
  ┌─────────┐             ┌─────────┐
  │ State D │ ←────────── │ State C │
  └─────────┘  condition4  └─────────┘
```

### 17.4 依赖注入模式

使用**DependencyInjector**共享数据：

```cpp
// 不使用全局变量
❌ global_ego_info;  // 不好

// 使用依赖注入
✅ injector_->ego_info();  // 好
```

---

## 18. 性能考虑

### 18.1 Task执行时间

典型Task的执行时间（参考值）：

| Task | 耗时 | 占比 |
|------|------|------|
| LaneFollowPath | 10-15ms | 15% |
| SpeedBoundsDecider | 5-8ms | 8% |
| PiecewiseJerkSpeed | 15-20ms | 20% |
| PathDecider | 1-2ms | 2% |
| 其他Tasks | 5-10ms | 10% |
| **总计** | **~80ms** | **80%** |
| 其他开销 | ~20ms | 20% |
| **规划周期总时间** | **~100ms** | **100%** |

### 18.2 优化建议

**如果规划耗时过长**：

1. **禁用非必要Task**：
```protobuf
# 注释掉可选的Task
# task { name: "SPEED_HEURISTIC_OPTIMIZER" ... }
```

2. **减少采样点数**：
```cpp
delta_s = 1.0;  // 从0.5增加到1.0
```

3. **降低优化精度**：
```cpp
max_iteration = 2000;  // 从4000降低
```

4. **使用热启动**：
```cpp
problem.SetWarmStart(previous_solution);
```

---

## 19. 总结

### 19.1 核心要点

**记住这个层级关系**：
```
Planner (策略)
  └─ Scenario (场景) = 一种驾驶情境
      └─ Stage (阶段) = 场景的执行步骤  
          └─ Task (任务) = 具体的算法
```

**记住这个执行流程**：
```
消息触发 → TrafficRules → Scenario选择 → Stage执行 → Task顺序执行 → 结果输出
```

**记住这个依赖关系**：
```
PathTask → PathData → SpeedTask → SpeedData → Trajectory
```

### 19.2 快速上手步骤

**步骤1**：理解三层架构（本文档）

**步骤2**：查看具体场景配置
```bash
cat modules/planning/scenarios/lane_follow/conf/pipeline.pb.txt
```

**步骤3**：阅读关键Task代码
```bash
cat modules/planning/tasks/lane_follow_path/README_cn.md
```

**步骤4**：运行并查看日志
```bash
grep "Scenario\|Stage\|Task" /apollo/data/log/planning.INFO
```

**步骤5**：在DreamView中可视化观察

### 19.3 学习路径建议

```
第1周：理解概念
  └─ Scenario、Stage、Task的定义和关系

第2周：熟悉流程
  └─ 完整执行流程、数据流转

第3周：深入代码
  └─ 阅读LaneFollow场景的所有代码

第4周：实践调试
  └─ 修改参数、添加日志、分析问题

第5周：扩展开发
  └─ 添加自定义Task/Stage/Scenario
```

---

## 20. 参考资源

**代码位置**：
- Scenario: `modules/planning/scenarios/`
- Stage: 在各scenario目录下
- Task: `modules/planning/tasks/`
- 基类: `modules/planning/planning_interface_base/`

**文档**：
- Planning组件README: `modules/planning/planning_component/README_cn.md`
- 各Scenario的README: `modules/planning/scenarios/*/README_cn.md`
- 各Task的README: `modules/planning/tasks/*/README_cn.md`

**可视化工具**：
- DreamView: 实时查看场景、阶段、任务状态
- Cyber_monitor: 查看消息流
- 日志文件: `/apollo/data/log/planning.INFO`

---

**文档版本**: v1.0  
**创建日期**: 2025-10-10  
**适用版本**: Apollo 9.0+  
**维护者**: Apollo Planning Team

**结语**：通过本文档，你应该已经清楚理解了Apollo Planning的核心概念。Scenario-Stage-Task的三层架构是Apollo Planning的精髓，掌握了这些概念，就掌握了Apollo Planning的钥匙！🔑

