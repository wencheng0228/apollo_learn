# Apollo 速度规划模块文档

## 1. 概述

速度规划模块是Apollo规划系统中的核心组件，负责在给定路径的基础上，生成安全、舒适且符合交通规则的速度曲线。速度规划将路径规划的结果转换为ST图（Station-Time图），并在该图上进行优化求解，生成包含位置、速度、加速度和加加速度的完整速度曲线。

### 1.1 触发机制说明

⚠️ **重要**：Apollo速度规划采用**消息触发机制**，而非定时器触发：

- **触发方式**：当收到新的`PredictionObstacles`消息时触发
- **运行频率**：通常为10Hz（100ms周期），由Prediction模块的发布频率决定
- **数据同步**：同时融合最新的`Chassis`和`Localization`消息
- **优势**：保证使用最新数据，减少延迟，提高资源利用率

详细的触发机制说明请参见 [第8.1节：触发机制](#81-触发机制)。

### 1.2 主要功能

- **ST图构建**：将障碍物和车辆状态映射到时间-路径坐标系
- **速度边界计算**：根据障碍物、道路限制和车辆动力学计算速度约束
- **速度优化**：使用优化算法生成平滑、安全的速度曲线
- **多场景支持**：支持跟车、超车、避让、停车等多种驾驶场景

## 2. 核心接口定义

### 2.1 基础类接口

#### 2.1.1 SpeedOptimizer 基类

```cpp
class SpeedOptimizer : public Task {
 public:
  virtual ~SpeedOptimizer() = default;
  
  // 主执行函数
  common::Status Execute(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

 protected:
  // 子类需要实现的核心处理函数
  virtual common::Status Process(const PathData& path_data,
                                 const common::TrajectoryPoint& init_point,
                                 SpeedData* const speed_data) = 0;

  // 调试信息记录
  void RecordDebugInfo(const SpeedData& speed_data);
  void RecordDebugInfo(const SpeedData& speed_data,
                       planning_internal::STGraphDebug* st_graph_debug);
};
```

#### 2.1.2 SpeedData 数据结构

```cpp
class SpeedData : public std::vector<common::SpeedPoint> {
 public:
  SpeedData() = default;
  virtual ~SpeedData() = default;

  explicit SpeedData(std::vector<common::SpeedPoint> speed_points);

  // 添加速度点
  void AppendSpeedPoint(const double s, const double time, 
                        const double v, const double a, const double da);

  // 根据时间评估速度点
  bool EvaluateByTime(const double time,
                      common::SpeedPoint* const speed_point) const;

  // 根据距离评估速度点
  bool EvaluateByS(const double s, 
                   common::SpeedPoint* const speed_point) const;

  // 获取总时间
  double TotalTime() const;

  // 获取总长度
  double TotalLength() const;

  virtual std::string DebugString() const;
};
```

#### 2.1.3 SpeedLimit 速度限制

```cpp
class SpeedLimit {
 public:
  SpeedLimit() = default;

  // 添加速度限制点
  void AppendSpeedLimit(const double s, const double v);

  // 获取所有速度限制点
  const std::vector<std::pair<double, double>>& speed_limit_points() const;

  // 根据距离获取速度限制
  double GetSpeedLimitByS(const double s) const;

  // 清空速度限制
  void Clear();

 private:
  // 速度限制点：(s, v) - 在距离s处的速度限制为v
  std::vector<std::pair<double, double>> speed_limit_points_;
};
```

#### 2.1.4 StGraphData ST图数据

```cpp
class StGraphData {
 public:
  StGraphData() = default;

  // 加载ST图数据
  void LoadData(const std::vector<const STBoundary*>& st_boundaries,
                const double min_s_on_st_boundaries,
                const apollo::common::TrajectoryPoint& init_point,
                const SpeedLimit& speed_limit, 
                const double cruise_speed,
                const double path_data_length, 
                const double total_time_by_conf,
                planning_internal::STGraphDebug* st_graph_debug);

  // 数据访问接口
  bool is_initialized() const;
  const std::vector<const STBoundary*>& st_boundaries() const;
  double min_s_on_st_boundaries() const;
  const apollo::common::TrajectoryPoint& init_point() const;
  const SpeedLimit& speed_limit() const;
  double cruise_speed() const;
  double path_length() const;
  double total_time_by_conf() const;

  // 设置ST可行驶边界
  bool SetSTDrivableBoundary(
      const std::vector<std::tuple<double, double, double>>& s_boundary,
      const std::vector<std::tuple<double, double, double>>& v_obs_info);

  const STDrivableBoundary& st_drivable_boundary() const;

 private:
  bool init_ = false;
  std::vector<const STBoundary*> st_boundaries_;
  double min_s_on_st_boundaries_ = 0.0;
  apollo::common::TrajectoryPoint init_point_;
  SpeedLimit speed_limit_;
  double cruise_speed_ = 0.0;
  double path_data_length_ = 0.0;
  double total_time_by_conf_ = 0.0;
  planning_internal::STGraphDebug* st_graph_debug_ = nullptr;
  STDrivableBoundary st_drivable_boundary_;
};
```

### 2.2 具体优化器接口

#### 2.2.1 PiecewiseJerkSpeedOptimizer（分段加加速度速度优化器）

```cpp
class PiecewiseJerkSpeedOptimizer : public SpeedOptimizer {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
  virtual ~PiecewiseJerkSpeedOptimizer() = default;

 private:
  common::Status Process(const PathData& path_data,
                         const common::TrajectoryPoint& init_point,
                         SpeedData* const speed_data) override;
  
  void AdjustInitStatus(
      const std::vector<std::pair<double, double>> s_dot_bound, 
      double delta_t,
      std::array<double, 3>& init_s);
  
  PiecewiseJerkSpeedOptimizerConfig config_;
};
```

#### 2.2.2 PiecewiseJerkSpeedNonlinearOptimizer（非线性分段加加速度速度优化器）

```cpp
class PiecewiseJerkSpeedNonlinearOptimizer : public SpeedOptimizer {
 public:
  PiecewiseJerkSpeedNonlinearOptimizer();
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
  virtual ~PiecewiseJerkSpeedNonlinearOptimizer() = default;

 private:
  common::Status Process(const PathData& path_data,
                         const common::TrajectoryPoint& init_point,
                         SpeedData* const speed_data) override;

  // 设置状态和边界
  common::Status SetUpStatesAndBounds(const PathData& path_data,
                                      const SpeedData& speed_data);

  // 检查速度限制可行性
  bool CheckSpeedLimitFeasibility();

  // 平滑速度限制
  common::Status SmoothSpeedLimit();

  // 平滑路径曲率
  common::Status SmoothPathCurvature(const PathData& path_data);

  // 二次规划优化
  common::Status OptimizeByQP(SpeedData* const speed_data,
                              std::vector<double>* distance,
                              std::vector<double>* velocity,
                              std::vector<double>* acceleration);

  // 非线性规划优化
  common::Status OptimizeByNLP(std::vector<double>* distance,
                               std::vector<double>* velocity,
                               std::vector<double>* acceleration);
};
```

#### 2.2.3 PathTimeHeuristicOptimizer（动态规划速度优化器）

```cpp
class PathTimeHeuristicOptimizer : public SpeedOptimizer {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  common::Status Process(const PathData& path_data,
                         const common::TrajectoryPoint& init_point,
                         SpeedData* const speed_data) override;

  // 在路径-时间图上搜索最优路径
  bool SearchPathTimeGraph(SpeedData* speed_data) const;

 private:
  common::TrajectoryPoint init_point_;
  SLBoundary adc_sl_boundary_;
  SpeedHeuristicOptimizerConfig config_;
};
```

### 2.3 决策器接口

#### 2.3.1 SpeedBoundsDecider（速度边界决策器）

```cpp
class SpeedBoundsDecider : public Decider {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  common::Status Process(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info) override;

  // 设置速度回退距离
  double SetSpeedFallbackDistance(PathDecision* const path_decision);

  // 记录ST图调试信息
  void RecordSTGraphDebug(
      const StGraphData& st_graph_data,
      planning_internal::STGraphDebug* st_graph_debug) const;

  SpeedBoundsDeciderConfig config_;
};
```

## 3. 数据结构

### 3.1 SpeedPoint（速度点）

```protobuf
message SpeedPoint {
  optional double s = 1;      // 累计距离 (m)
  optional double t = 2;      // 时间 (s)
  optional double v = 3;      // 速度 (m/s)
  optional double a = 4;      // 加速度 (m/s²)
  optional double da = 5;     // 加加速度 (m/s³)
}
```

### 3.2 TrajectoryPoint（轨迹点）

```protobuf
message TrajectoryPoint {
  optional PathPoint path_point = 1;      // 路径点
  optional double v = 2;                  // 线速度 (m/s)
  optional double a = 3;                  // 线加速度 (m/s²)
  optional double relative_time = 4;      // 相对时间 (s)
  optional double da = 5;                 // 纵向加加速度 (m/s³)
  optional double steer = 6;              // 方向盘转角
  optional GaussianInfo gaussian_info = 7; // 高斯概率信息
}
```

### 3.3 STBoundary（ST边界）

ST边界用于表示障碍物在ST图上的占据区域，包括以下类型：

```cpp
enum BoundaryType {
  UNKNOWN = 0,
  STOP = 1,        // 停止边界
  FOLLOW = 2,      // 跟车边界
  YIELD = 3,       // 让行边界
  OVERTAKE = 4,    // 超车边界
  KEEP_CLEAR = 5   // 保持清空边界
};
```

## 4. 输入输出

### 4.1 规划范围说明

⚠️ **重要概念**：Apollo速度规划是**固定时间范围规划**，而非全路径规划。

#### 规划范围特性

**时间范围（固定）**：
- 规划时域：`FLAGS_trajectory_time_length = 8.0` 秒（默认）
- ST图时间：`st_max_t = 8.0` 秒
- **每次规划固定覆盖未来8秒**

**空间范围（动态）**：
- 规划距离取决于优化结果，**不是固定值**
- 受限于路径长度：`path_data_length`
- ST图最大距离：`st_max_s = 100` 米
- **实际规划距离 = 8秒内能行驶的距离**

#### 末端条件

**默认情况（非目的地）**：
```cpp
// 参考值设置
std::vector<double> x_ref(num_of_knots, total_length);     // 期望走完全部路径
std::vector<double> dx_ref(num_of_knots, cruise_speed);    // 期望保持巡航速度
```

- 末端速度 **≠ 0**
- 末端速度 **≈ cruise_speed**（巡航速度，通常5-10 m/s）
- 鼓励车辆保持稳定速度

**特殊情况（需要停车）**：

只有在以下场景，末端速度才为0：

1. **到达目的地**
   ```cpp
   if (reference_line_info_->ReachedDestination()) {
     // 规划停止轨迹
     return Status::OK();
   }
   ```

2. **遇到STOP边界**
   - ST边界类型为 `STOP` 或 `YIELD`
   - 在障碍物前停车

3. **交通规则要求**
   - 红灯停车
   - STOP标志
   - 其他需要停车的场景

#### 为什么这样设计？

**优势**：

1. **滚动优化**
   - 每个规划周期（100ms）重新规划未来8秒
   - 类似"滑动窗口"，不断向前滚动
   - 始终有足够的前瞻距离

2. **计算效率**
   - 固定时间范围，计算量可控
   - 避免规划过长导致实时性问题

3. **动态适应**
   - 每次规划使用最新的感知和预测数据
   - 及时响应环境变化

4. **平滑衔接**
   - 保持末端速度便于下一周期轨迹拼接
   - 避免频繁加减速

#### 示例说明

**场景1：正常巡航**
```
当前时刻: t = 0s
规划范围: t = [0, 8s]
路径长度: 200m
巡航速度: 10 m/s

规划结果:
- 时间: 0-8s (固定)
- 距离: 约 80m (8s × 10m/s)
- 末端速度: ≈ 10 m/s (保持巡航)
- 末端距离: 80m (还有120m未规划)

下一周期(100ms后):
- 时间: 0.1-8.1s (窗口滑动)
- 重新规划未来8秒
```

**场景2：接近目的地**
```
当前时刻: t = 0s
距离目的地: 30m
巡航速度: 10 m/s

规划结果:
- 时间: 0-8s
- 距离: 0-30m (在目的地前停止)
- 末端速度: 0 m/s (到达目的地)
- 减速曲线: 平滑减速到0
```

**场景3：前方红灯**
```
当前时刻: t = 0s
距离停止线: 50m
巡航速度: 10 m/s

规划结果:
- 时间: 0-8s
- 距离: 0-50m (在停止线前停止)
- 末端速度: 0 m/s (红灯停车)
- ST边界: STOP类型，s = 50m
```

### 4.2 输入

#### 4.2.1 主要输入

1. **PathData（路径数据）**
   - 离散化路径点序列
   - 路径长度
   - 路径曲率信息

2. **初始状态（TrajectoryPoint）**
   - 当前位置 (x, y, s)
   - 当前速度 v
   - 当前加速度 a
   - 当前航向角 θ

3. **参考线信息（ReferenceLineInfo）**
   - 参考线
   - 路径决策信息
   - 巡航速度

4. **Frame（帧信息）**
   - 车辆状态
   - 障碍物信息
   - 规划起点

#### 4.2.2 约束条件输入

1. **ST边界（STBoundaries）**
   - 障碍物的ST边界
   - 边界类型（STOP/FOLLOW/YIELD/OVERTAKE）
   - 边界点序列

2. **速度限制（SpeedLimit）**
   - 地图速度限制
   - 路径曲率限制
   - 避让障碍物速度限制

3. **动力学约束**
   - 最大速度 v_max
   - 最大加速度 a_max
   - 最小加速度 a_min（最大减速度）
   - 最大加加速度 jerk_max
   - 最小加加速度 jerk_min

4. **配置参数**
   - 规划时域 total_time
   - 规划步长 delta_t
   - 权重系数（平滑性、舒适性、效率）

### 4.3 输出

#### 4.3.1 主要输出

**SpeedData（速度曲线）**

包含一系列SpeedPoint，描述完整的速度规划结果：

```cpp
std::vector<SpeedPoint> speed_points;
// 每个点包含：
// - s: 累计距离
// - t: 时间戳
// - v: 速度
// - a: 加速度
// - da: 加加速度
```

#### 4.3.2 输出特性

1. **时间域**：从当前时刻到规划时域末端（通常6-8秒）
2. **空间域**：从当前位置到规划路径终点
3. **采样频率**：通常为10Hz（delta_t = 0.1s）
4. **平滑性**：保证速度、加速度和加加速度连续

#### 4.3.3 输出约束保证

- 满足所有动力学约束
- 避免与障碍物的ST边界碰撞
- 符合速度限制要求
- 满足初始状态连续性

### 4.4 数据流图

```
输入数据流:
Frame → [车辆状态, 障碍物] → SpeedBoundsDecider
PathData → [路径点序列] → SpeedOptimizer
ReferenceLineInfo → [参考线, 决策] → SpeedOptimizer
TrajectoryPoint → [初始状态] → SpeedOptimizer

处理流程:
SpeedBoundsDecider:
  ├─ 构建ST图
  ├─ 映射障碍物边界
  └─ 计算速度限制
     ↓
  StGraphData → SpeedOptimizer:
     ├─ 提取约束边界
     ├─ 构建优化问题
     ├─ 求解优化问题
     └─ 生成速度曲线
        ↓
     SpeedData → 输出

输出数据流:
SpeedData → [速度点序列] → TrajectoryStitcher → ADCTrajectory
```

## 5. 算法原理

### 5.1 总体流程

速度规划的核心是将三维的时空规划问题转化为ST图上的优化问题：

```
1. ST图构建
   └─ 将障碍物和车辆映射到Station-Time坐标系

2. 边界计算
   ├─ 障碍物ST边界
   ├─ 速度限制边界
   └─ 动力学约束边界

3. 优化求解
   ├─ 构建代价函数
   ├─ 添加约束条件
   └─ 求解优化问题

4. 速度曲线生成
   └─ 输出SpeedData
```

### 5.2 主要算法

#### 5.2.1 动态规划算法（DP）

**算法：PathTimeHeuristicOptimizer**

**原理**：
- 将ST图离散化为网格
- 从起点开始逐层向前搜索
- 使用动态规划找到代价最小的路径

**代价函数**：
```
cost = w1 × obstacle_cost     // 障碍物代价
     + w2 × speed_cost        // 速度偏差代价
     + w3 × acceleration_cost // 加速度代价
     + w4 × jerk_cost         // 加加速度代价
```

**优点**：
- 能够处理非凸问题
- 保证找到全局最优解（在离散空间内）
- 适合复杂交通场景

**缺点**：
- 计算量大
- 解的平滑性依赖于离散化精度

#### 5.2.2 二次规划算法（QP）

**算法：PiecewiseJerkSpeedOptimizer**

**原理**：
将速度规划问题建模为二次规划问题：

**标准QP形式**：
```
minimize:  (1/2) × x^T × H × x + f^T × x

subject to:
  LB ≤ x ≤ UB           (变量边界约束)
  A_eq × x = b_eq       (等式约束)
  A × x ≤ b             (不等式约束)
```

**决策变量**：
```
x = [s_0, v_0, a_0, s_1, v_1, a_1, ..., s_n, v_n, a_n]^T
```

**目标函数**：

完整的目标函数包含以下各项（从代码中提取）：

$$
\begin{aligned}
J(\mathbf{x}) = &\underbrace{\sum_{i=0}^{n-1} w_{s\_ref} (s_i - s_{ref,i})^2}_{\text{位置参考跟踪代价}} \\
&+ \underbrace{\sum_{i=0}^{n-1} (w_{v\_ref,i} + penalty_{v,i}) (v_i - v_{ref,i})^2}_{\text{速度参考跟踪代价}} \\
&+ \underbrace{\sum_{i=0}^{n-1} w_{acc} a_i^2}_{\text{加速度平滑代价}} \\
&+ \underbrace{\sum_{i=0}^{n-2} w_{jerk} \left(\frac{a_{i+1} - a_i}{\Delta t}\right)^2}_{\text{加加速度（Jerk）代价}} \\
&+ \underbrace{w_{end\_s}(s_{n-1} - s_{end\_ref})^2}_{\text{末端位置代价}} \\
&+ \underbrace{w_{end\_v}(v_{n-1} - v_{end\_ref})^2}_{\text{末端速度代价}} \\
&+ \underbrace{w_{end\_a}(a_{n-1} - a_{end\_ref})^2}_{\text{末端加速度代价}}
\end{aligned}
$$

**各项详细解释**：

**1. 位置参考跟踪代价** (\(w_{s\_ref} \sum (s_i - s_{ref,i})^2\))
```cpp
// 代码位置：CalculateKernel()
columns[i].emplace_back(i, weight_x_ref_ / (scale_factor_[0] * scale_factor_[0]));

// 对应线性项：CalculateOffset()
q->at(i) += -2.0 * weight_x_ref_ * x_ref_[i] / scale_factor_[0];
```
- **含义**：使车辆尽量跟随参考s曲线
- **权重**：`weight_x_ref` (通常为10.0)
- **参考值**：`x_ref_[i]` 通常设为期望到达的距离
- **效果**：鼓励车辆尽量前进

**2. 速度参考跟踪代价** (\(\sum (w_{v\_ref,i} + penalty_{v,i}) (v_i - v_{ref,i})^2\))
```cpp
// 代码位置：CalculateKernel()
columns[n + i].emplace_back(n + i,
    (weight_dx_ref_[i] + penalty_dx_[i]) / (scale_factor_[1] * scale_factor_[1]));

// 对应线性项：CalculateOffset()
q->at(n + i) += -2.0 * weight_dx_ref_[i] * dx_ref_[i] / scale_factor_[1];
```
- **含义**：使速度尽量接近参考速度
- **权重**：`weight_dx_ref_[i]` (通常为10.0)
- **额外惩罚**：`penalty_dx_[i]` 在曲率大的地方增加（鼓励减速）
- **参考值**：`dx_ref_[i]` 通常设为巡航速度或速度限制
- **效果**：保持稳定的巡航速度

**3. 加速度平滑代价** (\(w_{acc} \sum a_i^2\))
```cpp
// 代码位置：CalculateKernel()
// 第一个点和最后一个点
columns[2*n].emplace_back(2*n, 
    (weight_ddx_ + weight_dddx_/delta_s_square) / (scale_factor_[2] * scale_factor_[2]));

// 中间点（包含Jerk项的耦合）
columns[2*n + i].emplace_back(2*n + i, 
    (weight_ddx_ + 2.0*weight_dddx_/delta_s_square) / (scale_factor_[2] * scale_factor_[2]));
```
- **含义**：惩罚大的加速度，提高乘坐舒适性
- **权重**：`weight_ddx_` (通常为100.0-500.0)
- **效果**：减少急加速和急减速

**4. 加加速度（Jerk）代价** (\(w_{jerk} \sum (jerk_i)^2\))
```cpp
// 代码位置：CalculateKernel()
// Jerk的定义：jerk_i = (a_{i+1} - a_i) / Δt
// 展开为：jerk_i² = (a_{i+1}² - 2*a_{i+1}*a_i + a_i²) / Δt²

// 对角项贡献（已包含在加速度项中）
weight_dddx_ / delta_s_square  // 端点
2.0 * weight_dddx_ / delta_s_square  // 中间点

// 非对角项（耦合项）
columns[2*n + i].emplace_back(2*n + i + 1,
    -2.0 * weight_dddx_ / delta_s_square / (scale_factor_[2] * scale_factor_[2]));
```
- **含义**：惩罚加速度的变化率，提高平滑性
- **权重**：`weight_dddx_` (通常为100.0-1000.0)
- **效果**：避免加速度突变，最重要的平滑性指标

**5. 末端状态代价** (末端约束)
```cpp
// 代码位置：CalculateKernel() - 末端点额外权重
columns[n - 1] += weight_end_state_[0]  // 末端位置
columns[2*n - 1] += weight_end_state_[1]  // 末端速度
columns[3*n - 1] += weight_end_state_[2]  // 末端加速度

// 对应线性项：CalculateOffset()
q->at(n - 1) += -2.0 * weight_end_state_[0] * end_state_ref_[0];
q->at(2*n - 1) += -2.0 * weight_end_state_[1] * end_state_ref_[1];
q->at(3*n - 1) += -2.0 * weight_end_state_[2] * end_state_ref_[2];
```
- **含义**：约束末端状态接近期望值
- **权重**：`weight_end_state_` (通常为10.0)
- **参考值**：`end_state_ref_` (通常为 {s_end, v_cruise, 0})
- **效果**：确保末端状态合理

**6. 曲率惩罚项** (\(penalty_{v,i}\))
```cpp
// 代码中penalty_dx的计算：
penalty_dx_[i] = std::fabs(path_point.kappa()) * config.kappa_penalty_weight();
```
- **含义**：在曲率大的地方增加速度惩罚，鼓励减速
- **权重**：`kappa_penalty_weight` (通常为100.0)
- **效果**：弯道自动减速

**QP标准形式的完整矩阵表示**：

$$
J(\mathbf{x}) = \frac{1}{2} \mathbf{x}^T \mathbf{H} \mathbf{x} + \mathbf{f}^T \mathbf{x}
$$

其中决策变量：
$$
\mathbf{x} = [s_0, s_1, \ldots, s_{n-1}, v_0, v_1, \ldots, v_{n-1}, a_0, a_1, \ldots, a_{n-1}]^T
$$

**Hessian矩阵 H 的构造（3n × 3n）**：

$$
\mathbf{H} = \begin{bmatrix}
\mathbf{H}_s & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{H}_v & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{H}_a + \mathbf{H}_{jerk}
\end{bmatrix}
$$

其中：
- $\mathbf{H}_s = \text{diag}(w_{s\_ref}, \ldots, w_{s\_ref} + w_{end\_s})$ (位置权重)
- $\mathbf{H}_v = \text{diag}(w_{v\_ref,0} + penalty_0, \ldots, w_{v\_ref,n-1} + penalty_{n-1} + w_{end\_v})$ (速度权重)
- $\mathbf{H}_a = \text{diag}(w_{acc} + w_{jerk}/\Delta t^2, \ldots, w_{acc} + 2w_{jerk}/\Delta t^2, \ldots, w_{acc} + w_{jerk}/\Delta t^2 + w_{end\_a})$ (加速度权重)
- $\mathbf{H}_{jerk}$ 是三对角矩阵，表示Jerk的耦合项

**Jerk耦合矩阵**：
$$
\mathbf{H}_{jerk} = \frac{w_{jerk}}{\Delta t^2} \begin{bmatrix}
1 & -1 & 0 & \cdots & 0 \\
-1 & 2 & -1 & \cdots & 0 \\
0 & -1 & 2 & \ddots & \vdots \\
\vdots & \ddots & \ddots & \ddots & -1 \\
0 & \cdots & 0 & -1 & 1
\end{bmatrix}_{n \times n}
$$

**线性项 f 的构造（3n × 1）**：

$$
\mathbf{f} = \begin{bmatrix}
-w_{s\_ref} s_{ref,0}, \ldots, -w_{s\_ref} s_{ref,n-2}, -(w_{s\_ref} s_{ref,n-1} + w_{end\_s} s_{end\_ref}) \\
-w_{v\_ref,0} v_{ref,0}, \ldots, -(w_{v\_ref,n-1} v_{ref,n-1} + w_{end\_v} v_{end\_ref}) \\
0, 0, \ldots, -w_{end\_a} a_{end\_ref}
\end{bmatrix}^T
$$

**代码实现对应**：

```cpp
// Hessian矩阵H的构造
void PiecewiseJerkSpeedProblem::CalculateKernel(...) {
  // 1. s的对角元素
  for (int i = 0; i < n - 1; ++i) {
    H[i,i] = weight_x_ref_;
  }
  H[n-1, n-1] = weight_x_ref_ + weight_end_state_[0];
  
  // 2. v的对角元素（包含曲率惩罚）
  for (int i = 0; i < n - 1; ++i) {
    H[n+i, n+i] = weight_dx_ref_[i] + penalty_dx_[i];
  }
  H[2n-1, 2n-1] = weight_dx_ref_[n-1] + penalty_dx_[n-1] + weight_end_state_[1];
  
  // 3. a的对角元素（包含Jerk贡献）
  H[2n, 2n] = weight_ddx_ + weight_dddx_/Δt²;  // 第一个点
  for (int i = 1; i < n - 1; ++i) {
    H[2n+i, 2n+i] = weight_ddx_ + 2.0*weight_dddx_/Δt²;  // 中间点
  }
  H[3n-1, 3n-1] = weight_ddx_ + weight_dddx_/Δt² + weight_end_state_[2];  // 最后点
  
  // 4. Jerk的非对角元素
  for (int i = 0; i < n - 1; ++i) {
    H[2n+i, 2n+i+1] = -2.0 * weight_dddx_/Δt²;  // a_i 和 a_{i+1} 的耦合
  }
}

// 线性项f的构造
void PiecewiseJerkSpeedProblem::CalculateOffset(std::vector<c_float>* q) {
  // 1. s的线性项
  for (int i = 0; i < n; ++i) {
    q[i] = -2.0 * weight_x_ref_ * x_ref_[i];
  }
  
  // 2. v的线性项
  for (int i = 0; i < n; ++i) {
    q[n + i] = -2.0 * weight_dx_ref_[i] * dx_ref_[i];
  }
  
  // 3. 末端状态线性项
  q[n - 1] += -2.0 * weight_end_state_[0] * end_state_ref_[0];
  q[2*n - 1] += -2.0 * weight_end_state_[1] * end_state_ref_[1];
  q[3*n - 1] += -2.0 * weight_end_state_[2] * end_state_ref_[2];
}
```

**数值示例**（假设n=3个点）：

输入参数：
```cpp
weight_x_ref = 10.0
weight_dx_ref = [10.0, 10.0, 10.0]
weight_ddx = 100.0
weight_dddx = 1000.0
weight_end_state = [10.0, 10.0, 10.0]
penalty_dx = [0.0, 50.0, 0.0]  // 中间点曲率大
Δt = 0.1s
```

则Hessian矩阵对角元素：
```
H[0,0] = 10.0 (s_0)
H[1,1] = 10.0 (s_1)
H[2,2] = 20.0 (s_2, 包含末端权重)

H[3,3] = 10.0 (v_0)
H[4,4] = 60.0 (v_1, 包含曲率惩罚50.0)
H[5,5] = 20.0 (v_2, 包含末端权重)

H[6,6] = 100.0 + 1000.0/0.01 = 100100.0 (a_0)
H[7,7] = 100.0 + 2*1000.0/0.01 = 200100.0 (a_1)
H[8,8] = 100.0 + 1000.0/0.01 + 10.0 = 100110.0 (a_2)

H[6,7] = H[7,6] = -2*1000.0/0.01 = -200000.0 (a_0与a_1耦合)
H[7,8] = H[8,7] = -200000.0 (a_1与a_2耦合)
```

**约束条件**：

1. **运动学约束**：
```cpp
// 代码位置：CalculateAffineConstraint()

// 速度连续性：v_{i+1} = v_i + 0.5*Δt*a_i + 0.5*Δt*a_{i+1}
// 代码：
variables[n + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
variables[n + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);
variables[2*n + i].emplace_back(constraint_index, -0.5 * delta_s_ * scale_factor_[1]);
variables[2*n + i + 1].emplace_back(constraint_index, -0.5 * delta_s_ * scale_factor_[1]);
```

$$
v_{i+1} = v_i + \frac{1}{2}(a_i + a_{i+1}) \Delta t
$$

```cpp
// 位置连续性：s_{i+1} = s_i + Δt*v_i + (1/3)Δt²*a_i + (1/6)Δt²*a_{i+1}
// 代码：
variables[i].emplace_back(constraint_index, -1.0 * scale_factor_[1] * scale_factor_[2]);
variables[i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[1] * scale_factor_[2]);
variables[n + i].emplace_back(constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);
variables[2*n + i].emplace_back(constraint_index, -delta_s_sq_/3.0 * scale_factor_[0] * scale_factor_[1]);
variables[2*n + i + 1].emplace_back(constraint_index, -delta_s_sq_/6.0 * scale_factor_[0] * scale_factor_[1]);
```

$$
s_{i+1} = s_i + v_i \Delta t + \frac{1}{3} a_i \Delta t^2 + \frac{1}{6} a_{i+1} \Delta t^2
$$

2. **动力学边界约束**：

```cpp
// 代码位置：CalculateAffineConstraint() - 变量边界
// s的边界
for (int i = 0; i < n; ++i) {
  lower_bounds[i] = x_bounds_[i].first;   // s_min (通常为0)
  upper_bounds[i] = x_bounds_[i].second;  // s_max (路径长度)
}

// v的边界
for (int i = 0; i < n; ++i) {
  lower_bounds[n + i] = dx_bounds_[i].first;   // v_min (通常为0)
  upper_bounds[n + i] = dx_bounds_[i].second;  // v_max (速度限制)
}

// a的边界
for (int i = 0; i < n; ++i) {
  lower_bounds[2*n + i] = ddx_bounds_[i].first;   // a_min (最大减速度)
  upper_bounds[2*n + i] = ddx_bounds_[i].second;  // a_max (最大加速度)
}
```

$$
\begin{aligned}
0 &\leq s_i \leq s_{max}(t_i) \\
0 &\leq v_i \leq v_{max}(t_i) \\
a_{min} &\leq a_i \leq a_{max}
\end{aligned}
$$

3. **加加速度（Jerk）约束**：

```cpp
// 代码位置：CalculateAffineConstraint()
// jerk_i = (a_{i+1} - a_i) / Δt
for (int i = 0; i + 1 < n; ++i) {
  variables[2*n + i].emplace_back(constraint_index, -1.0);
  variables[2*n + i + 1].emplace_back(constraint_index, 1.0);
  lower_bounds[constraint_index] = dddx_bound_.first * delta_s_;   // jerk_min * Δt
  upper_bounds[constraint_index] = dddx_bound_.second * delta_s_;  // jerk_max * Δt
}
```

$$
jerk_{min} \Delta t \leq a_{i+1} - a_i \leq jerk_{max} \Delta t
$$

4. **ST边界约束**：

ST边界通过**变量边界**实现，而非单独的约束：

```cpp
// 在PiecewiseJerkSpeedOptimizer::Process()中设置
for (int i = 0; i < num_of_knots; ++i) {
  double curr_t = i * delta_t;
  double s_lower_bound = 0.0;
  double s_upper_bound = total_length;
  
  // 遍历所有ST边界
  for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
    double s_lower = 0.0;
    double s_upper = 0.0;
    if (!boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
      continue;
    }
    
    switch (boundary->boundary_type()) {
      case STBoundary::STOP:
      case STBoundary::YIELD:
        s_upper_bound = std::fmin(s_upper_bound, s_upper);  // 不能超过障碍物
        break;
      case STBoundary::FOLLOW:
        s_upper_bound = std::fmin(s_upper_bound, s_upper);
        break;
      case STBoundary::OVERTAKE:
        s_lower_bound = std::fmax(s_lower_bound, s_lower);  // 必须超过障碍物
        break;
    }
  }
  
  s_bounds[i] = {s_lower_bound, s_upper_bound};
}

// 然后设置到QP问题
qp_problem.set_x_bounds(s_bounds);
```

$$
\text{对于STOP/YIELD边界: } s_i \leq s_{boundary\_upper}(t_i)
$$
$$
\text{对于OVERTAKE边界: } s_i \geq s_{boundary\_lower}(t_i)
$$

5. **速度限制约束**：

```cpp
// 在PiecewiseJerkSpeedOptimizer::Process()中设置
const SpeedLimit& speed_limit = st_graph_data.speed_limit();
std::vector<std::pair<double, double>> s_dot_bounds;

for (int i = 0; i < num_of_knots; ++i) {
  double curr_t = i * delta_t;
  SpeedPoint sp;
  reference_speed_data.EvaluateByTime(curr_t, &sp);
  const double path_s = sp.s();
  
  // 获取该位置的速度限制
  double v_upper_bound = FLAGS_planning_upper_speed_limit;
  v_upper_bound = std::fmin(speed_limit.GetSpeedLimitByS(path_s), v_upper_bound);
  
  s_dot_bounds.emplace_back(0.0, std::fmax(v_upper_bound, 0.0));
}

// 设置到QP问题
qp_problem.set_dx_bounds(std::move(s_dot_bounds));
```

$$
0 \leq v_i \leq \min(v_{map}(s_i), v_{curvature}(s_i), v_{upper\_limit})
$$

6. **初始状态约束**：

```cpp
// 代码位置：CalculateAffineConstraint()
// 通过等式约束实现
variables[0].emplace_back(constraint_index, 1.0);
lower_bounds[constraint_index] = x_init_[0];  // s_0 = s_init
upper_bounds[constraint_index] = x_init_[0];

variables[n].emplace_back(constraint_index, 1.0);
lower_bounds[constraint_index] = x_init_[1];  // v_0 = v_init
upper_bounds[constraint_index] = x_init_[1];

variables[2*n].emplace_back(constraint_index, 1.0);
lower_bounds[constraint_index] = x_init_[2];  // a_0 = a_init
upper_bounds[constraint_index] = x_init_[2];
```

$$
\begin{cases}
s_0 = s_{init} \\
v_0 = v_{init} \\
a_0 = a_{init}
\end{cases}
$$

**完整的QP问题构造总结**：

```cpp
// 从代码层面理解QP问题的完整构造过程

// Step 1: 创建QP问题
PiecewiseJerkSpeedProblem qp_problem(num_of_knots, delta_t, init_s);

// Step 2: 设置目标函数权重
qp_problem.set_weight_ddx(config_.acc_weight());           // w_acc = 100.0
qp_problem.set_weight_dddx(config_.jerk_weight());         // w_jerk = 1000.0

// Step 3: 设置位置参考（鼓励前进）
std::vector<double> x_ref(num_of_knots, total_length);    // 期望走完全程
qp_problem.set_x_ref(config_.ref_s_weight(), x_ref);      // w_s_ref = 10.0

// Step 4: 设置速度参考（保持巡航）
std::vector<double> dx_ref(num_of_knots, cruise_speed);   // 期望巡航速度
std::vector<double> dx_ref_weight(num_of_knots, config_.ref_v_weight());  // w_v_ref = 10.0
qp_problem.set_dx_ref(dx_ref_weight, dx_ref);

// Step 5: 设置曲率惩罚（弯道减速）
std::vector<double> penalty_dx(num_of_knots);
for (int i = 0; i < num_of_knots; ++i) {
  PathPoint path_point = path_data.GetPathPointWithPathS(s_i);
  penalty_dx[i] = std::fabs(path_point.kappa()) * config_.kappa_penalty_weight();
}
qp_problem.set_penalty_dx(penalty_dx);

// Step 6: 设置变量边界约束
qp_problem.set_x_bounds(s_bounds);        // 位置边界（ST边界）
qp_problem.set_dx_bounds(v_bounds);       // 速度边界（速度限制）
qp_problem.set_ddx_bounds(a_min, a_max);  // 加速度边界（动力学）
qp_problem.set_dddx_bound(jerk_bound);    // Jerk边界

// Step 7: 求解
bool success = qp_problem.Optimize();

// Step 8: 提取结果
const std::vector<double>& s = qp_problem.opt_x();
const std::vector<double>& v = qp_problem.opt_dx();
const std::vector<double>& a = qp_problem.opt_ddx();
```

**QP问题规模**（以80个点为例）：

```
决策变量数: 3 × 80 = 240
  ├─ s: 80个
  ├─ v: 80个
  └─ a: 80个

约束数量: 
  ├─ 变量边界: 240个
  ├─ 初始状态: 3个
  ├─ 速度连续性: 79个
  ├─ 位置连续性: 79个
  ├─ Jerk约束: 79个
  └─ 总计: ~480个约束

Hessian矩阵非零元: 
  ├─ 对角元素: 240个
  ├─ Jerk耦合: 79个
  └─ 总计: 319个非零元（稀疏矩阵）

OSQP求解时间: 5-20ms（取决于初值和约束）
```

**优点**：
- 求解速度快（凸二次规划）
- 解的质量高，平滑性好
- 适合实时系统
- 代码实现清晰，易于调试

**缺点**：
- 只能处理凸问题
- 需要合理的初始解
- 无法处理复杂的非线性约束（需要NLP）

#### 5.2.3 非线性优化算法（NLP）

**算法：PiecewiseJerkSpeedNonlinearOptimizer**

**原理**：
结合QP和NLP的优势，先用QP得到初始解，再用NLP精细优化。

**两阶段优化**：

**阶段1：QP优化**
- 忽略非线性约束（如向心加速度约束）
- 快速获得可行的初始解

**阶段2：NLP优化（使用IPOPT求解器）**

添加非线性约束：

1. **向心加速度约束**：
```
a_lateral = v² × κ(s) ≤ a_lateral_max
```
其中 κ(s) 是路径曲率

2. **非线性速度限制**：
```
v ≤ min(v_map, v_curvature, v_comfort)

其中:
v_curvature = √(a_lateral_max / κ(s))
v_comfort = √(a_comfort / κ(s))
```

**目标函数**：
```
minimize: w1 × ∫(s''')² dt      // 最小化加加速度（平滑性）
        + w2 × ∫(s - s_ref)² dt // 跟踪参考曲线（效率）
        + w3 × ∫(v - v_ref)² dt // 跟踪参考速度（效率）
```

**优点**：
- 能处理复杂非线性约束
- 考虑向心加速度，更加安全舒适
- 解的质量高

**缺点**：
- 计算量较大
- 可能陷入局部最优

#### 5.2.4 样条插值优化算法

**原理**：
使用5次多项式样条表示速度曲线。

**样条函数定义**：

将ST曲线分为n段，每段用5次多项式表示：

```
s = f_i(t) = a_0i + a_1i×t + a_2i×t² + a_3i×t³ + a_4i×t⁴ + a_5i×t⁵
```

**目标函数**：

```
cost_1 = ∑[w1×∫(f_i')²ds + w2×∫(f_i'')²ds + w3×∫(f_i''')²ds]
       (平滑性代价：速度、加速度、加加速度的平方积分)

cost_2 = ∑∑(f_i(t_j) - s_cruise_j)²
       (与巡航速度曲线的偏差)

cost_3 = ∑∑(f_i(t_j) - s_follow_j)²
       (与跟车速度曲线的偏差)

total_cost = cost_1 + cost_2 + cost_3
```

**约束条件**：

1. **初始点约束**：
```
f_i(t_0) = s_0
f_i'(t_0) = v_0
f_i''(t_0) = a_0
```

2. **单调性约束**：
车辆只能向前行驶
```
f_i(t_j) > f_i(t_{j-1})  ∀j
```

3. **平滑节点约束**：
相邻样条段在连接点处保持连续
```
f_k(t_k) = f_{k+1}(t_0)        // 位置连续
f_k'(t_k) = f_{k+1}'(t_0)      // 速度连续
f_k''(t_k) = f_{k+1}''(t_0)    // 加速度连续
f_k'''(t_k) = f_{k+1}'''(t_0)  // 加加速度连续
```

4. **ST边界约束**：
```
对每个采样点 t_j:
s_lower(t_j) ≤ f_i(t_j) ≤ s_upper(t_j)
```

5. **速度边界约束**：
```
v_min(t_j) ≤ f_i'(t_j) ≤ v_max(t_j)
```

### 5.3 ST图构建详解

#### 5.3.1 ST图定义

ST图是一个二维坐标系：
- **横轴（T）**：时间轴，表示未来时刻
- **纵轴（S）**：沿参考线的累计距离
- **曲线**：表示车辆在不同时刻的位置

#### 5.3.2 障碍物映射

**静态障碍物**：
在ST图上表现为垂直条带
```
s_lower ≤ s ≤ s_upper, ∀t
```

**动态障碍物**：
在ST图上表现为斜向四边形区域
```
对障碍物的每个预测轨迹点 (x_obs, y_obs, t):
1. 投影到参考线获得 s_obs
2. 考虑车辆和障碍物尺寸扩展得到 s_lower, s_upper
3. 连接各时刻的边界点形成ST边界多边形
```

#### 5.3.3 决策与边界类型

根据决策结果，为障碍物的ST边界分配不同类型：

1. **STOP边界**：
   - 必须在障碍物前方停止
   - 约束：`s(t) ≤ s_obstacle - safe_distance`

2. **FOLLOW边界**：
   - 跟随障碍物行驶
   - 约束：`s(t) ≤ s_obstacle(t) - follow_distance`

3. **YIELD边界**：
   - 让行给障碍物
   - 约束：`s(t) ≤ s_obstacle(t) - safe_distance`

4. **OVERTAKE边界**：
   - 超越障碍物
   - 约束：`s(t) ≥ s_obstacle(t) + safe_distance`

5. **KEEP_CLEAR边界**：
   - 保持区域清空（如路口）
   - 约束：不能停在该区域内

### 5.4 速度限制计算

**综合速度限制计算**：

```cpp
double GetSpeedLimit(double s) {
  double v_map = GetMapSpeedLimit(s);        // 地图限速
  double v_curv = GetCurvatureLimit(s);      // 曲率限速
  double v_nudge = GetNudgeSpeedLimit(s);    // 避让限速
  double v_junction = GetJunctionLimit(s);   // 路口限速
  
  return min(v_map, v_curv, v_nudge, v_junction);
}
```

**曲率限速计算**：
```
v_curv = √(a_lateral_max / |κ|)

其中：
- κ 是路径曲率
- a_lateral_max 是最大横向加速度（通常为 4 m/s²）
```

## 6. QP问题构造详解

### 6.1 目标函数代码解析

从代码层面完整理解QP目标函数的构造。

#### 6.1.1 完整目标函数

根据代码实现，完整的目标函数为：

$$
\begin{aligned}
J = &\sum_{i=0}^{n-1} w_{s} (s_i - s_{ref,i})^2 
    &&\text{← 位置跟踪} \\
  + &\sum_{i=0}^{n-1} (w_{v,i} + p_{\kappa,i}) (v_i - v_{ref,i})^2 
    &&\text{← 速度跟踪+曲率惩罚} \\
  + &\sum_{i=0}^{n-1} w_a a_i^2 
    &&\text{← 加速度平滑} \\
  + &\sum_{i=0}^{n-2} w_j \left(\frac{a_{i+1} - a_i}{\Delta t}\right)^2 
    &&\text{← Jerk平滑（最重要）} \\
  + &w_{end,s}(s_{n-1} - s_{end})^2 
    &&\text{← 末端位置} \\
  + &w_{end,v}(v_{n-1} - v_{end})^2 
    &&\text{← 末端速度} \\
  + &w_{end,a}(a_{n-1} - a_{end})^2 
    &&\text{← 末端加速度}
\end{aligned}
$$

#### 6.1.2 权重设置代码

```cpp
// 在PiecewiseJerkSpeedOptimizer::Process()中设置

// 基础权重
qp_problem.set_weight_ddx(config_.acc_weight());      // 100.0 - 500.0
qp_problem.set_weight_dddx(config_.jerk_weight());    // 100.0 - 1000.0

// 位置参考权重
qp_problem.set_x_ref(config_.ref_s_weight(), x_ref);  // 10.0

// 速度参考权重（可以是向量，每个点不同）
std::vector<double> dx_ref(num_of_knots, cruise_speed);
std::vector<double> dx_ref_weight(num_of_knots, config_.ref_v_weight());  // 10.0
qp_problem.set_dx_ref(dx_ref_weight, dx_ref);

// 曲率惩罚（动态计算）
std::vector<double> penalty_dx;
for (int i = 0; i < num_of_knots; ++i) {
  double curr_t = i * delta_t;
  SpeedPoint sp;
  reference_speed_data.EvaluateByTime(curr_t, &sp);
  PathPoint path_point = path_data.GetPathPointWithPathS(sp.s());
  
  // 曲率越大，惩罚越大
  penalty_dx.push_back(std::fabs(path_point.kappa()) * config_.kappa_penalty_weight());
}
qp_problem.set_penalty_dx(penalty_dx);
```

#### 6.1.3 代价项的物理意义

| 代价项 | 权重 | 物理意义 | 效果 |
|--------|------|---------|------|
| **位置跟踪** | w_s=10.0 | 鼓励车辆前进 | 尽量走得远 |
| **速度跟踪** | w_v=10.0 | 保持巡航速度 | 稳定行驶 |
| **曲率惩罚** | w_κ=100.0 | 弯道减速 | 安全转弯 |
| **加速度** | w_a=100.0 | 减少加速度 | 舒适性 |
| **Jerk** | w_j=1000.0 | 减少加加速度 | 平滑性⭐ |
| **末端状态** | w_end=10.0 | 约束终点 | 平滑衔接 |

**权重的相对重要性**：

```
w_jerk (1000.0) >> w_acc (100.0) >> w_v (10.0) ≈ w_s (10.0)
   ↑                  ↑                ↑
 最重要            重要            一般重要
 
决定平滑性      决定舒适性      决定效率
```

#### 6.1.4 曲率惩罚的作用

曲率惩罚是一个关键设计：

```cpp
// 假设路径曲率变化：
s=0-40m:  κ=0.0    → penalty=0     → 正常速度
s=40-60m: κ=0.1    → penalty=10.0  → 轻微减速
s=60-80m: κ=0.2    → penalty=20.0  → 明显减速
s=80-100m:κ=0.0    → penalty=0     → 恢复速度
```

**效果**：车辆会在弯道处自动减速，无需显式规则。

### 6.2 约束条件代码解析

#### 6.2.1 约束矩阵维度

```
约束类型 | 数量 | 维度 | 说明
---------|------|------|------
变量边界 | 3n | 240 | s/v/a的上下界
初始状态 | 3 | 3 | s_0/v_0/a_0固定
速度连续性 | n-1 | 79 | v_{i+1}与v_i/a_i/a_{i+1}关系
位置连续性 | n-1 | 79 | s_{i+1}与s_i/v_i/a_i/a_{i+1}关系
Jerk约束 | n-1 | 79 | a_{i+1}-a_i的范围
总计 | ~480 | 480 | 
```

#### 6.2.2 运动学约束推导

**速度连续性**（梯形积分）：

$$
v_{i+1} = v_i + \int_{t_i}^{t_{i+1}} a(t) dt \approx v_i + \frac{a_i + a_{i+1}}{2} \Delta t
$$

**位置连续性**（Simpson积分）：

$$
s_{i+1} = s_i + \int_{t_i}^{t_{i+1}} v(t) dt
$$

其中：
$$
v(t) = v_i + a_i(t - t_i) + \frac{1}{2}\frac{a_{i+1} - a_i}{\Delta t}(t - t_i)^2
$$

积分得：
$$
s_{i+1} = s_i + v_i \Delta t + \frac{1}{3} a_i \Delta t^2 + \frac{1}{6} a_{i+1} \Delta t^2
$$

这就是代码中的连续性约束来源！

### 6.3 完整的代码调用流程

```cpp
// 完整的速度规划QP求解流程

// 1. 在SpeedBoundsDecider中准备ST图数据
Status SpeedBoundsDecider::Process(...) {
  // 计算ST边界和速度限制
  st_graph_data->LoadData(boundaries, init_point, speed_limit, ...);
}

// 2. 在PiecewiseJerkSpeedOptimizer中求解
Status PiecewiseJerkSpeedOptimizer::Process(...) {
  // 2.1 获取ST图数据
  const StGraphData& st_graph_data = reference_line_info_->st_graph_data();
  
  // 2.2 计算s边界（基于ST边界）
  for (int i = 0; i < num_of_knots; ++i) {
    for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
      // 根据边界类型调整s的上下界
      if (boundary->boundary_type() == STOP) {
        s_upper = min(s_upper, boundary->upper_s(t_i));
      }
    }
    s_bounds[i] = {s_lower, s_upper};
  }
  
  // 2.3 计算v边界（基于速度限制）
  for (int i = 0; i < num_of_knots; ++i) {
    double v_limit = speed_limit.GetSpeedLimitByS(s_i);
    v_bounds[i] = {0.0, v_limit};
  }
  
  // 2.4 计算参考值
  x_ref = vector<double>(num_of_knots, total_length);  // 期望走完
  dx_ref = vector<double>(num_of_knots, cruise_speed); // 期望巡航
  
  // 2.5 计算曲率惩罚
  for (int i = 0; i < num_of_knots; ++i) {
    penalty_dx[i] = |kappa(s_i)| * kappa_penalty_weight;
  }
  
  // 2.6 创建并配置QP问题
  PiecewiseJerkSpeedProblem qp(num_of_knots, delta_t, init_s);
  qp.set_weight_ddx(acc_weight);
  qp.set_weight_dddx(jerk_weight);
  qp.set_x_ref(ref_s_weight, x_ref);
  qp.set_dx_ref(ref_v_weight, dx_ref);
  qp.set_penalty_dx(penalty_dx);
  qp.set_x_bounds(s_bounds);
  qp.set_dx_bounds(v_bounds);
  qp.set_ddx_bounds(a_min, a_max);
  qp.set_dddx_bound(jerk_bound);
  
  // 2.7 求解QP问题
  qp.Optimize();  // 内部调用OSQP
  
  // 2.8 提取结果
  opt_s = qp.opt_x();
  opt_v = qp.opt_dx();
  opt_a = qp.opt_ddx();
  
  // 2.9 构造SpeedData
  for (int i = 0; i < num_of_knots; ++i) {
    speed_data->AppendSpeedPoint(opt_s[i], i*delta_t, opt_v[i], opt_a[i], jerk_i);
  }
}
```

### 6.4 权重调参实战指南

#### 6.4.1 调参目标

| 目标 | 调整策略 | 示例 |
|------|---------|------|
| **更平滑** | 增大jerk_weight | 1000.0 → 2000.0 |
| **更舒适** | 增大acc_weight | 100.0 → 500.0 |
| **更快** | 增大ref_s_weight | 10.0 → 20.0 |
| **稳定巡航** | 增大ref_v_weight | 10.0 → 20.0 |
| **弯道更慢** | 增大kappa_penalty | 100.0 → 200.0 |

#### 6.4.2 典型参数组合

**舒适性优先**（乘用车）：
```cpp
acc_weight = 500.0      // 高
jerk_weight = 2000.0    // 很高
ref_v_weight = 10.0     // 中等
kappa_penalty = 150.0   // 较高
```

**效率优先**（物流车）：
```cpp
acc_weight = 100.0      // 中等
jerk_weight = 500.0     // 中等
ref_v_weight = 20.0     // 高（保持速度）
kappa_penalty = 50.0    // 较低（允许快速过弯）
```

**平衡配置**（默认）：
```cpp
acc_weight = 100.0
jerk_weight = 1000.0
ref_v_weight = 10.0
kappa_penalty = 100.0
```

## 7. 主要模块详解

### 7.1 SpeedBoundsDecider（速度边界决策器）

**职责**：构建ST图并计算速度约束边界

**处理流程**：

```cpp
Status SpeedBoundsDecider::Process(Frame* frame, 
                                   ReferenceLineInfo* reference_line_info) {
  // 1. 获取输入数据
  const PathData& path_data = reference_line_info->path_data();
  const TrajectoryPoint& init_point = frame->PlanningStartPoint();
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  PathDecision* path_decision = reference_line_info->path_decision();

  // 2. 映射障碍物到ST图
  STBoundaryMapper boundary_mapper(...);
  boundary_mapper.ComputeSTBoundary(path_decision);

  // 3. 创建速度限制
  SpeedLimitDecider speed_limit_decider(...);
  SpeedLimit speed_limit;
  speed_limit_decider.GetSpeedLimits(path_decision->obstacles(), &speed_limit);

  // 4. 获取搜索边界
  double path_length = path_data.discretized_path().Length();
  double total_time = config_.total_time();

  // 5. 加载ST图数据
  StGraphData* st_graph_data = reference_line_info->mutable_st_graph_data();
  st_graph_data->LoadData(boundaries, min_s_on_st_boundaries, init_point,
                          speed_limit, cruise_speed, path_length, total_time, ...);

  return Status::OK();
}
```

**输出**：StGraphData，包含：
- ST边界集合
- 速度限制曲线
- 初始状态
- 搜索范围

### 7.2 PiecewiseJerkSpeedOptimizer（QP速度优化器）

**职责**：使用二次规划求解速度曲线

**处理流程**：

```cpp
Status PiecewiseJerkSpeedOptimizer::Process(const PathData& path_data,
                                            const TrajectoryPoint& init_point,
                                            SpeedData* speed_data) {
  // 1. 获取ST图数据
  StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
  
  // 2. 设置初始状态
  std::array<double, 3> init_s = {0.0, init_point.v(), init_point.a()};
  
  // 3. 计算ST边界约束
  std::vector<std::pair<double, double>> s_bounds;
  for (int i = 0; i < num_of_knots; ++i) {
    double curr_t = i * delta_t;
    double s_lower_bound = 0.0;
    double s_upper_bound = total_length;
    
    // 根据障碍物ST边界计算s的上下界
    for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
      switch (boundary->boundary_type()) {
        case STBoundary::STOP:
        case STBoundary::YIELD:
          s_upper_bound = min(s_upper_bound, boundary->upper_bound(curr_t));
          break;
        case STBoundary::OVERTAKE:
          s_lower_bound = max(s_lower_bound, boundary->lower_bound(curr_t));
          break;
      }
    }
    s_bounds.emplace_back(s_lower_bound, s_upper_bound);
  }
  
  // 4. 计算速度边界约束
  std::vector<std::pair<double, double>> s_dot_bounds;
  const SpeedLimit& speed_limit = st_graph_data.speed_limit();
  for (int i = 0; i < num_of_knots; ++i) {
    double s = s_bounds[i].first;
    double v_limit = speed_limit.GetSpeedLimitByS(s);
    s_dot_bounds.emplace_back(0.0, v_limit);
  }
  
  // 5. 构建并求解QP问题
  PiecewiseJerkSpeedProblem qp_problem(num_of_knots, delta_t, init_s);
  qp_problem.set_weight_jerk(config_.jerk_weight());
  qp_problem.set_weight_acc(config_.acc_weight());
  qp_problem.set_weight_ref_speed(config_.ref_speed_weight());
  qp_problem.set_x_bounds(s_bounds);
  qp_problem.set_dx_bounds(s_dot_bounds);
  qp_problem.set_ddx_bounds(a_min, a_max);
  qp_problem.set_dddx_bound(jerk_min, jerk_max);
  
  bool success = qp_problem.Optimize();
  
  // 6. 提取结果
  const std::vector<double>& s = qp_problem.x();
  const std::vector<double>& v = qp_problem.x_derivative();
  const std::vector<double>& a = qp_problem.x_second_order_derivative();
  
  // 7. 构建SpeedData
  speed_data->clear();
  for (int i = 0; i < num_of_knots; ++i) {
    double jerk = (i > 0) ? (a[i] - a[i-1]) / delta_t : 0.0;
    speed_data->AppendSpeedPoint(s[i], i * delta_t, v[i], a[i], jerk);
  }
  
  return Status::OK();
}
```

### 7.3 PiecewiseJerkSpeedNonlinearOptimizer（NLP速度优化器）

**职责**：使用两阶段优化（QP+NLP）求解速度曲线

**处理流程**：

```cpp
Status PiecewiseJerkSpeedNonlinearOptimizer::Process(
    const PathData& path_data,
    const TrajectoryPoint& init_point,
    SpeedData* speed_data) {
    
  // 1. 安全检查
  if (path_data.discretized_path().empty()) {
    return Status(ErrorCode::PLANNING_ERROR, "Empty path data");
  }
  
  // 2. 设置状态和边界
  Status setup_status = SetUpStatesAndBounds(path_data, *speed_data);
  if (!setup_status.ok()) {
    speed_data->clear();
    return setup_status;
  }
  
  // 3. 第一阶段：QP优化
  std::vector<double> distance, velocity, acceleration;
  Status qp_status = OptimizeByQP(speed_data, &distance, &velocity, &acceleration);
  if (!qp_status.ok()) {
    speed_data->clear();
    return qp_status;
  }
  
  // 4. 检查速度限制可行性
  bool feasible = CheckSpeedLimitFeasibility();
  
  if (feasible) {
    // 5. 平滑路径曲率
    SmoothPathCurvature(path_data);
    
    // 6. 平滑速度限制
    SmoothSpeedLimit();
    
    // 7. 第二阶段：NLP优化
    Status nlp_status = OptimizeByNLP(&distance, &velocity, &acceleration);
    if (!nlp_status.ok()) {
      // NLP失败，使用QP结果
      AWARN << "NLP optimization failed, use QP result";
    }
  }
  
  // 8. 构建SpeedData
  speed_data->clear();
  for (size_t i = 0; i < distance.size(); ++i) {
    double jerk = 0.0;
    if (i > 0) {
      jerk = (acceleration[i] - acceleration[i-1]) / delta_t_;
    }
    speed_data->AppendSpeedPoint(distance[i], i * delta_t_, 
                                 velocity[i], acceleration[i], jerk);
  }
  
  return Status::OK();
}
```

**SetUpStatesAndBounds详解**：

```cpp
Status SetUpStatesAndBounds(const PathData& path_data, 
                            const SpeedData& speed_data) {
  // 1. 设置问题维度
  delta_t_ = 0.1;  // 时间步长
  total_length_ = path_data.discretized_path().Length();
  total_time_ = st_graph_data.total_time_by_conf();
  num_of_knots_ = static_cast<int>(total_time_ / delta_t_) + 1;
  
  // 2. 设置初始状态
  s_init_ = 0.0;
  s_dot_init_ = init_point.v();
  s_ddot_init_ = init_point.a();
  
  // 3. 设置动力学边界
  s_dot_max_ = vehicle_param.max_abs_speed_when_stopped();
  s_ddot_min_ = vehicle_param.max_deceleration();
  s_ddot_max_ = vehicle_param.max_acceleration();
  s_dddot_min_ = -config_.max_jerk();
  s_dddot_max_ = config_.max_jerk();
  
  // 4. 计算ST边界约束
  s_bounds_.clear();
  s_soft_bounds_.clear();
  for (int i = 0; i < num_of_knots_; ++i) {
    double t = i * delta_t_;
    double s_lower = 0.0;
    double s_upper = total_length_;
    
    // 考虑障碍物ST边界
    for (const auto* boundary : st_graph_data.st_boundaries()) {
      // ... 计算边界 ...
    }
    
    s_bounds_.emplace_back(s_lower, s_upper);
    s_soft_bounds_.emplace_back(s_soft_lower, s_soft_upper);
  }
  
  // 5. 设置速度限制
  speed_limit_ = st_graph_data.speed_limit();
  
  // 6. 设置参考速度
  cruise_speed_ = reference_line_info_->GetCruiseSpeed();
  
  return Status::OK();
}
```

### 7.4 PathTimeHeuristicOptimizer（动态规划优化器）

**职责**：使用动态规划在ST图上搜索最优路径

**处理流程**：

```cpp
Status PathTimeHeuristicOptimizer::Process(const PathData& path_data,
                                           const TrajectoryPoint& init_point,
                                           SpeedData* speed_data) {
  init_point_ = init_point;
  
  // 构建并搜索ST图
  if (!SearchPathTimeGraph(speed_data)) {
    const std::string msg = "Failed to search path time graph";
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  
  return Status::OK();
}

bool PathTimeHeuristicOptimizer::SearchPathTimeGraph(SpeedData* speed_data) {
  // 创建网格化ST图
  GriddedPathTimeGraph st_graph(
      st_graph_data, dp_st_speed_config, 
      reference_line_info_->path_decision()->obstacles().Items(),
      init_point_);
  
  // 动态规划搜索
  if (!st_graph.Search(speed_data).ok()) {
    AERROR << "Failed to search st graph";
    return false;
  }
  
  return true;
}
```

**GriddedPathTimeGraph搜索算法**：

```cpp
Status GriddedPathTimeGraph::Search(SpeedData* speed_data) {
  // 1. 初始化DP表
  // dp[t][s] 表示在时刻t到达位置s的最小代价
  
  // 2. 前向DP搜索
  for (int t_level = 0; t_level < num_t_levels; ++t_level) {
    for (int s_level = 0; s_level < num_s_levels; ++s_level) {
      // 检查当前点是否在ST边界内（可行）
      if (!IsPointFeasible(t_level, s_level)) {
        continue;
      }
      
      // 从所有可能的前驱节点转移
      for (int prev_s = 0; prev_s < num_s_levels; ++prev_s) {
        if (dp[t_level-1][prev_s].cost == infinity) {
          continue;
        }
        
        // 计算转移代价
        double cost = CalculateCost(t_level-1, prev_s, t_level, s_level);
        double total_cost = dp[t_level-1][prev_s].cost + cost;
        
        // 更新DP表
        if (total_cost < dp[t_level][s_level].cost) {
          dp[t_level][s_level].cost = total_cost;
          dp[t_level][s_level].pre_point = {t_level-1, prev_s};
        }
      }
    }
  }
  
  // 3. 反向回溯最优路径
  int best_s = FindBestEndingS();
  std::vector<STPoint> st_points;
  int t = num_t_levels - 1;
  int s = best_s;
  
  while (t >= 0) {
    st_points.push_back({s * s_resolution, t * t_resolution});
    auto pre = dp[t][s].pre_point;
    t = pre.t;
    s = pre.s;
  }
  
  std::reverse(st_points.begin(), st_points.end());
  
  // 4. 转换为SpeedData
  ConvertToSpeedData(st_points, speed_data);
  
  return Status::OK();
}
```

## 8. 配置参数

### 8.1 PiecewiseJerkSpeedOptimizer配置

```protobuf
message PiecewiseJerkSpeedOptimizerConfig {
  // QP权重
  optional double jerk_weight = 1 [default = 1.0];
  optional double acc_weight = 2 [default = 1.0];
  optional double ref_speed_weight = 3 [default = 10.0];
  
  // 动力学限制
  optional double max_acceleration = 4 [default = 3.0];   // m/s²
  optional double min_acceleration = 5 [default = -4.0];  // m/s²
  optional double max_jerk = 6 [default = 4.0];           // m/s³
  
  // 时间参数
  optional double total_time = 7 [default = 8.0];  // 规划时域(s)
  optional double delta_t = 8 [default = 0.1];     // 时间步长(s)
}
```

### 8.2 PiecewiseJerkSpeedNonlinearOptimizer配置

```protobuf
message PiecewiseJerkNonlinearSpeedOptimizerConfig {
  // QP权重
  optional double acc_weight = 1 [default = 1.0];
  optional double jerk_weight = 2 [default = 1.0];
  optional double kappa_penalty_weight = 3 [default = 100.0];
  optional double ref_s_weight = 4 [default = 10.0];
  optional double ref_v_weight = 5 [default = 10.0];
  
  // NLP权重
  optional double nlp_acc_weight = 6 [default = 1.0];
  optional double nlp_jerk_weight = 7 [default = 1.0];
  optional double nlp_curvature_weight = 8 [default = 10.0];
  
  // 安全边界
  optional double safe_distance_buffer = 9 [default = 3.0];  // m
  optional double soft_boundary_buffer = 10 [default = 1.0]; // m
  
  // 向心加速度限制
  optional double max_lateral_acceleration = 11 [default = 4.0]; // m/s²
}
```

### 8.3 SpeedBoundsDecider配置

```protobuf
message SpeedBoundsDeciderConfig {
  // ST图时域
  optional double total_time = 1 [default = 8.0];  // s
  
  // 边界buffer
  optional double boundary_buffer = 2 [default = 0.1];  // m
  
  // 速度限制
  optional double max_centric_acceleration_limit = 3 [default = 2.0];  // m/s²
  
  // 路口速度限制
  optional double minimal_kappa = 4 [default = 0.00001];
  optional double point_extension = 5 [default = 0.5];    // m
  optional double lowest_speed = 6 [default = 2.5];       // m/s
  optional double collision_safety_range = 7 [default = 1000.0];  // m
}
```

### 8.4 PathTimeHeuristic配置

```protobuf
message SpeedHeuristicOptimizerConfig {
  // DP网格参数
  optional uint32 s_samples = 1 [default = 20];
  optional uint32 t_samples = 2 [default = 10];
  optional double delta_t = 3 [default = 1.0];
  
  // 代价权重
  optional double speed_cost_weight = 4 [default = 10.0];
  optional double accel_cost_weight = 5 [default = 10.0];
  optional double jerk_cost_weight = 6 [default = 10.0];
  optional double obstacle_cost_weight = 7 [default = 100.0];
  optional double reference_cost_weight = 8 [default = 1.0];
  
  // 动力学限制
  optional double max_acceleration = 9 [default = 3.0];
  optional double max_deceleration = 10 [default = -4.0];
  optional double preferred_max_deceleration = 11 [default = -2.5];
  optional double preferred_max_acceleration = 12 [default = 2.0];
}
```

## 9. 调用流程

### 9.1 触发机制

Apollo的速度规划采用**消息触发机制**，而非定时器触发：

#### 9.1.1 消息驱动模式

```cpp
// PlanningComponent定义
class PlanningComponent final
    : public cyber::Component<prediction::PredictionObstacles,  // 触发消息
                              canbus::Chassis,                  // 融合消息
                              localization::LocalizationEstimate> { // 融合消息
 public:
  // 当收到新的PredictionObstacles消息时触发
  bool Proc(const std::shared_ptr<prediction::PredictionObstacles>& prediction_obstacles,
            const std::shared_ptr<canbus::Chassis>& chassis,
            const std::shared_ptr<localization::LocalizationEstimate>& localization_estimate) override;
};
```

**触发条件**：
- **主触发源**：`PredictionObstacles`消息到达
- **融合输入**：同时获取最新的`Chassis`和`LocalizationEstimate`消息
- **触发频率**：由Prediction模块的发布频率决定，通常为**10Hz**（100ms周期）

#### 9.1.2 触发流程

```
1. Perception模块检测障碍物 → 发布PerceptionObstacles
   ↓
2. Prediction模块预测障碍物轨迹 → 发布PredictionObstacles (10Hz)
   ↓
3. PlanningComponent收到PredictionObstacles → 触发Proc()
   ↓
4. 获取最新的Chassis和Localization数据
   ↓
5. 执行规划算法（包括速度规划）
   ↓
6. 发布ADCTrajectory给Control模块
```

#### 9.1.3 周期时间参数

虽然采用消息触发，但代码中仍有周期时间参数：

```cpp
// 代码位置：on_lane_planning.cc
// planning_loop_rate 默认为10 (即10Hz，100ms周期)
const double planning_cycle_time = 1.0 / static_cast<double>(FLAGS_planning_loop_rate);

// 用途：轨迹拼接时估算周期时间
std::vector<TrajectoryPoint> stitching_trajectory = 
    TrajectoryStitcher::ComputeStitchingTrajectory(
        chassis, vehicle_state, start_timestamp, 
        planning_cycle_time,  // <- 使用估算的周期时间
        FLAGS_trajectory_stitching_preserved_length, 
        true, last_publishable_trajectory, &replan_reason);
```

**注意**：`planning_cycle_time`不是用于定时触发，而是用于：
- 轨迹拼接计算
- 预估下一帧的时间戳
- 速度规划的时域设置

#### 9.1.4 为什么使用消息触发而非定时器？

**优点**：
1. **数据同步**：保证规划使用最新的感知和预测结果
2. **资源效率**：没有新数据时不浪费计算资源
3. **延迟最小**：收到数据立即处理，减少端到端延迟
4. **模块解耦**：各模块独立运行，通过消息通信

**实际运行频率**：
- **理论频率**：10Hz（由prediction模块控制）
- **实际频率**：取决于上游模块的实际发布频率，可能略有波动
- **最大频率限制**：由`FLAGS_planning_loop_rate`参数控制

#### 9.1.5 其他触发条件

除了正常的消息触发，还有以下特殊触发条件：

```cpp
// 1. 路由变更触发
if (util::IsDifferentRouting(last_command_, *local_view_.planning_command)) {
    // 清空历史，重新规划
    reference_line_provider_->Reset();
    injector_->history()->Clear();
    injector_->planning_context()->mutable_planning_status()->Clear();
    planner_->Reset(frame_.get());
}

// 2. PadMessage命令触发
if (pad_msg_.action() == PadMessage::CLEAR_PLANNING) {
    // 清空规划
    local_view_.planning_command = nullptr;
    planning_command_.Clear();
}

// 3. 重路由请求触发
CheckRerouting();  // 检查并处理重路由请求
```

### 9.2 整体调用链

```
消息到达触发:
PredictionObstacles消息 → PlanningComponent::Proc()
  ↓
数据融合:
  ├─ local_view_.prediction_obstacles = prediction_obstacles
  ├─ local_view_.chassis = chassis
  ├─ local_view_.localization_estimate = localization_estimate
  ├─ local_view_.traffic_light (异步订阅)
  └─ local_view_.planning_command (异步订阅)
  ↓
规划执行:
OnLanePlanning::RunOnce()
  └─> Planner::Plan()
      └─> PublicRoadPlanner::PlanOnReferenceLine()
          └─> Scenario::Process()
              └─> Stage::Process()
                  └─> Stage::ExecuteTaskOnReferenceLine()
                      ├─> SpeedBoundsDecider::Execute()
                      │   └─> SpeedBoundsDecider::Process()
                      │       ├─> STBoundaryMapper::ComputeSTBoundary()
                      │       ├─> SpeedLimitDecider::GetSpeedLimits()
                      │       └─> StGraphData::LoadData()
                      │
                      └─> SpeedOptimizer::Execute()
                          └─> SpeedOptimizer::Process()
                              ├─> PiecewiseJerkSpeedOptimizer::Process()
                              ├─> PiecewiseJerkSpeedNonlinearOptimizer::Process()
                              └─> PathTimeHeuristicOptimizer::Process()
  ↓
结果发布:
ADCTrajectory → planning_writer_->Write() → Control模块
```

### 9.3 典型场景调用示例

#### 场景1：正常巡航

```
任务序列:
1. SpeedBoundsDecider
   - 映射少量障碍物
   - 计算速度限制
   - 构建ST图

2. PiecewiseJerkSpeedOptimizer
   - 使用QP优化
   - 跟踪巡航速度
   - 生成平滑速度曲线
```

#### 场景2：跟车

```
任务序列:
1. SpeedBoundsDecider
   - 映射前车为FOLLOW边界
   - 计算跟车距离
   - 构建ST图

2. PiecewiseJerkSpeedNonlinearOptimizer
   - QP阶段：生成基础跟车曲线
   - NLP阶段：考虑路径曲率优化
   - 保持安全跟车距离
```

#### 场景3：避让停止

```
任务序列:
1. SpeedBoundsDecider
   - 映射障碍物为STOP边界
   - 计算停止距离
   - 构建ST图

2. PiecewiseJerkSpeedOptimizer
   - 规划减速曲线
   - 在障碍物前停止
   - 满足舒适性约束
```

## 10. 调试与可视化

### 10.1 调试信息

Apollo提供了丰富的调试信息：

```protobuf
message STGraphDebug {
  optional string name = 1;
  repeated StGraphBoundaryDebug boundary = 2;
  repeated SpeedPoint speed_profile = 3;
  message STGraphSpeedConstraint {
    repeated double t = 1;
    repeated double lower_bound = 2;
    repeated double upper_bound = 3;
  }
  optional STGraphSpeedConstraint speed_constraint = 4;
  repeated STGraphKernelCuiseRef kernel_cruise_ref = 5;
  repeated STGraphKernelFollowRef kernel_follow_ref = 6;
}
```

### 10.2 可视化工具

在DreamView中可以可视化：
- **ST图**：显示障碍物边界和规划的速度曲线
- **速度曲线**：显示速度、加速度、加加速度随时间的变化
- **SL图**：显示路径和障碍物的横向关系

### 10.3 日志记录

关键日志点：
```cpp
AINFO << "Speed optimization start";
ADEBUG << "ST boundary computed: " << boundary.DebugString();
ADEBUG << "Speed limit at s=" << s << " is " << speed_limit;
AINFO << "QP optimization takes " << time_ms << " ms";
AWARN << "NLP optimization failed, use QP result";
AERROR << "Speed optimization failed: " << error_msg;
```

## 11. 性能优化建议

### 11.1 计算效率

1. **算法选择**：
   - 简单场景：PiecewiseJerkSpeedOptimizer（最快）
   - 复杂场景：PiecewiseJerkSpeedNonlinearOptimizer（平衡）
   - 非凸场景：PathTimeHeuristicOptimizer（最慢但鲁棒）

2. **参数调优**：
   - 减少时间步数 num_of_knots
   - 降低DP网格密度
   - 调整优化器迭代次数

3. **并行计算**：
   - 多条参考线并行规划
   - ST边界计算并行化

### 11.2 解的质量

1. **权重调整**：
   - 增大 jerk_weight 提高平滑性
   - 增大 ref_speed_weight 提高跟踪性能
   - 增大 acc_weight 提高舒适性

2. **约束松弛**：
   - 使用软约束处理可能不可行的硬约束
   - 添加松弛变量避免无解

3. **初始值优化**：
   - 使用上一帧结果作为初始值
   - 使用简单方法生成初始解

## 12. 常见问题

### 12.1 优化失败

**原因**：
- ST边界过于严格，导致无可行解
- 速度限制不合理
- 动力学约束冲突

**解决方法**：
- 检查ST边界是否合理
- 放宽速度限制
- 使用软约束
- 回退到fallback策略

### 11.2 速度曲线抖动

**原因**：
- 权重设置不合理
- 约束频繁切换
- 时间步长过大

**解决方法**：
- 增大平滑性权重
- 添加约束缓冲区
- 减小时间步长
- 使用速度曲线滤波

### 11.3 跟车不稳定

**原因**：
- 跟车距离设置不合理
- 预测轨迹不准确
- 参考速度变化过快

**解决方法**：
- 调整跟车距离策略
- 改进障碍物预测
- 对参考速度进行平滑

## 12. 扩展与定制

### 12.1 添加新的优化器

1. 继承SpeedOptimizer基类
2. 实现Process函数
3. 注册为插件
4. 在配置文件中启用

示例：
```cpp
class CustomSpeedOptimizer : public SpeedOptimizer {
 public:
  bool Init(...) override;
  
 private:
  common::Status Process(const PathData& path_data,
                         const common::TrajectoryPoint& init_point,
                         SpeedData* const speed_data) override {
    // 自定义优化逻辑
    return Status::OK();
  }
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::CustomSpeedOptimizer, Task)
```

### 12.2 自定义代价函数

修改QP目标函数权重：
```cpp
qp_problem.set_weight_jerk(custom_jerk_weight);
qp_problem.set_weight_acc(custom_acc_weight);
qp_problem.set_weight_ref_speed(custom_ref_speed_weight);
```

### 12.3 添加新的约束

```cpp
// 添加自定义约束
qp_problem.AddCustomConstraint(constraint_matrix, constraint_bound);
```

## 13. 路径和速度结合生成轨迹

速度规划完成后，需要将路径（PathData）和速度曲线（SpeedData）结合起来，生成最终的轨迹（DiscretizedTrajectory）供控制模块使用。

### 13.1 核心方法

主要通过`ReferenceLineInfo::CombinePathAndSpeedProfile()`方法实现：

```cpp
bool ReferenceLineInfo::CombinePathAndSpeedProfile(
    const double relative_time,  // 相对时间偏移
    const double start_s,        // 起始s坐标偏移
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  
  // 1. 检查输入有效性
  if (path_data_.discretized_path().empty()) {
    AERROR << "path data is empty";
    return false;
  }
  
  if (speed_data_.empty()) {
    AERROR << "speed profile is empty";
    return false;
  }
  
  // 2. 设置时间采样分辨率
  // 使用变分辨率：近处密集，远处稀疏
  const double kDenseTimeResolution = FLAGS_trajectory_time_min_interval;    // 0.02s (50Hz)
  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;   // 0.1s (10Hz)
  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;    // 前1.0s使用密集采样
  
  // 3. 时间域采样，结合路径和速度
  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResolution 
                                                     : kSparseTimeResolution)) {
    // 3.1 根据时间获取速度点 (s, t, v, a, da)
    common::SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }
    
    // 3.2 检查是否超出路径长度
    if (speed_point.s() > path_data_.discretized_path().Length()) {
      break;
    }
    
    // 3.3 根据s在路径上插值获取路径点 (x, y, theta, kappa, s)
    common::PathPoint path_point = 
        path_data_.GetPathPointWithPathS(speed_point.s());
    path_point.set_s(path_point.s() + start_s);  // 加上起始偏移
    
    // 3.4 组合成轨迹点
    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);  // 路径信息
    trajectory_point.set_v(speed_point.v());                       // 速度
    trajectory_point.set_a(speed_point.a());                       // 加速度
    trajectory_point.set_relative_time(speed_point.t() + relative_time);  // 时间戳
    
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  
  // 4. 处理倒车情况
  if (path_data_.is_reverse_path()) {
    std::for_each(ptr_discretized_trajectory->begin(),
                  ptr_discretized_trajectory->end(),
                  [](common::TrajectoryPoint& trajectory_point) {
                    trajectory_point.set_v(-trajectory_point.v());   // 速度取反
                    trajectory_point.set_a(-trajectory_point.a());   // 加速度取反
                    trajectory_point.mutable_path_point()->set_s(
                        -trajectory_point.path_point().s());         // s取反
                  });
    ptr_discretized_trajectory->SetIsReversed(true);
  }
  
  return true;
}
```

### 13.2 关键步骤详解

#### 步骤1：时间域采样

采用**变分辨率采样**策略：
- **近期密集**：前1秒使用0.02s间隔（50Hz），为控制模块提供更精确的短期信息
- **远期稀疏**：1秒后使用0.1s间隔（10Hz），减少数据量

```cpp
时间采样示例：
t = [0.00, 0.02, 0.04, ..., 0.98, 1.00,  // 密集采样 50个点
     1.10, 1.20, 1.30, ..., 7.90, 8.00]  // 稀疏采样 70个点
总共约 120个点
```

#### 步骤2：根据时间获取速度信息

从SpeedData中根据时间插值获取速度点：

```cpp
// SpeedData::EvaluateByTime() 实现
bool SpeedData::EvaluateByTime(const double time, SpeedPoint* speed_point) const {
  // 在speed_data中查找对应时间的点，使用线性插值
  // 返回 (s, t, v, a, da)
  
  if (time < front().t()) {
    *speed_point = front();
    return true;
  }
  if (time > back().t()) {
    *speed_point = back();
    return true;
  }
  
  // 二分查找 + 线性插值
  auto it_lower = std::lower_bound(begin(), end(), time, 
                                   [](const SpeedPoint& p, double t) {
                                     return p.t() < t;
                                   });
  
  const SpeedPoint& p0 = *(it_lower - 1);
  const SpeedPoint& p1 = *it_lower;
  
  // 线性插值
  double ratio = (time - p0.t()) / (p1.t() - p0.t());
  speed_point->set_s(p0.s() + ratio * (p1.s() - p0.s()));
  speed_point->set_v(p0.v() + ratio * (p1.v() - p0.v()));
  speed_point->set_a(p0.a() + ratio * (p1.a() - p0.a()));
  speed_point->set_t(time);
  
  return true;
}
```

#### 步骤3：根据s获取路径信息

从PathData中根据累计距离s插值获取路径点：

```cpp
// PathData::GetPathPointWithPathS() 实现
common::PathPoint PathData::GetPathPointWithPathS(const double s) const {
  return discretized_path_.Evaluate(s);
}

// DiscretizedPath::Evaluate() 实现
PathPoint DiscretizedPath::Evaluate(const double path_s) const {
  // 在discretized_path中查找对应s的点，使用线性插值
  // 返回 (x, y, z, theta, kappa, s, dkappa)
  
  if (path_s <= front().s()) {
    return front();
  }
  if (path_s >= back().s()) {
    return back();
  }
  
  // 二分查找
  auto it_lower = std::lower_bound(begin(), end(), path_s,
                                   [](const PathPoint& p, double s) {
                                     return p.s() < s;
                                   });
  
  const PathPoint& p0 = *(it_lower - 1);
  const PathPoint& p1 = *it_lower;
  
  // 线性插值
  PathPoint result;
  double ratio = (path_s - p0.s()) / (p1.s() - p0.s());
  result.set_x(p0.x() + ratio * (p1.x() - p0.x()));
  result.set_y(p0.y() + ratio * (p1.y() - p0.y()));
  result.set_z(p0.z() + ratio * (p1.z() - p0.z()));
  result.set_theta(slerp(p0.theta(), p1.theta(), ratio));  // 角度插值
  result.set_kappa(p0.kappa() + ratio * (p1.kappa() - p0.kappa()));
  result.set_dkappa(p0.dkappa() + ratio * (p1.dkappa() - p0.dkappa()));
  result.set_s(path_s);
  
  return result;
}
```

#### 步骤4：组合成轨迹点

将路径点和速度点组合成完整的轨迹点：

```cpp
// TrajectoryPoint 数据结构
message TrajectoryPoint {
  // 路径信息（空间）
  optional PathPoint path_point = 1;  // x, y, z, theta, kappa, s, dkappa
  
  // 速度信息（时间）
  optional double v = 2;              // 速度 (m/s)
  optional double a = 3;              // 加速度 (m/s²)
  optional double relative_time = 4;  // 相对时间 (s)
  optional double da = 5;             // 加加速度 (m/s³)
  optional double steer = 6;          // 方向盘转角 (rad)
}
```

### 13.3 数据流示意图

```
输入：
PathData (路径)                    SpeedData (速度曲线)
  ├─ path_point[0]: (x,y,θ,κ,s)     ├─ speed_point[0]: (s,t,v,a,da)
  ├─ path_point[1]: (x,y,θ,κ,s)     ├─ speed_point[1]: (s,t,v,a,da)
  └─ ...                             └─ ...

处理过程：
  ┌─────────────────────────────────────────────────┐
  │  时间采样：t = 0.00, 0.02, 0.04, ..., 8.00     │
  └─────────────────────────────────────────────────┘
                         ↓
  ┌─────────────────────────────────────────────────┐
  │  对每个时刻 t_i:                                 │
  │  1. 从SpeedData插值: t_i → (s_i, v_i, a_i)     │
  │  2. 从PathData插值:  s_i → (x_i, y_i, θ_i, κ_i)│
  │  3. 组合: trajectory_point_i = {path_i + speed_i}│
  └─────────────────────────────────────────────────┘
                         ↓
输出：
DiscretizedTrajectory (离散轨迹)
  ├─ trajectory_point[0]: (x,y,θ,κ,s,v,a,t)
  ├─ trajectory_point[1]: (x,y,θ,κ,s,v,a,t)
  └─ ...
```

### 13.4 完整示例

假设我们有：

**PathData（路径）：**
```
point[0]: x=0.0,  y=0.0, θ=0.0,  κ=0.0,  s=0.0
point[1]: x=1.0,  y=0.0, θ=0.0,  κ=0.0,  s=1.0
point[2]: x=2.0,  y=0.1, θ=0.1,  κ=0.05, s=2.01
...
```

**SpeedData（速度曲线）：**
```
point[0]: s=0.0,  t=0.0, v=0.0,  a=2.0
point[1]: s=0.1,  t=0.1, v=0.2,  a=2.0
point[2]: s=0.4,  t=0.2, v=0.4,  a=2.0
...
```

**生成过程：**

```cpp
t = 0.0s:
  1. SpeedData.EvaluateByTime(0.0) → s=0.0, v=0.0, a=2.0
  2. PathData.GetPathPointWithPathS(0.0) → x=0.0, y=0.0, θ=0.0, κ=0.0
  3. TrajectoryPoint[0] = {x=0.0, y=0.0, θ=0.0, κ=0.0, s=0.0, 
                            v=0.0, a=2.0, t=0.0}

t = 0.02s:
  1. SpeedData.EvaluateByTime(0.02) → s=0.002, v=0.04, a=2.0 (线性插值)
  2. PathData.GetPathPointWithPathS(0.002) → x=0.002, y=0.0, θ=0.0, κ=0.0
  3. TrajectoryPoint[1] = {x=0.002, y=0.0, θ=0.0, κ=0.0, s=0.002,
                            v=0.04, a=2.0, t=0.02}

t = 0.1s:
  1. SpeedData.EvaluateByTime(0.1) → s=0.1, v=0.2, a=2.0
  2. PathData.GetPathPointWithPathS(0.1) → x=0.1, y=0.0, θ=0.0, κ=0.0
  3. TrajectoryPoint[5] = {x=0.1, y=0.0, θ=0.0, κ=0.0, s=0.1,
                            v=0.2, a=2.0, t=0.1}
...
```

### 13.5 特殊情况处理

#### 1. 倒车轨迹

对于倒车路径，需要调整速度和s的符号：

```cpp
if (path_data_.is_reverse_path()) {
  for (auto& point : trajectory) {
    point.set_v(-point.v());    // 速度取负
    point.set_a(-point.a());    // 加速度取负
    point.path_point().set_s(-point.path_point().s());  // s取负
  }
}
```

#### 2. 速度超出路径长度

当速度曲线规划的距离超过路径长度时，提前截断：

```cpp
if (speed_point.s() > path_data_.discretized_path().Length()) {
  break;  // 停止采样
}
```

#### 3. 边界情况

- **时间超出范围**：使用首/尾点
- **s超出范围**：使用首/尾点
- **插值精度**：使用线性插值（对于小间隔足够精确）

### 13.6 调用位置

轨迹组合在规划流程的最后阶段进行：

```cpp
// OnLanePlanning::RunOnce()
bool OnLanePlanning::RunOnce(const LocalView& local_view,
                             ADCTrajectory* const trajectory_pb) {
  // ... 路径规划 ...
  // ... 速度规划 ...
  
  // 选择最优参考线
  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  
  // 组合路径和速度生成轨迹
  const auto& path_data = best_ref_info->path_data();
  const auto& speed_data = best_ref_info->speed_data();
  
  DiscretizedTrajectory discretized_trajectory;
  best_ref_info->CombinePathAndSpeedProfile(
      0.0,  // relative_time
      path_data.frenet_frame_path().front().s(),  // start_s
      &discretized_trajectory);
  
  // 转换为protobuf格式发布
  discretized_trajectory.PopulateTrajectoryProtobuf(trajectory_pb);
  
  return true;
}
```

### 13.7 输出轨迹格式

最终生成的`ADCTrajectory`发布给控制模块：

```protobuf
message ADCTrajectory {
  optional Header header = 1;
  
  // 轨迹点序列（通常120个点，覆盖8秒）
  repeated TrajectoryPoint trajectory_point = 2;
  
  // 轨迹类型
  optional TrajectoryType trajectory_type = 3;
  
  // 总路径长度
  optional double total_path_length = 4;
  
  // 总时间长度
  optional double total_path_time = 5;
  
  // 规划延迟
  optional double latency_stats = 6;
  
  // 决策信息
  optional DecisionResult decision = 7;
  
  // 是否重规划
  optional bool is_replan = 8;
  
  // 档位
  optional canbus.Chassis.GearPosition gear = 9;
}
```

### 13.8 性能优化

1. **变分辨率采样**：近处密集远处稀疏，平衡精度和数据量
2. **二分查找**：O(log n)复杂度查找插值点
3. **预分配内存**：避免频繁的内存分配
4. **缓存查询结果**：对于相同的s/t查询使用缓存

### 13.9 质量检查

在组合过程中进行质量检查：

```cpp
// 检查轨迹连续性
for (size_t i = 1; i < trajectory.size(); ++i) {
  const auto& prev = trajectory[i-1];
  const auto& curr = trajectory[i];
  
  // 时间单调递增
  ACHECK(curr.relative_time() > prev.relative_time());
  
  // s单调递增（前进）
  if (!is_reverse) {
    ACHECK(curr.path_point().s() >= prev.path_point().s());
  }
  
  // 加速度连续性（避免突变）
  double da = (curr.a() - prev.a()) / (curr.relative_time() - prev.relative_time());
  ACHECK(std::abs(da) < max_jerk);
}
```

## 14. 参考文献

1. **Optimal Trajectory Generation for Autonomous Vehicles Under Centripetal Acceleration Constraints for In-lane Driving Scenarios**

2. **DL-IAPS and PJSO: A Path/Speed Decoupled Trajectory Optimization and its Application in Autonomous Driving**

3. **Apollo Planning代码仓库**
   - [SpeedOptimizer](https://github.com/ApolloAuto/apollo/tree/master/modules/planning/planning_interface_base/task_base/common)
   - [Speed Tasks](https://github.com/ApolloAuto/apollo/tree/master/modules/planning/tasks)

4. **Apollo官方文档**
   - [QP Spline ST Speed Optimizer](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/qp_spline_st_speed_optimizer.md)
   - [Piecewise Jerk Speed Optimizer](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/planning_piecewise_jerk_nonlinear_speed_optimizer.md)

## 15. 总结

### 15.1 完整的速度规划流程

Apollo的速度规划系统采用完整的分层架构，从输入到输出的完整流程如下：

```
1. 消息触发阶段（10Hz）
   └─ PredictionObstacles消息到达 → PlanningComponent::Proc()

2. ST图构建阶段
   ├─ 障碍物ST边界映射（STBoundaryMapper）
   ├─ 速度限制计算（SpeedLimitDecider）
   └─ ST图数据加载（StGraphData）

3. 速度优化阶段
   ├─ 选择优化算法（DP / QP / NLP）
   ├─ 构建目标函数和约束
   ├─ 求解优化问题
   └─ 生成速度曲线（SpeedData）

4. 轨迹组合阶段
   ├─ 时间域采样（变分辨率）
   ├─ 速度点插值（EvaluateByTime）
   ├─ 路径点插值（GetPathPointWithPathS）
   └─ 生成完整轨迹（DiscretizedTrajectory）

5. 输出发布阶段
   └─ ADCTrajectory → Control模块
```

### 15.2 主要特点

1. **消息驱动架构**
   - 由Prediction消息触发，保证数据实时性
   - 运行频率10Hz，满足实时性要求
   - 数据同步机制保证输入一致性

2. **多种优化算法**
   - **DP算法**：处理非凸问题，适合复杂场景
   - **QP算法**：快速求解，适合实时系统
   - **NLP算法**：精细优化，考虑非线性约束

3. **完善的约束处理**
   - 动力学约束（速度、加速度、加加速度）
   - 障碍物约束（ST边界）
   - 速度限制约束（地图、曲率、避让）
   - 舒适性约束（横向加速度）

4. **路径速度解耦**
   - 路径规划关注空间安全性
   - 速度规划关注时间舒适性
   - 最后通过插值方法优雅地结合

5. **高效的实现**
   - 优化算法在保证解质量的同时满足实时性
   - 变分辨率采样平衡精度和效率
   - 二分查找和线性插值保证性能

6. **良好的可扩展性**
   - 基于插件架构，方便添加新的优化器
   - 统一的接口定义，易于维护
   - 灵活的配置参数系统

7. **丰富的调试工具**
   - ST图可视化
   - 详细的日志信息
   - DreamView实时监控

### 15.3 核心数据流

```
输入数据：
  PathData (x, y, θ, κ, s)  +  InitPoint (v, a)  +  STBoundaries  +  SpeedLimits
                                      ↓
速度优化器：
  SpeedOptimizer → SpeedData (s, t, v, a, da)
                                      ↓
轨迹组合器：
  CombinePathAndSpeedProfile() → DiscretizedTrajectory (x,y,θ,κ,s,v,a,t)
                                      ↓
输出数据：
  ADCTrajectory → Control模块 → 车辆执行
```

### 15.4 关键技术点

| 技术点 | 说明 | 优势 |
|--------|------|------|
| ST图建模 | 将3D问题降维到2D | 简化问题复杂度 |
| 二次规划 | 凸优化快速求解 | 实时性好，解质量高 |
| 样条插值 | 5次多项式表示 | 平滑性好，连续可导 |
| 变分辨率采样 | 近密远疏 | 平衡精度和效率 |
| 线性插值 | 双向插值组合 | 计算简单，效果好 |
| 消息触发 | 事件驱动架构 | 数据同步，低延迟 |

### 15.5 应用建议

1. **算法选择**
   - 简单场景：`PiecewiseJerkSpeedOptimizer`
   - 复杂场景：`PiecewiseJerkSpeedNonlinearOptimizer`
   - 非凸场景：`PathTimeHeuristicOptimizer`

2. **参数调优**
   - 增大`jerk_weight`提高平滑性
   - 增大`ref_speed_weight`提高跟踪性能
   - 调整`safe_distance_buffer`平衡安全和效率

3. **性能优化**
   - 减少优化时域长度
   - 降低采样密度
   - 使用QP代替NLP

4. **质量保证**
   - 检查轨迹连续性
   - 验证约束满足情况
   - 监控计算耗时

### 15.6 典型问题排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 优化失败 | ST边界过严 | 放宽约束，使用软约束 |
| 速度抖动 | 权重不合理 | 增大平滑性权重 |
| 跟车不稳 | 预测不准 | 改进预测，增加buffer |
| 计算超时 | 参数设置不当 | 减少时域，降低精度 |
| 轨迹不连续 | 插值问题 | 检查采样间隔，增加点数 |

### 15.7 未来展望

1. **算法改进**
   - 引入学习方法优化参数
   - 考虑更多舒适性指标
   - 支持多模态优化

2. **性能提升**
   - GPU加速优化求解
   - 预测性优化
   - 分布式计算

3. **功能扩展**
   - 更多场景支持
   - 自适应参数调整
   - 交互式规划

通过深入理解Apollo速度规划的完整流程，从触发机制、ST图构建、优化求解到轨迹组合，开发者可以更好地使用、调试和扩展这个强大的规划系统，在不同驾驶场景下生成安全、舒适、高效的速度规划结果。

---

**文档版本**：v2.0  
**创建日期**：2025-10-10  
**最后更新**：2025-10-10  
**适用版本**：Apollo 9.0+  
**维护者**：Apollo Planning Team

