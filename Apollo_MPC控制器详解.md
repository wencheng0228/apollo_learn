# Apollo MPC控制器详解

> 基于Apollo源代码：`modules/control/controllers/mpc_controller/`

## 目录

1. [关键公式速查](#关键公式速查)
2. [控制器概述](#控制器概述)
3. [状态与控制变量](#状态与控制变量)
4. [车辆动力学模型](#车辆动力学模型)
5. [代码实现详解](#代码实现详解)
6. [MPC优化问题](#mpc优化问题数学表达式)
7. [OSQP求解器](#osqp求解器)
8. [前馈补偿](#前馈补偿数学推导)
9. [配置参数](#配置参数详解)
10. [调参指南](#调试与调参指南)

---

## 关键公式速查

### 状态空间模型

**连续时间**：
```
ẋ = A_c(v_x)·x + B_c·u

x = [e_y, ė_y, e_φ, ė_φ, e_s, e_v]ᵀ ∈ ℝ⁶
u = [δ_f, a]ᵀ ∈ ℝ²
```

**离散时间（Tustin）**：
```
x(k+1) = A_d·x(k) + B_d·u(k)

A_d = (I - T_s/2·A_c)^(-1)·(I + T_s/2·A_c)
B_d = T_s·B_c
```

### A矩阵关键元素

```
A(1,1) = -(c_f+c_r)/(m·v_x)           # 横向速度阻尼
A(1,2) = (c_f+c_r)/m                   # 轮胎侧偏力系数
A(1,3) = (l_r·c_r-l_f·c_f)/(m·v_x) - v_x  # 横摆耦合
A(3,1) = (l_r·c_r-l_f·c_f)/(I_z·v_x)  # 横向力矩系数
A(3,3) = -(l_f²·c_f+l_r²·c_r)/(I_z·v_x)  # 横摆阻尼
```

### B矩阵

```
B(1,0) = c_f/m                 # 前轮转角→横向加速度
B(3,0) = l_f·c_f/I_z           # 前轮转角→横摆角加速度
B(5,1) = -1                    # 加速度指令→速度误差变化率
```

### 误差计算公式

```
# 横向误差（Frenet坐标系）
e_y = cos(θ_ref)·(y - y_ref) - sin(θ_ref)·(x - x_ref)

# 航向角误差
e_φ = normalize(θ - θ_ref) ∈ [-π, π]

# 横向误差变化率
ė_y = v_x·sin(e_φ)

# 航向角误差变化率
ė_φ = ω - κ_ref·v_ref
```

### MPC优化问题

```
minimize   Σ[k=1 to N] x(k)ᵀ·Q·x(k) + Σ[k=0 to N-1] u(k)ᵀ·R·u(k)
   ξ

subject to:
  x(k+1) = A_d·x(k) + B_d·u(k)  (动力学约束)
  x(0) = x_current                (初始状态)
  u_min ≤ u(k) ≤ u_max           (控制约束)
  x_min ≤ x(k) ≤ x_max           (状态约束)

决策变量：ξ = [x(0), x(1), ..., x(N), u(0), u(1), ..., u(N-1)]ᵀ ∈ ℝ⁸⁶
```

### 代价函数权重

```
Q = diag(q₁, q₂, q₃, q₄, q₅, q₆)
典型值：[0.05, 0, 1.0, 0, 0.1, 0.1]
       ↑ lateral_error权重最关键

R = diag(r₁, r₂)
典型值：[0.01, 0.01]
       ↑ 越大越平滑
```

### 前馈补偿

```
# 运动学前馈（低速/倒车）
δ_ff = L·κ

# 动力学前馈（高速）
δ_ff = L·κ + K_v·v²·κ

其中：K_v = l_r·m/(2·c_f·L) - l_f·m/(2·c_r·L)
```

### 约束边界

```
# 前轮转角
δ_f ∈ [-δ_max, δ_max]
典型值：±0.515 rad (±29.5°)

# 加速度
a ∈ [a_min, a_max]
典型值：[-4.0, 2.0] m/s²

# 航向角误差
e_φ ∈ [-π, π]
```

---

## 控制器概述

### 代码文件结构

```
modules/control/controllers/mpc_controller/
├── mpc_controller.h              # 主控制器头文件
├── mpc_controller.cc             # 主控制器实现
├── proto/
│   └── mpc_controller.proto      # 配置文件定义
└── README_cn.md                  # 说明文档

modules/common/math/
├── mpc_osqp.h                    # OSQP求解器封装
└── mpc_osqp.cc
```

### 控制器特点

Apollo的MPC控制器是**横向+纵向联合控制器**，同时输出方向盘转角和加速度命令。

**关键特性**：
- 预测时域：`horizon_ = 10`步
- 采样时间：`ts_ = 0.01`秒（默认）
- 状态维度：6维（横向误差、航向角误差、纵向误差）
- 控制维度：2维（方向盘、加速度）

---

## 状态与控制变量

### 状态变量定义

代码位置：`mpc_controller.h:187`

```cpp
const int basic_state_size_ = 6;
```

6维状态向量（`mpc_controller.cc:724-748`）：

```cpp
// 状态矩阵更新
matrix_state_(0, 0) = debug->lateral_error();           // 横向误差 (m)
matrix_state_(1, 0) = debug->lateral_error_rate();      // 横向误差变化率 (m/s)
matrix_state_(2, 0) = debug->heading_error();           // 航向角误差 (rad)
matrix_state_(3, 0) = debug->heading_error_rate();      // 航向角误差变化率 (rad/s)
matrix_state_(4, 0) = debug->station_error();           // 纵向位置误差 (m)
matrix_state_(5, 0) = debug->speed_error();             // 速度误差 (m/s)
```

**状态向量定义**：
```
x = [lateral_error,          // e_y：横向偏差
     lateral_error_rate,     // e_y'：横向偏差变化率
     heading_error,          // e_φ：航向角偏差  
     heading_error_rate,     // e_φ'：航向角偏差变化率
     station_error,          // e_s：纵向位置偏差
     speed_error]ᵀ           // e_v：速度偏差
```

### 控制变量定义

代码位置：`mpc_controller.h:189`

```cpp
const int controls_ = 2;
```

2维控制向量（`mpc_controller.cc:482-483`）：

```cpp
control[0](0, 0) = control_cmd.at(0);  // 前轮转角 (rad)
control[0](1, 0) = control_cmd.at(1);  // 加速度 (m/s²)
```

**控制向量定义**：
```
u = [δ_f,  // 前轮转角 (steering angle)
     a]ᵀ   // 加速度 (acceleration)
```

### 约束边界

代码位置：`mpc_controller.cc:445-459`

```cpp
// 控制量约束
Matrix lower_bound(controls_, 1);
lower_bound << -wheel_single_direction_max_degree_, max_deceleration_;

Matrix upper_bound(controls_, 1);
upper_bound << wheel_single_direction_max_degree_, max_acceleration_;

// 状态约束
const double max = std::numeric_limits<double>::max();
Matrix lower_state_bound(basic_state_size_, 1);
Matrix upper_state_bound(basic_state_size_, 1);

// lateral_error, lateral_error_rate, heading_error, heading_error_rate
// station_error, station_error_rate
lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max,
    -1.0 * max, -1.0 * max;
upper_state_bound << max, max, M_PI, max, max, max;
```

**典型约束值**：
- 前轮转角：`±wheel_single_direction_max_degree_`（通常±0.5 rad）
- 加速度：`[max_deceleration_, max_acceleration_]`（通常[-4.0, 2.0] m/s²）
- 航向角误差：`±π`

---

## 车辆动力学模型

### 模型类型

Apollo MPC使用**线性二自由度自行车模型**（Bicycle Model）。

代码位置：`mpc_controller.cc:216-243`

### 数学符号定义

| 符号 | 含义 | 单位 | 代码变量 |
|------|------|------|---------|
| `e_y` | 横向误差（lateral error） | m | `lateral_error` |
| `ė_y` | 横向误差变化率 | m/s | `lateral_error_rate` |
| `e_φ` | 航向角误差（heading error） | rad | `heading_error` |
| `ė_φ` | 航向角误差变化率 | rad/s | `heading_error_rate` |
| `e_s` | 纵向位置误差（station error） | m | `station_error` |
| `e_v` | 速度误差（speed error） | m/s | `speed_error` |
| `δ_f` | 前轮转角（steering angle） | rad | `control[0](0,0)` |
| `a` | 加速度（acceleration） | m/s² | `control[0](1,0)` |
| `v_x` | 纵向速度 | m/s | `linear_velocity()` |
| `ω` | 横摆角速度 | rad/s | `angular_velocity()` |
| `m` | 车辆质量 | kg | `mass_` |
| `I_z` | 绕z轴转动惯量 | kg·m² | `iz_` |
| `l_f` | 质心到前轴距离 | m | `lf_` |
| `l_r` | 质心到后轴距离 | m | `lr_` |
| `c_f` | 前轮侧偏刚度 | N/rad | `cf_` |
| `c_r` | 后轮侧偏刚度 | N/rad | `cr_` |

### 连续时间状态方程

**状态向量**：
```
x = [e_y, ė_y, e_φ, ė_φ, e_s, e_v]ᵀ ∈ ℝ⁶
```

**控制向量**：
```
u = [δ_f, a]ᵀ ∈ ℝ²
```

**连续时间状态空间方程**：
```
ẋ = A_c·x + B_c·u

其中：
ẋ = [ė_y, ë_y, ė_φ, ë_φ, ė_s, ė_v]ᵀ
```

**代码实现**：

```cpp
// Matrix init operations.
matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
matrix_a_(0, 1) = 1.0;
matrix_a_(1, 2) = (cf_ + cr_) / mass_;
matrix_a_(2, 3) = 1.0;
matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
matrix_a_(4, 5) = 1.0;
matrix_a_(5, 5) = 0.0;

matrix_a_coeff_ = Matrix::Zero(basic_state_size_, basic_state_size_);
matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
matrix_a_coeff_(2, 3) = 1.0;
matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
```

### A矩阵数学推导

**前向驱动模式**（`mpc_controller.cc:373-386`）：

```cpp
// A matrix (Gear Drive)
matrix_a_(0, 1) = 1.0;
matrix_a_coeff_(0, 2) = 0.0;
```

**A_c 矩阵的完整数学表达式**：

```
      ┌                                                                    ┐
      │  0           1                0                    0          0  0 │  ← ė_y方程
      │                                                                    │
      │  0    -(c_f+c_r)        (c_f+c_r)     (l_r·c_r-l_f·c_f)           │
A_c = │       ─────────          ────────     ─────────────────  - v_x  0 0│  ← ë_y方程
      │         m·v_x               m              m·v_x                   │
      │                                                                    │
      │  0           0                0                    1          0  0 │  ← ė_φ方程
      │                                                                    │
      │  0    (l_r·c_r-l_f·c_f)  (l_f·c_f-l_r·c_r)  -(l_f²·c_f+l_r²·c_r) │
      │       ─────────────────  ────────────────   ─────────────────  0 0│  ← ë_φ方程
      │           I_z·v_x              I_z               I_z·v_x          │
      │                                                                    │
      │  0           0                0                    0          0  1 │  ← ė_s方程
      │                                                                    │
      │  0           0                0                    0          0  0 │  ← ė_v方程
      └                                                                    ┘
```

**A矩阵分块结构**：

```
      ┌────────────────┬─────┐
      │  横向动力学块    │  0  │
A_c = │  (4×4)         │     │
      ├────────────────┼─────┤
      │      0         │纵向块│
      │                │(2×2)│
      └────────────────┴─────┘
```

**各元素的物理意义**：

| 矩阵元素 | 数学表达式 | 代码 | 物理意义 |
|---------|-----------|------|---------|
| A(0,1) | 1 | `matrix_a_(0, 1) = 1.0` | ė_y = ė_y（定义） |
| A(1,1) | `-(c_f+c_r)/(m·v_x)` | `matrix_a_coeff_(1,1) / v` | 横向速度阻尼 |
| A(1,2) | `(c_f+c_r)/m` | `(cf_+cr_)/mass_` | 轮胎侧偏力→横向加速度 |
| A(1,3) | `(l_r·c_r-l_f·c_f)/(m·v_x) - v_x` | `matrix_a_coeff_(1,3)/v` | 横摆角速度对横向加速度的影响 |
| A(2,3) | 1 | `matrix_a_(2, 3) = 1.0` | ė_φ = ė_φ（定义） |
| A(3,1) | `(l_r·c_r-l_f·c_f)/(I_z·v_x)` | `matrix_a_coeff_(3,1) / v` | 横向速度→横摆力矩 |
| A(3,2) | `(l_f·c_f-l_r·c_r)/I_z` | `(lf_*cf_-lr_*cr_)/iz_` | 轮胎侧偏力矩→横摆角加速度 |
| A(3,3) | `-(l_f²·c_f+l_r²·c_r)/(I_z·v_x)` | `matrix_a_coeff_(3,3) / v` | 横摆角速度阻尼 |
| A(4,5) | 1 | `matrix_a_(4, 5) = 1.0` | ė_s = e_v（位置积分速度） |
| A(5,5) | 0 | `matrix_a_(5, 5) = 0.0` | 速度误差不自衰减 |

**倒车模式**（`mpc_controller.cc:358-371`）：

```cpp
// A matrix (Gear Reverse)
cf_ = -control_conf_.cf();
cr_ = -control_conf_.cr();
matrix_a_(0, 1) = 0.0;
matrix_a_coeff_(0, 2) = 1.0;
```

倒车时的A矩阵：
```
     [0,        0,               1.0*v,                 0,    0, 0]
     [0, -(cf+cr)/(m*v),     (cf+cr)/m,    (lr*cr-lf*cf)/(m*v), 0, 0]
A =  [0,        0,                     0,                 1,    0, 0]
     [0, (lr*cr-lf*cf)/(Iz*v), (lf*cf-lr*cr)/Iz, -(lf²*cf+lr²*cr)/(Iz*v), 0, 0]
     [0,        0,                     0,                 0,    0, 1]
     [0,        0,                     0,                 0,    0, 0]
```

### B矩阵数学推导

代码位置：`mpc_controller.cc:234-240`

```cpp
matrix_b_ = Matrix::Zero(basic_state_size_, controls_);
matrix_bd_ = Matrix::Zero(basic_state_size_, controls_);
matrix_b_(1, 0) = cf_ / mass_;
matrix_b_(3, 0) = lf_ * cf_ / iz_;
matrix_b_(4, 1) = 0.0;
matrix_b_(5, 1) = -1.0;
matrix_bd_ = matrix_b_ * ts_;
```

**B_c 矩阵的完整数学表达式**：

```
      ┌            ┐
      │   0     0  │  ← 对ė_y的影响（无直接影响）
      │            │
      │  c_f       │
B_c = │  ───    0  │  ← 对ë_y的影响（前轮转角产生侧向力）
      │   m        │
      │            │
      │   0     0  │  ← 对ė_φ的影响（无直接影响）
      │            │
      │ l_f·c_f    │
      │ ──────  0  │  ← 对ë_φ的影响（前轮转角产生横摆力矩）
      │   I_z      │
      │            │
      │   0     0  │  ← 对ė_s的影响（无直接影响）
      │            │
      │   0    -1  │  ← 对ė_v的影响（加速度指令）
      └            ┘
```

**控制输入的物理影响**：

| 控制输入 | 影响的状态 | 数学关系 | 代码 |
|---------|-----------|---------|------|
| `δ_f` | `ë_y` | `ë_y += (c_f/m)·δ_f` | `matrix_b_(1,0) = cf_/mass_` |
| `δ_f` | `ë_φ` | `ë_φ += (l_f·c_f/I_z)·δ_f` | `matrix_b_(3,0) = lf_*cf_/iz_` |
| `a` | `ė_v` | `ė_v = -a` | `matrix_b_(5,1) = -1.0` |

**物理解释**：

1. **前轮转角 δ_f**：
   - 产生前轮侧偏力：`F_yf = c_f·α_f ≈ c_f·δ_f`
   - 侧向力产生横向加速度：`ë_y = F_yf/m = (c_f/m)·δ_f`
   - 侧向力产生横摆力矩：`M_z = l_f·F_yf` → `ë_φ = M_z/I_z = (l_f·c_f/I_z)·δ_f`

2. **加速度 a**：
   - 直接作用于纵向动力学
   - 速度误差变化率：`ė_v = v_ref - v = -(v̇ - v̇_ref) = -a`（假设参考加速度为0）

### 车辆参数

代码位置：`mpc_controller.cc:77-115`

```cpp
// 从配置文件加载参数
cf_ = control_conf_.cf();           // 前轮侧偏刚度 (N/rad)
cr_ = control_conf_.cr();           // 后轮侧偏刚度 (N/rad)
wheelbase_ = vehicle_param_.wheel_base();  // 轴距 (m)
steer_ratio_ = vehicle_param_.steer_ratio();  // 方向盘传动比
mass_ = mass_front + mass_rear;     // 车辆质量 (kg)

// 计算质心位置
lf_ = wheelbase_ * (1.0 - mass_front / mass_);  // 质心到前轴距离
lr_ = wheelbase_ * (1.0 - mass_rear / mass_);   // 质心到后轴距离

// 计算转动惯量
iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;  // 绕z轴转动惯量
```

**典型数值**：
- `cf = cr = 155494.663 N/rad`
- `wheelbase = 2.85 m`
- `mass ≈ 1500 kg`
- `lf ≈ 1.2 m, lr ≈ 1.3 m`
- `iz ≈ 3000 kg·m²`

### 离散化方法

代码位置：`mpc_controller.cc:758-760`

```cpp
Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
             (matrix_i + ts_ * 0.5 * matrix_a_);
```

**离散化数学公式**：

使用**双线性变换（Tustin方法/梯形积分）**：

```
连续系统：ẋ(t) = A_c·x(t) + B_c·u(t)

离散化后：x(k+1) = A_d·x(k) + B_d·u(k)
```

**Tustin变换公式**：

```
A_d = (I - T_s/2 · A_c)^(-1) · (I + T_s/2 · A_c)

B_d = T_s · B_c

其中：T_s = 采样时间（默认0.01秒）
```

**代码对应**：
```cpp
// 计算 (I - Ts/2·A)
Matrix temp1 = I - ts_ * 0.5 * matrix_a_;

// 计算 (I + Ts/2·A)
Matrix temp2 = I + ts_ * 0.5 * matrix_a_;

// A_d = temp1^(-1) · temp2
matrix_ad_ = temp1.inverse() * temp2;

// B_d = Ts · B
matrix_bd_ = matrix_b_ * ts_;
```

**Tustin法的优势**：

| 方法 | 公式 | 精度 | 稳定性 |
|------|------|------|--------|
| 前向欧拉 | `A_d = I + T_s·A_c` | O(T_s) | 条件稳定 |
| 后向欧拉 | `A_d = (I - T_s·A_c)^(-1)` | O(T_s) | 无条件稳定 |
| **Tustin** | `A_d = (I-T_s/2·A_c)^(-1)(I+T_s/2·A_c)` | **O(T_s²)** | **无条件稳定** |

**离散系统方程**：

```
x(k+1) = A_d(v_x)·x(k) + B_d·u(k)

展开为：
┌     ┐       ┌         ┐   ┌     ┐       ┌         ┐   ┌     ┐
│ e_y │       │         │   │ e_y │       │         │   │ δ_f │
│ ė_y │       │         │   │ ė_y │       │         │   │  a  │
│ e_φ │   =   │   A_d   │ · │ e_φ │   +   │   B_d   │ · └     ┘
│ ė_φ │       │         │   │ ė_φ │       │         │
│ e_s │       │         │   │ e_s │       │         │
│ e_v │(k+1)  │         │   │ e_v │(k)    │         │
└     ┘       └         ┘   └     ┘       └         ┘
```

### 速度相关的矩阵更新

代码位置：`mpc_controller.cc:750-757`

```cpp
void MPCController::UpdateMatrix(SimpleMPCDebug *debug) {
  const double v = std::max(injector_->vehicle_state()->linear_velocity(),
                            minimum_speed_protection_);
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
  // ... 离散化
}
```

**关键点**：
- A矩阵的某些元素与速度v成反比
- 需要在每个控制周期根据当前车速更新矩阵
- 设置最小速度保护（`minimum_speed_protection_ = 0.1 m/s`）避免除零

---

## 代码实现详解

### 初始化流程

代码位置：`mpc_controller.cc:196-281`

```cpp
Status MPCController::Init(std::shared_ptr<DependencyInjector> injector) {
  // 1. 加载配置文件
  if (!ControlTask::LoadConfig<MPCControllerConf>(&control_conf_)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_INIT_ERROR, "...");
  }
  
  // 2. 加载控制参数
  if (!LoadControlConf()) {
    AERROR << "failed to load control conf";
    return Status(...);
  }
  
  // 3. 初始化矩阵 A, B
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  // ... (矩阵初始化代码见前文)
  
  // 4. 初始化权重矩阵 Q, R
  matrix_r_ = Matrix::Identity(controls_, controls_);
  matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  
  int r_param_size = control_conf_.matrix_r_size();
  for (int i = 0; i < r_param_size; ++i) {
    matrix_r_(i, i) = control_conf_.matrix_r(i);
  }
  
  int q_param_size = control_conf_.matrix_q_size();
  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf_.matrix_q(i);
  }
  
  // 5. 初始化滤波器
  InitializeFilters();
  
  // 6. 加载增益调度器
  LoadMPCGainScheduler();
  
  return Status::OK();
}
```

### 主控制循环

代码位置：`mpc_controller.cc:337-702`

```cpp
Status MPCController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    ControlCommand *cmd) {
  
  // 1. 初始化轨迹分析器
  trajectory_analyzer_ = std::move(TrajectoryAnalyzer(planning_published_trajectory));
  
  // 2. 计算纵向误差
  ComputeLongitudinalErrors(&trajectory_analyzer_, debug);
  
  // 3. 更新状态
  UpdateState(debug);
  
  // 4. 更新矩阵 A, B（依赖当前车速）
  UpdateMatrix(debug);
  
  // 5. 前馈更新
  FeedforwardUpdate(debug);
  
  // 6. 增益调度（根据车速调整权重）
  if (FLAGS_enable_gain_scheduler) {
    matrix_q_updated_(0, 0) = matrix_q_(0, 0) *
        lat_err_interpolation_->Interpolate(vehicle_state->linear_velocity());
    matrix_q_updated_(2, 2) = matrix_q_(2, 2) *
        heading_err_interpolation_->Interpolate(vehicle_state->linear_velocity());
    // ... 其他权重调整
  }
  
  // 7. 准备OSQP求解器
  apollo::common::math::MpcOsqp mpc_osqp(
      matrix_ad_, matrix_bd_, matrix_q_updated_, matrix_r_updated_,
      matrix_state_, lower_bound, upper_bound, lower_state_bound,
      upper_state_bound, reference_state, mpc_max_iteration_, horizon_, mpc_eps_);
  
  // 8. 求解QP问题
  if (!mpc_osqp.Solve(&control_cmd)) {
    AERROR << "MPC OSQP solver failed";
  } else {
    control[0](0, 0) = control_cmd.at(0);  // 方向盘
    control[0](1, 0) = control_cmd.at(1);  // 加速度
  }
  
  // 9. 转换为方向盘百分比
  steer_angle_feedback = Wheel2SteerPct(control[0](0, 0));
  acc_feedback = control[0](1, 0);
  
  // 10. 添加前馈补偿
  double steer_angle = steer_angle_feedback + steer_angle_feedforwardterm_updated_
                      + steer_angle_ff_compensation;
  
  // 11. 滤波和限幅
  steer_angle = digital_filter_.Filter(steer_angle);
  steer_angle = common::math::Clamp(steer_angle, -100.0, 100.0);
  cmd->set_steering_target(steer_angle);
  
  // 12. 纵向控制命令
  double acceleration_cmd = acc_feedback + debug->acceleration_reference();
  // ... 油门/刹车转换逻辑
  
  return Status::OK();
}
```

### 状态计算

代码位置：`mpc_controller.cc:724-748`

```cpp
void MPCController::UpdateState(SimpleMPCDebug *debug) {
  // 计算质心位置
  const auto &com = injector_->vehicle_state()->ComputeCOMPosition(lr_);
  
  // 计算横向误差
  ComputeLateralErrors(com.x(), com.y(),
                       injector_->vehicle_state()->heading(),
                       injector_->vehicle_state()->linear_velocity(),
                       injector_->vehicle_state()->angular_velocity(),
                       injector_->vehicle_state()->linear_acceleration(),
                       trajectory_analyzer_, debug);
  
  // 状态矩阵更新
  matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(3, 0) = debug->heading_error_rate();
  matrix_state_(4, 0) = debug->station_error();
  matrix_state_(5, 0) = debug->speed_error();
}
```

### 横向误差计算

代码位置：`mpc_controller.cc:782-908`

```cpp
void MPCController::ComputeLateralErrors(
    const double x, const double y, const double theta,
    const double linear_v, const double angular_v, const double linear_a,
    const TrajectoryAnalyzer &trajectory_analyzer, SimpleMPCDebug *debug) {
  
  // 1. 查找最近匹配点
  const auto matched_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);
  
  // 2. 计算横向误差
  const double dx = x - matched_point.path_point().x();
  const double dy = y - matched_point.path_point().y();
  const double cos_matched_theta = std::cos(matched_point.path_point().theta());
  const double sin_matched_theta = std::sin(matched_point.path_point().theta());
  
  double lateral_error = cos_matched_theta * dy - sin_matched_theta * dx;
  debug->set_lateral_error(lateral_error);
  
  // 3. 计算航向角误差
  debug->set_ref_heading(matched_point.path_point().theta());
  double delta_theta = common::math::NormalizeAngle(theta - debug->ref_heading());
  debug->set_heading_error(delta_theta);
  
  // 4. 计算横向误差变化率
  const double sin_delta_theta = std::sin(delta_theta);
  double lateral_error_dot = linear_v * sin_delta_theta;
  debug->set_lateral_error_rate(lateral_error_dot);
  
  // 5. 计算航向角误差变化率
  debug->set_curvature(matched_point.path_point().kappa());
  debug->set_ref_heading_rate(debug->curvature() * matched_point.v());
  debug->set_heading_error_rate(angular_v - debug->ref_heading_rate());
}
```

**误差计算数学公式**：

**1. 横向误差 e_y（垂直于参考轨迹）**：

```
设车辆当前位置：P_vehicle = (x, y)
    参考轨迹最近点：P_ref = (x_ref, y_ref)
    参考轨迹方向：θ_ref

位置偏差向量：Δp = P_vehicle - P_ref = [x - x_ref, y - y_ref]ᵀ

参考轨迹的切向量：t_ref = [cos(θ_ref), sin(θ_ref)]ᵀ
参考轨迹的法向量：n_ref = [-sin(θ_ref), cos(θ_ref)]ᵀ

横向误差（沿法向量方向）：
e_y = Δp · n_ref = (x - x_ref)·(-sin(θ_ref)) + (y - y_ref)·cos(θ_ref)
    = cos(θ_ref)·(y - y_ref) - sin(θ_ref)·(x - x_ref)
```

**对应代码**：
```cpp
const double dx = x - matched_point.path_point().x();
const double dy = y - matched_point.path_point().y();
lateral_error = cos_matched_theta * dy - sin_matched_theta * dx;
```

**2. 航向角误差 e_φ**：

```
e_φ = normalize(θ_vehicle - θ_ref) ∈ [-π, π]

其中normalize函数将角度归一化到[-π, π]区间
```

**对应代码**：
```cpp
double delta_theta = common::math::NormalizeAngle(theta - debug->ref_heading());
```

**3. 横向误差变化率 ė_y**：

```
ė_y = d(e_y)/dt

在Frenet坐标系下，横向误差变化率可近似为：
ė_y ≈ v_x · sin(e_φ)

其中 v_x 是车辆纵向速度
```

**推导**：
```
横向速度：v_y = ė_y
在小角度近似下，横向速度等于车辆速度在法向上的投影：
v_y = v · sin(θ_vehicle - θ_ref) = v · sin(e_φ)
```

**对应代码**：
```cpp
double lateral_error_dot = linear_v * sin_delta_theta;
```

**4. 航向角误差变化率 ė_φ**：

```
ė_φ = ω_vehicle - ω_ref

其中：
ω_vehicle：车辆横摆角速度（实测）
ω_ref：参考轨迹的角速度

参考轨迹的角速度可由曲率和速度计算：
ω_ref = κ_ref · v_ref

其中 κ_ref 是参考轨迹的曲率
```

**对应代码**：
```cpp
debug->set_ref_heading_rate(debug->curvature() * matched_point.v());
debug->set_heading_error_rate(angular_v - debug->ref_heading_rate());
```

**误差计算示意图**：

```
                      参考轨迹
                         ↓
        ┌────────────────●────────────────┐
        │          (x_ref, y_ref)         │
        │           θ_ref →               │
        │                                 │
        │                                 │
        │              ↗ e_y (横向误差)    │
        │             /                   │
        │            /                    │
        │           ● (x, y)              │
        │        车辆位置                  │
        │        θ_vehicle                │
        │                                 │
        │    e_φ = θ_vehicle - θ_ref     │
        │       (航向角误差)               │
        └─────────────────────────────────┘
```

### 纵向误差计算

代码位置：`mpc_controller.cc:910-969`

```cpp
void MPCController::ComputeLongitudinalErrors(
    const TrajectoryAnalyzer *trajectory_analyzer, SimpleMPCDebug *debug) {
  
  // 1. 查找匹配点
  const auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
      injector_->vehicle_state()->x(), injector_->vehicle_state()->y());
  
  // 2. 转换到Frenet坐标系
  double s_matched, s_dot_matched, d_matched, d_dot_matched;
  trajectory_analyzer->ToTrajectoryFrame(
      vehicle_state->x(), vehicle_state->y(), vehicle_state->heading(),
      vehicle_state->linear_velocity(), matched_point,
      &s_matched, &s_dot_matched, &d_matched, &d_dot_matched);
  
  // 3. 获取参考点
  const double current_control_time = Clock::NowInSeconds();
  TrajectoryPoint reference_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(current_control_time);
  
  // 4. 计算纵向误差
  debug->set_station_error(reference_point.path_point().s() - s_matched);
  debug->set_speed_error(reference_point.v() - s_dot_matched);
  debug->set_acceleration_error(reference_point.a() - lon_acceleration);
}
```

---

## MPC优化问题数学表达式

### 问题定义

**预测时域**：`N = horizon_ = 10`

**决策变量**：
```
ξ = [x(0), x(1), ..., x(N), u(0), u(1), ..., u(N-1)]ᵀ

其中：
x(k) ∈ ℝ⁶  (状态向量，k=0,1,...,N)
u(k) ∈ ℝ²  (控制向量，k=0,1,...,N-1)

总决策变量维度：n_ξ = 6×(N+1) + 2×N = 6×11 + 2×10 = 86
```

### MPC优化问题的完整形式

```
minimize   J(x, u)
  ξ

subject to:
  (1) x(k+1) = A_d·x(k) + B_d·u(k),  k=0,1,...,N-1  (动力学约束)
  (2) x(0) = x_current                              (初始状态约束)
  (3) u_min ≤ u(k) ≤ u_max,  k=0,1,...,N-1         (控制约束)
  (4) x_min ≤ x(k) ≤ x_max,  k=1,2,...,N           (状态约束)
```

### 代价函数

```
       N                    N-1
J = Σ  x(k)ᵀ·Q·x(k)  +  Σ  u(k)ᵀ·R·u(k)
     k=1                  k=0

其中：
Q ∈ ℝ⁶ˣ⁶  (状态权重矩阵，半正定)
R ∈ ℝ²ˣ²  (控制权重矩阵，正定)
```

**展开形式**：

```
J = Σ [q₁·e_y² + q₂·ė_y² + q₃·e_φ² + q₄·ė_φ² + q₅·e_s² + q₆·e_v²]
    + Σ [r₁·δ_f² + r₂·a²]
```

### QP标准形式

将MPC问题转换为标准QP：

```
minimize:   (1/2)·ξᵀ·P·ξ + qᵀ·ξ
   ξ

subject to: l ≤ A_cons·ξ ≤ u
```

其中：
- `ξ`：决策变量（状态+控制序列），维度86×1
- `P`：Hessian矩阵（二次项），维度86×86
- `q`：梯度向量（一次项），维度86×1
- `A_cons`：约束矩阵
- `l, u`：约束上下界

### 权重矩阵Q

代码位置：`mpc_controller.cc:250-268`

```cpp
matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);

int q_param_size = control_conf_.matrix_q_size();
for (int i = 0; i < q_param_size; ++i) {
  matrix_q_(i, i) = control_conf_.matrix_q(i);
}
```

Q矩阵是对角矩阵：
```
Q = diag(q0, q1, q2, q3, q4, q5)
```

**配置说明**（`mpc_controller.proto:24-26`）：
```protobuf
// output variable (control state) weight matrix
// (lateral_error, lateral_error_rate, heading_error, heading_error_rate,
//  station_error, speed_error)
repeated double matrix_q = 9;
```

**典型配置值**：
```
matrix_q: [0.05, 0.0, 1.0, 0.0, 0.1, 0.1]

对应：
q0 = 0.05  # lateral_error权重
q1 = 0.0   # lateral_error_rate权重（通常为0）
q2 = 1.0   # heading_error权重（最重要）
q3 = 0.0   # heading_error_rate权重（通常为0）
q4 = 0.1   # station_error权重
q5 = 0.1   # speed_error权重
```

### 权重矩阵R

代码位置：`mpc_controller.cc:248-255`

```cpp
matrix_r_ = Matrix::Identity(controls_, controls_);

int r_param_size = control_conf_.matrix_r_size();
for (int i = 0; i < r_param_size; ++i) {
  matrix_r_(i, i) = control_conf_.matrix_r(i);
}
```

R矩阵是对角矩阵：
```
R = diag(r0, r1)
```

**配置说明**（`mpc_controller.proto:28-30`）：
```protobuf
// manipulated variable weight matrix
// (steer, acceleration)
repeated double matrix_r = 10;
```

**典型配置值**：
```
matrix_r: [0.01, 0.01]

对应：
r0 = 0.01  # steering权重
r1 = 0.01  # acceleration权重
```

### 增益调度

代码位置：`mpc_controller.cc:405-423`

```cpp
if (FLAGS_enable_gain_scheduler) {
  // 根据车速调整横向误差权重
  matrix_q_updated_(0, 0) =
      matrix_q_(0, 0) *
      lat_err_interpolation_->Interpolate(vehicle_state->linear_velocity());
  
  // 根据车速调整航向角误差权重
  matrix_q_updated_(2, 2) =
      matrix_q_(2, 2) *
      heading_err_interpolation_->Interpolate(vehicle_state->linear_velocity());
  
  // 根据车速调整方向盘权重
  matrix_r_updated_(0, 0) =
      matrix_r_(0, 0) *
      steer_weight_interpolation_->Interpolate(vehicle_state->linear_velocity());
} else {
  matrix_q_updated_ = matrix_q_;
  matrix_r_updated_ = matrix_r_;
}
```

**增益调度的作用**：
- **低速**：增大横向误差权重，提高精度
- **高速**：增大航向角误差权重，保持稳定性
- **高速**：增大方向盘权重，避免过度转向

---

## 约束条件设置

### 控制约束

代码位置：`mpc_controller.cc:445-449`

```cpp
Matrix lower_bound(controls_, 1);
lower_bound << -wheel_single_direction_max_degree_, max_deceleration_;

Matrix upper_bound(controls_, 1);
upper_bound << wheel_single_direction_max_degree_, max_acceleration_;
```

**约束1：前轮转角**

```cpp
// mpc_controller.cc:100-101
wheel_single_direction_max_degree_ =
    steer_single_direction_max_degree_ / steer_ratio_ / 180 * M_PI;
```

计算过程：
```
方向盘最大转角（度）→ 前轮最大转角（度）→ 前轮最大转角（弧度）

示例：
steer_single_direction_max_degree = 470°  # 方向盘最大转角
steer_ratio = 16                          # 方向盘传动比
wheel_single_direction_max_degree = 470 / 16 / 180 * π ≈ 0.515 rad ≈ 29.5°
```

**约束2：加速度**

```cpp
// mpc_controller.cc:102-103
max_acceleration_ = vehicle_param_.max_acceleration();
max_deceleration_ = vehicle_param_.max_deceleration();
```

典型值：
- `max_acceleration_ = 2.0 m/s²`
- `max_deceleration_ = -4.0 m/s²`

### 状态约束

代码位置：`mpc_controller.cc:451-459`

```cpp
const double max = std::numeric_limits<double>::max();
Matrix lower_state_bound(basic_state_size_, 1);
Matrix upper_state_bound(basic_state_size_, 1);

lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max,
    -1.0 * max, -1.0 * max;
upper_state_bound << max, max, M_PI, max, max, max;
```

**实际约束**：
- 横向误差：无限制（用权重惩罚）
- 航向角误差：`[-π, π]`
- 其他状态：无限制

### 方向盘速率限制

代码在后处理中实现（`mpc_controller.cc:545-557`）：

```cpp
if (FLAGS_set_steer_limit) {
  const double steer_limit =
      std::atan(max_lat_acc_ * wheelbase_ /
                (vehicle_state->linear_velocity() *
                 vehicle_state->linear_velocity())) *
      steer_ratio_ * 180 / M_PI /
      steer_single_direction_max_degree_ * 100;
  
  double steer_angle_limited =
      common::math::Clamp(steer_angle, -steer_limit, steer_limit);
}
```

**侧向加速度约束**：

```
a_lat_max = v² / R_min
R_min = L / tan(δ_max)
δ_max = atan(a_lat_max * L / v²)
```

---

## OSQP求解器

### MpcOsqp类定义

代码位置：`modules/common/math/mpc_osqp.h:33-84`

```cpp
class MpcOsqp {
 public:
  MpcOsqp(const Eigen::MatrixXd &matrix_a, const Eigen::MatrixXd &matrix_b,
          const Eigen::MatrixXd &matrix_q, const Eigen::MatrixXd &matrix_r,
          const Eigen::MatrixXd &matrix_initial_x,
          const Eigen::MatrixXd &matrix_u_lower,
          const Eigen::MatrixXd &matrix_u_upper,
          const Eigen::MatrixXd &matrix_x_lower,
          const Eigen::MatrixXd &matrix_x_upper,
          const Eigen::MatrixXd &matrix_x_ref,
          const int max_iter, const int horizon, const double eps_abs);
  
  bool Solve(std::vector<double> *control_cmd);
  
 private:
  void CalculateKernel(/*...*/);          // 计算P矩阵
  void CalculateGradient();               // 计算q向量
  void CalculateEqualityConstraint(/*...*/);  // 计算A矩阵
  void CalculateConstraintVectors();      // 计算l, u向量
  // ...
};
```

### 求解器构造

代码位置：`mpc_controller.cc:473-477`

```cpp
apollo::common::math::MpcOsqp mpc_osqp(
    matrix_ad_,           // 离散化A矩阵
    matrix_bd_,           // 离散化B矩阵
    matrix_q_updated_,    // 状态权重矩阵
    matrix_r_updated_,    // 控制权重矩阵
    matrix_state_,        // 当前状态
    lower_bound,          // 控制下界
    upper_bound,          // 控制上界
    lower_state_bound,    // 状态下界
    upper_state_bound,    // 状态上界
    reference_state,      // 参考状态（通常为0）
    mpc_max_iteration_,   // 最大迭代次数
    horizon_,             // 预测时域
    mpc_eps_);            // 收敛阈值
```

### P矩阵计算（Hessian）

代码位置：`mpc_osqp.cc:53-92`

```cpp
void MpcOsqp::CalculateKernel(std::vector<c_float> *P_data,
                              std::vector<c_int> *P_indices,
                              std::vector<c_int> *P_indptr) {
  // 状态部分: Q矩阵的对角元素
  for (size_t i = 0; i <= horizon_; ++i) {
    for (size_t j = 0; j < state_dim_; ++j) {
      columns[i * state_dim_ + j].emplace_back(
          i * state_dim_ + j, matrix_q_(j, j));
    }
  }
  
  // 控制部分: R矩阵的对角元素
  const size_t state_total_dim = state_dim_ * (horizon_ + 1);
  for (size_t i = 0; i < horizon_; ++i) {
    for (size_t j = 0; j < control_dim_; ++j) {
      columns[i * control_dim_ + j + state_total_dim].emplace_back(
          state_total_dim + i * control_dim_ + j, matrix_r_(j, j));
    }
  }
}
```

**P矩阵的数学结构**：

```
P = diag(Q, Q, ..., Q, R, R, ..., R)
         ↑ N+1个      ↑ N个

详细展开（86×86）：
         ┌                                                  ┐
         │ Q  0  0 ... 0  |  0  0 ... 0                    │  ← x(0)
         │ 0  Q  0 ... 0  |  0  0 ... 0                    │  ← x(1)
         │ 0  0  Q ... 0  |  0  0 ... 0                    │  ← x(2)
         │ ...            |  ...                           │
         │ 0  0  0 ... Q  |  0  0 ... 0                    │  ← x(N)
P = 2·   │───────────────────────────────────              │
         │ 0  0  0 ... 0  |  R  0 ... 0                    │  ← u(0)
         │ 0  0  0 ... 0  |  0  R ... 0                    │  ← u(1)
         │ ...            |  ...                           │
         │ 0  0  0 ... 0  |  0  0 ... R                    │  ← u(N-1)
         └                                                  ┘

其中Q是6×6对角矩阵，R是2×2对角矩阵
```

**注意**：代码中P矩阵直接用于OSQP，而OSQP的QP形式是：
```
minimize (1/2)·ξᵀ·P·ξ + qᵀ·ξ
```

因此P中已经包含了因子2（对应代价函数中没有1/2）。

**P矩阵的稀疏性**：

```
非零元素数量 = (N+1)×state_dim + N×control_dim
             = 11×6 + 10×2
             = 66 + 20
             = 86

稀疏率 = 86 / (86×86) = 1.16%  ← 非常稀疏！
```

**Q矩阵块展开**（每个6×6块）：
```
     ┌                        ┐
     │ q₁  0   0   0   0   0  │
     │ 0   q₂  0   0   0   0  │
Q =  │ 0   0   q₃  0   0   0  │
     │ 0   0   0   q₄  0   0  │
     │ 0   0   0   0   q₅  0  │
     │ 0   0   0   0   0   q₆ │
     └                        ┘

典型值：Q = diag(0.05, 0, 1.0, 0, 0.1, 0.1)
```

**R矩阵块展开**（每个2×2块）：
```
     ┌        ┐
R =  │ r₁  0  │
     │ 0   r₂ │
     └        ┘

典型值：R = diag(0.01, 0.01)
```

**P矩阵维度验证**：
```
决策变量：ξ ∈ ℝ⁸⁶
P矩阵：P ∈ ℝ⁸⁶ˣ⁸⁶

分块：
- 状态块：66×66 (前11个6×6的Q矩阵)
- 控制块：20×20 (后10个2×2的R矩阵)
- 总计：86×86 ✓
```

### q向量计算（梯度）

代码位置：`mpc_osqp.cc:95-107`

```cpp
void MpcOsqp::CalculateGradient() {
  gradient_ = Eigen::VectorXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_);
  
  for (size_t i = 0; i < horizon_ + 1; ++i) {
    gradient_.block(i * state_dim_, 0, state_dim_, 1) =
        -1.0 * matrix_q_ * matrix_x_ref_;
  }
}
```

由于参考状态通常为0（`matrix_x_ref_ = 0`），梯度向量也为0：
```
q = 0
```

### A矩阵计算（动力学约束）

代码位置：`mpc_osqp.cc:109-192`

```cpp
void MpcOsqp::CalculateEqualityConstraint(std::vector<c_float> *A_data,
                                          std::vector<c_int> *A_indices,
                                          std::vector<c_int> *A_indptr) {
  // 1. 状态转移约束: x_{k+1} = A*x_k + B*u_k
  // 2. 初始状态约束: x_0 = x_init
  // 3. 状态边界约束: x_min ≤ x_k ≤ x_max
  // 4. 控制边界约束: u_min ≤ u_k ≤ u_max
}
```

### 求解过程

代码位置：`mpc_osqp.cc:281-340`

```cpp
bool MpcOsqp::Solve(std::vector<double> *control_cmd) {
  // 1. 计算梯度和约束
  CalculateGradient();
  CalculateConstraintVectors();
  
  // 2. 准备OSQP数据结构
  OSQPData *data = Data();
  OSQPSettings *settings = Settings();
  
  // 3. 设置求解器
  OSQPWorkspace *osqp_workspace = osqp_setup(data, settings);
  
  // 4. 求解
  osqp_solve(osqp_workspace);
  
  // 5. 检查状态
  auto status = osqp_workspace->info->status_val;
  if (status < 0 || (status != 1 && status != 2)) {
    AERROR << "failed optimization status";
    return false;
  }
  
  // 6. 提取控制命令（只用第一步）
  size_t first_control = state_dim_ * (horizon_ + 1);
  for (size_t i = 0; i < control_dim_; ++i) {
    control_cmd->at(i) = osqp_workspace->solution->x[i + first_control];
  }
  
  // 7. 清理
  osqp_cleanup(osqp_workspace);
  return true;
}
```

### OSQP设置

代码位置：`mpc_osqp.cc:219-234`

```cpp
OSQPSettings *MpcOsqp::Settings() {
  OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  osqp_set_default_settings(settings);
  
  settings->polish = true;              // 精炼解（提高精度）
  settings->scaled_termination = true;  // 缩放终止条件
  settings->verbose = false;            // 不打印详细信息
  settings->max_iter = max_iteration_;  // 最大迭代次数
  settings->eps_abs = eps_abs_;         // 绝对误差阈值
  
  return settings;
}
```

**典型配置**：
- `max_iteration = 150`
- `eps_abs = 1e-4`

---

## 前馈补偿数学推导

### 基础前馈

代码位置：`mpc_controller.cc:767-780`

```cpp
void MPCController::FeedforwardUpdate(SimpleMPCDebug *debug) {
  const double v = injector_->vehicle_state()->linear_velocity();
  const double kv =
      lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
  
  if (control_conf_.use_kinematic_model() &&
      injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    // 倒车时使用运动学模型
    steer_angle_feedforwardterm_ =
        Wheel2SteerPct(wheelbase_ * debug->curvature());
  } else {
    // 前进时使用动力学模型
    steer_angle_feedforwardterm_ = Wheel2SteerPct(
        wheelbase_ * debug->curvature() + kv * v * v * debug->curvature());
  }
}
```

### 前馈补偿数学公式

**1. 运动学前馈**（低速/倒车）：

**阿克曼转向几何**：

```
设车辆以恒定曲率 κ 行驶，前轮转角 δ_f，轴距 L

转弯半径：R = 1/κ

根据阿克曼几何：
tan(δ_f) ≈ L/R = L·κ  (小角度近似)

因此：
δ_ff = L·κ
```

**对应代码**：
```cpp
steer_angle_feedforwardterm_ = Wheel2SteerPct(wheelbase_ * debug->curvature());
```

**2. 动力学前馈**（高速）：

**稳态转向分析**：

在稳态转向时（ė_y = 0, ė_φ = 0），车辆以恒定曲率行驶：

```
横向动力学方程：
0 = -(c_f + c_r)/(m·v) · v_y + (c_f + c_r)/m · δ_f + [(l_r·c_r - l_f·c_f)/(m·v) - v] · ω

横摆动力学方程：
0 = (l_r·c_r - l_f·c_f)/(I_z·v) · v_y + (l_f·c_f - l_r·c_r)/I_z · δ_f - (l_f²·c_f + l_r²·c_r)/(I_z·v) · ω
```

在稳态转向时，横摆角速度与曲率关系：
```
ω = v·κ
```

求解稳态时的前轮转角 δ_f（稳态解）：

经过代数运算（消元求解），得到：

```
δ_ff = L·κ + K_v·v²·κ

其中：K_v 是稳定性因子
K_v = (l_r·m)/(2·c_f·L) - (l_f·m)/(2·c_r·L)
```

**对应代码**：
```cpp
const double kv = lr_ * mass_ / 2 / cf_ / wheelbase_ 
                - lf_ * mass_ / 2 / cr_ / wheelbase_;

steer_angle_feedforwardterm_ = Wheel2SteerPct(
    wheelbase_ * debug->curvature() + kv * v * v * debug->curvature());
```

**K_v 的物理意义**：

- `K_v > 0`：**不足转向**（前驱车常见），高速时需要更大转角
- `K_v = 0`：**中性转向**（理想情况，c_f/l_f = c_r/l_r）
- `K_v < 0`：**过度转向**（后驱车常见），高速时需要更小转角

**前馈补偿项分解**：

```
δ_ff = δ_kinematic + δ_dynamic

δ_kinematic = L·κ           ← 几何项（运动学）
δ_dynamic = K_v·v²·κ        ← 动力学项（轮胎侧偏）

总前馈转角：
δ_ff = L·κ·(1 + K_v·v²/L)
```

### 前馈补偿的作用

**为什么需要前馈？**

MPC只根据当前误差反馈控制，对于**期望轨迹的曲率**是被动响应的。前馈补偿可以：

1. **减少稳态误差**：直接补偿轨迹曲率所需的转角
2. **加快响应速度**：不等误差积累就施加控制
3. **提高跟踪精度**：特别是高曲率路段

**最终控制律**：

```
δ_total = δ_feedback + δ_feedforward + δ_compensation

其中：
δ_feedback：MPC优化输出（基于误差反馈）
δ_feedforward：基础前馈（基于轨迹曲率）
δ_compensation：高级补偿（见下节）
```

**对应代码**（`mpc_controller.cc:541-543`）：
```cpp
double steer_angle =
    steer_angle_feedback + steer_angle_feedforwardterm_updated_ +
    steer_angle_ff_compensation + steer_angle_feedback_augment;
```

### 高级前馈补偿

代码位置：`mpc_controller.cc:492-516`

```cpp
if (enable_mpc_feedforward_compensation_) {
  unconstrained_control_diff =
      Wheel2SteerPct(control[0](0, 0) - unconstrained_control);
  
  if (fabs(unconstrained_control_diff) <= unconstrained_control_diff_limit_) {
    // MPC求解结果接近无约束解，使用完整补偿
    steer_angle_ff_compensation =
        Wheel2SteerPct(debug->curvature() *
                       (control_gain[0](0, 2) *
                            (lr_ - lf_ / cr_ * mass_ * v * v / wheelbase_) -
                        addition_gain[0](0, 0) * v));
  } else {
    // MPC求解结果受约束影响，按比例缩放补偿
    control_gain_truncation_ratio = control[0](0, 0) / unconstrained_control;
    steer_angle_ff_compensation = (...)  * control_gain_truncation_ratio;
  }
}
```

**作用**：
- 当MPC解接近无约束解时，应用完整前馈补偿
- 当MPC解受约束限制时，按比例缩小补偿，避免过度前馈

---

## 配置参数详解

### 配置文件示例

```protobuf
ts: 0.01                          # 采样时间 10ms
cf: 155494.663                    # 前轮侧偏刚度
cr: 155494.663                    # 后轮侧偏刚度
mass_fl: 600                      # 前左质量
mass_fr: 600                      # 前右质量
mass_rl: 500                      # 后左质量
mass_rr: 500                      # 后右质量
eps: 0.01                         # 收敛阈值
max_iteration: 150                # 最大迭代次数
max_lateral_acceleration: 5.0     # 最大侧向加速度

# 状态权重 (6个)
matrix_q: [0.05, 0.0, 1.0, 0.0, 0.1, 0.1]

# 控制权重 (2个)
matrix_r: [0.01, 0.01]
```

### 关键参数说明

| 参数 | 含义 | 典型值 | 影响 |
|------|------|--------|------|
| `ts` | 采样时间 | 0.01s | 越小精度越高，计算量越大 |
| `cf`, `cr` | 轮胎侧偏刚度 | 155494 N/rad | 影响模型准确性 |
| `matrix_q[0]` | 横向误差权重 | 0.05 | 越大横向跟踪越紧 |
| `matrix_q[2]` | 航向角误差权重 | 1.0 | 越大姿态越稳定 |
| `matrix_r[0]` | 方向盘权重 | 0.01 | 越大方向盘越平滑 |
| `max_iteration` | 最大迭代次数 | 150 | 影响计算时间和精度 |
| `horizon` | 预测时域 | 10 | 越大前瞻性越强，计算量越大 |

---

## 调试与调参指南

### 权重调参步骤

**步骤1：设置基准值**
```
matrix_q: [0.05, 0.0, 1.0, 0.0, 0.1, 0.1]
matrix_r: [0.01, 0.01]
```

**步骤2：观察现象，调整权重**

| 现象 | 可能原因 | 调整方案 |
|------|---------|---------|
| 横向偏差大 | `q[0]`太小 | 增大`matrix_q[0]` |
| 姿态不稳 | `q[2]`太小 | 增大`matrix_q[2]` |
| 方向盘抖动 | `r[0]`太小 | 增大`matrix_r[0]` |
| 加速度波动 | `r[1]`太小 | 增大`matrix_r[1]` |
| 响应迟缓 | 权重比失衡 | 减小R或增大Q |

**步骤3：高速优化**

启用增益调度，根据车速自动调整权重：
```
enable_gain_scheduler: true

lat_err_gain_scheduler {
  scheduler { speed: 2.0  ratio: 1.0 }
  scheduler { speed: 10.0 ratio: 0.5 }
  scheduler { speed: 20.0 ratio: 0.2 }
}
```

### 常见问题排查

**问题1：MPC求解失败**

```
AERROR << "MPC OSQP solver failed"
```

**排查**：
1. 检查约束是否冲突
2. 增大`max_iteration`
3. 放宽`eps`阈值
4. 检查矩阵A, B是否正确更新

**问题2：控制命令跳变**

**排查**：
1. 增大控制权重R
2. 检查滤波器设置（`cutoff_freq`）
3. 启用数字滤波器

**问题3：低速抖动**

**排查**：
1. 检查`minimum_speed_protection_`（默认0.1 m/s）
2. 启用`enable_look_ahead_back_control`
3. 低速时切换到PID或Stanley

### 日志分析

启用CSV调试日志：
```bash
cyber_launch start modules/control/launch/control.launch
--flagfile=modules/control/conf/control.conf
--enable_csv_debug=true
```

日志位置：`/tmp/mpc_controller_*.csv`

关键字段：
- `lateral_error`：横向误差
- `heading_error`：航向角误差
- `steer_angle_feedback`：MPC输出的方向盘角度
- `steer_angle_feedforward`：前馈项
- `acceleration_cmd`：加速度命令

---

## 代码流程图

```
ComputeControlCommand()
│
├──> 1. TrajectoryAnalyzer初始化
│
├──> 2. ComputeLongitudinalErrors()
│         └──> 计算纵向位置、速度误差
│
├──> 3. UpdateState()
│         ├──> ComputeLateralErrors()
│         │      └──> 计算横向、航向角误差
│         └──> 更新matrix_state_
│
├──> 4. UpdateMatrix()
│         ├──> 根据当前车速更新A矩阵
│         └──> 双线性变换离散化
│
├──> 5. FeedforwardUpdate()
│         └──> 计算前馈项
│
├──> 6. 增益调度（可选）
│         └──> 根据车速调整Q, R矩阵
│
├──> 7. MpcOsqp构造
│         ├──> CalculateKernel() → P矩阵
│         ├──> CalculateGradient() → q向量
│         ├──> CalculateEqualityConstraint() → A矩阵
│         └──> CalculateConstraintVectors() → l, u向量
│
├──> 8. OSQP求解
│         └──> 提取第一步控制量
│
├──> 9. 方向盘转换
│         └──> Wheel2SteerPct()
│
├──> 10. 前馈补偿
│          └──> steer_angle = feedback + feedforward + compensation
│
├──> 11. 滤波限幅
│          └──> digital_filter_.Filter()
│
└──> 12. 输出控制命令
           ├──> cmd->set_steering_target()
           ├──> cmd->set_throttle()
           └──> cmd->set_brake()
```

---

## 参考文献

### Apollo源代码

- **MPC控制器**：`modules/control/controllers/mpc_controller/`
- **OSQP求解器**：`modules/common/math/mpc_osqp.cc`
- **轨迹分析器**：`modules/control/control_component/controller_task_base/common/trajectory_analyzer.h`
- **配置文件**：`modules/control/controllers/mpc_controller/proto/mpc_controller.proto`

### 学术论文

1. **Falcone et al.**, "Predictive Active Steering Control for Autonomous Vehicle Systems", IEEE Trans. Control Systems Technology, 2007
2. **Rajamani**, "Vehicle Dynamics and Control", Springer, 2012
3. **Kong et al.**, "Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design", IEEE IV, 2015

### OSQP文档

- 官方网站：https://osqp.org/
- GitHub：https://github.com/osqp/osqp
- 文档：https://osqp.org/docs/

---

*文档版本：v2.0（基于Apollo实际代码）*  
*最后更新：2025年10月*  
*代码版本：Apollo 9.0*

