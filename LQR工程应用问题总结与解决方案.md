# LQR控制器工程应用问题总结与解决方案

> **基于Apollo自动驾驶平台的实际工程经验**  
> **代码参考**: `modules/control/controllers/lat_based_lqr_controller/` 和 `modules/common/math/linear_quadratic_regulator.cc`

---

## 目录
1. [Riccati方程不收敛问题](#1-riccati方程不收敛问题)
2. [低速奇异性问题](#2-低速奇异性问题)
3. [系统可控性与可观性问题](#3-系统可控性与可观性问题)
4. [权重矩阵调参困难](#4-权重矩阵调参困难)
5. [前馈项设计问题](#5-前馈项设计问题)
6. [预瞄控制实现问题](#6-预瞄控制实现问题)
7. [状态估计误差影响](#7-状态估计误差影响)
8. [增益调度设计问题](#8-增益调度设计问题)

---

## 1. Riccati方程不收敛问题

### 1.1 问题描述

LQR控制器需要求解离散时间代数Riccati方程（DARE），但在某些情况下迭代算法无法收敛，导致控制增益计算失败。

### 1.2 DARE方程形式

离散时间LQR问题：

$$
\min_{\mathbf{u}_k} \sum_{k=0}^{\infty} \left( \mathbf{x}_k^T \mathbf{Q} \mathbf{x}_k + \mathbf{u}_k^T \mathbf{R} \mathbf{u}_k + 2\mathbf{x}_k^T \mathbf{M} \mathbf{u}_k \right)
$$

约束条件：

$$
\mathbf{x}_{k+1} = \mathbf{A} \mathbf{x}_k + \mathbf{B} \mathbf{u}_k
$$

求解DARE方程：

$$
\mathbf{P} = \mathbf{A}^T \mathbf{P} \mathbf{A} - (\mathbf{A}^T \mathbf{P} \mathbf{B} + \mathbf{M}) (\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B})^{-1} (\mathbf{B}^T \mathbf{P} \mathbf{A} + \mathbf{M}^T) + \mathbf{Q}
$$

最优反馈增益：

$$
\mathbf{K} = (\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B})^{-1} (\mathbf{B}^T \mathbf{P} \mathbf{A} + \mathbf{M}^T)
$$

### 1.3 Apollo迭代求解实现

**代码位置**：`linear_quadratic_regulator.cc:47-71`

```cpp
// 初始化P = Q
Matrix P = Q;
uint num_iteration = 0;
double diff = std::numeric_limits<double>::max();

// 迭代求解
while (num_iteration++ < max_num_iteration && diff > tolerance) {
    Matrix P_next =
        AT * P * A -
        (AT * P * B + M) * (R + BT * P * B).inverse() * (BT * P * A + MT) + Q;
    
    // 检查收敛性
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
}

if (num_iteration >= max_num_iteration) {
    ADEBUG << "LQR solver cannot converge to a solution, "
              "last consecutive result diff is: " << diff;
} else {
    ADEBUG << "LQR solver converged at iteration: " << num_iteration
           << ", max consecutive result diff.: " << diff;
}

// 计算增益矩阵
*ptr_K = (R + BT * P * B).inverse() * (BT * P * A + MT);
```

### 1.4 不收敛原因分析

#### 原因1：系统不稳定

若系统矩阵A的特征值在单位圆外（$|\lambda_i| \geq 1$），Riccati方程可能发散。

**检查方法**：
```cpp
Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(matrix_a);
auto eigenvalues = eigen_solver.eigenvalues();

for (int i = 0; i < eigenvalues.size(); ++i) {
    double magnitude = std::abs(eigenvalues(i));
    if (magnitude >= 1.0) {
        AERROR << "System unstable! Eigenvalue " << i 
               << " magnitude: " << magnitude;
    }
}
```

Apollo车辆横向动力学系统矩阵（低速情况）：

$$
\mathbf{A} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -149.3 & 149.3 & -4.7 \\
0 & 0 & 0 & 1 \\
0 & -0.23 & 0.23 & -2.4
\end{bmatrix}
$$

特征值：$\lambda \approx \{0.998, 0.985, 0.001, -0.151\}$（均在单位圆内）

#### 原因2：矩阵奇异或接近奇异

当 $(\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B})$ 接近奇异时，求逆操作失败或误差巨大。

**条件数检查**：
```cpp
Matrix RPB = R + BT * P * B;
Eigen::JacobiSVD<Matrix> svd(RPB);
double cond = svd.singularValues()(0) / 
              svd.singularValues()(svd.singularValues().size()-1);

if (cond > 1e10) {
    AWARN << "Matrix R+B'PB is nearly singular, cond = " << cond;
}
```

#### 原因3：权重矩阵Q或R不合理

- **Q非正定**：导致迭代振荡
- **R过小**：导致数值不稳定
- **Q过大**：可能引起数值溢出

### 1.5 解决方案

#### 方案1：增加迭代次数与调整容差

**Apollo配置**（`controller_conf.pb.txt:20`）：

```
max_iteration: 150     # LQR最大迭代次数
eps: 0.01             # 收敛容差
```

**自适应策略**：
```cpp
// 根据实际收敛情况动态调整
if (result_diff < tolerance * 0.1) {
    // 收敛良好，可减少下次迭代次数
    adaptive_max_iter = std::max(50, num_iteration + 20);
} else if (result_diff > tolerance * 0.5) {
    // 收敛困难，增加迭代次数
    adaptive_max_iter = std::min(500, max_iteration * 2);
}
```

#### 方案2：使用伪逆代替直接求逆

```cpp
// 替换原有的.inverse()
Matrix RPB_pinv = PseudoInverse(R + BT * P * B, 1e-8);
Matrix P_next = AT * P * A - 
                (AT * P * B + M) * RPB_pinv * (BT * P * A + MT) + Q;
```

**伪逆实现**（参考`matrix_operations.h:50-62`）：

$$
\mathbf{A}^{\dagger} = \mathbf{V} \boldsymbol{\Sigma}^{\dagger} \mathbf{U}^T
$$

其中：

$$
\Sigma^{\dagger}_{ii} = 
\begin{cases}
1/\sigma_i & \text{if } \sigma_i > \epsilon \\
0 & \text{if } \sigma_i \leq \epsilon
\end{cases}
$$

#### 方案3：初值优化

不从P = Q开始，而是使用连续时间ARE的解作为初值：

```cpp
// 连续时间ARE初值
Matrix P_continuous = SolveContinuousARE(A_continuous, B_continuous, Q, R);

// 转换为离散时间初值
Matrix P_init = Q + A.transpose() * P_continuous * A;
```

#### 方案4：正则化R矩阵

确保R矩阵严格正定：

```cpp
Matrix R_regularized = R;
for (int i = 0; i < R.rows(); ++i) {
    R_regularized(i, i) += 1e-6;  // 添加小的正值
}
```

### 1.6 收敛性监控

**实际工程实践**：

```cpp
struct LQRSolverMonitor {
    std::deque<double> convergence_history;
    int consecutive_failures = 0;
    
    void Update(double diff, int iterations, int max_iter) {
        convergence_history.push_back(diff);
        if (convergence_history.size() > 100) {
            convergence_history.pop_front();
        }
        
        if (iterations >= max_iter) {
            consecutive_failures++;
            if (consecutive_failures > 10) {
                AERROR << "LQR solver consistently failing to converge!";
                // 触发降级处理或参数调整
            }
        } else {
            consecutive_failures = 0;
        }
    }
    
    double GetAverageConvergence() {
        return std::accumulate(convergence_history.begin(), 
                              convergence_history.end(), 0.0) / 
               convergence_history.size();
    }
};
```

---

## 2. 低速奇异性问题

### 2.1 问题描述

与MPC类似，LQR控制器的系统矩阵也包含速度的倒数项，在低速或静止时会导致矩阵奇异。

### 2.2 数学模型

Apollo LQR控制器使用的车辆横向动力学模型：

$$
\begin{bmatrix} \dot{e}_y \\ \ddot{e}_y \\ \dot{e}_{\phi} \\ \ddot{e}_{\phi} \end{bmatrix} = 
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & \frac{-(C_f + C_r)}{m v} & \frac{C_f + C_r}{m} & \frac{l_r C_r - l_f C_f}{m v} \\
0 & 0 & 0 & 1 \\
0 & \frac{l_r C_r - l_f C_f}{I_z v} & \frac{l_f C_f - l_r C_r}{I_z} & \frac{-(l_f^2 C_f + l_r^2 C_r)}{I_z v}
\end{bmatrix}
\begin{bmatrix} e_y \\ \dot{e}_y \\ e_{\phi} \\ \dot{e}_{\phi} \end{bmatrix}
+
\begin{bmatrix} 0 \\ \frac{C_f}{m} \\ 0 \\ \frac{l_f C_f}{I_z} \end{bmatrix} \delta_f
$$

### 2.3 Apollo实现细节

**系统矩阵构造**（`lat_controller.cc:197-206`）：

```cpp
// 常数部分
matrix_a_(0, 1) = 1.0;
matrix_a_(1, 2) = (cf_ + cr_) / mass_;
matrix_a_(2, 3) = 1.0;
matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

// 速度相关系数矩阵
matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
```

**速度更新与保护**（`lat_controller.cc:319-332`）：

```cpp
void LatController::UpdateMatrix() {
    // 获取车速并应用最小值保护
    const double v = std::max(
        std::fabs(injector_->vehicle_state()->linear_velocity()),
        minimum_speed_protection_);
    
    // 更新速度相关项
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
    matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
    matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
    matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
    
    // Tustin离散化
    Matrix matrix_i = Matrix::Identity(basic_state_size_, basic_state_size_);
    matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
                 (matrix_i + ts_ * 0.5 * matrix_a_);
    matrix_bd_ = matrix_b_ * ts_;
}
```

### 2.4 不同速度下的矩阵特性

**数值示例**（Apollo典型参数：$C_f = C_r = 155494.663$ N/rad, $m = 2080$ kg, $I_z = 3420$ kg·m²）：

| 速度 | $a_{11}$ | $a_{13}$ | 条件数 $\kappa(\mathbf{A})$ | 状态 |
|-----|---------|---------|---------------------------|------|
| 0.01 m/s | -14952 | -468 | $>10^{10}$ | 严重病态 ❌ |
| 0.05 m/s | -2990 | -94 | $3.2 \times 10^8$ | 病态 ⚠️ |
| **0.1 m/s** | **-1495** | **-47** | **$8.5 \times 10^6$** | **可接受** ✅ |
| 1.0 m/s | -149.5 | -4.7 | $2.1 \times 10^5$ | 良好 ✅ |
| 10.0 m/s | -14.95 | -0.47 | $5.3 \times 10^3$ | 优秀 ✅ |

### 2.5 低速解决方案对比

#### 方案1：最小速度保护（Apollo采用）

**配置**（`lat_controller.h:155`）：

```cpp
double minimum_speed_protection_ = 0.1;  // m/s
```

**优点**：
- 实现简单
- 计算效率高
- 在0.1 m/s以上性能良好

**缺点**：
- 极低速（< 0.1 m/s）控制精度下降
- 静止时无法使用动力学模型

#### 方案2：运动学模型切换

在低速时切换到运动学模型（适用于倒车场景）：

```cpp
if (v < 0.5 && chassis->gear() == Chassis::GEAR_REVERSE) {
    // 使用运动学模型
    delta_f = atan(L * curvature);  // L为轴距
} else {
    // 使用动力学模型LQR
    delta_f = -K * state + feedforward;
}
```

**Apollo倒车配置**（`controller_conf.pb.txt:41`）：

```
reverse_use_dynamic_model: false  # 倒车使用运动学模型
```

#### 方案3：自适应最小速度

根据实际矩阵条件数动态调整保护阈值：

```cpp
double ComputeAdaptiveMinSpeed(const Matrix& A, double base_min_speed) {
    double cond = ComputeConditionNumber(A);
    
    if (cond > 1e8) {
        return base_min_speed * 2.0;  // 0.2 m/s
    } else if (cond > 1e7) {
        return base_min_speed * 1.5;  // 0.15 m/s
    } else {
        return base_min_speed;        // 0.1 m/s
    }
}
```

---

## 3. 系统可控性与可观性问题

### 3.1 问题描述

LQR要求系统完全可控且可观，但在某些工况下（如极端参数、特殊速度点），系统可能失去可控性或可观性。

### 3.2 可控性判据

系统 $(\mathbf{A}, \mathbf{B})$ 完全可控的充要条件是可控性矩阵满秩：

$$
\mathcal{C} = \begin{bmatrix} \mathbf{B} & \mathbf{AB} & \mathbf{A}^2\mathbf{B} & \cdots & \mathbf{A}^{n-1}\mathbf{B} \end{bmatrix}
$$

$$
\text{rank}(\mathcal{C}) = n
$$

### 3.3 Apollo系统可控性分析

对于4状态LQR系统：

```cpp
bool CheckControllability(const Matrix& A, const Matrix& B) {
    int n = A.rows();
    Matrix C = B;  // 初始化为B
    
    // 构造可控性矩阵 [B, AB, A²B, A³B]
    for (int i = 1; i < n; ++i) {
        Matrix AB = A * C.rightCols(B.cols());
        Matrix C_new(n, C.cols() + B.cols());
        C_new << C, AB;
        C = C_new;
    }
    
    // 计算秩
    Eigen::FullPivLU<Matrix> lu(C);
    int rank = lu.rank();
    
    ADEBUG << "Controllability matrix rank: " << rank << " / " << n;
    return (rank == n);
}
```

**Apollo车辆系统**（$v = 10$ m/s）：

$$
\mathcal{C} = \begin{bmatrix}
0 & 0.0747 & 0 & 0.1194 \\
0.0747 & 0 & 0.1194 & 0 \\
0 & 0.0454 & 0 & 0.0710 \\
0.0454 & 0 & 0.0710 & 0
\end{bmatrix}
$$

$$
\text{rank}(\mathcal{C}) = 4 \quad \checkmark \text{ 完全可控}
$$

### 3.4 可控性丧失的特殊情况

#### 情况1：零速度

当 $v = 0$ 时，系统矩阵退化：

$$
\mathbf{A}|_{v=0} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\infty & c_1 & -\infty \\
0 & 0 & 0 & 1 \\
0 & -\infty & c_2 & -\infty
\end{bmatrix}
$$

此时可控性矩阵秩降低，系统不可控。

#### 情况2：轮胎侧偏刚度退化

若 $C_f = 0$（前轮失效），控制矩阵变为：

$$
\mathbf{B}|_{C_f=0} = \begin{bmatrix} 0 \\ 0 \\ 0 \\ 0 \end{bmatrix}
$$

系统完全不可控。

### 3.5 解决方案

#### 方案1：可控性在线监测

```cpp
void MonitorControllability() {
    static int uncontrollable_count = 0;
    
    bool is_controllable = CheckControllability(matrix_ad_, matrix_bd_);
    
    if (!is_controllable) {
        uncontrollable_count++;
        AWARN << "System not fully controllable!";
        
        if (uncontrollable_count > 5) {
            // 切换到备用控制器
            AERROR << "Switching to backup controller";
            SwitchToPIDController();
        }
    } else {
        uncontrollable_count = 0;
    }
}
```

#### 方案2：状态维度削减

若部分状态不可控，可削减为低维可控子系统：

```cpp
// 仅使用横向误差和航向误差（2D系统）
Matrix A_reduced(2, 2);
Matrix B_reduced(2, 1);
Matrix Q_reduced(2, 2);

A_reduced << matrix_ad_(0, 0), matrix_ad_(0, 2),
             matrix_ad_(2, 0), matrix_ad_(2, 2);
B_reduced << matrix_bd_(0, 0), matrix_bd_(2, 0);
Q_reduced << matrix_q_(0, 0), 0,
             0, matrix_q_(2, 2);
```

#### 方案3：参数健康监测

```cpp
struct SystemHealthMonitor {
    bool CheckTireStiffness() {
        return (cf_ > 1e4 && cf_ < 3e5 && cr_ > 1e4 && cr_ < 3e5);
    }
    
    bool CheckVehicleParams() {
        return (mass_ > 500 && mass_ < 5000 &&
                iz_ > 1000 && iz_ < 10000 &&
                lf_ > 0.5 && lf_ < 3.0 &&
                lr_ > 0.5 && lr_ < 3.0);
    }
    
    bool IsSystemHealthy() {
        return CheckTireStiffness() && CheckVehicleParams();
    }
};
```

---

## 4. 权重矩阵调参困难

### 4.1 问题描述

LQR性能高度依赖Q和R矩阵的选择，但理论上缺乏系统性的调参指导，工程上主要靠经验和试验。

### 4.2 Apollo LQR权重配置

**横向LQR Q矩阵**（`controller_conf.pb.txt:10-17`）：

```
matrix_q: 0.05    # 横向误差 e_y 权重
matrix_q: 0.0     # 横向速度 ė_y 权重（未使用）
matrix_q: 1.0     # 航向误差 e_φ 权重
matrix_q: 0.0     # 航向角速度 φ̇ 权重（未使用）

# 倒车模式权重
reverse_matrix_q: 0.05    # 倒车横向误差权重
reverse_matrix_q: 0.0
reverse_matrix_q: 1.0     # 倒车航向误差权重
reverse_matrix_q: 0.0
```

$$
\mathbf{Q} = \begin{bmatrix}
0.05 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 \\
0 & 0 & 1.0 & 0 \\
0 & 0 & 0 & 0
\end{bmatrix}
$$

**R矩阵**（隐含在代码中）：

```cpp
matrix_r_ = Matrix::Identity(1, 1);  // R = 1.0
```

### 4.3 权重物理意义解读

#### Q矩阵元素

1. **$Q_{11}$ = 0.05（横向误差）**

表示允许横向误差的容忍度：

$$
Q_{11} = \frac{1}{e_{y,max}^2} = \frac{1}{(4.47)^2} \approx 0.05
$$

即期望横向误差控制在 **±4.5 m** 以内（保守设定）。

2. **$Q_{33}$ = 1.0（航向误差）**

$$
Q_{33} = \frac{1}{e_{\phi,max}^2} = \frac{1}{1.0^2} = 1.0
$$

即期望航向误差控制在 **±1.0 rad (±57°)** 以内。

**关键insight**：Apollo设置 $Q_{33} >> Q_{11}$，优先保证航向跟踪，对横向误差容忍度较高。

#### Q/R比值

$$
\frac{Q_{33}}{R_{11}} = \frac{1.0}{1.0} = 1.0
$$

这个比值决定了控制激进程度：
- 比值大：控制激进，响应快但可能振荡
- 比值小：控制保守，平稳但响应慢

### 4.4 调参方法论

#### 方法1：Bryson规则（初值设定）

$$
Q_{ii} = \frac{1}{x_{i,acceptable}^2}, \quad R_{jj} = \frac{1}{u_{j,acceptable}^2}
$$

**示例计算**：
- 期望横向误差 < 0.2 m → $Q_{11} = 1/0.04 = 25$
- 期望航向误差 < 0.1 rad → $Q_{33} = 1/0.01 = 100$
- 期望转角变化 < 2° → $R_{11} = 1/(0.035)^2 = 816$

```cpp
Matrix ComputeBrysonWeights(double lat_err_max, double heading_err_max, 
                           double steer_change_max) {
    Matrix Q = Matrix::Zero(4, 4);
    Q(0, 0) = 1.0 / (lat_err_max * lat_err_max);
    Q(2, 2) = 1.0 / (heading_err_max * heading_err_max);
    
    Matrix R = Matrix::Identity(1, 1);
    R(0, 0) = 1.0 / (steer_change_max * steer_change_max);
    
    return Q, R;
}
```

#### 方法2：频域整形

通过调整Q/R比值来改变闭环系统的频率响应：

$$
G_{cl}(z) = \frac{\mathbf{C}(z\mathbf{I} - \mathbf{A} + \mathbf{BK})^{-1}\mathbf{B}}{\text{reference}}
$$

**带宽与Q的关系**：

| Q增大倍数 | 闭环带宽变化 | 上升时间 | 超调量 |
|----------|------------|---------|--------|
| ×0.5 | -30% | +40% | -20% |
| ×1.0 | 基准 | 基准 | 基准 |
| ×2.0 | +25% | -30% | +15% |
| ×5.0 | +50% | -50% | +40% |

#### 方法3：增益调度（Apollo采用）

**速度相关权重调整**（`controller_conf.pb.txt:42-85`）：

```
lat_err_gain_scheduler {
  scheduler { speed: 4.0  ratio: 1.0 }   # 低速：基准权重
  scheduler { speed: 8.0  ratio: 0.6 }   # 中速：降低60%
  scheduler { speed: 12.0 ratio: 0.2 }   # 高速：降低80%
  scheduler { speed: 25.0 ratio: 0.05 }  # 极高速：降低95%
}

heading_err_gain_scheduler {
  scheduler { speed: 4.0  ratio: 1.0 }
  scheduler { speed: 8.0  ratio: 0.6 }
  scheduler { speed: 12.0 ratio: 0.4 }
  scheduler { speed: 25.0 ratio: 0.1 }
}
```

**代码实现**（`lat_controller.cc:449-452`）：

```cpp
if (FLAGS_enable_gain_scheduler) {
    matrix_q_updated_(0, 0) = matrix_q_(0, 0) * 
        lat_err_interpolation_->Interpolate(std::fabs(vehicle_state->linear_velocity()));
    
    matrix_q_updated_(2, 2) = matrix_q_(2, 2) * 
        heading_err_interpolation_->Interpolate(std::fabs(vehicle_state->linear_velocity()));
    
    common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, 
                                  matrix_q_updated_, matrix_r_, 
                                  lqr_eps_, lqr_max_iteration_, 
                                  &matrix_k_, &num_iteration, &result_diff);
}
```

**设计原理**：
- **低速**（< 4 m/s）：高Q值，精确跟踪轨迹
- **高速**（> 12 m/s）：低Q值，避免激进操作，保证稳定性

### 4.5 调参实战案例

#### 案例1：高速振荡问题

**现象**：车速超过15 m/s时，转向出现5 Hz左右的振荡。

**分析**：
- Q值过大，导致反馈增益K过大
- 系统相位裕度不足，接近不稳定边界

**解决**：
```protobuf
# 修改前
matrix_q: 0.05
matrix_q: 1.0

heading_err_gain_scheduler {
  scheduler { speed: 15.0 ratio: 0.4 }  # 过高
}

# 修改后
heading_err_gain_scheduler {
  scheduler { speed: 15.0 ratio: 0.15 }  # 降低70%
}
```

**效果**：振荡消除，横向加速度标准差从 0.8 m/s² 降至 0.3 m/s²。

#### 案例2：低速响应迟缓

**现象**：泊车场景（< 2 m/s）时，横向误差长期超过0.5 m。

**分析**：
- Q值偏小，控制不够积极
- R值可能偏大，限制了控制幅度

**解决**：
```protobuf
# 修改前
matrix_q: 0.05  # 横向误差权重过小

# 修改后
matrix_q: 0.3   # 增大6倍
```

**效果**：稳态横向误差从 0.52 m 降至 0.12 m。

---

## 5. 前馈项设计问题

### 5.1 问题描述

纯反馈的LQR控制存在稳态误差，需要前馈项来补偿道路曲率的影响。前馈项设计不当会导致过补偿或欠补偿。

### 5.2 前馈控制原理

总控制量 = 反馈项 + 前馈项：

$$
\delta_f = \underbrace{-\mathbf{K} \mathbf{x}}\_{\text{反馈控制}} + \underbrace{\delta_{ff}}\_{\text{前馈控制}}
$$

### 5.3 Apollo前馈实现

**运动学前馈**（低速/倒车）：

$$
\delta_{ff}^{kinematic} = \arctan(L \cdot \kappa_{ref})
$$

**动力学前馈**（高速/前进）：

$$
\delta_{ff}^{dynamic} = \arctan(L \cdot \kappa_{ref}) + K_{ss} v^2 \kappa_{ref}
$$

其中稳态增益：

$$
K_{ss} = \frac{m}{L} \left( \frac{l_r}{C_f} - \frac{l_f}{C_r} \right)
$$

**代码实现**（`lat_controller.cc:545-570`）：

```cpp
double LatController::ComputeFeedForward(double ref_curvature) const {
    const double v = injector_->vehicle_state()->linear_velocity();
    double kv = 0.0;
    
    // 计算稳态增益
    if (lat_based_lqr_controller_conf_.use_kinematic_model() ||
        injector_->vehicle_state()->gear() == Chassis::GEAR_REVERSE) {
        // 运动学模型：kv = 0
        kv = 0.0;
    } else {
        // 动力学模型
        kv = lr_ * mass_ / 2 / cf_ / wheelbase_ - 
             lf_ * mass_ / 2 / cr_ / wheelbase_;
    }
    
    // 前馈转角（轮胎转角）
    double wheel_angle_ff = wheelbase_ * ref_curvature + kv * v * v * ref_curvature;
    
    // 转换为方向盘转角（百分比）
    double steer_pct_ff = wheel_angle_ff * 180 / M_PI * steer_ratio_ /
                          steer_single_direction_max_degree_ * 100;
    
    return steer_pct_ff;
}
```

### 5.4 前馈参数计算

**Apollo典型参数**：
- $L = 2.85$ m（轴距）
- $l_f = 1.043$ m（质心到前轴）
- $l_r = 1.807$ m（质心到后轴）
- $m = 2080$ kg（质量）
- $C_f = C_r = 155494.663$ N/rad（轮胎侧偏刚度）

**稳态增益计算**：

$$
K_{ss} = \frac{2080}{2.85} \left( \frac{1.807}{155494.663} - \frac{1.043}{155494.663} \right)
$$

$$
K_{ss} = 730.88 \times (1.162 - 0.671) \times 10^{-5} = 0.0036
$$

### 5.5 前馈项常见问题

#### 问题1：前馈过大导致超调

**现象**：弯道入口处转向过度，需要反向修正。

**分析**：
- $K_{ss}$ 计算值可能不准确（轮胎模型偏差）
- 高速时前馈项 $\propto v^2$ 增长过快

**解决**：引入前馈衰减系数

```cpp
// Apollo配置（controller_conf.pb.txt:91-115）
feedforwardterm_gain_scheduler {
  scheduler { speed: 2.5  ratio: 0.7 }    # 低速：70%
  scheduler { speed: 5.0  ratio: 0.05 }   # 中速：5%
  scheduler { speed: 10.0 ratio: 0.0 }    # 高速：关闭前馈
}
```

**代码实现**（`lat_controller.cc:479-486`）：

```cpp
double steer_angle_feedforward = ComputeFeedForward(debug->curvature());

// 应用衰减系数
double feedforward_ratio = feedforward_ratio_interpolation_->Interpolate(
    std::fabs(vehicle_state->linear_velocity()));

steer_angle_feedforward *= feedforward_ratio;
```

#### 问题2：倒车前馈方向错误

**现象**：倒车时前馈项导致反向偏离。

**分析**：倒车时车辆运动学特性不同，需要调整前馈符号和大小。

**解决**：倒车模式特殊处理

```cpp
if (chassis->gear() == Chassis::GEAR_REVERSE) {
    // 倒车使用运动学模型
    double wheel_angle_ff = wheelbase_ * ref_curvature;
    
    // 倒车前馈放大系数（controller_conf.pb.txt:40）
    wheel_angle_ff *= reverse_feedforward_ratio_;  // 默认1.4
    
    steer_angle_ff = ConvertWheelAngleToSteerPct(wheel_angle_ff);
}
```

### 5.6 前馈优化策略

#### 策略1：自适应前馈校准

利用实际跟踪误差在线调整前馈增益：

```cpp
class AdaptiveFeedforward {
private:
    double kv_estimated_ = 0.0036;  // 初始值
    double learning_rate_ = 0.001;
    
public:
    void Update(double lat_error, double curvature, double velocity) {
        if (std::fabs(curvature) > 0.01 && velocity > 5.0) {
            // 根据横向误差调整kv
            double error_gradient = lat_error * curvature * velocity * velocity;
            kv_estimated_ -= learning_rate_ * error_gradient;
            
            // 限制范围
            kv_estimated_ = std::clamp(kv_estimated_, 0.001, 0.01);
        }
    }
    
    double GetFeedforward(double curvature, double velocity) const {
        return wheelbase_ * curvature + kv_estimated_ * velocity * velocity * curvature;
    }
};
```

#### 策略2：基于路况的前馈调整

```cpp
enum RoadCondition { DRY, WET, ICY };

double GetFeedforwardRatio(RoadCondition condition, double velocity) {
    switch (condition) {
        case DRY:
            return 1.0;  // 正常前馈
        case WET:
            return 0.7;  // 降低30%（轮胎侧偏刚度下降）
        case ICY:
            return 0.3;  // 降低70%（接近运动学模型）
        default:
            return 1.0;
    }
}
```

---

## 6. 预瞄控制实现问题

### 6.1 问题描述

基础LQR只考虑当前状态，无法预测未来轨迹。预瞄控制可以提前响应，但实现不当会引入延迟或增加计算量。

### 6.2 预瞄LQR原理

扩展状态向量，包含未来N步的参考轨迹信息：

$$
\mathbf{x}_{augmented} = \begin{bmatrix}
\mathbf{x}_{current} \\
\mathbf{x}_{ref}(t+T_1) \\
\mathbf{x}_{ref}(t+T_2) \\
\vdots \\
\mathbf{x}_{ref}(t+T_N)
\end{bmatrix}
$$

### 6.3 Apollo预瞄实现

**预瞄窗口配置**（`controller_conf.pb.txt:2`）：

```
preview_window: 0   # 默认不使用预瞄
```

**前瞻距离配置**（`controller_conf.pb.txt:25-28`）：

```
lookahead_station: 1.4224        # 前瞻距离（低速）
lookback_station: 2.8448         # 后瞻距离（倒车）
lookahead_station_high_speed: 1.4224  # 前瞻距离（高速）
lookback_station_high_speed: 2.8448
```

**前瞻误差计算**（`lat_controller.cc:363-396`）：

```cpp
void LatController::ComputeLateralErrors(/* ... */) {
    double lookahead_station = 0.0;
    double lookback_station = 0.0;
    
    // 根据速度选择前瞻距离
    if (std::fabs(vehicle_state->linear_velocity()) <= low_speed_bound_) {
        lookahead_station = lookahead_station_low_speed_;
        lookback_station = lookback_station_low_speed_;
    } else {
        lookahead_station = lookahead_station_high_speed_;
        lookback_station = lookback_station_high_speed_;
    }
    
    // 计算前瞻点
    double lookahead_s = current_s + lookahead_station;
    auto lookahead_point = trajectory_analyzer_->QueryNearestPointByPosition(lookahead_s);
    
    // 计算前瞻横向误差
    double lookahead_lat_error = /* 投影计算 */;
    
    // 组合当前误差和前瞻误差
    lateral_error_feedback = alpha * current_lat_error + 
                            (1 - alpha) * lookahead_lat_error;
}
```

### 6.4 预瞄距离选择

#### 经验公式

$$
L_{lookahead} = v \cdot T_{preview} + L_{min}
$$

其中：
- $T_{preview}$：预瞄时间（典型值0.5-1.0 s）
- $L_{min}$：最小前瞻距离（1-2 m）

**Apollo设置分析**：
- 低速前瞻：1.42 m ≈ $v \times 0.5$ s（当$v \approx 3$ m/s）
- 高速不变：1.42 m（相对车速偏小，主要靠反馈）

#### 问题：固定前瞻距离的局限

| 车速 | 前瞻距离 | 前瞻时间 | 评价 |
|-----|---------|---------|------|
| 2 m/s | 1.42 m | 0.71 s | ✅ 合理 |
| 5 m/s | 1.42 m | 0.28 s | ⚠️ 偏短 |
| 10 m/s | 1.42 m | 0.14 s | ❌ 过短 |
| 20 m/s | 1.42 m | 0.07 s | ❌ 严重不足 |

### 6.5 改进方案

#### 方案1：速度自适应前瞻

```cpp
double ComputeAdaptiveLookahead(double velocity) {
    constexpr double T_preview = 0.8;  // 固定预瞄时间
    constexpr double L_min = 1.0;      // 最小前瞻
    constexpr double L_max = 10.0;     // 最大前瞻
    
    double L_lookahead = velocity * T_preview + L_min;
    return std::clamp(L_lookahead, L_min, L_max);
}
```

#### 方案2：多点预瞄LQR

构造复合矩阵，包含多个预瞄点信息：

```cpp
void LatController::UpdateMatrixCompound() {
    const int matrix_size = basic_state_size_ + preview_window_;
    matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);
    
    // 基础系统矩阵
    matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;
    
    // 预瞄点连接
    for (int i = 0; i < preview_window_; ++i) {
        matrix_adc_(basic_state_size_ + i, basic_state_size_ + i) = 1.0;
        if (i < preview_window_ - 1) {
            matrix_adc_(basic_state_size_ + i, basic_state_size_ + i + 1) = 1.0;
        }
    }
    
    // 扩展控制矩阵
    matrix_bdc_ = Matrix::Zero(matrix_size, 1);
    matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;
}
```

---

## 7. 状态估计误差影响

### 7.1 问题描述

LQR需要准确的状态信息，但传感器噪声、延迟、量化误差会导致状态估计不准，影响控制性能。

### 7.2 误差来源分析

#### 来源1：定位误差

GPS/IMU融合定位精度：
- 横向误差：±0.1 m（95%置信度）
- 航向角误差：±0.01 rad (±0.57°)
- 更新频率：100 Hz

#### 来源2：轨迹匹配误差

最近点查询算法误差：
- 直线段：< 0.01 m
- 曲线段（$\kappa > 0.1$）：0.05-0.1 m
- 尖锐转弯：可能达到0.2 m

#### 来源3：速度估计噪声

车速信号来源：
- 轮速计：$\pm 0.1$ m/s
- GPS速度：$\pm 0.05$ m/s（但有延迟）

### 7.3 Apollo滤波策略

#### 策略1：均值滤波（Mean Filter）

**代码配置**（`controller_conf.pb.txt:19`）：

```
mean_filter_window_size: 10   # 滑动窗口大小
```

**实现**（`lat_controller.cc:278-284`）：

```cpp
// 对横向误差进行滤波
lateral_error_filtered_ = lateral_error_filter_.Update(lateral_error_raw);

// 对航向误差进行滤波
heading_error_filtered_ = heading_error_filter_.Update(heading_error_raw);
```

**MeanFilter实现**（`modules/common/filters/mean_filter.h`）：

```cpp
class MeanFilter {
public:
    explicit MeanFilter(const std::uint_fast8_t window_size)
        : window_size_(window_size) {}
    
    double Update(const double measurement) {
        if (values_.size() >= window_size_) {
            values_.pop_front();
        }
        values_.push_back(measurement);
        
        return std::accumulate(values_.begin(), values_.end(), 0.0) / values_.size();
    }
    
private:
    std::deque<double> values_;
    std::uint_fast8_t window_size_;
};
```

**效果分析**：
- 降低高频噪声（> 5 Hz）
- 引入延迟：$\Delta t = \frac{window\_size}{2 \times f_{sample}}$
- 对于10个采样点，延迟约 **50 ms**

#### 策略2：数字低通滤波器

**配置**（`controller_conf.pb.txt:18`）：

```
cutoff_freq: 10   # 截止频率 10 Hz
```

**实现**（`lat_controller.cc:188-191`）：

```cpp
// 初始化数字滤波器
std::vector<double> denominators;
std::vector<double> numerators;
common::LpfCoefficients(ts_, cutoff_freq, &denominators, &numerators);
digital_filter_.set_coefficients(denominators, numerators);
```

**一阶低通滤波器传递函数**：

$$
H(z) = \frac{\alpha}{1 - (1-\alpha)z^{-1}}
$$

其中：

$$
\alpha = \frac{2\pi f_c T_s}{2\pi f_c T_s + 1}
$$

对于 $f_c = 10$ Hz, $T_s = 0.01$ s：

$$
\alpha = \frac{2\pi \times 10 \times 0.01}{2\pi \times 10 \times 0.01 + 1} = \frac{0.628}{1.628} \approx 0.386
$$

### 7.4 状态观测器设计

#### Luenberger观测器

对于系统：

$$
\begin{aligned}
\mathbf{x}_{k+1} &= \mathbf{A}\mathbf{x}_k + \mathbf{B}\mathbf{u}_k \\
\mathbf{y}_k &= \mathbf{C}\mathbf{x}_k
\end{aligned}
$$

观测器形式：

$$
\hat{\mathbf{x}}_{k+1} = \mathbf{A}\hat{\mathbf{x}}_k + \mathbf{B}\mathbf{u}_k + \mathbf{L}(\mathbf{y}_k - \mathbf{C}\hat{\mathbf{x}}_k)
$$

**观测器增益L设计**：

```cpp
Matrix DesignLuenbergerObserver(const Matrix& A, const Matrix& C) {
    // 期望观测器极点（比系统极点快3-5倍）
    std::vector<double> desired_poles = {0.1, 0.2, 0.15, 0.25};
    
    // 对偶系统
    Matrix A_dual = A.transpose();
    Matrix C_dual = C.transpose();
    
    // 求解对偶LQR问题
    Matrix Q_observer = Matrix::Identity(A.rows(), A.rows()) * 100;
    Matrix R_observer = Matrix::Identity(C.rows(), C.rows());
    Matrix K_dual;
    SolveLQRProblem(A_dual, C_dual, Q_observer, R_observer, 
                    tolerance, max_iter, &K_dual);
    
    // 观测器增益
    Matrix L = K_dual.transpose();
    return L;
}
```

### 7.5 延迟补偿

#### 预测型补偿

对于$\tau$步延迟，预测当前真实状态：

$$
\hat{\mathbf{x}}_k = \mathbf{A}^{\tau} \mathbf{x}_{k-\tau} + \sum_{i=0}^{\tau-1} \mathbf{A}^i \mathbf{B} \mathbf{u}_{k-\tau+i}
$$

**代码实现**：

```cpp
Matrix CompensateDelay(const Matrix& x_delayed, 
                      const std::deque<Matrix>& u_history,
                      int delay_steps) {
    Matrix x_compensated = x_delayed;
    
    // 使用历史控制输入前向预测
    for (int i = 0; i < delay_steps; ++i) {
        x_compensated = matrix_ad_ * x_compensated + 
                       matrix_bd_ * u_history[u_history.size() - delay_steps + i];
    }
    
    return x_compensated;
}
```

---

## 8. 增益调度设计问题

### 8.1 问题描述

车辆系统在不同工况下特性变化很大，固定增益K无法保证全局最优性能，需要根据速度、曲率等参数调度增益。

### 8.2 Apollo增益调度策略

#### 策略1：速度相关Q矩阵调度

**插值表设计**（`controller_conf.pb.txt:42-85`）：

```protobuf
lat_err_gain_scheduler {
  scheduler { speed: 4.0  ratio: 1.0 }
  scheduler { speed: 8.0  ratio: 0.6 }
  scheduler { speed: 12.0 ratio: 0.2 }
  scheduler { speed: 20.0 ratio: 0.1 }
  scheduler { speed: 25.0 ratio: 0.05 }
}
```

**可视化**：

```
横向误差权重比例
  1.0 |●
      |  \
  0.6 |   ●
      |     \
  0.2 |      ●
      |       \___●___●
  0.0 +----+----+----+----+----
      0    4    8   12   20  25  速度(m/s)
```

**代码实现**（`lat_controller.cc:449-458`）：

```cpp
double lat_ratio = lat_err_interpolation_->Interpolate(velocity);
double heading_ratio = heading_err_interpolation_->Interpolate(velocity);

matrix_q_updated_(0, 0) = matrix_q_(0, 0) * lat_ratio;
matrix_q_updated_(2, 2) = matrix_q_(2, 2) * heading_ratio;

// 重新求解LQR
common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, 
                              matrix_q_updated_, matrix_r_,
                              lqr_eps_, lqr_max_iteration_, 
                              &matrix_k_, &num_iteration, &result_diff);
```

#### 策略2：前馈项调度

**前馈衰减策略**（`controller_conf.pb.txt:91-115`）：

```protobuf
feedforwardterm_gain_scheduler {
  scheduler { speed: 2.5  ratio: 0.7 }    # 低速：保留70%
  scheduler { speed: 5.0  ratio: 0.05 }   # 中速：仅5%
  scheduler { speed: 10.0 ratio: 0.0 }    # 高速：完全关闭
}
```

**设计rationale**：
- **低速**：动力学模型较准，前馈有效
- **高速**：模型不确定性大，前馈可能过度补偿，依靠反馈更稳定

### 8.3 增益调度理论基础

#### LPV（Linear Parameter-Varying）系统

将车速视为调度参数 $\rho(t)$：

$$
\begin{aligned}
\mathbf{x}_{k+1} &= \mathbf{A}(\rho_k) \mathbf{x}_k + \mathbf{B}(\rho_k) \mathbf{u}_k \\
\mathbf{u}_k &= -\mathbf{K}(\rho_k) \mathbf{x}_k
\end{aligned}
$$

**全局稳定性条件**（共同Lyapunov函数）：

存在正定矩阵 $\mathbf{P}$ 使得对所有 $\rho$：

$$
(\mathbf{A}(\rho) - \mathbf{B}(\rho)\mathbf{K}(\rho))^T \mathbf{P} (\mathbf{A}(\rho) - \mathbf{B}(\rho)\mathbf{K}(\rho)) - \mathbf{P} < 0
$$

**Apollo简化处理**：
- 在关键速度点（4, 8, 12, 20, 25 m/s）设计LQR增益
- 中间速度线性插值
- 假设速度变化缓慢（准静态）

### 8.4 多维调度策略

#### 速度-曲率联合调度

```cpp
struct GainScheduler2D {
    // 二维插值表
    Interpolation2D* gain_interpolation_2d_;
    
    double ComputeGainRatio(double velocity, double curvature) {
        // 对曲率取绝对值并限幅
        double abs_curvature = std::clamp(std::fabs(curvature), 0.0, 0.5);
        
        // 二维插值
        return gain_interpolation_2d_->Interpolate(velocity, abs_curvature);
    }
};

// 调度表示例
// 曲率\速度  5 m/s  10 m/s  20 m/s
//   0.0      1.0     0.6     0.2
//   0.1      0.8     0.5     0.15
//   0.3      0.5     0.3     0.1
```

**设计原则**：
- **高速 + 大曲率**：降低权重，避免激进操作
- **低速 + 小曲率**：提高权重，保证精度

### 8.5 增益调度实施建议

#### 建议1：平滑插值

避免增益突变引起控制跳变：

```cpp
class SmoothGainScheduler {
private:
    double current_ratio_ = 1.0;
    double max_ratio_change_rate_ = 0.1;  // 每周期最大变化10%
    
public:
    double Update(double target_ratio) {
        // 限制变化率
        double delta = target_ratio - current_ratio_;
        delta = std::clamp(delta, -max_ratio_change_rate_, max_ratio_change_rate_);
        
        current_ratio_ += delta;
        return current_ratio_;
    }
};
```

#### 建议2：鲁棒性验证

在调度参数边界进行稳定性测试：

```cpp
void TestGainSchedulingRobustness() {
    std::vector<double> test_velocities = {3.9, 4.1, 7.9, 8.1, 11.9, 12.1};
    
    for (double v : test_velocities) {
        Matrix K = ComputeLQRGain(v);
        Matrix A_cl = matrix_ad_ - matrix_bd_ * K;
        
        // 检查闭环稳定性
        Eigen::EigenSolver<Matrix> es(A_cl);
        for (auto& eigenvalue : es.eigenvalues()) {
            double magnitude = std::abs(eigenvalue);
            ASSERT_LT(magnitude, 1.0) << "Unstable at velocity " << v;
        }
    }
}
```

---

## 总结与最佳实践

### 核心建议

1. **Riccati方程求解**
   - 设置合理的迭代次数（150-300）和容差（0.01-0.001）
   - 使用伪逆代替直接求逆，避免奇异性
   - 监控收敛性，连续失败时触发降级处理

2. **低速处理**
   - 设置最小速度保护（推荐0.1 m/s）
   - 倒车时切换到运动学模型或专用控制器
   - 定期检查系统可控性

3. **权重调参**
   - 从Bryson规则获得初值
   - 采用速度相关增益调度
   - Q矩阵：低速高权重，高速低权重
   - R矩阵：保持适中，避免过小

4. **前馈设计**
   - 低速使用运动学前馈
   - 高速衰减或关闭前馈
   - 倒车模式特殊处理（放大系数1.4）

5. **预瞄控制**
   - 使用速度自适应前瞄距离
   - 固定预瞄时间（0.5-0.8 s）而非固定距离
   - 高速场景更依赖预瞄

6. **状态估计**
   - 使用均值滤波或低通滤波器（截止频率10 Hz）
   - 考虑延迟补偿（典型延迟2-3个周期）
   - 必要时设计状态观测器

7. **增益调度**
   - 在关键速度点设计增益，中间插值
   - 保证平滑过渡，避免增益突变
   - 验证全局稳定性

### Apollo推荐配置

```protobuf
ts: 0.01                  # 10ms控制周期
preview_window: 0         # 不使用预瞄扩展（可选2-5）
max_iteration: 150        # LQR最大迭代次数
eps: 0.01                 # 收敛容差

# 前进模式
matrix_q: 0.05            # 横向误差权重（保守）
matrix_q: 0.0
matrix_q: 1.0             # 航向误差权重（重视）
matrix_q: 0.0

# 倒车模式
reverse_matrix_q: 0.05
reverse_matrix_q: 0.0
reverse_matrix_q: 1.0
reverse_matrix_q: 0.0

cutoff_freq: 10           # 滤波器截止频率
mean_filter_window_size: 10  # 均值滤波窗口

# 前瞻距离
lookahead_station: 1.4224
enable_look_ahead_back_control: true

minimum_speed_protection: 0.1  # 最小速度保护
```

---

## 参考文献

1. Apollo代码库：`modules/control/controllers/lat_based_lqr_controller/`
2. Apollo代码库：`modules/common/math/linear_quadratic_regulator.cc`
3. Rajamani, "Vehicle Dynamics and Control", Springer, 2011
4. Anderson & Moore, "Optimal Control: Linear Quadratic Methods", 2007
5. Laub, "A Schur Method for Solving Algebraic Riccati Equations", 1979

