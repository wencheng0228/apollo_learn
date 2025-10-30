# 双轮差速机器人MPC控制器完整数学推导（稀疏形式）

> **基于Apollo实际使用的稀疏QP形式，从状态空间方程到OSQP求解的完整推导**

---

## 目录

1. [预备知识：线性化与离散化](#1-预备知识线性化与离散化)
2. [稀疏形式MPC基本概念](#2-稀疏形式mpc基本概念)
3. [决策变量定义](#3-决策变量定义)
4. [等式约束构造（动力学约束）](#4-等式约束构造动力学约束)
5. [目标函数设计（6项）](#5-目标函数设计6项)
6. [不等式约束构造](#6-不等式约束构造)
7. [完整QP问题形式](#7-完整qp问题形式)
8. [完整数值示例](#8-完整数值示例)
9. [参数调优指南](#9-参数调优指南)

---

## 核心公式速查

### 目标函数（6项）

| 序号 | 目标函数项 | 数学表示 | 权重矩阵 |
|------|-----------|---------|---------|
| 1️⃣ | 位姿误差 | $\mathbf{X}_e^\top \mathbf{Q}_1 \mathbf{X}_e$ | $\mathbf{Q}_1 = \text{diag}(q_x, q_y, q_\theta)$ |
| 2️⃣ | 速度误差 | $\Delta\mathbf{u}^\top \mathbf{R}_1 \Delta\mathbf{u}$ | $\mathbf{R}_1 = \text{diag}(r_v, r_\omega)$ |
| 3️⃣ | 速度误差变化率（加速度平滑） | $\delta\mathbf{u}^\top \mathbf{R}_2 \delta\mathbf{u}$ | $\mathbf{R}_2 = \text{diag}(r_a, r_\alpha)$ |
| 4️⃣ | 终点位姿误差 | $\mathbf{X}_e(N)^\top \mathbf{Q}_f \mathbf{X}_e(N)$ | $\mathbf{Q}_f = 1000 \cdot \mathbf{Q}_1$ |
| 5️⃣ | 终点速度误差 | $\Delta\mathbf{u}(N)^\top \mathbf{R}_f \Delta\mathbf{u}(N)$ | $\mathbf{R}_f = 100 \cdot \mathbf{R}_1$ |
| 6️⃣ | 终点加速度（防止抖动） | $\delta\mathbf{u}(N)^\top \mathbf{R}_a \delta\mathbf{u}(N)$ | $\mathbf{R}_a = 10 \cdot \mathbf{R}_2$ |

> **设计理由**：
> - ❌ **去掉了位姿误差变化率项**：Apollo MPC实践证明该项冗余（权重通常设为0）
> - ✅ **保留了终点加速度项**：显式约束终点静止状态，防止到点时抖动

### 约束汇总（N=10示例）

| 约束类型 | 数量 | 说明 |
|---------|------|------|
| 等式约束（动力学） | 30 | 每步3个，共10步 |
| 状态边界约束 | 30 | 通常设为无穷 |
| 控制边界约束 | 20 | 速度和角速度限制 |
| 加速度约束 | 18 | 控制变化率限制 |
| **约束总数** | **98** | **30等式+68不等式** |
| **决策变量** | **50** | **30状态+20控制** |

---

## 1. 预备知识：线性化与离散化

### 1.1 连续时间状态方程

**线性化误差动力学**：

$$
\dot{\mathbf{X}}_e = \mathbf{A}\mathbf{X}_e + \mathbf{B}\Delta\mathbf{u}
$$

其中：

$$
\mathbf{A} = \begin{bmatrix} 
0 & \omega_r & 0 \\ 
-\omega_r & 0 & v_r \\ 
0 & 0 & 0 
\end{bmatrix}, \quad
\mathbf{B} = \begin{bmatrix} 
-1 & 0 \\ 
0 & 0 \\ 
0 & -1 
\end{bmatrix}
$$

$$
\mathbf{X}_e = \begin{bmatrix} 
x_e \\ 
y_e \\ 
\theta_e 
\end{bmatrix}, \quad
\Delta\mathbf{u} = \begin{bmatrix} 
\Delta v \\ 
\Delta\omega 
\end{bmatrix} = \begin{bmatrix} 
v - v_r \\ 
\omega - \omega_r 
\end{bmatrix}
$$

**符号说明**：
- $\mathbf{X}_e$：车体坐标系下的位姿误差 $(x_e, y_e, \theta_e)$
- $\Delta\mathbf{u}$：速度误差（相对于参考轨迹）
- $v_r, \omega_r$：参考线速度和角速度

### 1.2 欧拉离散化

采样时间为 $T$，使用前向欧拉法：

$$
\dot{\mathbf{X}}_e \approx \frac{\mathbf{X}_e(k+1) - \mathbf{X}_e(k)}{T}
$$

整理为**离散状态方程**：

$$
\boxed{\mathbf{X}_e(k+1) = \mathbf{A}_d \mathbf{X}_e(k) + \mathbf{B}_d \Delta\mathbf{u}(k)}
$$

其中：

$$
\boxed{
\mathbf{A}_d = \mathbf{I} + T\mathbf{A} = \begin{bmatrix} 
1 & T\omega_r & 0 \\ 
-T\omega_r & 1 & Tv_r \\ 
0 & 0 & 1 
\end{bmatrix}
}
$$

$$
\boxed{
\mathbf{B}_d = T\mathbf{B} = \begin{bmatrix} 
-T & 0 \\ 
0 & 0 \\ 
0 & -T 
\end{bmatrix}
}
$$

**注意**：$T$ 是采样时间（如 0.1s），欧拉法精度为 $O(T^2)$。

---

## 2. 稀疏形式MPC基本概念

### 2.1 什么是稀疏形式？

**核心思想**：将未来时域内的**所有状态和控制都作为优化变量**，通过等式约束保证它们满足动力学关系。

**与传统方法的对比**：

| 方法 | 决策变量 | 状态如何得到 | 优势 |
|------|---------|------------|------|
| **传统紧凑形式** | 只有控制 $\mathbf{U}$ | 通过预测方程计算 | 变量少，理论优雅 |
| **稀疏形式（本文）** | 状态+控制 $\boldsymbol{\xi} = [\boldsymbol{\Xi}, \mathbf{U}]$ | 通过等式约束保证 | 易实现，易约束，Apollo采用 |

### 2.2 为什么选择稀疏形式？

1. **Hessian矩阵简单**：直接块对角，无需复杂矩阵乘法
2. **状态约束直观**：可以直接约束状态边界
3. **实现简单**：约束构造清晰，物理意义明确
4. **工业验证**：Apollo、Autoware等主流自动驾驶系统都采用此形式
5. **OSQP友好**：稀疏矩阵结构，求解器优化效果好

### 2.3 预测时域设置

对于预测时域 $N=10$，采样周期 $T=0.1$s：

- **预测距离**：$N \times T \times v_r = 10 \times 0.1 \times 1.0 = 1.0$ m
- **状态维度**：$n_x = 3$ (位姿误差)
- **控制维度**：$n_u = 2$ (速度误差)
- **决策变量总数**：$N \times n_x + N \times n_u = 10 \times 3 + 10 \times 2 = 50$

---

## 3. 决策变量定义

### 3.1 决策变量排列

稀疏形式的决策变量包含状态和控制：

$$
\boldsymbol{\xi} = \begin{bmatrix}
\mathbf{X}_e(k+1) \\ 
\mathbf{X}_e(k+2) \\ 
\vdots \\ 
\mathbf{X}_e(k+N) \\
\Delta\mathbf{u}(k) \\ 
\Delta\mathbf{u}(k+1) \\ 
\vdots \\ 
\Delta\mathbf{u}(k+N-1) 
\end{bmatrix}_{(Nn_x + Nn_u) \times 1}
$$

展开为：

$$
\boldsymbol{\xi} = \begin{bmatrix}
x_e(k+1) \\ y_e(k+1) \\ \theta_e(k+1) \\
x_e(k+2) \\ y_e(k+2) \\ \theta_e(k+2) \\
\vdots \\ 
x_e(k+10) \\ y_e(k+10) \\ \theta_e(k+10) \\
\Delta v(k) \\ \Delta\omega(k) \\
\Delta v(k+1) \\ \Delta\omega(k+1) \\
\vdots \\
\Delta v(k+9) \\ \Delta\omega(k+9)
\end{bmatrix}_{50 \times 1}
$$

### 3.2 索引规则

为了方便编程实现，定义索引规则：

- **状态变量索引**：`ξ[0:30]` 
  - $\mathbf{X}_e(k+i)$ 对应 `ξ[3i:3i+3]`, $i=1,2,\ldots,10$
  
- **控制变量索引**：`ξ[30:50]`
  - $\Delta\mathbf{u}(k+i)$ 对应 `ξ[30+2i:30+2i+2]`, $i=0,1,\ldots,9$

**示例**（Python风格）：
```python
# 第1步状态：ξ[0:3]   = [x_e(k+1), y_e(k+1), θ_e(k+1)]
# 第2步状态：ξ[3:6]   = [x_e(k+2), y_e(k+2), θ_e(k+2)]
# 第10步状态：ξ[27:30] = [x_e(k+10), y_e(k+10), θ_e(k+10)]

# 第1步控制：ξ[30:32] = [Δv(k), Δω(k)]
# 第2步控制：ξ[32:34] = [Δv(k+1), Δω(k+1)]
# 第10步控制：ξ[48:50] = [Δv(k+9), Δω(k+9)]
```

### 3.3 维度汇总（N=10）

| 变量类型 | 符号 | 维度 | 说明 |
|---------|------|------|------|
| 状态序列 | $\boldsymbol{\Xi}$ | $30 \times 1$ | 10步×3状态 |
| 控制序列 | $\mathbf{U}$ | $20 \times 1$ | 10步×2控制 |
| **决策变量** | $\boldsymbol{\xi}$ | **50 × 1** | **总优化变量** |

---

## 4. 等式约束构造（动力学约束）

### 4.1 约束原理

稀疏形式的核心：通过**等式约束**保证状态和控制满足动力学关系。

对于每一步 $i = 0, 1, \ldots, N-1$：

$$
\mathbf{X}_e(k+i+1) = \mathbf{A}_d \mathbf{X}_e(k+i) + \mathbf{B}_d \Delta\mathbf{u}(k+i)
$$

改写为标准形式（左边=0）：

$$
-\mathbf{X}_e(k+i+1) + \mathbf{A}_d \mathbf{X}_e(k+i) + \mathbf{B}_d \Delta\mathbf{u}(k+i) = \mathbf{0}
$$

### 4.2 第一步约束（特殊处理）

对于 $i=0$：

$$
-\mathbf{X}_e(k+1) + \mathbf{A}_d \mathbf{X}_e(k) + \mathbf{B}_d \Delta\mathbf{u}(k) = \mathbf{0}
$$

其中 $\mathbf{X}_e(k)$ 是**已知的初始状态**，需要移到右边：

$$
-\mathbf{X}_e(k+1) + \mathbf{B}_d \Delta\mathbf{u}(k) = -\mathbf{A}_d \mathbf{X}_e(k)
$$

### 4.3 后续步约束

对于 $i=1,2,\ldots,N-1$：

$$
-\mathbf{X}_e(k+i+1) + \mathbf{A}_d \mathbf{X}_e(k+i) + \mathbf{B}_d \Delta\mathbf{u}(k+i) = \mathbf{0}
$$

### 4.4 等式约束矩阵形式

将所有等式约束堆叠为矩阵形式：

$$
\mathbf{A}_{eq} \boldsymbol{\xi} = \mathbf{b}_{eq}
$$

其中：

**约束矩阵** $\mathbf{A}_{eq} \in \mathbb{R}^{30 \times 50}$：

$$
\mathbf{A}_{eq} = \begin{bmatrix}
-\mathbf{I}_3 & \mathbf{0} & \mathbf{0} & \cdots & \mathbf{0} & \mathbf{B}_d & \mathbf{0} & \cdots & \mathbf{0} \\
\mathbf{A}_d & -\mathbf{I}_3 & \mathbf{0} & \cdots & \mathbf{0} & \mathbf{0} & \mathbf{B}_d & \cdots & \mathbf{0} \\
\mathbf{0} & \mathbf{A}_d & -\mathbf{I}_3 & \cdots & \mathbf{0} & \mathbf{0} & \mathbf{0} & \cdots & \mathbf{0} \\
\vdots & \vdots & \vdots & \ddots & \vdots & \vdots & \vdots & \ddots & \vdots \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \cdots & -\mathbf{I}_3 & \mathbf{0} & \mathbf{0} & \cdots & \mathbf{B}_d
\end{bmatrix}_{30 \times 50}
$$

**右端项** $\mathbf{b}_{eq} \in \mathbb{R}^{30 \times 1}$：

$$
\mathbf{b}_{eq} = \begin{bmatrix}
-\mathbf{A}_d \mathbf{X}_e(k) \\
\mathbf{0}_3 \\
\mathbf{0}_3 \\
\vdots \\
\mathbf{0}_3
\end{bmatrix}_{30 \times 1}
$$

### 4.5 约束矩阵结构解析

对于第 $i$ 步约束（$i=1,2,\ldots,9$），矩阵 $\mathbf{A}_{eq}$ 的第 $3i$ 到 $3i+2$ 行：

- **列 $3(i-1)$ 到 $3(i-1)+2$**：系数矩阵 $\mathbf{A}_d$ （对应 $\mathbf{X}_e(k+i)$）
- **列 $3i$ 到 $3i+2$**：系数矩阵 $-\mathbf{I}_3$ （对应 $\mathbf{X}_e(k+i+1)$）
- **列 $30+2i$ 到 $30+2i+1$**：系数矩阵 $\mathbf{B}_d$ （对应 $\Delta\mathbf{u}(k+i)$）
- **其他列**：全零

**稀疏性**：矩阵 $\mathbf{A}_{eq}$ 高度稀疏，每行只有约 $3+3+2=8$ 个非零元素。

### 4.6 等式约束数值示例

假设 $T=0.1$, $v_r=1.0$, $\omega_r=0.1$, $\mathbf{X}_e(k) = [0.5, 0.2, 0.1]^\top$：

$$
\mathbf{A}_d = \begin{bmatrix}
1 & 0.01 & 0 \\
-0.01 & 1 & 0.1 \\
0 & 0 & 1
\end{bmatrix}, \quad
\mathbf{B}_d = \begin{bmatrix}
-0.1 & 0 \\
0 & 0 \\
0 & -0.1
\end{bmatrix}
$$

$$
\mathbf{b}_{eq}[0:3] = -\mathbf{A}_d \begin{bmatrix} 0.5 \\ 0.2 \\ 0.1 \end{bmatrix} = \begin{bmatrix}
-0.502 \\
-0.195 \\
-0.1
\end{bmatrix}
$$

---

## 5. 目标函数设计（6项）

### 5.1 目标函数设计原则

MPC的目标函数设计需要权衡多个控制目标：

1. **跟踪精度**：最小化位姿误差
2. **控制平滑**：避免控制量剧烈变化
3. **能量效率**：减少不必要的控制努力
4. **终点约束**：确保准确到达目标位置并停止
5. **舒适性**：避免加速度突变

### 5.2 完整目标函数

基于工程实践和Apollo MPC经验，目标函数包含以下6项：

$$
\boxed{
\begin{align}
J = & \sum_{i=1}^{N} \mathbf{X}_e^\top(k+i) \mathbf{Q}_1 \mathbf{X}_e(k+i) 
    && \text{1️⃣ 位姿误差} \\
& + \sum_{i=0}^{N-1} \Delta\mathbf{u}^\top(k+i) \mathbf{R}_1 \Delta\mathbf{u}(k+i) 
    && \text{2️⃣ 速度误差} \\
& + \sum_{i=1}^{N-1} \delta\mathbf{u}^\top(k+i) \mathbf{R}_2 \delta\mathbf{u}(k+i) 
    && \text{3️⃣ 速度误差变化率（加速度平滑）} \\
& + \mathbf{X}_e^\top(k+N) \mathbf{Q}_f \mathbf{X}_e(k+N) 
    && \text{4️⃣ 终点位姿误差} \\
& + \Delta\mathbf{u}^\top(k+N-1) \mathbf{R}_f \Delta\mathbf{u}(k+N-1) 
    && \text{5️⃣ 终点速度误差} \\
& + \delta\mathbf{u}^\top(k+N-1) \mathbf{R}_a \delta\mathbf{u}(k+N-1) 
    && \text{6️⃣ 终点加速度（防止抖动）}
\end{align}
}
$$

其中 $\delta\mathbf{u}(k+i) = \Delta\mathbf{u}(k+i) - \Delta\mathbf{u}(k+i-1)$ 是控制变化率（加速度）。

### 5.3 各项详细解释

#### 5.3.1 第1项：位姿误差（核心项）

$$
J_1 = \sum_{i=1}^{N} \mathbf{X}_e^\top(k+i) \mathbf{Q}_1 \mathbf{X}_e(k+i)
$$

**物理意义**：惩罚预测时域内所有时刻的位姿误差。

**展开形式**：

$$
J_1 = \sum_{i=1}^{N} \begin{bmatrix} x_e(k+i) \\ y_e(k+i) \\ \theta_e(k+i) \end{bmatrix}^\top 
\begin{bmatrix} q_x & 0 & 0 \\ 0 & q_y & 0 \\ 0 & 0 & q_\theta \end{bmatrix}
\begin{bmatrix} x_e(k+i) \\ y_e(k+i) \\ \theta_e(k+i) \end{bmatrix}
$$

$$
= \sum_{i=1}^{N} \left( q_x \cdot x_e^2(k+i) + q_y \cdot y_e^2(k+i) + q_\theta \cdot \theta_e^2(k+i) \right)
$$

**作用**：
- $q_x$ 大 → 严格控制纵向偏差
- $q_y$ 大 → 严格控制横向偏差
- $q_\theta$ 大 → 严格控制角度偏差

**调优建议**：
- 直线跟踪：$q_y > q_x$（横向更重要）
- 转弯跟踪：$q_\theta$ 适当增大
- 推荐：$\mathbf{Q}_1 = \text{diag}(100, 100, 10)$

#### 5.3.2 第2项：速度误差（控制平滑项）

$$
J_2 = \sum_{i=0}^{N-1} \Delta\mathbf{u}^\top(k+i) \mathbf{R}_1 \Delta\mathbf{u}(k+i)
$$

**物理意义**：惩罚速度误差，避免过大的控制输入。

**展开形式**：

$$
J_2 = \sum_{i=0}^{N-1} \begin{bmatrix} \Delta v(k+i) \\ \Delta\omega(k+i) \end{bmatrix}^\top
\begin{bmatrix} r_v & 0 \\ 0 & r_\omega \end{bmatrix}
\begin{bmatrix} \Delta v(k+i) \\ \Delta\omega(k+i) \end{bmatrix}
$$

$$
= \sum_{i=0}^{N-1} \left( r_v \cdot (\Delta v)^2(k+i) + r_\omega \cdot (\Delta\omega)^2(k+i) \right)
$$

**作用**：
- 防止控制量偏离参考值过大
- 减少能量消耗
- 提高控制平滑性

**注意**：
- $r_v$ 太大 → 跟踪变慢（不愿意加速）
- $r_v$ 太小 → 控制激进
- 推荐：$\mathbf{R}_1 = \text{diag}(0.1, 0.1)$（相对Q1较小）

#### 5.3.3 第3项：加速度平滑（舒适性项）

$$
J_3 = \sum_{i=1}^{N-1} \delta\mathbf{u}^\top(k+i) \mathbf{R}_2 \delta\mathbf{u}(k+i)
$$

其中 $\delta\mathbf{u}(k+i) = \Delta\mathbf{u}(k+i) - \Delta\mathbf{u}(k+i-1)$ 是控制变化率。

**物理意义**：惩罚速度的变化率（即加速度），避免加速度突变。

**展开形式**：

$$
\delta\mathbf{u}(k+i) = \begin{bmatrix}
\Delta v(k+i) - \Delta v(k+i-1) \\
\Delta\omega(k+i) - \Delta\omega(k+i-1)
\end{bmatrix}
= \begin{bmatrix}
\delta v(k+i) \\
\delta \omega(k+i)
\end{bmatrix}
$$

$$
J_3 = \sum_{i=1}^{N-1} \left( r_a \cdot (\delta v)^2(k+i) + r_\alpha \cdot (\delta\omega)^2(k+i) \right)
$$

**作用**：
- 避免速度突变（乘客舒适性）
- 减少机械磨损
- 提高轨迹平滑度

**索引说明**：
- 从 $i=1$ 开始（因为需要 $i-1$）
- 到 $i=N-1$ 结束（共 $N-1$ 项）

**调优建议**：
- 推荐：$\mathbf{R}_2 = \text{diag}(1, 1)$
- 如果轨迹抖动 → 增大 $\mathbf{R}_2$
- 如果响应太慢 → 减小 $\mathbf{R}_2$

#### 5.3.4 第4项：终点位姿误差（强调项）

$$
J_4 = \mathbf{X}_e^\top(k+N) \mathbf{Q}_f \mathbf{X}_e(k+N)
$$

**物理意义**：额外强调终点位姿误差，确保精确到达目标。

**展开形式**：

$$
J_4 = \begin{bmatrix} x_e(k+N) \\ y_e(k+N) \\ \theta_e(k+N) \end{bmatrix}^\top
\begin{bmatrix} q_{f,x} & 0 & 0 \\ 0 & q_{f,y} & 0 \\ 0 & 0 & q_{f,\theta} \end{bmatrix}
\begin{bmatrix} x_e(k+N) \\ y_e(k+N) \\ \theta_e(k+N) \end{bmatrix}
$$

$$
= q_{f,x} \cdot x_e^2(k+N) + q_{f,y} \cdot y_e^2(k+N) + q_{f,\theta} \cdot \theta_e^2(k+N)
$$

**作用**：
- 单独惩罚预测时域的最后一步
- 通常设置为 $\mathbf{Q}_f = 1000 \times \mathbf{Q}_1$（强调！）

**为什么需要？**
- 第1项惩罚所有步，每步权重相同
- 第4项额外强调终点，形成"漏斗"效应
- 确保到达终点时误差足够小

#### 5.3.5 第5项：终点速度误差（停止项）

$$
J_5 = \Delta\mathbf{u}^\top(k+N-1) \mathbf{R}_f \Delta\mathbf{u}(k+N-1)
$$

**物理意义**：额外强调最后一步的速度误差，确保终点停止。

**展开形式**：

$$
J_5 = \begin{bmatrix} \Delta v(k+N-1) \\ \Delta\omega(k+N-1) \end{bmatrix}^\top
\begin{bmatrix} r_{f,v} & 0 \\ 0 & r_{f,\omega} \end{bmatrix}
\begin{bmatrix} \Delta v(k+N-1) \\ \Delta\omega(k+N-1) \end{bmatrix}
$$

$$
= r_{f,v} \cdot (\Delta v)^2(k+N-1) + r_{f,\omega} \cdot (\Delta\omega)^2(k+N-1)
$$

**作用**：
- 强制最后一步速度接近参考值（通常为0）
- 通常设置为 $\mathbf{R}_f = 100 \times \mathbf{R}_1$

**为什么索引是 $k+N-1$？**
- 控制序列：$\Delta\mathbf{u}(k), \Delta\mathbf{u}(k+1), \ldots, \Delta\mathbf{u}(k+N-1)$
- 最后一个控制是 $\Delta\mathbf{u}(k+N-1)$

#### 5.3.6 第6项：终点加速度（防抖动项）

$$
J_6 = \delta\mathbf{u}^\top(k+N-1) \mathbf{R}_a \delta\mathbf{u}(k+N-1)
$$

**物理意义**：额外强调最后一步的加速度，防止到点时抖动。

**展开形式**：

$$
\delta\mathbf{u}(k+N-1) = \Delta\mathbf{u}(k+N-1) - \Delta\mathbf{u}(k+N-2)
$$

$$
J_6 = r_{a,v} \cdot (\delta v)^2(k+N-1) + r_{a,\omega} \cdot (\delta\omega)^2(k+N-1)
$$

**作用**：
- 确保到达终点时加速度为0
- 防止到点瞬间的小幅抖动
- 通常设置为 $\mathbf{R}_a = 10 \times \mathbf{R}_2$（适中权重）

**为什么需要？**
- 理论上：第3项+第5项能保证终点加速度接近0
- 实际上：数值误差可能导致残余抖动
- 工程保险：显式约束更可靠

**权重设置为10倍而非100倍的原因**：
- 第3项已全程平滑
- 第6项只是额外强调
- 10倍足够，100倍会过度约束

### 5.4 目标函数的分组理解

可以将6项分为3组：

| 组别 | 包含项 | 作用 | 权重关系 |
|------|--------|------|---------|
| **路径组** | 1、4 | 位姿跟踪 | $\mathbf{Q}_f \gg \mathbf{Q}_1$ |
| **控制组** | 2、5 | 速度平滑 | $\mathbf{R}_f \gg \mathbf{R}_1$ |
| **舒适组** | 3、6 | 加速度平滑 | $\mathbf{R}_a > \mathbf{R}_2$ |

**权重递进设计**：
- 常规项（1、2、3）：全程优化
- 终点强调项（4、5）：强力约束（100-1000倍）
- 终点保险项（6）：适度约束（10倍）

### 5.5 权重矩阵定义

| 权重 | 维度 | 物理意义 | 推荐值 | 说明 |
|------|------|---------|--------|------|
| $\mathbf{Q}_1$ | $3 \times 3$ | 位姿误差惩罚 | $\text{diag}(100, 100, 10)$ | 核心项，确保跟踪精度 |
| $\mathbf{R}_1$ | $2 \times 2$ | 速度误差惩罚 | $\text{diag}(0.1, 0.1)$ | 控制平滑，参考Apollo |
| $\mathbf{R}_2$ | $2 \times 2$ | 加速度惩罚 | $\text{diag}(1, 1)$ | 避免速度突变 |
| $\mathbf{Q}_f$ | $3 \times 3$ | 终点位姿（强调） | $1000 \cdot \mathbf{Q}_1$ | 精确到达目标 |
| $\mathbf{R}_f$ | $2 \times 2$ | 终点速度（强调） | $100 \cdot \mathbf{R}_1$ | 确保停止 |
| $\mathbf{R}_a$ | $2 \times 2$ | 终点加速度（强调） | $10 \cdot \mathbf{R}_2$ | 防止到点抖动⭐ |

### 5.6 稀疏形式的Hessian矩阵构造（详细）

#### 5.6.1 目标函数的二次型转换

在稀疏形式下，决策变量为：

$$
\boldsymbol{\xi} = \begin{bmatrix}
\boldsymbol{\Xi} \\  
\mathbf{U}
\end{bmatrix} = \begin{bmatrix}
\mathbf{X}_e(k+1) \\ \vdots \\ \mathbf{X}_e(k+N) \\
\Delta\mathbf{u}(k) \\ \vdots \\ \Delta\mathbf{u}(k+N-1)
\end{bmatrix}_{50 \times 1}
$$

目标函数可以直接写为标准二次型：

$$
J = \frac{1}{2} \boldsymbol{\xi}^\top \mathbf{P} \boldsymbol{\xi}
$$

**关键优势**：状态和控制都是决策变量，无需通过预测方程消元，Hessian矩阵构造非常简单！

#### 5.6.2 Hessian矩阵的块对角结构

**Hessian矩阵** $\mathbf{P} \in \mathbb{R}^{50 \times 50}$ 是**块对角**结构：

$$
\mathbf{P} = \begin{bmatrix}
\bar{\mathbf{Q}} & \mathbf{0} \\
\mathbf{0} & \bar{\mathbf{R}} + \mathbf{R}_{accel}
\end{bmatrix}_{50 \times 50}
$$

**说明**：
- 左上角 $\bar{\mathbf{Q}}$ (30×30)：对应状态变量 $\boldsymbol{\Xi}$
- 右下角 $\bar{\mathbf{R}} + \mathbf{R}_{accel}$ (20×20)：对应控制变量 $\mathbf{U}$
- 非对角块全为零：状态和控制在Hessian中不耦合

#### 5.6.3 状态部分Hessian（对应第1、4项）

第1项（位姿误差）和第4项（终点位姿误差）对应决策变量中的状态部分。

**构造方法**：

$$
J_1 + J_4 = \sum_{i=1}^{N-1} \mathbf{X}_e^\top(k+i) \mathbf{Q}_1 \mathbf{X}_e(k+i) + \mathbf{X}_e^\top(k+N) (\mathbf{Q}_1 + \mathbf{Q}_f) \mathbf{X}_e(k+N)
$$

可以写为：

$$
J_1 + J_4 = \boldsymbol{\Xi}^\top \bar{\mathbf{Q}} \boldsymbol{\Xi}
$$

其中：

$$
\bar{\mathbf{Q}} = \text{diag}(\underbrace{\mathbf{Q}_1, \mathbf{Q}_1, \ldots, \mathbf{Q}_1}_{N-1\text{个}}, \mathbf{Q}_1 + \mathbf{Q}_f)_{30 \times 30}
$$

**展开形式**（N=10）：

$$
\bar{\mathbf{Q}} = \begin{bmatrix}
\mathbf{Q}_1 & & & & & \\
& \mathbf{Q}_1 & & & & \\
& & \ddots & & & \\
& & & \mathbf{Q}_1 & & \\
& & & & \mathbf{Q}_1 & \\
& & & & & \mathbf{Q}_1 + \mathbf{Q}_f
\end{bmatrix}_{30 \times 30}
$$

**注意**：
- 前9个块对角：$\mathbf{Q}_1$ (每个3×3)
- 最后一个块对角：$\mathbf{Q}_1 + \mathbf{Q}_f$ (3×3)

**数值示例**（$\mathbf{Q}_1 = \text{diag}(100, 100, 10)$, $\mathbf{Q}_f = 1000 \times \mathbf{Q}_1$）：

- $\bar{\mathbf{Q}}[0:27, 0:27]$：前9步，对角线重复 `[100, 100, 10]`
- $\bar{\mathbf{Q}}[27:30, 27:30]$：第10步，对角线为 `[100100, 100100, 10010]`

#### 5.6.4 控制部分Hessian（对应第2、5项）

第2项（速度误差）和第5项（终点速度误差）对应决策变量中的控制部分。

**构造方法**：

$$
J_2 + J_5 = \sum_{i=0}^{N-2} \Delta\mathbf{u}^\top(k+i) \mathbf{R}_1 \Delta\mathbf{u}(k+i) + \Delta\mathbf{u}^\top(k+N-1) (\mathbf{R}_1 + \mathbf{R}_f) \Delta\mathbf{u}(k+N-1)
$$

可以写为：

$$
J_2 + J_5 = \mathbf{U}^\top \bar{\mathbf{R}} \mathbf{U}
$$

其中：

$$
\bar{\mathbf{R}} = \text{diag}(\underbrace{\mathbf{R}_1, \mathbf{R}_1, \ldots, \mathbf{R}_1}_{N-1\text{个}}, \mathbf{R}_1 + \mathbf{R}_f)_{20 \times 20}
$$

**展开形式**（N=10）：

$$
\bar{\mathbf{R}} = \begin{bmatrix}
\mathbf{R}_1 & & & & \\
& \mathbf{R}_1 & & & \\
& & \ddots & & \\
& & & \mathbf{R}_1 & \\
& & & & \mathbf{R}_1 + \mathbf{R}_f
\end{bmatrix}_{20 \times 20}
$$

**数值示例**（$\mathbf{R}_1 = \text{diag}(0.1, 0.1)$, $\mathbf{R}_f = 100 \times \mathbf{R}_1$）：

- $\bar{\mathbf{R}}[0:18, 0:18]$：前9步，对角线重复 `[0.1, 0.1]`
- $\bar{\mathbf{R}}[18:20, 18:20]$：第10步，对角线为 `[10.1, 10.1]`

#### 5.6.5 加速度平滑Hessian（对应第3、6项）

第3项（加速度平滑）和第6项（终点加速度）涉及控制变化率，需要通过差分矩阵实现。

**第3项展开**：

$$
J_3 = \sum_{i=1}^{N-1} \delta\mathbf{u}^\top(k+i) \mathbf{R}_2 \delta\mathbf{u}(k+i)
$$

定义差分向量：

$$
\boldsymbol{\delta} = \begin{bmatrix}
\delta\mathbf{u}(k+1) \\
\delta\mathbf{u}(k+2) \\
\vdots \\
\delta\mathbf{u}(k+N-1)
\end{bmatrix}_{18 \times 1} = \mathbf{D}_{accel} \mathbf{U}
$$

其中差分矩阵 $\mathbf{D}_{accel} \in \mathbb{R}^{18 \times 20}$：

$$
\mathbf{D}_{accel} = \begin{bmatrix}
-\mathbf{I}_2 & \mathbf{I}_2 & \mathbf{0} & \cdots & \mathbf{0} \\
\mathbf{0} & -\mathbf{I}_2 & \mathbf{I}_2 & \cdots & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & -\mathbf{I}_2 & \cdots & \mathbf{0} \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \cdots & \mathbf{I}_2
\end{bmatrix}_{18 \times 20}
$$

**差分矩阵详细说明**：

- **行数**：18 = (N-1) × 2，因为有N-1个控制变化率，每个2维
- **列数**：20 = N × 2，作用于N个控制量
- **每一行**：对应一个 $\delta\mathbf{u}(k+i) = \Delta\mathbf{u}(k+i) - \Delta\mathbf{u}(k+i-1)$
- **第1-2行**：$\begin{bmatrix} -\mathbf{I}_2 & \mathbf{I}_2 & \mathbf{0} & \cdots \end{bmatrix}$，计算 $\delta\mathbf{u}(k+1) = \Delta\mathbf{u}(k+1) - \Delta\mathbf{u}(k)$
- **第3-4行**：$\begin{bmatrix} \mathbf{0} & -\mathbf{I}_2 & \mathbf{I}_2 & \cdots \end{bmatrix}$，计算 $\delta\mathbf{u}(k+2) = \Delta\mathbf{u}(k+2) - \Delta\mathbf{u}(k+1)$
- ...以此类推

**差分矩阵的具体形式**（展开 $\mathbf{I}_2$）：

$$
\mathbf{D}_{accel} = \begin{bmatrix}
-1 & 0 & 1 & 0 & 0 & 0 & \cdots & 0 & 0 \\
0 & -1 & 0 & 1 & 0 & 0 & \cdots & 0 & 0 \\
0 & 0 & -1 & 0 & 1 & 0 & \cdots & 0 & 0 \\
0 & 0 & 0 & -1 & 0 & 1 & \cdots & 0 & 0 \\
\vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \ddots & \vdots & \vdots \\
0 & 0 & 0 & 0 & 0 & 0 & \cdots & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & \cdots & 0 & 1
\end{bmatrix}_{18 \times 20}
$$

**第3+6项合并**：

$$
J_3 + J_6 = \sum_{i=1}^{N-2} \delta\mathbf{u}^\top(k+i) \mathbf{R}_2 \delta\mathbf{u}(k+i) + \delta\mathbf{u}^\top(k+N-1) (\mathbf{R}_2 + \mathbf{R}_a) \delta\mathbf{u}(k+N-1)
$$

$$
= \boldsymbol{\delta}^\top \bar{\mathbf{R}}_2 \boldsymbol{\delta} = (\mathbf{D}_{accel} \mathbf{U})^\top \bar{\mathbf{R}}_2 (\mathbf{D}_{accel} \mathbf{U})
$$

$$
= \mathbf{U}^\top (\mathbf{D}_{accel}^\top \bar{\mathbf{R}}_2 \mathbf{D}_{accel}) \mathbf{U}
$$

其中权重矩阵：

$$
\bar{\mathbf{R}}_2 = \text{diag}(\underbrace{\mathbf{R}_2, \mathbf{R}_2, \ldots, \mathbf{R}_2}_{N-2\text{个}}, \mathbf{R}_2 + \mathbf{R}_a)_{18 \times 18}
$$

因此加速度Hessian为：

$$
\boxed{\mathbf{R}_{accel} = \mathbf{D}_{accel}^\top \bar{\mathbf{R}}_2 \mathbf{D}_{accel} \quad (20 \times 20)}
$$

**数值示例**（$\mathbf{R}_2 = \text{diag}(1, 1)$, $\mathbf{R}_a = 10 \times \mathbf{R}_2$）：

$$
\bar{\mathbf{R}}_2 = \text{diag}(\underbrace{1, 1, 1, 1, \ldots, 1, 1}_{16\text{个}}, 11, 11)_{18 \times 18}
$$

#### 5.6.6 完整Hessian矩阵

综合第1-6项，完整的Hessian矩阵为：

$$
\boxed{
\mathbf{P} = \begin{bmatrix}
\bar{\mathbf{Q}} & \mathbf{0}_{30 \times 20} \\
\mathbf{0}_{20 \times 30} & \bar{\mathbf{R}} + \mathbf{R}_{accel}
\end{bmatrix}_{50 \times 50}
}
$$

**维度验证**：
- $\bar{\mathbf{Q}}$：30×30（状态部分）
- $\bar{\mathbf{R}} + \mathbf{R}_{accel}$：20×20（控制部分）
- 总维度：50×50 ✓

**稀疏性**：
- $\bar{\mathbf{Q}}$：块对角，每块3×3，非零元素 = 30×3 = 90
- $\bar{\mathbf{R}}$：块对角，每块2×2，非零元素 = 20×2 = 40
- $\mathbf{R}_{accel}$：三对角带状，非零元素 ≈ 18×4 = 72
- 总非零元素 ≈ 202 / 2500 ≈ 8%（高度稀疏）

**关键优势**：
- ✅ 块对角结构，无需复杂的 $\boldsymbol{\Theta}^\top \bar{\mathbf{Q}} \boldsymbol{\Theta}$ 矩阵乘法
- ✅ 高度稀疏，OSQP求解快
- ✅ 物理意义清晰，每一块对应一个目标项
- ✅ 易于调参，修改权重直接修改对角元素

#### 5.6.7 Hessian构造的Python实现框架

```python
import numpy as np
from scipy import sparse

# 权重矩阵
Q1 = np.diag([100, 100, 10])
R1 = np.diag([0.1, 0.1])
R2 = np.diag([1, 1])
Qf = 1000 * Q1
Rf = 100 * R1
Ra = 10 * R2

N = 10  # 预测步数

# 1. 状态部分Hessian (30×30)
Q_bar = sparse.block_diag([Q1]*(N-1) + [Q1 + Qf], format='csc')

# 2. 控制部分Hessian (20×20)
R_bar = sparse.block_diag([R1]*(N-1) + [R1 + Rf], format='csc')

# 3. 差分矩阵 (18×20)
D_accel = sparse.lil_matrix((18, 20))
for i in range(9):  # N-1 = 9
    D_accel[2*i:2*i+2, 2*i:2*i+2] = -np.eye(2)
    D_accel[2*i:2*i+2, 2*i+2:2*i+4] = np.eye(2)
D_accel = D_accel.tocsc()

# 4. 加速度权重 (18×18)
R2_bar = sparse.block_diag([R2]*(N-2) + [R2 + Ra], format='csc')

# 5. 加速度Hessian (20×20)
R_accel = D_accel.T @ R2_bar @ D_accel

# 6. 完整Hessian (50×50)
P = sparse.block_diag([Q_bar, R_bar + R_accel], format='csc')
```

### 5.7 梯度向量

对于稀疏形式，由于目标函数是纯二次型（无一次项），梯度向量为零：

$$
\boxed{\mathbf{q} = \mathbf{0}_{50 \times 1}}
$$

**说明**：
- 紧凑形式中，梯度 $\mathbf{f} = 2\boldsymbol{\Theta}^\top \bar{\mathbf{Q}} \boldsymbol{\Phi} \mathbf{X}_e(k)$ 不为零（因为初始状态影响）
- 稀疏形式中，初始状态通过等式约束引入，目标函数中不含一次项
- 这使得稀疏形式的构造更加简洁

### 5.8 数值示例

假设权重设置为：

$$
\begin{align}
\mathbf{Q}_1 &= \text{diag}(100, 100, 10) \\
\mathbf{R}_1 &= \text{diag}(0.1, 0.1) \\
\mathbf{R}_2 &= \text{diag}(1, 1) \\
\mathbf{Q}_f &= 1000 \times \mathbf{Q}_1 = \text{diag}(100000, 100000, 10000) \\
\mathbf{R}_f &= 100 \times \mathbf{R}_1 = \text{diag}(10, 10) \\
\mathbf{R}_a &= 10 \times \mathbf{R}_2 = \text{diag}(10, 10)
\end{align}
$$

则Hessian矩阵的对角元素为：

**状态部分**：
- 前9步：每步对角线为 $[100, 100, 10]$
- 第10步：对角线为 $[100100, 100100, 10010]$（增加终点惩罚）

**控制部分**（需要加上加速度项）：
- 基础权重：前9步 $[0.1, 0.1]$，第10步 $[10.1, 10.1]$
- 加上 $\mathbf{D}_{accel}^\top \bar{\mathbf{R}}_2 \mathbf{D}_{accel}$ 的贡献

---

## 6. 不等式约束构造（详细）

### 6.1 约束设计原则

MPC的约束设计需要考虑：

1. **物理限制**：电机最大速度、最大加速度
2. **安全边界**：避免状态超出安全范围
3. **舒适性**：加速度不能太大
4. **硬件能力**：执行器的实际能力

### 6.2 约束类型汇总

稀疏形式共有3类不等式约束：

| 约束类型 | 数量 | 符号 | 说明 | 是否必需 |
|---------|------|------|------|---------|
| 状态边界 | 30 | $\mathbf{X}_{e,\min} \leq \mathbf{X}_e \leq \mathbf{X}_{e,\max}$ | 通常设为无穷 | 可选 |
| 控制边界 | 20 | $\mathbf{U}_{\min} \leq \Delta\mathbf{u} \leq \mathbf{U}_{\max}$ | 速度和角速度限制 | **必需** |
| 加速度约束 | 18 | $\boldsymbol{\Delta}_{\min} \leq \delta\mathbf{u} \leq \boldsymbol{\Delta}_{\max}$ | 控制变化率限制 | **必需** |

**总约束数**：30 + 20 + 18 = 68个不等式约束

### 6.3 状态边界约束（详细）

#### 6.3.1 约束定义

对于预测时域内的每一步 $i=1,2,\ldots,N$：

$$
\begin{bmatrix}
x_{e,\min} \\ y_{e,\min} \\ \theta_{e,\min}
\end{bmatrix}
\leq
\begin{bmatrix}
x_e(k+i) \\ y_e(k+i) \\ \theta_e(k+i)
\end{bmatrix}
\leq
\begin{bmatrix}
x_{e,\max} \\ y_{e,\max} \\ \theta_{e,\max}
\end{bmatrix}
$$

**物理意义**：
- $x_{e,\min}, x_{e,\max}$：限制纵向偏差（通常不限制）
- $y_{e,\min}, y_{e,\max}$：限制横向偏差（例如：车道宽度）
- $\theta_{e,\min}, \theta_{e,\max}$：限制角度偏差（通常 $[-\pi, \pi]$）

#### 6.3.2 矩阵形式推导

将所有N步的状态约束堆叠：

$$
\begin{bmatrix}
\mathbf{X}_{e,\min} \\
\mathbf{X}_{e,\min} \\
\vdots \\
\mathbf{X}_{e,\min}
\end{bmatrix}_{30 \times 1}
\leq
\begin{bmatrix}
\mathbf{X}_e(k+1) \\
\mathbf{X}_e(k+2) \\
\vdots \\
\mathbf{X}_e(k+N)
\end{bmatrix}_{30 \times 1}
\leq
\begin{bmatrix}
\mathbf{X}_{e,\max} \\
\mathbf{X}_{e,\max} \\
\vdots \\
\mathbf{X}_{e,\max}
\end{bmatrix}_{30 \times 1}
$$

在稀疏形式中，决策变量 $\boldsymbol{\xi} = [\boldsymbol{\Xi}, \mathbf{U}]$，状态变量占据前30个元素。

因此约束矩阵需要**选择**决策变量的前30个元素：

$$
\mathbf{A}_{state} = \begin{bmatrix}
\mathbf{I}_{30} & \mathbf{0}_{30 \times 20}
\end{bmatrix}_{30 \times 50}
$$

**矩阵结构解释**：
- $\mathbf{I}_{30}$：30×30单位矩阵，选择状态变量
- $\mathbf{0}_{30 \times 20}$：30×20零矩阵，控制变量不参与此约束

**约束形式**：

$$
\mathbf{l}_{state} \leq \mathbf{A}_{state} \boldsymbol{\xi} \leq \mathbf{u}_{state}
$$

**边界向量**：

$$
\mathbf{l}_{state} = \begin{bmatrix}
x_{e,\min} \\ y_{e,\min} \\ \theta_{e,\min} \\
x_{e,\min} \\ y_{e,\min} \\ \theta_{e,\min} \\
\vdots \\
x_{e,\min} \\ y_{e,\min} \\ \theta_{e,\min}
\end{bmatrix}_{30 \times 1}, \quad
\mathbf{u}_{state} = \begin{bmatrix}
x_{e,\max} \\ y_{e,\max} \\ \theta_{e,\max} \\
x_{e,\max} \\ y_{e,\max} \\ \theta_{e,\max} \\
\vdots \\
x_{e,\max} \\ y_{e,\max} \\ \theta_{e,\max}
\end{bmatrix}_{30 \times 1}
$$

#### 6.3.3 推荐设置

**方案1：不约束状态（推荐）**

通常位姿误差不做硬约束（软约束通过目标函数实现）：

$$
\mathbf{l}_{state} = \begin{bmatrix}
-\infty \\ -\infty \\ -\pi \\
-\infty \\ -\infty \\ -\pi \\
\vdots
\end{bmatrix}, \quad
\mathbf{u}_{state} = \begin{bmatrix}
+\infty \\ +\infty \\ +\pi \\
+\infty \\ +\infty \\ +\pi \\
\vdots
\end{bmatrix}
$$

**方案2：约束横向偏差（车道保持）**

如果需要车道保持（横向偏差 ≤ 0.5m）：

$$
\mathbf{l}_{state} = \begin{bmatrix}
-\infty \\ -0.5 \\ -\pi \\
\vdots
\end{bmatrix}, \quad
\mathbf{u}_{state} = \begin{bmatrix}
+\infty \\ 0.5 \\ +\pi \\
\vdots
\end{bmatrix}
$$

**Python实现**：

```python
import numpy as np

N = 10
# 方案1：不约束
l_state = np.tile([-np.inf, -np.inf, -np.pi], N)
u_state = np.tile([np.inf, np.inf, np.pi], N)

# 方案2：约束横向偏差
l_state = np.tile([-np.inf, -0.5, -np.pi], N)
u_state = np.tile([np.inf, 0.5, np.pi], N)
```

### 6.4 控制边界约束（详细）

#### 6.4.1 约束定义

对于预测时域内的每一步 $i=0,1,\ldots,N-1$：

$$
\begin{bmatrix}
v_{\min} \\ \omega_{\min}
\end{bmatrix}
\leq
\begin{bmatrix}
v(k+i) \\ \omega(k+i)
\end{bmatrix}
\leq
\begin{bmatrix}
v_{\max} \\ \omega_{\max}
\end{bmatrix}
$$

**物理意义**：
- $v_{\min}, v_{\max}$：线速度限制（电机能力）
- $\omega_{\min}, \omega_{\max}$：角速度限制（转向能力）

**关键问题**：决策变量是速度**误差** $\Delta\mathbf{u}$，而非实际速度！

需要转换：

$$
v(k+i) = v_r + \Delta v(k+i), \quad \omega(k+i) = \omega_r + \Delta\omega(k+i)
$$

因此约束变为：

$$
\begin{bmatrix}
v_{\min} - v_r \\ \omega_{\min} - \omega_r
\end{bmatrix}
\leq
\begin{bmatrix}
\Delta v(k+i) \\ \Delta\omega(k+i)
\end{bmatrix}
\leq
\begin{bmatrix}
v_{\max} - v_r \\ \omega_{\max} - \omega_r
\end{bmatrix}
$$

**注意**：右端边界会随参考速度 $v_r, \omega_r$ 动态变化！

#### 6.4.2 矩阵形式推导

将所有N步的控制约束堆叠：

$$
\begin{bmatrix}
v_{\min} - v_r \\ \omega_{\min} - \omega_r \\
v_{\min} - v_r \\ \omega_{\min} - \omega_r \\
\vdots \\
v_{\min} - v_r \\ \omega_{\min} - \omega_r
\end{bmatrix}_{20 \times 1}
\leq
\begin{bmatrix}
\Delta\mathbf{u}(k) \\
\Delta\mathbf{u}(k+1) \\
\vdots \\
\Delta\mathbf{u}(k+N-1)
\end{bmatrix}_{20 \times 1}
\leq
\begin{bmatrix}
v_{\max} - v_r \\ \omega_{\max} - \omega_r \\
v_{\max} - v_r \\ \omega_{\max} - \omega_r \\
\vdots \\
v_{\max} - v_r \\ \omega_{\max} - \omega_r
\end{bmatrix}_{20 \times 1}
$$

在稀疏形式中，控制变量占据决策变量的后20个元素（索引30-49）。

因此约束矩阵需要**选择**决策变量的后20个元素：

$$
\mathbf{A}_{control} = \begin{bmatrix}
\mathbf{0}_{20 \times 30} & \mathbf{I}_{20}
\end{bmatrix}_{20 \times 50}
$$

**矩阵结构解释**：
- $\mathbf{0}_{20 \times 30}$：20×30零矩阵，状态变量不参与此约束
- $\mathbf{I}_{20}$：20×20单位矩阵，选择控制变量

**约束形式**：

$$
\mathbf{l}_{control} \leq \mathbf{A}_{control} \boldsymbol{\xi} \leq \mathbf{u}_{control}
$$

**边界向量**：

$$
\mathbf{l}_{control} = \begin{bmatrix}
v_{\min} - v_r \\ \omega_{\min} - \omega_r \\
v_{\min} - v_r \\ \omega_{\min} - \omega_r \\
\vdots \\
v_{\min} - v_r \\ \omega_{\min} - \omega_r
\end{bmatrix}_{20 \times 1}, \quad
\mathbf{u}_{control} = \begin{bmatrix}
v_{\max} - v_r \\ \omega_{\max} - \omega_r \\
v_{\max} - v_r \\ \omega_{\max} - \omega_r \\
\vdots \\
v_{\max} - v_r \\ \omega_{\max} - \omega_r
\end{bmatrix}_{20 \times 1}
$$

#### 6.4.3 推荐设置和数值示例

**典型的速度和角速度限制**：

$$
\begin{align}
v_{\min}, v_{\max} &= [0, 2] \text{ m/s} \\
\omega_{\min}, \omega_{\max} &= [-1, 1] \text{ rad/s} \quad \text{(约 ±57°/s)}
\end{align}
$$

**数值示例1**：直线行驶（$v_r=1.0$ m/s, $\omega_r=0.1$ rad/s）

$$
\mathbf{l}_{control} = \begin{bmatrix}
0-1.0 \\ -1-0.1 \\
\vdots
\end{bmatrix} = \begin{bmatrix}
-1.0 \\ -1.1 \\
\vdots
\end{bmatrix}, \quad
\mathbf{u}_{control} = \begin{bmatrix}
2-1.0 \\ 1-0.1 \\
\vdots
\end{bmatrix} = \begin{bmatrix}
1.0 \\ 0.9 \\
\vdots
\end{bmatrix}
$$

**数值示例2**：停车场景（$v_r=0$ m/s, $\omega_r=0$ rad/s）

$$
\mathbf{l}_{control} = \begin{bmatrix}
-0 \\ -1 \\
\vdots
\end{bmatrix} = \begin{bmatrix}
0 \\ -1 \\
\vdots
\end{bmatrix}, \quad
\mathbf{u}_{control} = \begin{bmatrix}
2 \\ 1 \\
\vdots
\end{bmatrix}
$$

**注意不对称性**：
- 直线行驶时：线速度可加速1.0m/s或减速1.0m/s（对称）
- 直线行驶时：角速度可增加0.9rad/s或减小1.1rad/s（不对称）
- 这是因为参考角速度本身就是0.1rad/s

**Python实现**：

```python
import numpy as np

# 硬件限制
v_min, v_max = 0.0, 2.0
omega_min, omega_max = -1.0, 1.0

# 参考速度（每个时刻可能不同）
v_r = 1.0
omega_r = 0.1

N = 10

# 构造边界向量
l_control = np.tile([v_min - v_r, omega_min - omega_r], N)
u_control = np.tile([v_max - v_r, omega_max - omega_r], N)

# 如果参考速度时变
for i in range(N):
    v_r_i = get_reference_speed(i)  # 假设函数
    omega_r_i = get_reference_angular_speed(i)
    l_control[2*i:2*i+2] = [v_min - v_r_i, omega_min - omega_r_i]
    u_control[2*i:2*i+2] = [v_max - v_r_i, omega_max - omega_r_i]
```

#### 6.4.4 约束可行性检查

**重要**：需要确保约束有解！

**可能的无解情况**：
- 参考速度超出硬件限制：$v_r > v_{\max}$ 或 $v_r < v_{\min}$
- 会导致 $l_{control} > u_{control}$（矛盾）

**检查方法**：

```python
# 检查可行性
assert v_min <= v_r <= v_max, "参考速度超出限制！"
assert omega_min <= omega_r <= omega_max, "参考角速度超出限制！"
assert (l_control <= u_control).all(), "约束上下界矛盾！"
```

### 6.5 加速度约束（详细）

#### 6.5.1 约束定义

对于相邻两步之间的控制变化率 $i=0,1,\ldots,N-2$：

$$
\begin{bmatrix}
a_{\min} \\ \alpha_{\min}
\end{bmatrix}
\leq
\begin{bmatrix}
\Delta v(k+i+1) - \Delta v(k+i) \\
\Delta\omega(k+i+1) - \Delta\omega(k+i)
\end{bmatrix}
\leq
\begin{bmatrix}
a_{\max} \\ \alpha_{\max}
\end{bmatrix}
$$

**物理意义**：
- $a_{\min}, a_{\max}$：线加速度限制（加速/制动能力）
- $\alpha_{\min}, \alpha_{\max}$：角加速度限制（转向变化率）

**为什么需要？**
- 防止速度突变（机械冲击）
- 提高乘客舒适性
- 保护执行器

**控制变化率数量**：
- 控制序列有N个：$\Delta\mathbf{u}(k), \ldots, \Delta\mathbf{u}(k+N-1)$
- 变化率有N-1个：$\delta\mathbf{u}(k+1), \ldots, \delta\mathbf{u}(k+N-1)$
- 每个2维，共 $(N-1) \times 2 = 18$ 个约束

#### 6.5.2 矩阵形式推导

定义控制变化率向量：

$$
\delta\mathbf{u}(k+i) = \Delta\mathbf{u}(k+i) - \Delta\mathbf{u}(k+i-1), \quad i=1,\ldots,N-1
$$

堆叠所有变化率：

$$
\boldsymbol{\delta} = \begin{bmatrix}
\delta\mathbf{u}(k+1) \\
\delta\mathbf{u}(k+2) \\
\vdots \\
\delta\mathbf{u}(k+N-1)
\end{bmatrix}_{18 \times 1} = \mathbf{D}_{accel} \mathbf{U}
$$

其中差分矩阵 $\mathbf{D}_{accel} \in \mathbb{R}^{18 \times 20}$：

$$
\mathbf{D}_{accel} = \begin{bmatrix}
-\mathbf{I}_2 & \mathbf{I}_2 & \mathbf{0} & \cdots & \mathbf{0} \\
\mathbf{0} & -\mathbf{I}_2 & \mathbf{I}_2 & \cdots & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & -\mathbf{I}_2 & \cdots & \mathbf{0} \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \cdots & \mathbf{I}_2
\end{bmatrix}_{18 \times 20}
$$

**展开为标量形式**（更清晰）：

$$
\mathbf{D}_{accel} = \begin{bmatrix}
-1 & 0 & 1 & 0 & 0 & 0 & \cdots & 0 & 0 \\
0 & -1 & 0 & 1 & 0 & 0 & \cdots & 0 & 0 \\
0 & 0 & -1 & 0 & 1 & 0 & \cdots & 0 & 0 \\
0 & 0 & 0 & -1 & 0 & 1 & \cdots & 0 & 0 \\
\vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \ddots & \vdots & \vdots \\
0 & 0 & 0 & 0 & 0 & 0 & \cdots & -1 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & \cdots & 0 & -1 \\
0 & 0 & 0 & 0 & 0 & 0 & \cdots & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & \cdots & 0 & 1
\end{bmatrix}_{18 \times 20}
$$

**差分矩阵详细说明**：

- **第1-2行**：$\begin{bmatrix} -1 & 0 & 1 & 0 & 0 & \cdots \\ 0 & -1 & 0 & 1 & 0 & \cdots \end{bmatrix}$
  - 计算 $\delta\mathbf{u}(k+1) = \Delta\mathbf{u}(k+1) - \Delta\mathbf{u}(k)$
  - 第1行：$-\Delta v(k) + \Delta v(k+1)$
  - 第2行：$-\Delta\omega(k) + \Delta\omega(k+1)$

- **第3-4行**：$\begin{bmatrix} 0 & 0 & -1 & 0 & 1 & 0 & \cdots \\ 0 & 0 & 0 & -1 & 0 & 1 & \cdots \end{bmatrix}$
  - 计算 $\delta\mathbf{u}(k+2) = \Delta\mathbf{u}(k+2) - \Delta\mathbf{u}(k+1)$

- ...以此类推

在稀疏形式中，控制变量占据决策变量的后20个元素，因此：

$$
\mathbf{A}_{accel} = \begin{bmatrix}
\mathbf{0}_{18 \times 30} & \mathbf{D}_{accel}
\end{bmatrix}_{18 \times 50}
$$

**矩阵结构解释**：
- $\mathbf{0}_{18 \times 30}$：状态变量不参与此约束
- $\mathbf{D}_{accel}$：只作用于控制变量部分

**约束形式**：

$$
\mathbf{l}_{accel} \leq \mathbf{A}_{accel} \boldsymbol{\xi} \leq \mathbf{u}_{accel}
$$

**边界向量**：

$$
\mathbf{l}_{accel} = \begin{bmatrix}
a_{\min} \\ \alpha_{\min} \\
a_{\min} \\ \alpha_{\min} \\
\vdots \\
a_{\min} \\ \alpha_{\min}
\end{bmatrix}_{18 \times 1}, \quad
\mathbf{u}_{accel} = \begin{bmatrix}
a_{\max} \\ \alpha_{\max} \\
a_{\max} \\ \alpha_{\max} \\
\vdots \\
a_{\max} \\ \alpha_{\max}
\end{bmatrix}_{18 \times 1}
$$

#### 6.5.3 推荐设置和数值示例

**典型的加速度限制**：

$$
\begin{align}
a_{\min}, a_{\max} &= [-2, 2] \text{ m/s}^2 \\
\alpha_{\min}, \alpha_{\max} &= [-1, 1] \text{ rad/s}^2
\end{align}
$$

**选择依据**：
- 轿车舒适加速度：0.5-1.5 m/s²
- 紧急制动加速度：5-8 m/s²
- MPC通常取中间值：2 m/s²（兼顾舒适性和响应性）

**Python实现**：

```python
import numpy as np
from scipy import sparse

N = 10
nu = 2  # 控制维度

# 构造差分矩阵 D_accel (18×20)
D_accel = sparse.lil_matrix((18, 20))
for i in range(9):  # N-1 = 9
    # 第2i到2i+1行
    D_accel[2*i:2*i+2, 2*i:2*i+2] = -np.eye(2)  # -I
    D_accel[2*i:2*i+2, 2*i+2:2*i+4] = np.eye(2)  # +I
D_accel = D_accel.tocsc()

# 构造边界向量
a_min, a_max = -2.0, 2.0
alpha_min, alpha_max = -1.0, 1.0
l_accel = np.tile([a_min, alpha_min], 9)
u_accel = np.tile([a_max, alpha_max], 9)

# 完整约束矩阵（作用于决策变量）
A_accel = sparse.hstack([
    sparse.csc_matrix((18, 30)),  # 状态部分（零矩阵）
    D_accel                        # 控制部分
], format='csc')
```

#### 6.5.4 加速度约束的物理解释

**数值示例**：假设采样时间 $T=0.1$s，加速度限制 $a_{\max}=2$ m/s²

则单步最大速度变化：

$$
\Delta v_{\max}(k+i+1) - \Delta v_{\max}(k+i) \leq a_{\max} = 2 \text{ m/s}^2
$$

**对比单位**：
- 速度误差 $\Delta v$：m/s
- 速度误差变化率 $\delta v$：m/s（每采样周期）
- 实际加速度：$\frac{\delta v}{T} = \frac{2}{0.1} = 20$ m/s²？

**注意**：这里的约束是速度误差的变化，而非实际加速度！

**如果要约束实际加速度**：

$$
\frac{\delta v}{T} \leq a_{\max} \quad \Rightarrow \quad \delta v \leq T \cdot a_{\max} = 0.1 \times 2 = 0.2 \text{ m/s}
$$

因此边界应该是：

```python
l_accel = np.tile([T * a_min, T * alpha_min], 9)
u_accel = np.tile([T * a_max, T * alpha_max], 9)
```

### 6.6 约束汇总和完整构造

#### 6.6.1 不等式约束堆叠

所有3类不等式约束堆叠为：

$$
\mathbf{l}_{ineq} \leq \mathbf{A}_{ineq} \boldsymbol{\xi} \leq \mathbf{u}_{ineq}
$$

其中：

$$
\mathbf{A}_{ineq} = \begin{bmatrix}
\mathbf{A}_{state} \\
\mathbf{A}_{control} \\
\mathbf{A}_{accel}
\end{bmatrix}_{68 \times 50}, \quad
\mathbf{l}_{ineq} = \begin{bmatrix}
\mathbf{l}_{state} \\
\mathbf{l}_{control} \\
\mathbf{l}_{accel}
\end{bmatrix}_{68 \times 1}, \quad
\mathbf{u}_{ineq} = \begin{bmatrix}
\mathbf{u}_{state} \\
\mathbf{u}_{control} \\
\mathbf{u}_{accel}
\end{bmatrix}_{68 \times 1}
$$

**维度验证**：
- $\mathbf{A}_{state}$：30×50
- $\mathbf{A}_{control}$：20×50
- $\mathbf{A}_{accel}$：18×50
- 堆叠后：(30+20+18)×50 = 68×50 ✓

#### 6.6.2 约束矩阵的稀疏结构

**$\mathbf{A}_{ineq}$ 的完整结构**：

$$
\mathbf{A}_{ineq} = \begin{bmatrix}
\mathbf{I}_{30} & \mathbf{0}_{30 \times 20} \\
\mathbf{0}_{20 \times 30} & \mathbf{I}_{20} \\
\mathbf{0}_{18 \times 30} & \mathbf{D}_{accel}
\end{bmatrix}_{68 \times 50}
$$

**稀疏性分析**：
- 总元素：68 × 50 = 3400
- 非零元素：30×30 + 20×20 + 18×4 ≈ 1372
- 稀疏度：40%（相对稠密）

**块状结构**：
- 前30行：只涉及状态变量（前30列非零）
- 中20行：只涉及控制变量（后20列非零）
- 后18行：只涉及控制变量（后20列非零，三对角带状）

#### 6.6.3 完整约束构造的Python实现

```python
import numpy as np
from scipy import sparse

def build_inequality_constraints(N, v_r, omega_r, v_min, v_max, omega_min, omega_max, a_min, a_max, alpha_min, alpha_max):
    """
    构造完整的不等式约束
    
    参数:
        N: 预测步数
        v_r, omega_r: 参考速度
        v_min, v_max: 速度限制
        omega_min, omega_max: 角速度限制
        a_min, a_max: 线加速度限制
        alpha_min, alpha_max: 角加速度限制
    
    返回:
        A_ineq: 约束矩阵 (68×50)
        l_ineq, u_ineq: 约束边界 (68×1)
    """
    nx, nu = 3, 2
    
    # 1. 状态边界约束 (30×50)
    A_state = sparse.hstack([
        sparse.eye(N*nx),
        sparse.csc_matrix((N*nx, N*nu))
    ], format='csc')
    
    l_state = np.tile([-np.inf, -np.inf, -np.pi], N)
    u_state = np.tile([np.inf, np.inf, np.pi], N)
    
    # 2. 控制边界约束 (20×50)
    A_control = sparse.hstack([
        sparse.csc_matrix((N*nu, N*nx)),
        sparse.eye(N*nu)
    ], format='csc')
    
    l_control = np.tile([v_min - v_r, omega_min - omega_r], N)
    u_control = np.tile([v_max - v_r, omega_max - omega_r], N)
    
    # 3. 加速度约束 (18×50)
    D_accel = sparse.lil_matrix(((N-1)*nu, N*nu))
    for i in range(N-1):
        D_accel[nu*i:nu*i+nu, nu*i:nu*i+nu] = -np.eye(nu)
        D_accel[nu*i:nu*i+nu, nu*i+nu:nu*i+2*nu] = np.eye(nu)
    D_accel = D_accel.tocsc()
    
    A_accel = sparse.hstack([
        sparse.csc_matrix(((N-1)*nu, N*nx)),
        D_accel
    ], format='csc')
    
    T = 0.1  # 采样时间
    l_accel = np.tile([T*a_min, T*alpha_min], N-1)
    u_accel = np.tile([T*a_max, T*alpha_max], N-1)
    
    # 4. 堆叠所有约束
    A_ineq = sparse.vstack([A_state, A_control, A_accel], format='csc')
    l_ineq = np.concatenate([l_state, l_control, l_accel])
    u_ineq = np.concatenate([u_state, u_control, u_accel])
    
    return A_ineq, l_ineq, u_ineq


# 使用示例
N = 10
v_r, omega_r = 1.0, 0.1
v_min, v_max = 0.0, 2.0
omega_min, omega_max = -1.0, 1.0
a_min, a_max = -2.0, 2.0
alpha_min, alpha_max = -1.0, 1.0

A_ineq, l_ineq, u_ineq = build_inequality_constraints(
    N, v_r, omega_r, v_min, v_max, omega_min, omega_max, 
    a_min, a_max, alpha_min, alpha_max
)

print(f"约束矩阵维度: {A_ineq.shape}")  # (68, 50)
print(f"非零元素数: {A_ineq.nnz}")
print(f"稀疏度: {100 * A_ineq.nnz / (A_ineq.shape[0] * A_ineq.shape[1]):.1f}%")
```

#### 6.6.4 约束维度汇总表

| 约束类型 | 约束矩阵 | 维度 | 下界 | 上界 | 说明 |
|---------|---------|------|------|------|------|
| 状态边界 | $\mathbf{A}_{state}$ | 30×50 | $\mathbf{l}_{state}$ | $\mathbf{u}_{state}$ | 通常无穷 |
| 控制边界 | $\mathbf{A}_{control}$ | 20×50 | $\mathbf{l}_{control}$ | $\mathbf{u}_{control}$ | 动态变化 |
| 加速度 | $\mathbf{A}_{accel}$ | 18×50 | $\mathbf{l}_{accel}$ | $\mathbf{u}_{accel}$ | 乘以T |
| **汇总** | $\mathbf{A}_{ineq}$ | **68×50** | $\mathbf{l}_{ineq}$ | $\mathbf{u}_{ineq}$ | **总约束** |

---

## 7. 完整QP问题形式

### 7.1 标准OSQP形式

稀疏形式MPC问题的完整QP形式：

$$
\boxed{
\begin{align}
\min_{\boldsymbol{\xi}} \quad & \frac{1}{2} \boldsymbol{\xi}^\top \mathbf{P} \boldsymbol{\xi} \\
\text{s.t.} \quad & \mathbf{l}_{all} \leq \mathbf{A}_{all} \boldsymbol{\xi} \leq \mathbf{u}_{all}
\end{align}
}
$$

### 7.2 所有约束堆叠

将等式约束和不等式约束堆叠在一起：

$$
\mathbf{A}_{all} = \begin{bmatrix}
\mathbf{A}_{eq} \\
\mathbf{A}_{state} \\
\mathbf{A}_{control} \\
\mathbf{A}_{accel}
\end{bmatrix}_{98 \times 50}
$$

$$
\mathbf{l}_{all} = \begin{bmatrix}
\mathbf{b}_{eq} \\
\mathbf{l}_{state} \\
\mathbf{l}_{control} \\
\mathbf{l}_{accel}
\end{bmatrix}_{98 \times 1}, \quad
\mathbf{u}_{all} = \begin{bmatrix}
\mathbf{b}_{eq} \\
\mathbf{u}_{state} \\
\mathbf{u}_{control} \\
\mathbf{u}_{accel}
\end{bmatrix}_{98 \times 1}
$$

**注意**：等式约束通过设置 $\mathbf{l}_{eq} = \mathbf{u}_{eq} = \mathbf{b}_{eq}$ 实现。

### 7.3 问题规模汇总（N=10）

| 项目 | 符号 | 维度 | 说明 |
|------|------|------|------|
| 决策变量 | $\boldsymbol{\xi}$ | 50 | 30状态+20控制 |
| Hessian矩阵 | $\mathbf{P}$ | $50 \times 50$ | 块对角稀疏 |
| 梯度向量 | $\mathbf{q}$ | 50 | 全零 |
| 约束矩阵 | $\mathbf{A}_{all}$ | $98 \times 50$ | 稀疏 |
| 等式约束 | $\mathbf{A}_{eq}$ | $30 \times 50$ | 动力学 |
| 状态边界 | $\mathbf{A}_{state}$ | $30 \times 50$ | 通常无穷 |
| 控制边界 | $\mathbf{A}_{control}$ | $20 \times 50$ | 速度限制 |
| 加速度约束 | $\mathbf{A}_{accel}$ | $18 \times 50$ | 加速度限制 |

### 7.4 稀疏性分析

**Hessian矩阵**：
- 总元素：$50 \times 50 = 2500$
- 非零元素：约 $30 \times 3 + 20 \times 2 + 18 \times 4 = 202$
- 稀疏度：$202 / 2500 \approx 8\%$

**约束矩阵**：
- 总元素：$98 \times 50 = 4900$
- 非零元素：约 $30 \times 8 + 30 \times 3 + 20 \times 2 + 18 \times 4 = 442$
- 稀疏度：$442 / 4900 \approx 9\%$

**OSQP优势**：稀疏矩阵求解器对这种高度稀疏的结构优化效果极好。

### 7.5 求解与控制

OSQP求解得到：

$$
\boldsymbol{\xi}^* = \begin{bmatrix}
\mathbf{X}_e^*(k+1) \\
\mathbf{X}_e^*(k+2) \\
\vdots \\
\mathbf{X}_e^*(k+N) \\
\Delta\mathbf{u}^*(k) \\
\Delta\mathbf{u}^*(k+1) \\
\vdots \\
\Delta\mathbf{u}^*(k+N-1)
\end{bmatrix}
$$

**MPC滚动时域策略**：只应用第一个控制量

$$
\Delta\mathbf{u}^*(k) = \begin{bmatrix} \Delta v^*(k) \\ \Delta\omega^*(k) \end{bmatrix} = \boldsymbol{\xi}^*[30:32]
$$

**实际控制输出**：

$$
\boxed{
\begin{align}
v_{\text{cmd}} &= v_r + \Delta v^*(k) \\
\omega_{\text{cmd}} &= \omega_r + \Delta\omega^*(k)
\end{align}
}
$$

### 7.6 滚动时域控制流程

```
初始化: k = 0, 给定初始位姿误差 X_e(0)
while 未到达目标:
    1. 测量当前误差状态 X_e(k)
    2. 获取参考速度 v_r(k), ω_r(k)
    3. 更新 b_eq（包含 X_e(k)）
    4. 更新 l_control, u_control（包含 v_r, ω_r）
    5. 调用OSQP求解 → ξ*
    6. 提取第一个控制量: v_cmd = v_r + Δv*(k), ω_cmd = ω_r + Δω*(k)
    7. 应用控制，等待采样周期 T
    8. k = k + 1
end
```

---

## 8. 完整数值示例

### 8.1 参数设置

| 参数 | 值 | 说明 |
|------|---|------|
| $T$ | 0.1 s | 采样时间 |
| $N$ | 10 | 预测步数 |
| $v_r$ | 1.0 m/s | 参考线速度 |
| $\omega_r$ | 0.1 rad/s | 参考角速度 |
| $v_{\min}, v_{\max}$ | [0, 2] m/s | 速度范围 |
| $\omega_{\min}, \omega_{\max}$ | [-1, 1] rad/s | 角速度范围 |
| $a_{\min}, a_{\max}$ | [-2, 2] m/s² | 线加速度范围 |
| $\alpha_{\min}, \alpha_{\max}$ | [-1, 1] rad/s² | 角加速度范围 |

### 8.2 权重矩阵

$$
\begin{align}
\mathbf{Q}_1 &= \text{diag}(100, 100, 10) && \text{位姿误差} \\
\mathbf{R}_1 &= \text{diag}(0.1, 0.1) && \text{速度误差} \\
\mathbf{R}_2 &= \text{diag}(1, 1) && \text{加速度} \\
\mathbf{Q}_f &= 1000 \times \mathbf{Q}_1 && \text{终点位姿} \\
\mathbf{R}_f &= 100 \times \mathbf{R}_1 && \text{终点速度} \\
\mathbf{R}_a &= 10 \times \mathbf{R}_2 && \text{终点加速度（适中权重）}
\end{align}
$$

### 8.3 离散化矩阵计算

$$
\mathbf{A} = \begin{bmatrix}
0 & 0.1 & 0 \\
-0.1 & 0 & 1 \\
0 & 0 & 0
\end{bmatrix}, \quad
\mathbf{B} = \begin{bmatrix}
-1 & 0 \\
0 & 0 \\
0 & -1
\end{bmatrix}
$$

$$
\mathbf{A}_d = \mathbf{I} + 0.1 \mathbf{A} = \begin{bmatrix}
1 & 0.01 & 0 \\
-0.01 & 1 & 0.1 \\
0 & 0 & 1
\end{bmatrix}, \quad
\mathbf{B}_d = 0.1 \mathbf{B} = \begin{bmatrix}
-0.1 & 0 \\
0 & 0 \\
0 & -0.1
\end{bmatrix}
$$

### 8.4 初始状态示例

假设当前误差状态：

$$
\mathbf{X}_e(k) = \begin{bmatrix} 0.5 \\ 0.2 \\ 0.1 \end{bmatrix} \text{ (m, m, rad)}
$$

则等式约束右端项前3个元素：

$$
\mathbf{b}_{eq}[0:3] = -\mathbf{A}_d \mathbf{X}_e(k) = -\begin{bmatrix}
1 & 0.01 & 0 \\
-0.01 & 1 & 0.1 \\
0 & 0 & 1
\end{bmatrix} \begin{bmatrix}
0.5 \\ 0.2 \\ 0.1
\end{bmatrix} = \begin{bmatrix}
-0.502 \\
-0.195 \\
-0.1
\end{bmatrix}
$$

### 8.5 矩阵维度汇总

| 矩阵/向量 | 维度 | 说明 |
|----------|------|------|
| $\boldsymbol{\xi}$ | $50 \times 1$ | 决策变量 |
| $\mathbf{P}$ | $50 \times 50$ | Hessian（块对角） |
| $\mathbf{q}$ | $50 \times 1$ | 梯度（全零） |
| $\mathbf{A}_{eq}$ | $30 \times 50$ | 等式约束矩阵 |
| $\mathbf{b}_{eq}$ | $30 \times 1$ | 等式约束右端项 |
| $\mathbf{A}_{state}$ | $30 \times 50$ | 状态边界约束 |
| $\mathbf{A}_{control}$ | $20 \times 50$ | 控制边界约束 |
| $\mathbf{A}_{accel}$ | $18 \times 50$ | 加速度约束 |
| $\mathbf{A}_{all}$ | $98 \times 50$ | 所有约束堆叠 |

### 8.6 求解结果示例

假设OSQP求解成功，得到最优解 $\boldsymbol{\xi}^*$：

**提取结果**：
- 第一步预测状态：$\mathbf{X}_e^*(k+1) = \boldsymbol{\xi}^*[0:3]$
- 第一步控制：$\Delta\mathbf{u}^*(k) = \boldsymbol{\xi}^*[30:32]$

**控制输出**：
```
Δv*(k) = -0.234 m/s
Δω*(k) = 0.056 rad/s

实际控制：
v_cmd = 1.0 + (-0.234) = 0.766 m/s
ω_cmd = 0.1 + 0.056 = 0.156 rad/s
```

**验证动力学约束**（应接近零）：
```
residual = A_eq @ ξ* - b_eq
||residual|| = 2.3e-7  ✓（满足精度要求）
```

---

## 9. 参数调优指南

### 9.1 权重矩阵调优

#### Q1：位姿误差权重

| 现象 | 原因 | 调整 |
|------|------|------|
| 跟踪误差大 | Q1太小 | 增大Q1 |
| 控制激进、抖动 | Q1太大 | 减小Q1或增大R |
| 横向偏移严重 | $q_x$或$q_y$太小 | 增大横/纵向权重 |
| 角度误差大 | $q_\theta$太小 | 增大角度权重 |

**推荐**：$\mathbf{Q}_1 = \text{diag}(100, 100, 10)$

#### R1：速度误差权重

| 现象 | 原因 | 调整 |
|------|------|------|
| 控制抖动 | R1太小 | 增大R1 |
| 跟踪迟钝 | R1太大 | 减小R1 |
| 速度振荡 | R1与Q1不匹配 | 调整R1/Q1比例 |

**推荐**：$\mathbf{R}_1 = \text{diag}(0.1, 0.1)$

#### R2：加速度权重

| 现象 | 原因 | 调整 |
|------|------|------|
| 加速度突变 | R2太小 | 增大R2 |
| 响应过慢 | R2太大 | 减小R2 |
| 启动/刹车不平滑 | R2太小 | 增大R2 |

**推荐**：$\mathbf{R}_2 = \text{diag}(1, 1)$

#### 终点权重

| 权重 | 倍数 | 说明 |
|------|------|------|
| $\mathbf{Q}_f$ | $1000 \times \mathbf{Q}_1$ | 强制终点位姿收敛 |
| $\mathbf{R}_f$ | $100 \times \mathbf{R}_1$ | 强制终点速度为0 |
| $\mathbf{R}_a$ | $10 \times \mathbf{R}_2$ | 防止终点抖动⭐ |

**调优策略**：
- 如果终点停不住：增大 $\mathbf{Q}_f$ 和 $\mathbf{R}_f$
- 如果到点抖动：增大 $\mathbf{R}_a$
- 如果减速过于保守：减小 $\mathbf{R}_a$

### 9.2 预测步数 N

| N | 优点 | 缺点 | 场景 |
|---|------|------|------|
| 5 | 计算快，实时性好 | 预见性差 | 简单直线，低速 |
| **10** | **平衡** | **-** | **推荐** |
| 20 | 预见性好，轨迹平滑 | 计算慢，变量多 | 复杂轨迹，高速 |

**经验公式**：

$$
N \approx \frac{\text{期望预见距离}}{v_r \cdot T}
$$

例如：预见距离1m，速度1m/s，采样时间0.1s → $N = 1/(1 \times 0.1) = 10$

### 9.3 约束设置

#### 速度约束
```python
v_min, v_max = 0.0, 2.0        # m/s
omega_min, omega_max = -1.57, 1.57  # rad/s (±90°/s)
```

#### 加速度约束
```python
a_min, a_max = -2.0, 2.0       # m/s²
alpha_min, alpha_max = -1.0, 1.0    # rad/s²
```

**调整原则**：
- 约束越紧，安全性越高，但灵活性越差
- 约束应基于实际硬件能力（电机极限）
- 保守设定，留有安全余量

### 9.4 采样时间 T

| T (s) | 优点 | 缺点 | 适用场景 |
|-------|------|------|---------|
| 0.05 | 精度高，反应快 | 计算频繁 | 高速、高精度 |
| **0.1** | **平衡** | **-** | **推荐** |
| 0.2 | 计算轻松 | 精度降低 | 低速、简单任务 |

**选择依据**：
- 系统动态特性：快速系统需要小T
- 计算能力：嵌入式设备可能需要大T
- 控制精度要求：高精度需要小T

### 9.5 调试技巧

1. **检查约束矩阵维度**
   ```
   assert P.shape == (50, 50)
   assert A_all.shape == (98, 50)
   assert b_eq.shape[0] == 30
   ```

2. **验证等式约束**
   ```
   residual = A_eq @ solution - b_eq
   print(f"动力学残差: {np.linalg.norm(residual):.6f}")
   # 应 < 1e-5
   ```

3. **可视化预测轨迹**
   ```python
   # 提取预测状态
   x_pred = solution[0::3]  # 每3个取1个
   y_pred = solution[1::3]
   plt.plot(x_pred, y_pred, 'o-', label='MPC预测')
   ```

4. **监控求解时间**
```python
   import time
   t0 = time.time()
   result = prob.solve()
   dt = time.time() - t0
   print(f"求解时间: {dt*1000:.2f} ms")
   # 应 < T*1000 (即采样周期)
   ```

5. **检查目标函数值**
```python
   J = 0.5 * solution.T @ P @ solution
   print(f"目标函数值: {J:.4f}")
   # 应随时间递减
   ```

### 9.6 常见问题排查

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 求解失败 | 约束冲突 | 检查上下界是否合理 |
| 动力学残差大 | 等式约束错误 | 验证A_eq和b_eq构造 |
| 速度慢 | 稠密矩阵 | 使用sparse格式 |
| 结果抖动 | 权重不当 | 增大R1或R2 |
| 到点不停 | 终点权重小 | 增大Qf和Rf |
| 轨迹振荡 | Q/R不平衡 | 调整权重比例 |
| 求解时间长 | N太大或T太小 | 减小N或增大T |

---

## 参考文献

1. **Model Predictive Control**: Camacho & Alba (2007)
2. **Nonlinear Model Predictive Control**: Grüne & Pannek (2017)
3. **Apollo MPC实现**: `modules/common/math/mpc_osqp.cc`
4. **OSQP Documentation**: https://osqp.org/
5. **差速驱动运动学**: Lynch & Park - Modern Robotics (2017)

---

## 附录：符号表

| 符号 | 含义 | 维度 |
|------|------|------|
| $N$ | 预测时域步数 | 标量 |
| $T$ | 采样时间 | 标量 |
| $n_x$ | 状态维度 | 标量 (=3) |
| $n_u$ | 控制维度 | 标量 (=2) |
| $\mathbf{X}_e(k)$ | 第k步位姿误差 | $3 \times 1$ |
| $\Delta\mathbf{u}(k)$ | 第k步速度误差 | $2 \times 1$ |
| $\boldsymbol{\xi}$ | 决策变量 | $50 \times 1$ |
| $\mathbf{A}_d, \mathbf{B}_d$ | 离散化矩阵 | $3 \times 3$, $3 \times 2$ |
| $\mathbf{P}$ | Hessian矩阵 | $50 \times 50$ |
| $\mathbf{A}_{eq}$ | 等式约束矩阵 | $30 \times 50$ |
| $\mathbf{A}_{ineq}$ | 不等式约束矩阵 | $68 \times 50$ |
| $\mathbf{Q}_1, \mathbf{R}_1, \mathbf{R}_2$ | 权重矩阵 | $3 \times 3$, $2 \times 2$, $2 \times 2$ |
