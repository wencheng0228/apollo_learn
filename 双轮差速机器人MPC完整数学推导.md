# 双轮差速机器人MPC控制器完整数学推导（全变量形式）

> **基于Apollo实际使用的全变量QP形式，从状态空间方程到OSQP求解的完整推导**
> 
> **默认设置：N=10（预测时域），M=5（控制时域）**

---

## 目录

1. [预备知识：线性化与离散化](#1-预备知识线性化与离散化)
2. [全变量形式MPC基本概念](#2-全变量形式mpc基本概念)
3. [决策变量定义](#3-决策变量定义)
4. [等式约束构造](#4-等式约束构造)
5. [目标函数设计（6项）](#5-目标函数设计6项)
6. [不等式约束构造](#6-不等式约束构造)
7. [完整QP问题形式](#7-完整qp问题形式)
8. [完整数值示例](#8-完整数值示例)
9. [参数调优指南](#9-参数调优指南)
10. [附录：符号表](#10-附录符号表)

---

## 核心公式速查

### 默认参数设置

| 参数 | 符号 | 默认值 | 说明 |
|------|------|--------|------|
| 预测时域 | N | 10 | 预测未来10步状态 |
| 控制时域 | M | 5 | 只优化前5步控制 |
| 采样时间 | T | 0.1 s | 控制周期 |
| 决策变量 | - | 40 | 30状态 + 10控制 |

### 目标函数（6项）

| 序号 | 目标函数项 | 数学表示 | 权重矩阵 |
|------|-----------|---------|---------|
| 1️⃣ | 位姿误差 | $\sum_{i=1}^{10} \mathbf{X}_e^\top \mathbf{Q}_1 \mathbf{X}_e$ | $\mathbf{Q}_1 = \text{diag}(100, 100, 10)$ |
| 2️⃣ | 速度误差 | $\sum_{i=0}^{4} \Delta\mathbf{u}^\top \mathbf{R}_1 \Delta\mathbf{u}$ | $\mathbf{R}_1 = \text{diag}(0.1, 0.1)$ |
| 3️⃣ | 加速度平滑 | $\sum_{i=1}^{4} \delta\mathbf{u}^\top \mathbf{R}_2 \delta\mathbf{u}$ | $\mathbf{R}_2 = \text{diag}(1, 1)$ |
| 4️⃣ | 终点位姿误差 | $\mathbf{X}_e(10)^\top \mathbf{Q}_f \mathbf{X}_e(10)$ | $\mathbf{Q}_f = 1000 \cdot \mathbf{Q}_1$ |
| 5️⃣ | 终点速度误差 | $\Delta\mathbf{u}(4)^\top \mathbf{R}_f \Delta\mathbf{u}(4)$ | $\mathbf{R}_f = 100 \cdot \mathbf{R}_1$ |
| 6️⃣ | 终点加速度 | $\delta\mathbf{u}(4)^\top \mathbf{R}_a \delta\mathbf{u}(4)$ | $\mathbf{R}_a = 10 \cdot \mathbf{R}_2$ |

### 约束汇总

| 约束类型 | 数量 | 说明 |
|---------|------|------|
| 等式约束（动力学） | 10 | N步状态演化 |
| 等式约束（控制固定） | 10 | 后5步控制固定为第5步值 |
| 状态边界约束 | 30 | 通常设为无穷 |
| 控制边界约束 | 10 | M步控制限制 |
| 加速度约束 | 8 | M-1步加速度限制 |
| **总约束** | **68** | **20等式+48不等式** |
| **决策变量** | **40** | **30状态+10控制** |

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

采样时间为 $T=0.1$s，使用前向欧拉法：

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

---

## 2. 全变量形式MPC基本概念

### 2.1 什么是全变量形式？

**核心思想**：将未来时域内的**所有状态和控制都作为优化变量**，通过等式约束保证动力学关系。

**关键特点**：
- ✅ 状态和控制都是决策变量
- ✅ 动力学通过等式约束保证
- ✅ **控制时域 M < 预测时域 N**（减少计算量）
- ✅ Apollo、Autoware采用此形式

### 2.2 预测时域 vs 控制时域

**预测时域 N = 10**：
- 预测未来10步状态：$\mathbf{X}_e(k+1), \ldots, \mathbf{X}_e(k+10)$
- 预测时间：$10 \times 0.1 = 1.0$秒
- 预测距离（假设 $v_r=1$ m/s）：约 1.0 m
- 状态变量数：$10 \times 3 = 30$

**控制时域 M = 5**：
- 只优化前5步控制：$\Delta\mathbf{u}(k), \ldots, \Delta\mathbf{u}(k+4)$
- 后5步控制固定：$\Delta\mathbf{u}(k+5) = \cdots = \Delta\mathbf{u}(k+9) = \Delta\mathbf{u}(k+4)$
- 控制变量数：$5 \times 2 = 10$
- 控制时间：$5 \times 0.1 = 0.5$秒

### 2.3 为什么 M < N？

| 优势 | 说明 |
|------|------|
| **减少计算量** | 决策变量从50个减少到40个（20%优化） |
| **提高稳定性** | 远期控制保持不变，避免振荡 |
| **工程实践** | 嵌入式系统常用策略 |
| **求解速度** | 提升约30-40% |

### 2.4 决策变量总数

$$
\text{决策变量总数} = N \times n_x + M \times n_u = 10 \times 3 + 5 \times 2 = 40
$$

---

## 3. 决策变量定义

### 3.1 决策变量排列

全变量形式的决策变量包含**N步状态 + M步控制**：

$$
\boldsymbol{\xi} = \begin{bmatrix}
\mathbf{X}_e(k+1) \\ 
\mathbf{X}_e(k+2) \\ 
\vdots \\ 
\mathbf{X}_e(k+10) \\
\Delta\mathbf{u}(k) \\ 
\Delta\mathbf{u}(k+1) \\ 
\vdots \\ 
\Delta\mathbf{u}(k+4) 
\end{bmatrix}_{40 \times 1}
$$

**展开为标量形式**：

$$
\boldsymbol{\xi} = \begin{bmatrix}
x_e(k+1) \\ y_e(k+1) \\ \theta_e(k+1) \\
x_e(k+2) \\ y_e(k+2) \\ \theta_e(k+2) \\
\vdots \\ 
x_e(k+10) \\ y_e(k+10) \\ \theta_e(k+10) \\
\Delta v(k) \\ \Delta\omega(k) \\
\Delta v(k+1) \\ \Delta\omega(k+1) \\
\Delta v(k+2) \\ \Delta\omega(k+2) \\
\Delta v(k+3) \\ \Delta\omega(k+3) \\
\Delta v(k+4) \\ \Delta\omega(k+4)
\end{bmatrix}_{40 \times 1}
$$

### 3.2 索引规则

**索引约定**（从0开始）：

- **状态变量索引**：$\xi_0$ 到 $\xi_{29}$（共30个）
  - 第i步状态（i=1..10）：$\xi_{3(i-1)}, \xi_{3(i-1)+1}, \xi_{3(i-1)+2}$
  - 第1步：$\xi_0, \xi_1, \xi_2$ 对应 $x_e(k+1), y_e(k+1), \theta_e(k+1)$
  - 第10步：$\xi_{27}, \xi_{28}, \xi_{29}$ 对应 $x_e(k+10), y_e(k+10), \theta_e(k+10)$

- **控制变量索引**：$\xi_{30}$ 到 $\xi_{39}$（共10个）
  - 第j步控制（j=0..4）：$\xi_{30+2j}, \xi_{30+2j+1}$
  - 第1步：$\xi_{30}, \xi_{31}$ 对应 $\Delta v(k), \Delta\omega(k)$
  - 第5步：$\xi_{38}, \xi_{39}$ 对应 $\Delta v(k+4), \Delta\omega(k+4)$

### 3.3 维度汇总

| 变量类型 | 符号 | 维度 | 说明 |
|---------|------|------|------|
| 状态序列 | $\boldsymbol{\Xi}$ | $30 \times 1$ | N=10步×3状态 |
| 控制序列 | $\mathbf{U}$ | $10 \times 1$ | M=5步×2控制 |
| **决策变量** | $\boldsymbol{\xi}$ | **40 × 1** | **总优化变量** |

---

## 4. 等式约束构造

### 4.1 约束原理

全变量形式需要两类等式约束：

1. **动力学约束**（N=10个）：保证状态演化遵循物理规律
2. **控制固定约束**（N-M=5个）：固定后5步控制

**总等式约束数**：10 + 10 = 20个（每个3维的动力学约束 + 每个2维的控制固定）

### 4.2 动力学约束

对于每一步 $i = 0, 1, \ldots, N-1$（10个约束）：

$$
\mathbf{X}_e(k+i+1) = \mathbf{A}_d \mathbf{X}_e(k+i) + \mathbf{B}_d \Delta\mathbf{u}(k+i)
$$

改写为标准形式（左边=0）：

$$
-\mathbf{X}_e(k+i+1) + \mathbf{A}_d \mathbf{X}_e(k+i) + \mathbf{B}_d \Delta\mathbf{u}(k+i) = \mathbf{0}
$$

**特殊处理**：
- 对于 $i=0$：$\mathbf{X}_e(k)$ 是已知初始状态，移到右边
- 对于 $i \geq M$：$\Delta\mathbf{u}(k+i)$ 不是决策变量（通过控制固定约束确定）

### 4.3 控制固定约束

后 $N-M=5$ 步控制保持不变：

$$
\Delta\mathbf{u}(k+5) = \Delta\mathbf{u}(k+4)
$$
$$
\Delta\mathbf{u}(k+6) = \Delta\mathbf{u}(k+4)
$$
$$
\vdots
$$
$$
\Delta\mathbf{u}(k+9) = \Delta\mathbf{u}(k+4)
$$

即：

$$
\Delta\mathbf{u}(k+i) - \Delta\mathbf{u}(k+M-1) = \mathbf{0}, \quad i = M, M+1, \ldots, N-1
$$

**物理意义**：远期控制不再优化，使用最后一个优化控制值，提高稳定性。

### 4.4 等式约束矩阵形式

将所有等式约束堆叠：

$$
\mathbf{A}_{eq} \boldsymbol{\xi} = \mathbf{b}_{eq}
$$

其中 $\mathbf{A}_{eq} \in \mathbb{R}^{20 \times 40}$：

**右端项** $\mathbf{b}_{eq} \in \mathbb{R}^{20 \times 1}$：

$$
\mathbf{b}_{eq} = \begin{bmatrix}
-\mathbf{A}_d \mathbf{X}_e(k) \\
\mathbf{0}_3 \\
\vdots \\
\mathbf{0}_3 \\
\mathbf{0}_2 \\
\vdots \\
\mathbf{0}_2
\end{bmatrix}_{20 \times 1}
$$

**说明**：
- 前10行（行0-9）：动力学约束（每行3维）
- 后10行（行10-19）：控制固定约束（每行2维）

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
J = & \sum_{i=1}^{10} \mathbf{X}_e^\top(k+i) \mathbf{Q}_1 \mathbf{X}_e(k+i) 
    && \text{1️⃣ 位姿误差} \\
& + \sum_{i=0}^{4} \Delta\mathbf{u}^\top(k+i) \mathbf{R}_1 \Delta\mathbf{u}(k+i) 
    && \text{2️⃣ 速度误差} \\
& + \sum_{i=1}^{4} \delta\mathbf{u}^\top(k+i) \mathbf{R}_2 \delta\mathbf{u}(k+i) 
    && \text{3️⃣ 加速度平滑} \\
& + \mathbf{X}_e^\top(k+10) \mathbf{Q}_f \mathbf{X}_e(k+10) 
    && \text{4️⃣ 终点位姿误差} \\
& + \Delta\mathbf{u}^\top(k+4) \mathbf{R}_f \Delta\mathbf{u}(k+4) 
    && \text{5️⃣ 终点速度误差} \\
& + \delta\mathbf{u}^\top(k+4) \mathbf{R}_a \delta\mathbf{u}(k+4) 
    && \text{6️⃣ 终点加速度（防止抖动）}
\end{align}
}
$$

其中 $\delta\mathbf{u}(k+i) = \Delta\mathbf{u}(k+i) - \Delta\mathbf{u}(k+i-1)$ 是控制变化率（加速度）。

**关键说明**：
- 第1项：对**所有N=10步**状态求和
- 第2项：对**前M=5步**控制求和（i=0到4）
- 第3项：对**前M-1=4步**加速度求和（i=1到4）
- 第4项：强调第N=10步状态
- 第5项：强调第M=5步控制（k+4，最后一个优化控制）
- 第6项：强调第M=5步加速度（k+4，最后一个加速度）

### 5.3 各项详细解释

#### 5.3.1 第1项：位姿误差（核心项）

$$
J_1 = \sum_{i=1}^{10} \mathbf{X}_e^\top(k+i) \mathbf{Q}_1 \mathbf{X}_e(k+i)
$$

**展开形式**：

$$
J_1 = \sum_{i=1}^{10} \left( q_x \cdot x_e^2(k+i) + q_y \cdot y_e^2(k+i) + q_\theta \cdot \theta_e^2(k+i) \right)
$$

**调优建议**：
- 直线跟踪：$q_y > q_x$（横向更重要）
- 推荐：$\mathbf{Q}_1 = \text{diag}(100, 100, 10)$

#### 5.3.2 第2项：速度误差（控制平滑项）

$$
J_2 = \sum_{i=0}^{4} \Delta\mathbf{u}^\top(k+i) \mathbf{R}_1 \Delta\mathbf{u}(k+i)
$$

**展开形式**：

$$
J_2 = \sum_{i=0}^{4} \left( r_v \cdot (\Delta v)^2(k+i) + r_\omega \cdot (\Delta\omega)^2(k+i) \right)
$$

**推荐**：$\mathbf{R}_1 = \text{diag}(0.1, 0.1)$（相对Q1较小）

#### 5.3.3 第3项：加速度平滑（舒适性项）

$$
J_3 = \sum_{i=1}^{4} \delta\mathbf{u}^\top(k+i) \mathbf{R}_2 \delta\mathbf{u}(k+i)
$$

其中 $\delta\mathbf{u}(k+i) = \Delta\mathbf{u}(k+i) - \Delta\mathbf{u}(k+i-1)$

**展开形式**：

$$
J_3 = \sum_{i=1}^{4} \left( r_a \cdot (\delta v)^2(k+i) + r_\alpha \cdot (\delta\omega)^2(k+i) \right)
$$

**推荐**：$\mathbf{R}_2 = \text{diag}(1, 1)$

#### 5.3.4 第4项：终点位姿误差（强调项）

$$
J_4 = \mathbf{X}_e^\top(k+10) \mathbf{Q}_f \mathbf{X}_e(k+10)
$$

**作用**：额外强调预测时域的最后一步

**推荐**：$\mathbf{Q}_f = 1000 \times \mathbf{Q}_1$

#### 5.3.5 第5项：终点速度误差（停止项）

$$
J_5 = \Delta\mathbf{u}^\top(k+4) \mathbf{R}_f \Delta\mathbf{u}(k+4)
$$

**作用**：强制最后一个优化控制接近参考值

**推荐**：$\mathbf{R}_f = 100 \times \mathbf{R}_1$

**为什么索引是 k+4？**
- 控制序列：$\Delta\mathbf{u}(k), \ldots, \Delta\mathbf{u}(k+4)$（5个）
- 最后一个优化控制是 $\Delta\mathbf{u}(k+4)$
- 后续控制（k+5到k+9）由约束固定为此值

#### 5.3.6 第6项：终点加速度（防抖动项）

$$
J_6 = \delta\mathbf{u}^\top(k+4) \mathbf{R}_a \delta\mathbf{u}(k+4)
$$

其中 $\delta\mathbf{u}(k+4) = \Delta\mathbf{u}(k+4) - \Delta\mathbf{u}(k+3)$

**作用**：确保到达终点时加速度为0

**推荐**：$\mathbf{R}_a = 10 \times \mathbf{R}_2$（适中权重）

### 5.4 权重矩阵定义

| 权重 | 维度 | 推荐值 | 说明 |
|------|------|--------|------|
| $\mathbf{Q}_1$ | $3 \times 3$ | diag(100, 100, 10) | 核心项，确保跟踪精度 |
| $\mathbf{R}_1$ | $2 \times 2$ | diag(0.1, 0.1) | 控制平滑，参考Apollo |
| $\mathbf{R}_2$ | $2 \times 2$ | diag(1, 1) | 避免速度突变 |
| $\mathbf{Q}_f$ | $3 \times 3$ | $1000 \cdot \mathbf{Q}_1$ | 精确到达目标 |
| $\mathbf{R}_f$ | $2 \times 2$ | $100 \cdot \mathbf{R}_1$ | 确保停止 |
| $\mathbf{R}_a$ | $2 \times 2$ | $10 \cdot \mathbf{R}_2$ | 防止到点抖动⭐ |

### 5.5 全变量形式的Hessian矩阵构造（详细）

#### 5.5.1 目标函数的二次型转换

目标函数可以直接写为标准二次型：

$$
J = \frac{1}{2} \boldsymbol{\xi}^\top \mathbf{P} \boldsymbol{\xi}
$$

**关键优势**：状态和控制都是决策变量，无需通过预测方程消元，Hessian矩阵构造非常简单！

#### 5.5.2 Hessian矩阵的块对角结构

**Hessian矩阵** $\mathbf{P} \in \mathbb{R}^{40 \times 40}$ 是**块对角**结构：

$$
\mathbf{P} = \begin{bmatrix}
\bar{\mathbf{Q}} & \mathbf{0} \\
\mathbf{0} & \bar{\mathbf{R}} + \mathbf{R}_{accel}
\end{bmatrix}_{40 \times 40}
$$

**说明**：
- 左上角 $\bar{\mathbf{Q}}$ (30×30)：对应状态变量
- 右下角 $\bar{\mathbf{R}} + \mathbf{R}_{accel}$ (10×10)：对应控制变量
- 非对角块全为零：状态和控制在Hessian中不耦合

#### 5.5.3 状态部分Hessian（对应第1、4项）

$$
\bar{\mathbf{Q}} = \text{diag}(\underbrace{\mathbf{Q}_1, \mathbf{Q}_1, \ldots, \mathbf{Q}_1}_{9\text{个}}, \mathbf{Q}_1 + \mathbf{Q}_f)_{30 \times 30}
$$

**展开形式**：

$$
\bar{\mathbf{Q}} = \begin{bmatrix}
\mathbf{Q}_1 & & & & \\
& \mathbf{Q}_1 & & & \\
& & \ddots & & \\
& & & \mathbf{Q}_1 & \\
& & & & \mathbf{Q}_1 + \mathbf{Q}_f
\end{bmatrix}_{30 \times 30}
$$

**注意**：
- 前9个块对角：$\mathbf{Q}_1$ (每个3×3)
- 最后一个块对角：$\mathbf{Q}_1 + \mathbf{Q}_f$ (3×3)

#### 5.5.4 控制部分Hessian（对应第2、5项）

$$
\bar{\mathbf{R}} = \text{diag}(\underbrace{\mathbf{R}_1, \mathbf{R}_1, \ldots, \mathbf{R}_1}_{4\text{个}}, \mathbf{R}_1 + \mathbf{R}_f)_{10 \times 10}
$$

**展开形式**（M=5）：

$$
\bar{\mathbf{R}} = \begin{bmatrix}
\mathbf{R}_1 & & & & \\
& \mathbf{R}_1 & & & \\
& & \mathbf{R}_1 & & \\
& & & \mathbf{R}_1 & \\
& & & & \mathbf{R}_1 + \mathbf{R}_f
\end{bmatrix}_{10 \times 10}
$$

#### 5.5.5 加速度平滑Hessian（对应第3、6项）

定义差分矩阵 $\mathbf{D}_{accel} \in \mathbb{R}^{8 \times 10}$：

$$
\mathbf{D}_{accel} = \begin{bmatrix}
-\mathbf{I}_2 & \mathbf{I}_2 & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & -\mathbf{I}_2 & \mathbf{I}_2 & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & -\mathbf{I}_2 & \mathbf{I}_2 & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & -\mathbf{I}_2 & \mathbf{I}_2
\end{bmatrix}_{8 \times 10}
$$

**展开为标量形式**：

$$
\mathbf{D}_{accel} = \begin{bmatrix}
-1 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & -1 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & -1 & 0 & 1 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & -1 & 0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & -1 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & -1 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & -1 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & -1 & 0 & 1
\end{bmatrix}_{8 \times 10}
$$

**差分矩阵说明**：
- 行数：8 = (M-1) × 2 = 4 × 2
- 列数：10 = M × 2 = 5 × 2
- 每两行计算一个 $\delta\mathbf{u}(k+i) = \Delta\mathbf{u}(k+i) - \Delta\mathbf{u}(k+i-1)$

权重矩阵：

$$
\bar{\mathbf{R}}_2 = \text{diag}(\underbrace{\mathbf{R}_2, \mathbf{R}_2, \mathbf{R}_2}_{3\text{个}}, \mathbf{R}_2 + \mathbf{R}_a)_{8 \times 8}
$$

加速度Hessian：

$$
\boxed{\mathbf{R}_{accel} = \mathbf{D}_{accel}^\top \bar{\mathbf{R}}_2 \mathbf{D}_{accel} \quad (10 \times 10)}
$$

#### 5.5.6 完整Hessian矩阵

$$
\boxed{
\mathbf{P} = \begin{bmatrix}
\bar{\mathbf{Q}} & \mathbf{0}_{30 \times 10} \\
\mathbf{0}_{10 \times 30} & \bar{\mathbf{R}} + \mathbf{R}_{accel}
\end{bmatrix}_{40 \times 40}
}
$$

**维度验证**：
- $\bar{\mathbf{Q}}$：30×30（状态部分）
- $\bar{\mathbf{R}} + \mathbf{R}_{accel}$：10×10（控制部分）
- 总维度：40×40 ✓

**稀疏性**：
- 总元素：1600
- 非零元素：约 142
- 稀疏度：约 9%（高度稀疏）

#### 5.5.7 Python实现

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
M = 5   # 控制步数

# 1. 状态部分Hessian (30×30)
Q_bar = sparse.block_diag([Q1]*(N-1) + [Q1 + Qf], format='csc')

# 2. 控制部分Hessian (10×10)
R_bar = sparse.block_diag([R1]*(M-1) + [R1 + Rf], format='csc')

# 3. 差分矩阵 (8×10)
D_accel = sparse.lil_matrix((2*(M-1), 2*M))
for i in range(M-1):
    D_accel[2*i:2*i+2, 2*i:2*i+2] = -np.eye(2)
    D_accel[2*i:2*i+2, 2*i+2:2*i+4] = np.eye(2)
D_accel = D_accel.tocsc()

# 4. 加速度权重 (8×8)
R2_bar = sparse.block_diag([R2]*(M-2) + [R2 + Ra], format='csc')

# 5. 加速度Hessian (10×10)
R_accel = D_accel.T @ R2_bar @ D_accel

# 6. 完整Hessian (40×40)
P = sparse.block_diag([Q_bar, R_bar + R_accel], format='csc')

# 7. 梯度向量（全零）
q = np.zeros(40)
```

### 5.6 梯度向量

对于全变量形式，由于目标函数是纯二次型（无一次项），梯度向量为零：

$$
\boxed{\mathbf{q} = \mathbf{0}_{40 \times 1}}
$$

---

## 6. 不等式约束构造（详细）

### 6.1 约束设计原则

MPC的约束设计需要考虑：

1. **物理限制**：电机最大速度、最大加速度
2. **安全边界**：避免状态超出安全范围
3. **舒适性**：加速度不能太大
4. **硬件能力**：执行器的实际能力

### 6.2 约束类型汇总

| 约束类型 | 数量 | 符号 | 说明 | 是否必需 |
|---------|------|------|------|---------|
| 状态边界 | 30 | $\mathbf{X}_{e,\min} \leq \mathbf{X}_e \leq \mathbf{X}_{e,\max}$ | 通常设为无穷 | 可选 |
| 控制边界 | 10 | $\mathbf{U}_{\min} \leq \Delta\mathbf{u} \leq \mathbf{U}_{\max}$ | M=5步速度限制 | **必需** |
| 加速度约束 | 8 | $\boldsymbol{\Delta}_{\min} \leq \delta\mathbf{u} \leq \boldsymbol{\Delta}_{\max}$ | M-1=4步加速度限制 | **必需** |

**总约束数**：30 + 10 + 8 = 48个不等式约束

### 6.3 状态边界约束（详细）

#### 6.3.1 约束定义

对于预测时域内的每一步 $i=1,2,\ldots,10$：

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

#### 6.3.2 矩阵形式

在全变量形式中，状态变量占据前30个元素：

$$
\mathbf{A}_{state} = \begin{bmatrix}
\mathbf{I}_{30} & \mathbf{0}_{30 \times 10}
\end{bmatrix}_{30 \times 40}
$$

$$
\mathbf{l}_{state} \leq \mathbf{A}_{state} \boldsymbol{\xi} \leq \mathbf{u}_{state}
$$

#### 6.3.3 推荐设置（不约束状态）

$$
\mathbf{l}_{state} = \begin{bmatrix}
-\infty \\ -\infty \\ -\pi \\
\vdots
\end{bmatrix}_{30 \times 1}, \quad
\mathbf{u}_{state} = \begin{bmatrix}
+\infty \\ +\infty \\ +\pi \\
\vdots
\end{bmatrix}_{30 \times 1}
$$

```python
l_state = np.tile([-np.inf, -np.inf, -np.pi], N)
u_state = np.tile([np.inf, np.inf, np.pi], N)
```

### 6.4 控制边界约束（详细）

#### 6.4.1 约束定义

对于前M=5步控制 $i=0,1,\ldots,4$：

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

**关键**：决策变量是速度**误差** $\Delta\mathbf{u}$，需要转换：

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

#### 6.4.2 矩阵形式

控制变量占据后10个元素（索引30-39）：

$$
\mathbf{A}_{control} = \begin{bmatrix}
\mathbf{0}_{10 \times 30} & \mathbf{I}_{10}
\end{bmatrix}_{10 \times 40}
$$

$$
\mathbf{l}_{control} \leq \mathbf{A}_{control} \boldsymbol{\xi} \leq \mathbf{u}_{control}
$$

其中：

$$
\mathbf{l}_{control} = \begin{bmatrix}
v_{\min} - v_r \\ \omega_{\min} - \omega_r \\
\vdots
\end{bmatrix}_{10 \times 1}, \quad
\mathbf{u}_{control} = \begin{bmatrix}
v_{\max} - v_r \\ \omega_{\max} - \omega_r \\
\vdots
\end{bmatrix}_{10 \times 1}
$$

#### 6.4.3 推荐设置

典型限制：
- $v_{\min}, v_{\max} = [0, 2]$ m/s
- $\omega_{\min}, \omega_{\max} = [-1, 1]$ rad/s

```python
v_min, v_max = 0.0, 2.0
omega_min, omega_max = -1.0, 1.0
v_r, omega_r = 1.0, 0.1

l_control = np.tile([v_min - v_r, omega_min - omega_r], M)
u_control = np.tile([v_max - v_r, omega_max - omega_r], M)
```

### 6.5 加速度约束（详细）

#### 6.5.1 约束定义

对于相邻两步之间的控制变化率 $i=1,2,3,4$：

$$
\begin{bmatrix}
a_{\min} \\ \alpha_{\min}
\end{bmatrix}
\leq
\begin{bmatrix}
\Delta v(k+i) - \Delta v(k+i-1) \\
\Delta\omega(k+i) - \Delta\omega(k+i-1)
\end{bmatrix}
\leq
\begin{bmatrix}
a_{\max} \\ \alpha_{\max}
\end{bmatrix}
$$

**控制变化率数量**（M=5）：
- 控制序列有5个：$\Delta\mathbf{u}(k), \ldots, \Delta\mathbf{u}(k+4)$
- 变化率有4个：$\delta\mathbf{u}(k+1), \ldots, \delta\mathbf{u}(k+4)$
- 每个2维，共 8 个约束

#### 6.5.2 矩阵形式

差分矩阵 $\mathbf{D}_{accel} \in \mathbb{R}^{8 \times 10}$：

$$
\mathbf{D}_{accel} = \begin{bmatrix}
-\mathbf{I}_2 & \mathbf{I}_2 & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & -\mathbf{I}_2 & \mathbf{I}_2 & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & -\mathbf{I}_2 & \mathbf{I}_2 & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & -\mathbf{I}_2 & \mathbf{I}_2
\end{bmatrix}_{8 \times 10}
$$

$$
\mathbf{A}_{accel} = \begin{bmatrix}
\mathbf{0}_{8 \times 30} & \mathbf{D}_{accel}
\end{bmatrix}_{8 \times 40}
$$

$$
\mathbf{l}_{accel} \leq \mathbf{A}_{accel} \boldsymbol{\xi} \leq \mathbf{u}_{accel}
$$

#### 6.5.3 推荐设置

**注意**：边界需要乘以采样时间T，因为约束的是每采样周期的速度变化：

$$
\delta v \leq T \cdot a_{\max}
$$

典型限制：
- $a_{\min}, a_{\max} = [-2, 2]$ m/s²
- $\alpha_{\min}, \alpha_{\max} = [-1, 1]$ rad/s²
- $T = 0.1$ s

边界向量：

$$
\mathbf{l}_{accel} = \begin{bmatrix}
T \cdot a_{\min} \\ T \cdot \alpha_{\min} \\
\vdots
\end{bmatrix}_{8 \times 1} = \begin{bmatrix}
-0.2 \\ -0.1 \\
\vdots
\end{bmatrix}, \quad
\mathbf{u}_{accel} = \begin{bmatrix}
T \cdot a_{\max} \\ T \cdot \alpha_{\max} \\
\vdots
\end{bmatrix}_{8 \times 1} = \begin{bmatrix}
0.2 \\ 0.1 \\
\vdots
\end{bmatrix}
$$

```python
T = 0.1
a_min, a_max = -2.0, 2.0
alpha_min, alpha_max = -1.0, 1.0

l_accel = np.tile([T*a_min, T*alpha_min], M-1)  # [-0.2, -0.1, -0.2, -0.1, ...]
u_accel = np.tile([T*a_max, T*alpha_max], M-1)  # [0.2, 0.1, 0.2, 0.1, ...]
```

### 6.6 约束汇总

所有不等式约束堆叠：

$$
\mathbf{A}_{ineq} = \begin{bmatrix}
\mathbf{A}_{state} \\
\mathbf{A}_{control} \\
\mathbf{A}_{accel}
\end{bmatrix}_{48 \times 40}, \quad
\mathbf{l}_{ineq} = \begin{bmatrix}
\mathbf{l}_{state} \\
\mathbf{l}_{control} \\
\mathbf{l}_{accel}
\end{bmatrix}_{48 \times 1}, \quad
\mathbf{u}_{ineq} = \begin{bmatrix}
\mathbf{u}_{state} \\
\mathbf{u}_{control} \\
\mathbf{u}_{accel}
\end{bmatrix}_{48 \times 1}
$$

**维度验证**：
- $\mathbf{A}_{state}$：30×40
- $\mathbf{A}_{control}$：10×40
- $\mathbf{A}_{accel}$：8×40
- 堆叠后：(30+10+8)×40 = 48×40 ✓

### 6.7 Python完整实现

```python
import numpy as np
from scipy import sparse

def build_inequality_constraints(N, M, v_r, omega_r, 
                                  v_min, v_max, omega_min, omega_max,
                                  a_min, a_max, alpha_min, alpha_max, T=0.1):
    """
    构造完整的不等式约束
    
    返回:
        A_ineq: 约束矩阵 (48×40)
        l_ineq, u_ineq: 约束边界 (48×1)
    """
    nx, nu = 3, 2
    
    # 1. 状态边界约束 (30×40)
    A_state = sparse.hstack([
        sparse.eye(N*nx),
        sparse.csc_matrix((N*nx, M*nu))
    ], format='csc')
    
    l_state = np.tile([-np.inf, -np.inf, -np.pi], N)
    u_state = np.tile([np.inf, np.inf, np.pi], N)
    
    # 2. 控制边界约束 (10×40)
    A_control = sparse.hstack([
        sparse.csc_matrix((M*nu, N*nx)),
        sparse.eye(M*nu)
    ], format='csc')
    
    l_control = np.tile([v_min - v_r, omega_min - omega_r], M)
    u_control = np.tile([v_max - v_r, omega_max - omega_r], M)
    
    # 3. 加速度约束 (8×40)
    D_accel = sparse.lil_matrix(((M-1)*nu, M*nu))
    for i in range(M-1):
        D_accel[nu*i:nu*i+nu, nu*i:nu*i+nu] = -np.eye(nu)
        D_accel[nu*i:nu*i+nu, nu*i+nu:nu*i+2*nu] = np.eye(nu)
    D_accel = D_accel.tocsc()
    
    A_accel = sparse.hstack([
        sparse.csc_matrix(((M-1)*nu, N*nx)),
        D_accel
    ], format='csc')
    
    l_accel = np.tile([T*a_min, T*alpha_min], M-1)
    u_accel = np.tile([T*a_max, T*alpha_max], M-1)
    
    # 4. 堆叠所有约束
    A_ineq = sparse.vstack([A_state, A_control, A_accel], format='csc')
    l_ineq = np.concatenate([l_state, l_control, l_accel])
    u_ineq = np.concatenate([u_state, u_control, u_accel])
    
    return A_ineq, l_ineq, u_ineq

# 使用示例
N, M = 10, 5
v_r, omega_r = 1.0, 0.1
v_min, v_max = 0.0, 2.0
omega_min, omega_max = -1.0, 1.0
a_min, a_max = -2.0, 2.0
alpha_min, alpha_max = -1.0, 1.0

A_ineq, l_ineq, u_ineq = build_inequality_constraints(
    N, M, v_r, omega_r, v_min, v_max, omega_min, omega_max,
    a_min, a_max, alpha_min, alpha_max
)
```

---

## 7. 完整QP问题形式

### 7.1 标准OSQP形式

$$
\boxed{
\begin{align}
\min_{\boldsymbol{\xi}} \quad & \frac{1}{2} \boldsymbol{\xi}^\top \mathbf{P} \boldsymbol{\xi} \\
\text{s.t.} \quad & \mathbf{l}_{all} \leq \mathbf{A}_{all} \boldsymbol{\xi} \leq \mathbf{u}_{all}
\end{align}
}
$$

### 7.2 所有约束堆叠

$$
\mathbf{A}_{all} = \begin{bmatrix}
\mathbf{A}_{eq} \\
\mathbf{A}_{ineq}
\end{bmatrix}_{68 \times 40}
$$

$$
\mathbf{l}_{all} = \begin{bmatrix}
\mathbf{b}_{eq} \\
\mathbf{l}_{ineq}
\end{bmatrix}_{68 \times 1}, \quad
\mathbf{u}_{all} = \begin{bmatrix}
\mathbf{b}_{eq} \\
\mathbf{u}_{ineq}
\end{bmatrix}_{68 \times 1}
$$

**注意**：等式约束通过设置 $\mathbf{l}_{eq} = \mathbf{u}_{eq} = \mathbf{b}_{eq}$ 实现。

### 7.3 问题规模汇总

| 项目 | 符号 | 维度 | 说明 |
|------|------|------|------|
| 决策变量 | $\boldsymbol{\xi}$ | 40 | 30状态+10控制 |
| Hessian矩阵 | $\mathbf{P}$ | $40 \times 40$ | 块对角稀疏 |
| 梯度向量 | $\mathbf{q}$ | 40 | 全零 |
| 约束矩阵 | $\mathbf{A}_{all}$ | $68 \times 40$ | 稀疏 |
| 等式约束 | $\mathbf{A}_{eq}$ | $20 \times 40$ | 动力学+控制固定 |
| 不等式约束 | $\mathbf{A}_{ineq}$ | $48 \times 40$ | 状态+控制+加速度 |

### 7.4 求解与控制

OSQP求解得到 $\boldsymbol{\xi}^*$，提取第一个控制量：

$$
\Delta\mathbf{u}^*(k) = \boldsymbol{\xi}^*[30:32] = \begin{bmatrix} \Delta v^*(k) \\ \Delta\omega^*(k) \end{bmatrix}
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

### 7.5 滚动时域控制流程

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
| $M$ | 5 | 控制步数 |
| $v_r$ | 1.0 m/s | 参考线速度 |
| $\omega_r$ | 0.1 rad/s | 参考角速度 |
| $v_{\min}, v_{\max}$ | [0, 2] m/s | 速度范围 |
| $\omega_{\min}, \omega_{\max}$ | [-1, 1] rad/s | 角速度范围 |
| $a_{\min}, a_{\max}$ | [-2, 2] m/s² | 线加速度范围 |
| $\alpha_{\min}, \alpha_{\max}$ | [-1, 1] rad/s² | 角加速度范围 |

### 8.2 权重矩阵

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

### 8.3 离散化矩阵计算

假设 $v_r=1.0$ m/s, $\omega_r=0.1$ rad/s, $T=0.1$ s：

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

### 8.4 初始状态示例

假设当前误差状态：

$$
\mathbf{X}_e(k) = \begin{bmatrix} 0.5 \\ 0.2 \\ 0.1 \end{bmatrix} \text{ (m, m, rad)}
$$

则等式约束右端项前3个元素：

$$
\mathbf{b}_{eq}[0:3] = -\mathbf{A}_d \mathbf{X}_e(k) = \begin{bmatrix}
-0.502 \\
-0.195 \\
-0.1
\end{bmatrix}
$$

### 8.5 矩阵维度汇总

| 矩阵/向量 | 维度 | 说明 |
|----------|------|------|
| $\boldsymbol{\xi}$ | $40 \times 1$ | 决策变量 |
| $\mathbf{P}$ | $40 \times 40$ | Hessian（块对角） |
| $\mathbf{q}$ | $40 \times 1$ | 梯度（全零） |
| $\mathbf{A}_{eq}$ | $20 \times 40$ | 等式约束矩阵 |
| $\mathbf{b}_{eq}$ | $20 \times 1$ | 等式约束右端项 |
| $\mathbf{A}_{ineq}$ | $48 \times 40$ | 不等式约束矩阵 |
| $\mathbf{A}_{all}$ | $68 \times 40$ | 所有约束堆叠 |

---

## 9. 参数调优指南

### 9.1 权重矩阵调优

#### Q1：位姿误差权重

| 现象 | 原因 | 调整 |
|------|------|------|
| 跟踪误差大 | Q1太小 | 增大Q1 |
| 控制激进、抖动 | Q1太大 | 减小Q1或增大R |
| 横向偏移严重 | $q_y$太小 | 增大横向权重 |

**推荐**：$\mathbf{Q}_1 = \text{diag}(100, 100, 10)$

#### R1：速度误差权重

| 现象 | 原因 | 调整 |
|------|------|------|
| 控制抖动 | R1太小 | 增大R1 |
| 跟踪迟钝 | R1太大 | 减小R1 |

**推荐**：$\mathbf{R}_1 = \text{diag}(0.1, 0.1)$

#### R2：加速度权重

| 现象 | 原因 | 调整 |
|------|------|------|
| 加速度突变 | R2太小 | 增大R2 |
| 响应过慢 | R2太大 | 减小R2 |

**推荐**：$\mathbf{R}_2 = \text{diag}(1, 1)$

#### 终点权重

| 权重 | 倍数 | 说明 |
|------|------|------|
| $\mathbf{Q}_f$ | $1000 \times \mathbf{Q}_1$ | 强制终点位姿收敛 |
| $\mathbf{R}_f$ | $100 \times \mathbf{R}_1$ | 强制终点速度为0 |
| $\mathbf{R}_a$ | $10 \times \mathbf{R}_2$ | 防止终点抖动 |

### 9.2 时域参数选择

#### 预测时域 N

| N | 优点 | 缺点 | 场景 |
|---|------|------|------|
| 5 | 计算快 | 预见性差 | 简单直线，低速 |
| **10** | **平衡** | **-** | **推荐** |
| 20 | 预见性好 | 计算慢 | 复杂轨迹，高速 |

**经验公式**：

$$
N \approx \frac{\text{期望预见距离}}{v_r \cdot T}
$$

#### 控制时域 M

| M/N 比例 | 决策变量 | 求解速度 | 稳定性 | 适用场景 |
|---------|---------|---------|--------|---------|
| **M=N/2** | 基准 | 基准 | 高 | **推荐（本文默认）** |
| M=N/3 | 减少13% | 提升10% | 很高 | 嵌入式系统 |
| M=N | 增加25% | 降低30% | 中等 | 精度优先 |

**默认推荐**：N=10, M=5

### 9.3 约束设置

**速度约束**：
```python
v_min, v_max = 0.0, 2.0        # m/s
omega_min, omega_max = -1.0, 1.0  # rad/s (±57°/s)
```

**加速度约束**：
```python
a_min, a_max = -2.0, 2.0       # m/s²
alpha_min, alpha_max = -1.0, 1.0    # rad/s²
```

**调整原则**：
- 约束越紧，安全性越高，但灵活性越差
- 约束应基于实际硬件能力（电机极限）
- 保守设定，留有安全余量

### 9.4 调试技巧

1. **检查约束矩阵维度**
   ```python
   assert P.shape == (40, 40)
   assert A_all.shape == (68, 40)
   assert b_eq.shape[0] == 20
   ```

2. **验证等式约束**
   ```python
   residual = A_eq @ solution - b_eq
   print(f"动力学残差: {np.linalg.norm(residual):.6f}")
   # 应 < 1e-5
   ```

3. **监控求解时间**
   ```python
   import time
   t0 = time.time()
   result = prob.solve()
   dt = time.time() - t0
   print(f"求解时间: {dt*1000:.2f} ms")
   # 应 < 100 ms
   ```

4. **检查可行性**
   ```python
   assert v_min <= v_r <= v_max, "参考速度超出限制！"
   assert (l_control <= u_control).all(), "约束上下界矛盾！"
   ```

### 9.5 常见问题排查

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 求解失败 | 约束冲突 | 检查上下界是否合理 |
| 动力学残差大 | 等式约束错误 | 验证A_eq和b_eq构造 |
| 速度慢 | 稠密矩阵 | 使用sparse格式 |
| 结果抖动 | 权重不当 | 增大R1或R2 |
| 到点不停 | 终点权重小 | 增大Qf和Rf |
| 轨迹振荡 | Q/R不平衡 | 调整权重比例 |

---

## 10. 附录：符号表

### 10.1 基本参数

| 符号 | 含义 | 默认值 |
|------|------|--------|
| $N$ | 预测时域步数 | 10 |
| $M$ | 控制时域步数 | 5 |
| $T$ | 采样时间 | 0.1 s |
| $n_x$ | 状态维度 | 3 |
| $n_u$ | 控制维度 | 2 |

### 10.2 状态和控制

| 符号 | 含义 | 维度 |
|------|------|------|
| $\mathbf{X}_e(k)$ | 第k步位姿误差 | $3 \times 1$ |
| $\Delta\mathbf{u}(k)$ | 第k步速度误差 | $2 \times 1$ |
| $\delta\mathbf{u}(k)$ | 第k步控制变化率（加速度） | $2 \times 1$ |
| $v_r, \omega_r$ | 参考线速度和角速度 | 标量 |

### 10.3 决策变量和矩阵

| 符号 | 含义 | 维度 |
|------|------|------|
| $\boldsymbol{\xi}$ | 决策变量 | $40 \times 1$ |
| $\boldsymbol{\Xi}$ | 状态序列 | $30 \times 1$ |
| $\mathbf{U}$ | 控制序列 | $10 \times 1$ |
| $\mathbf{A}_d, \mathbf{B}_d$ | 离散化矩阵 | $3 \times 3$, $3 \times 2$ |

### 10.4 QP问题矩阵

| 符号 | 含义 | 维度 |
|------|------|------|
| $\mathbf{P}$ | Hessian矩阵 | $40 \times 40$ |
| $\mathbf{q}$ | 梯度向量 | $40 \times 1$ |
| $\mathbf{A}_{eq}$ | 等式约束矩阵 | $20 \times 40$ |
| $\mathbf{b}_{eq}$ | 等式约束右端项 | $20 \times 1$ |
| $\mathbf{A}_{ineq}$ | 不等式约束矩阵 | $48 \times 40$ |
| $\mathbf{l}_{ineq}, \mathbf{u}_{ineq}$ | 不等式约束边界 | $48 \times 1$ |

### 10.5 权重矩阵

| 符号 | 含义 | 推荐值 |
|------|------|--------|
| $\mathbf{Q}_1$ | 位姿误差权重 | diag(100, 100, 10) |
| $\mathbf{R}_1$ | 速度误差权重 | diag(0.1, 0.1) |
| $\mathbf{R}_2$ | 加速度权重 | diag(1, 1) |
| $\mathbf{Q}_f$ | 终点位姿权重 | $1000 \times \mathbf{Q}_1$ |
| $\mathbf{R}_f$ | 终点速度权重 | $100 \times \mathbf{R}_1$ |
| $\mathbf{R}_a$ | 终点加速度权重 | $10 \times \mathbf{R}_2$ |

### 10.6 重要关系

$$
\begin{align}
\text{决策变量总数} &= N \times n_x + M \times n_u = 40 \\
\text{等式约束数} &= N \times n_x + (N-M) \times n_u = 20 \\
\text{不等式约束数} &= N \times n_x + M \times n_u + (M-1) \times n_u = 48 \\
\text{预测时间} &= N \times T = 1.0 \text{ s} \\
\text{控制时间} &= M \times T = 0.5 \text{ s}
\end{align}
$$

---

## 参考文献

1. **Model Predictive Control**: Camacho & Alba (2007)
2. **Nonlinear Model Predictive Control**: Grüne & Pannek (2017)
3. **Apollo MPC实现**: `modules/common/math/mpc_osqp.cc`
4. **OSQP Documentation**: https://osqp.org/
5. **差速驱动运动学**: Lynch & Park - Modern Robotics (2017)

---

**文档完成！✅**

本文档提供了基于 **M=5, N=10** 的双轮差速机器人MPC控制器的完整数学推导，包含：
- ✅ 全变量形式QP构造
- ✅ 6项目标函数详细推导
- ✅ Hessian矩阵完整构造
- ✅ 等式约束（动力学+控制固定）
- ✅ 不等式约束（状态+控制+加速度）
- ✅ Python实现代码
- ✅ 参数调优指南
