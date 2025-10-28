# 双轮差速机器人MPC控制器完整数学推导

> **从连续时间状态空间方程到QP求解的完整推导**

---

## 目录

1. [预备知识：线性化与离散化](#1-预备知识线性化与离散化)
2. [预测模型构建](#2-预测模型构建)
3. [目标函数设计（6项约束）](#3-目标函数设计6项约束)
4. [约束条件构建](#4-约束条件构建)
5. [QP标准形式](#5-qp标准形式)
6. [完整数值示例](#6-完整数值示例)
7. [求解算法](#7-求解算法)
8. [参数调优指南](#8-参数调优指南)
9. [设计决策说明](#9-设计决策说明)

---

## 核心公式速查

| 序号 | 目标函数项 | 数学表示 | 权重矩阵 |
|------|-----------|---------|---------|
| 1️⃣ | 位姿误差 | $\mathbf{X}_e^\top \mathbf{Q}_1 \mathbf{X}_e$ | $\mathbf{Q}_1 = \text{diag}(q_x, q_y, q_\theta)$ |
| 2️⃣ | 速度误差 | $\Delta\mathbf{u}^\top \mathbf{R}_1 \Delta\mathbf{u}$ | $\mathbf{R}_1 = \text{diag}(r_v, r_\omega)$ |
| 3️⃣ | 速度误差变化率（加速度平滑） | $\delta\mathbf{u}^\top \mathbf{R}_2 \delta\mathbf{u}$ | $\mathbf{R}_2 = \text{diag}(r_a, r_\alpha)$ |
| 4️⃣ | 终点位姿误差 | $\mathbf{X}_e(N)^\top \mathbf{Q}_f \mathbf{X}_e(N)$ | $\mathbf{Q}_f = 1000 \cdot \mathbf{Q}_1$ |
| 5️⃣ | 终点速度误差 | $\Delta\mathbf{u}(N)^\top \mathbf{R}_f \Delta\mathbf{u}(N)$ | $\mathbf{R}_f = 100 \cdot \mathbf{R}_1$ |
| 6️⃣ | 终点加速度（防止抖动） | $\delta\mathbf{u}(N)^\top \mathbf{R}_a \delta\mathbf{u}(N)$ | $\mathbf{R}_a = 10 \cdot \mathbf{R}_2$ |

> **设计理由**：
> - ❌ **去掉了位姿误差变化率项**：Apollo MPC实践证明该项冗余（权重通常设为0），且已被位姿误差和速度误差间接约束
> - ✅ **保留了终点加速度项**：显式约束终点静止状态，防止到点时抖动，确保完全平稳停止

---

## 1. 预备知识：线性化与离散化

### 1.1 连续时间状态方程（你已推导）

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
- $\dot{\mathbf{X}}_e$：**位姿误差变化率**（目标函数第2项需要）

### 1.2 欧拉离散化

采样时间为 $T$，使用前向欧拉法：

$$
\dot{\mathbf{X}}_e \approx \frac{\mathbf{X}_e(k+1) - \mathbf{X}_e(k)}{T}
$$

代入连续方程得：

$$
\mathbf{X}_e(k+1) = \mathbf{X}_e(k) + T(\mathbf{A}\mathbf{X}_e(k) + \mathbf{B}\Delta\mathbf{u}(k))
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

**注意**：$T$ 是采样时间（如 0.1s），欧拉法精度为 $O(T^2)$，$T$ 越小越准确。

---

## 2. 预测模型构建

### 2.1 递推展开

从当前时刻 $k$ 开始，预测未来 $N$ 步：

$$
\begin{align}
\mathbf{X}_e(k+1) &= \mathbf{A}_d \mathbf{X}_e(k) + \mathbf{B}_d \Delta\mathbf{u}(k) \\
\mathbf{X}_e(k+2) &= \mathbf{A}_d^2 \mathbf{X}_e(k) + \mathbf{A}_d\mathbf{B}_d \Delta\mathbf{u}(k) + \mathbf{B}_d \Delta\mathbf{u}(k+1) \\
\mathbf{X}_e(k+3) &= \mathbf{A}_d^3 \mathbf{X}_e(k) + \mathbf{A}_d^2\mathbf{B}_d \Delta\mathbf{u}(k) + \mathbf{A}_d\mathbf{B}_d \Delta\mathbf{u}(k+1) + \mathbf{B}_d \Delta\mathbf{u}(k+2) \\
&\vdots \\
\mathbf{X}_e(k+N) &= \mathbf{A}_d^N \mathbf{X}_e(k) + \sum_{i=0}^{N-1} \mathbf{A}_d^{N-1-i}\mathbf{B}_d \Delta\mathbf{u}(k+i)
\end{align}
$$

### 2.2 矩阵形式

定义预测向量（$n_x=3$，$n_u=2$）：

$$
\boldsymbol{\Xi} = \begin{bmatrix} 
\mathbf{X}_e(k+1) \\ 
\mathbf{X}_e(k+2) \\ 
\vdots \\ 
\mathbf{X}_e(k+N) 
\end{bmatrix}_{Nn_x \times 1}, \quad
\mathbf{U} = \begin{bmatrix} 
\Delta\mathbf{u}(k) \\ 
\Delta\mathbf{u}(k+1) \\ 
\vdots \\ 
\Delta\mathbf{u}(k+N-1) 
\end{bmatrix}_{Nn_u \times 1}
$$

构造预测矩阵：

$$
\boldsymbol{\Phi} = \begin{bmatrix} 
\mathbf{A}_d \\ 
\mathbf{A}_d^2 \\ 
\mathbf{A}_d^3 \\ 
\vdots \\ 
\mathbf{A}_d^N 
\end{bmatrix}_{Nn_x \times n_x}
$$

$$
\boldsymbol{\Theta} = \begin{bmatrix}
\mathbf{B}_d & \mathbf{0} & \mathbf{0} & \cdots & \mathbf{0} \\
\mathbf{A}_d\mathbf{B}_d & \mathbf{B}_d & \mathbf{0} & \cdots & \mathbf{0} \\
\mathbf{A}_d^2\mathbf{B}_d & \mathbf{A}_d\mathbf{B}_d & \mathbf{B}_d & \cdots & \mathbf{0} \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
\mathbf{A}_d^{N-1}\mathbf{B}_d & \mathbf{A}_d^{N-2}\mathbf{B}_d & \mathbf{A}_d^{N-3}\mathbf{B}_d & \cdots & \mathbf{B}_d
\end{bmatrix}_{Nn_x \times Nn_u}
$$

**预测方程**：

$$
\boxed{\boldsymbol{\Xi} = \boldsymbol{\Phi} \mathbf{X}_e(k) + \boldsymbol{\Theta} \mathbf{U}}
$$

### 2.3 速度误差变化率（加速度）

定义**控制变化率向量**：

$$
\boldsymbol{\Delta} = \begin{bmatrix}
\delta\mathbf{u}(k+1) \\
\delta\mathbf{u}(k+2) \\
\vdots \\
\delta\mathbf{u}(k+N-1)
\end{bmatrix}_{(N-1)n_u \times 1}, \quad
\delta\mathbf{u}(k+i) = \Delta\mathbf{u}(k+i) - \Delta\mathbf{u}(k+i-1)
$$

构造**差分矩阵** $\mathbf{D} \in \mathbb{R}^{(N-1)n_u \times Nn_u}$：

$$
\mathbf{D} = \begin{bmatrix}
-\mathbf{I} & \mathbf{I} & \mathbf{0} & \cdots & \mathbf{0} \\
\mathbf{0} & -\mathbf{I} & \mathbf{I} & \cdots & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & -\mathbf{I} & \cdots & \mathbf{0} \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \cdots & \mathbf{I}
\end{bmatrix}_{(N-1) \times N \text{ blocks}}
$$

其中 $\mathbf{I} \in \mathbb{R}^{n_u \times n_u}$ 是单位矩阵。

则有：

$$
\boxed{\boldsymbol{\Delta} = \mathbf{D} \mathbf{U}}
$$


---

## 3. 目标函数设计（6项约束）

### 3.1 完整目标函数

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

**权重矩阵定义**：

| 权重 | 维度 | 物理意义 | 推荐值 | 说明 |
|------|------|---------|--------|------|
| $\mathbf{Q}_1$ | $3 \times 3$ | 位姿误差惩罚 | $\text{diag}(100, 100, 10)$ | 核心项，确保跟踪精度 |
| $\mathbf{R}_1$ | $2 \times 2$ | 速度误差惩罚 | $\text{diag}(0.1, 0.1)$ | 控制平滑，参考Apollo |
| $\mathbf{R}_2$ | $2 \times 2$ | 加速度惩罚 | $\text{diag}(1, 1)$ | 避免速度突变 |
| $\mathbf{Q}_f$ | $3 \times 3$ | 终点位姿（强调） | $1000 \cdot \mathbf{Q}_1$ | 精确到达目标 |
| $\mathbf{R}_f$ | $2 \times 2$ | 终点速度（强调） | $100 \cdot \mathbf{R}_1$ | 确保停止 |
| $\mathbf{R}_a$ | $2 \times 2$ | 终点加速度（强调） | $10 \cdot \mathbf{R}_2$ | 防止到点抖动⭐ |

> **关键决策**：
> - ❌ **去掉位姿误差变化率** $\dot{\mathbf{X}}_e^\top \mathbf{Q}_2 \dot{\mathbf{X}}_e$：Apollo实践验证该项冗余（通常权重=0），已被第1、2项间接约束
> - ✅ **保留终点加速度**：虽然理论上第3+5项能保证，但显式约束更可靠，防止数值误差导致抖动
> - ⭐ **终点加速度权重设为10倍**（而非100倍）：因为第3项已全程平滑，第6项只是额外强调，避免过度约束

### 3.2 矩阵形式展开

#### 3.2.1 第1项：位姿误差

$$
J_1 = \sum_{i=1}^{N} \mathbf{X}_e^\top(k+i) \mathbf{Q}_1 \mathbf{X}_e(k+i) = \boldsymbol{\Xi}^\top \bar{\mathbf{Q}}_1 \boldsymbol{\Xi}
$$

其中：

$$
\bar{\mathbf{Q}}_1 = \begin{bmatrix}
\mathbf{Q}_1 & & & \\
& \mathbf{Q}_1 & & \\
& & \ddots & \\
& & & \mathbf{Q}_1
\end{bmatrix}_{Nn_x \times Nn_x}
$$

代入 $\boldsymbol{\Xi} = \boldsymbol{\Phi} \mathbf{X}_e(k) + \boldsymbol{\Theta} \mathbf{U}$：

$$
J_1 = (\boldsymbol{\Phi} \mathbf{X}_e(k) + \boldsymbol{\Theta} \mathbf{U})^\top \bar{\mathbf{Q}}_1 (\boldsymbol{\Phi} \mathbf{X}_e(k) + \boldsymbol{\Theta} \mathbf{U})
$$

展开：

$$
J_1 = \mathbf{U}^\top (\boldsymbol{\Theta}^\top \bar{\mathbf{Q}}_1 \boldsymbol{\Theta}) \mathbf{U} + 2\mathbf{X}_e^\top(k) \boldsymbol{\Phi}^\top \bar{\mathbf{Q}}_1 \boldsymbol{\Theta} \mathbf{U} + \text{const}
$$

#### 3.2.2 第2项：速度误差

$$
J_2 = \sum_{i=0}^{N-1} \Delta\mathbf{u}^\top(k+i) \mathbf{R}_1 \Delta\mathbf{u}(k+i) = \mathbf{U}^\top \bar{\mathbf{R}}_1 \mathbf{U}
$$

其中：

$$
\bar{\mathbf{R}}_1 = \begin{bmatrix}
\mathbf{R}_1 & & & \\
& \mathbf{R}_1 & & \\
& & \ddots & \\
& & & \mathbf{R}_1
\end{bmatrix}_{Nn_u \times Nn_u}
$$

#### 3.2.3 第3项：速度误差变化率（加速度平滑）

$$
J_3 = \sum_{i=1}^{N-1} \delta\mathbf{u}^\top(k+i) \mathbf{R}_2 \delta\mathbf{u}(k+i) = \boldsymbol{\Delta}^\top \bar{\mathbf{R}}_2 \boldsymbol{\Delta}
$$

其中 $\bar{\mathbf{R}}_2 = \text{diag}(\mathbf{R}_2, \mathbf{R}_2, \ldots, \mathbf{R}_2)_{(N-1)n_u \times (N-1)n_u}$。

代入 $\boldsymbol{\Delta} = \mathbf{D}\mathbf{U}$：

$$
J_3 = \mathbf{U}^\top (\mathbf{D}^\top \bar{\mathbf{R}}_2 \mathbf{D}) \mathbf{U}
$$

#### 3.2.4 第4项：终点位姿误差

为了处理终点，修改 $\bar{\mathbf{Q}}_1$ 的最后一个块对角元素：

$$
\bar{\mathbf{Q}}_1^{\text{aug}} = \begin{bmatrix}
\mathbf{Q}_1 & & & \\
& \mathbf{Q}_1 & & \\
& & \ddots & \\
& & & \mathbf{Q}_1 + \mathbf{Q}_f
\end{bmatrix}
$$

这样第1项和第4项可以合并。

#### 3.2.5 第5项：终点速度误差

修改 $\bar{\mathbf{R}}_1$ 的最后一个块对角元素：

$$
\bar{\mathbf{R}}_1^{\text{aug}} = \begin{bmatrix}
\mathbf{R}_1 & & & \\
& \mathbf{R}_1 & & \\
& & \ddots & \\
& & & \mathbf{R}_1 + \mathbf{R}_f
\end{bmatrix}
$$

#### 3.2.6 第6项：终点加速度（防止抖动）

修改 $\bar{\mathbf{R}}_2$ 的最后一个块对角元素：

$$
\bar{\mathbf{R}}_2^{\text{aug}} = \begin{bmatrix}
\mathbf{R}_2 & & & \\
& \mathbf{R}_2 & & \\
& & \ddots & \\
& & & \mathbf{R}_2 + \mathbf{R}_a
\end{bmatrix}
$$

**说明**：虽然第3项（全程加速度平滑）+ 第5项（终点速度为0）理论上能保证终点加速度接近0，但显式约束第6项可以：
- 补偿数值误差（OSQP精度限制）
- 防止到点时的小幅抖动
- 确保机械系统完全静止
- 权重设为 $\mathbf{R}_a = 10 \cdot \mathbf{R}_2$（适中，不过度约束）

### 3.3 标准二次型形式

综合以上6项，去掉常数项，得到关于 $\mathbf{U}$ 的标准QP形式：

$$
J = \frac{1}{2}\mathbf{U}^\top \mathbf{H} \mathbf{U} + \mathbf{f}^\top \mathbf{U}
$$

其中：

$$
\boxed{
\begin{align}
\mathbf{H} = 2 \Big( & \boldsymbol{\Theta}^\top \bar{\mathbf{Q}}_1^{\text{aug}} \boldsymbol{\Theta} 
    && \text{（位姿误差+终点位姿）} \\
& + \bar{\mathbf{R}}_1^{\text{aug}} 
    && \text{（速度误差+终点速度）} \\
& + \mathbf{D}^\top \bar{\mathbf{R}}_2^{\text{aug}} \mathbf{D} 
    && \text{（加速度+终点加速度）} \Big)
\end{align}
}
$$

$$
\boxed{
\mathbf{f} = 2 \boldsymbol{\Theta}^\top \bar{\mathbf{Q}}_1^{\text{aug}} \boldsymbol{\Phi} \mathbf{X}_e(k)
}
$$

**注意因子2**：因为标准QP形式是 $\frac{1}{2}\mathbf{U}^\top \mathbf{H} \mathbf{U}$，所以 $\mathbf{H}$ 和 $\mathbf{f}$ 都需要乘以2。

**简化说明**：相比7项设计，我们去掉了位姿误差变化率项 $\boldsymbol{\Theta}_v^\top \bar{\mathbf{Q}}_2 \boldsymbol{\Theta}_v$，因为：
1. Apollo MPC实践证明该项冗余（通常权重设为0）
2. 已被位姿误差和速度误差间接约束
3. 去掉后Hessian矩阵更稀疏，求解更快

---

## 4. 约束条件构建

### 4.1 控制量约束（速度限制）

$$
\begin{bmatrix}
v_{\min} - v_r \\
\omega_{\min} - \omega_r
\end{bmatrix} \leq \begin{bmatrix}
\Delta v(k+i) \\
\Delta\omega(k+i)
\end{bmatrix} \leq \begin{bmatrix}
v_{\max} - v_r \\
\omega_{\max} - \omega_r
\end{bmatrix}, \quad i=0,\ldots,N-1
$$

写成向量形式：

$$
\boxed{\mathbf{U}_{\min} \leq \mathbf{U} \leq \mathbf{U}_{\max}}
$$

其中：

$$
\mathbf{U}_{\min} = \begin{bmatrix}
v_{\min} - v_r \\ \omega_{\min} - \omega_r \\
v_{\min} - v_r \\ \omega_{\min} - \omega_r \\
\vdots
\end{bmatrix}_{Nn_u \times 1}, \quad
\mathbf{U}_{\max} = \begin{bmatrix}
v_{\max} - v_r \\ \omega_{\max} - \omega_r \\
v_{\max} - v_r \\ \omega_{\max} - \omega_r \\
\vdots
\end{bmatrix}_{Nn_u \times 1}
$$

### 4.2 加速度约束（速度变化率限制）

$$
\begin{bmatrix}
a_{\min} \\
\alpha_{\min}
\end{bmatrix} \leq \begin{bmatrix}
\delta v(k+i) \\
\delta\omega(k+i)
\end{bmatrix} \leq \begin{bmatrix}
a_{\max} \\
\alpha_{\max}
\end{bmatrix}, \quad i=1,\ldots,N-1
$$

利用 $\boldsymbol{\Delta} = \mathbf{D}\mathbf{U}$：

$$
\boxed{\boldsymbol{\Delta}_{\min} \leq \mathbf{D}\mathbf{U} \leq \boldsymbol{\Delta}_{\max}}
$$

其中：

$$
\boldsymbol{\Delta}_{\min} = \begin{bmatrix}
a_{\min} \\ \alpha_{\min} \\
a_{\min} \\ \alpha_{\min} \\
\vdots
\end{bmatrix}_{(N-1)n_u \times 1}, \quad
\boldsymbol{\Delta}_{\max} = \begin{bmatrix}
a_{\max} \\ \alpha_{\max} \\
a_{\max} \\ \alpha_{\max} \\
\vdots
\end{bmatrix}_{(N-1)n_u \times 1}
$$

### 4.3 状态约束（可选）

如果需要限制位姿误差：

$$
\mathbf{X}_{e,\min} \leq \mathbf{X}_e(k+i) \leq \mathbf{X}_{e,\max}, \quad i=1,\ldots,N
$$

转换为关于 $\mathbf{U}$ 的约束：

$$
\boxed{\boldsymbol{\Xi}_{\min} - \boldsymbol{\Phi}\mathbf{X}_e(k) \leq \boldsymbol{\Theta}\mathbf{U} \leq \boldsymbol{\Xi}_{\max} - \boldsymbol{\Phi}\mathbf{X}_e(k)}
$$

通常设置：

$$
\mathbf{X}_{e,\min} = \begin{bmatrix} -\infty \\ -\infty \\ -\pi \end{bmatrix}, \quad
\mathbf{X}_{e,\max} = \begin{bmatrix} +\infty \\ +\infty \\ +\pi \end{bmatrix}
$$

---

## 5. QP标准形式

### 5.1 完整QP问题

$$
\boxed{
\begin{align}
\min_{\mathbf{U}} \quad & \frac{1}{2} \mathbf{U}^\top \mathbf{H} \mathbf{U} + \mathbf{f}^\top \mathbf{U} \\
\text{s.t.} \quad & \mathbf{U}_{\min} \leq \mathbf{U} \leq \mathbf{U}_{\max} \quad \text{（速度约束）} \\
& \boldsymbol{\Delta}_{\min} \leq \mathbf{D}\mathbf{U} \leq \boldsymbol{\Delta}_{\max} \quad \text{（加速度约束）} \\
& \boldsymbol{\Xi}_{\min} - \boldsymbol{\Phi}\mathbf{X}_e(k) \leq \boldsymbol{\Theta}\mathbf{U} \leq \boldsymbol{\Xi}_{\max} - \boldsymbol{\Phi}\mathbf{X}_e(k) \quad \text{（状态约束）}
\end{align}
}
$$

### 5.2 OSQP求解器格式

OSQP标准形式：

$$
\min_{\mathbf{x}} \quad \frac{1}{2} \mathbf{x}^\top \mathbf{P} \mathbf{x} + \mathbf{q}^\top \mathbf{x} \quad \text{s.t.} \quad \mathbf{l} \leq \mathbf{A}\mathbf{x} \leq \mathbf{u}
$$

**映射关系**：

| OSQP | MPC | 维度 |
|------|-----|------|
| $\mathbf{x}$ | $\mathbf{U}$ | $Nn_u \times 1$ |
| $\mathbf{P}$ | $\mathbf{H}$ | $Nn_u \times Nn_u$ |
| $\mathbf{q}$ | $\mathbf{f}$ | $Nn_u \times 1$ |

**约束矩阵**：

$$
\mathbf{A} = \begin{bmatrix}
\mathbf{I}_{Nn_u} \\
\mathbf{D} \\
\boldsymbol{\Theta}
\end{bmatrix}, \quad
\mathbf{l} = \begin{bmatrix}
\mathbf{U}_{\min} \\
\boldsymbol{\Delta}_{\min} \\
\boldsymbol{\Xi}_{\min} - \boldsymbol{\Phi}\mathbf{X}_e(k)
\end{bmatrix}, \quad
\mathbf{u} = \begin{bmatrix}
\mathbf{U}_{\max} \\
\boldsymbol{\Delta}_{\max} \\
\boldsymbol{\Xi}_{\max} - \boldsymbol{\Phi}\mathbf{X}_e(k)
\end{bmatrix}
$$

### 5.3 求解与控制

OSQP求解得到：

$$
\mathbf{U}^* = \begin{bmatrix}
\Delta\mathbf{u}^*(k) \\
\Delta\mathbf{u}^*(k+1) \\
\vdots \\
\Delta\mathbf{u}^*(k+N-1)
\end{bmatrix}
$$

**MPC滚动时域策略**：只应用第一个控制量

$$
\Delta\mathbf{u}^*(k) = \begin{bmatrix} \Delta v^*(k) \\ \Delta\omega^*(k) \end{bmatrix}
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

---

## 6. 完整数值示例

### 6.1 参数设置

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

### 6.2 权重矩阵

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

### 6.3 离散化矩阵计算

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

### 6.4 初始状态示例

假设当前误差状态：

$$
\mathbf{X}_e(k) = \begin{bmatrix} 0.5 \\ 0.2 \\ 0.1 \end{bmatrix} \text{ (m, m, rad)}
$$

### 6.5 矩阵维度汇总（N=10）

| 矩阵 | 维度 | 说明 |
|------|------|------|
| $\mathbf{U}$ | $20 \times 1$ | 10步×2控制 |
| $\boldsymbol{\Xi}$ | $30 \times 1$ | 10步×3状态 |
| $\boldsymbol{\Delta}$ | $18 \times 1$ | 9步×2加速度 |
| $\boldsymbol{\Phi}$ | $30 \times 3$ | 预测矩阵 |
| $\boldsymbol{\Theta}$ | $30 \times 20$ | 控制影响矩阵 |
| $\mathbf{D}$ | $18 \times 20$ | 差分矩阵 |
| $\mathbf{H}$ | $20 \times 20$ | Hessian矩阵 |
| $\mathbf{f}$ | $20 \times 1$ | 梯度向量 |

---

## 7. 求解算法

### 7.1 Python实现框架

```python
import numpy as np
import osqp
from scipy import sparse

class DifferentialDriveMPC:
    def __init__(self, N=10, T=0.1):
        self.N = N
        self.T = T
        self.nx = 3  # 状态维度
        self.nu = 2  # 控制维度
        
        # 权重矩阵（基于决策：去掉位姿变化率，保留终点加速度）
        self.Q1 = np.diag([100, 100, 10])    # 位姿误差
        self.R1 = np.diag([0.1, 0.1])        # 速度误差
        self.R2 = np.diag([1, 1])            # 加速度平滑
        self.Qf = 1000 * self.Q1             # 终点位姿（强）
        self.Rf = 100 * self.R1              # 终点速度（强）
        self.Ra = 10 * self.R2               # 终点加速度（适中，防止抖动）
        
    def discretize(self, v_r, omega_r):
        """离散化"""
        T = self.T
        # 连续矩阵
        A = np.array([
            [0, omega_r, 0],
            [-omega_r, 0, v_r],
            [0, 0, 0]
        ])
        B = np.array([
            [-1, 0],
            [0, 0],
            [0, -1]
        ])
        # 离散化
        Ad = np.eye(3) + T * A
        Bd = T * B
        return Ad, Bd, A, B
    
    def build_prediction_matrices(self, Ad, Bd):
        """构造预测矩阵 Φ 和 Θ"""
        N, nx, nu = self.N, self.nx, self.nu
        
        # Φ矩阵 (Nnx × nx)
        Phi = np.zeros((N*nx, nx))
        Ak = Ad.copy()
        for i in range(N):
            Phi[i*nx:(i+1)*nx, :] = Ak
            Ak = Ak @ Ad
        
        # Θ矩阵 (Nnx × Nnu)
        Theta = np.zeros((N*nx, N*nu))
        for i in range(N):
            for j in range(i+1):
                if i == j:
                    Theta[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = Bd
                else:
                    Theta[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = \
                        np.linalg.matrix_power(Ad, i-j) @ Bd
        
        return Phi, Theta
    
    def build_difference_matrix(self):
        """构造差分矩阵 D"""
        N, nu = self.N, self.nu
        D = np.zeros(((N-1)*nu, N*nu))
        for i in range(N-1):
            D[i*nu:(i+1)*nu, i*nu:(i+1)*nu] = -np.eye(nu)
            D[i*nu:(i+1)*nu, (i+1)*nu:(i+2)*nu] = np.eye(nu)
        return D
    
    def build_hessian(self, Theta, D):
        """构造Hessian矩阵 H（6项设计）"""
        N, nx, nu = self.N, self.nx, self.nu
        
        # 增广权重矩阵
        Q1_bar = sparse.block_diag([self.Q1]*(N-1) + [self.Q1 + self.Qf])
        R1_bar = sparse.block_diag([self.R1]*(N-1) + [self.R1 + self.Rf])
        R2_bar = sparse.block_diag([self.R2]*(N-2) + [self.R2 + self.Ra])
        
        # 各项贡献（去掉了位姿变化率项）
        H = 2 * (
            Theta.T @ Q1_bar @ Theta +        # 位姿误差+终点位姿
            R1_bar +                           # 速度误差+终点速度
            D.T @ R2_bar @ D                  # 加速度+终点加速度
        )
        
        return H
    
    def build_gradient(self, Theta, Phi, X_e_k):
        """构造梯度向量 f（6项设计）"""
        N = self.N
        
        Q1_bar = sparse.block_diag([self.Q1]*(N-1) + [self.Q1 + self.Qf])
        
        f = 2 * Theta.T @ Q1_bar @ Phi @ X_e_k
        
        return f
    
    def solve(self, X_e_k, v_r, omega_r):
        """求解MPC"""
        # 离散化
        Ad, Bd, A, B = self.discretize(v_r, omega_r)
        
        # 预测矩阵
        Phi, Theta = self.build_prediction_matrices(Ad, Bd)
        D = self.build_difference_matrix()
        
        # Hessian和梯度（简化版，去掉位姿变化率）
        H = self.build_hessian(Theta, D)
        f = self.build_gradient(Theta, Phi, X_e_k)
        
        # 约束
        N, nu = self.N, self.nu
        
        # 控制量约束
        v_min, v_max = 0.0, 2.0
        omega_min, omega_max = -1.0, 1.0
        U_min = np.tile([v_min - v_r, omega_min - omega_r], N)
        U_max = np.tile([v_max - v_r, omega_max - omega_r], N)
        
        # 加速度约束
        a_min, a_max = -2.0, 2.0
        alpha_min, alpha_max = -1.0, 1.0
        Delta_min = np.tile([a_min, alpha_min], N-1)
        Delta_max = np.tile([a_max, alpha_max], N-1)
        
        # 约束矩阵
        A_constraint = sparse.vstack([
            sparse.eye(N*nu),
            D
        ])
        
        l = np.concatenate([U_min, Delta_min])
        u = np.concatenate([U_max, Delta_max])
        
        # OSQP求解
        prob = osqp.OSQP()
        prob.setup(P=sparse.csc_matrix(H), q=f, 
                   A=sparse.csc_matrix(A_constraint), l=l, u=u,
                   verbose=False, eps_abs=1e-4, eps_rel=1e-4)
        
        res = prob.solve()
        
        if res.info.status != 'solved':
            print(f"Warning: OSQP status = {res.info.status}")
            return None
        
        # 提取第一个控制量
        U_opt = res.x
        delta_v_opt = U_opt[0]
        delta_omega_opt = U_opt[1]
        
        # 恢复实际控制
        v_cmd = v_r + delta_v_opt
        omega_cmd = omega_r + delta_omega_opt
        
        return v_cmd, omega_cmd, U_opt

# 使用示例
if __name__ == "__main__":
    mpc = DifferentialDriveMPC(N=10, T=0.1)
    
    # 当前误差状态
    X_e_k = np.array([0.5, 0.2, 0.1])  # [x_e, y_e, theta_e]
    
    # 参考速度
    v_r, omega_r = 1.0, 0.1
    
    # 求解
    v_cmd, omega_cmd, U_opt = mpc.solve(X_e_k, v_r, omega_r)
    
    print(f"控制输出: v = {v_cmd:.3f} m/s, ω = {omega_cmd:.3f} rad/s")
    print(f"预测的终点加速度: δv = {U_opt[18] - U_opt[16]:.4f}, δω = {U_opt[19] - U_opt[17]:.4f}")
    print("✅ 基于6项设计：去掉位姿变化率，保留终点加速度防止抖动")
```

### 7.2 滚动时域控制流程

```
初始化: k = 0
while 未到达目标:
    1. 测量当前误差状态 X_e(k)
    2. 获取参考速度 v_r, ω_r
    3. 构造 H, f, A, l, u
    4. 调用OSQP求解 → U_opt
    5. 应用第一个控制量: v_cmd, ω_cmd
    6. 等待采样周期 T
    7. k = k + 1
end
```

---

## 8. 参数调优指南

### 8.1 权重矩阵调优

#### **Q1：位姿误差权重**

| 现象 | 原因 | 调整 |
|------|------|------|
| 跟踪误差大 | Q1太小 | 增大Q1 |
| 控制激进、抖动 | Q1太大 | 减小Q1或增大R |
| 横向偏移严重 | $q_x$或$q_y$太小 | 增大横/纵向权重 |

**推荐**：$\mathbf{Q}_1 = \text{diag}(100, 100, 10)$

#### **Q2：位姿变化率权重**

| 作用 | 效果 | 推荐值 |
|------|------|--------|
| 限制状态变化速度 | 更平滑的轨迹 | $\mathbf{Q}_2 = \text{diag}(1, 1, 0.1)$ |

**调优**：
- Q2太大 → 机器人反应迟钝
- Q2太小 → 状态变化剧烈

#### **R1：速度误差权重**

| 现象 | 原因 | 调整 |
|------|------|------|
| 控制抖动 | R1太小 | 增大R1 |
| 跟踪迟钝 | R1太大 | 减小R1 |

**推荐**：$\mathbf{R}_1 = \text{diag}(0.1, 0.1)$

#### **R2：加速度权重**

| 现象 | 原因 | 调整 |
|------|------|------|
| 加速度突变 | R2太小 | 增大R2 |
| 响应过慢 | R2太大 | 减小R2 |

**推荐**：$\mathbf{R}_2 = \text{diag}(1, 1)$

#### **终点权重**

| 权重 | 倍数 | 说明 |
|------|------|------|
| $\mathbf{Q}_f$ | $1000 \times \mathbf{Q}_1$ | 强制终点位姿收敛 |
| $\mathbf{R}_f$ | $100 \times \mathbf{R}_1$ | 强制终点速度为0 |
| $\mathbf{R}_a$ | $100 \times \mathbf{R}_2$ | 强制终点加速度为0 |

### 8.2 预测步数 N

| N | 优点 | 缺点 | 场景 |
|---|------|------|------|
| 5 | 计算快 | 预见性差 | 简单直线 |
| 10 | 平衡 | - | **推荐** |
| 20 | 预见性好 | 计算慢 | 复杂轨迹 |

**经验公式**：$N \approx \frac{\text{预见距离}}{v_r \cdot T}$

### 8.3 约束设置

```python
# 速度约束
v_min, v_max = 0.0, 2.0        # m/s
omega_min, omega_max = -1.57, 1.57  # rad/s (±90°/s)

# 加速度约束
a_min, a_max = -2.0, 2.0       # m/s²
alpha_min, alpha_max = -1.0, 1.0    # rad/s²
```

### 8.4 调试技巧

1. **可视化预测轨迹**：画出未来N步的预测路径
2. **监控求解时间**：应 < 采样周期T
3. **检查目标函数值**：应随时间递减
4. **验证约束满足**：确保所有约束都不违反
5. **测试极端情况**：大误差、急转弯、停车等

---

## 9. 设计决策说明

### 9.1 决策过程

**初始设计**：7项目标函数 → **最终决策**：去掉第2项（位姿误差变化率），保留6项

### 9.2 为什么去掉位姿误差变化率？

#### **理论分析：该项冗余**

$$
\dot{\mathbf{X}}_e = \mathbf{A}\mathbf{X}_e + \mathbf{B}\Delta\mathbf{u}
$$

位姿误差变化率已被 $\mathbf{X}_e$（第1项）和 $\Delta\mathbf{u}$（第2项）完全确定，惩罚它等于间接重复惩罚第1、2项。

#### **Apollo实践验证**

```cpp
// Apollo MPC权重配置
Q = diag([0.05,  // lateral_error
          0.0,   // lateral_error_rate ⚠️ 权重=0！
          1.0,   // heading_error
          0.0,   // heading_error_rate ⚠️ 权重=0！
          0.1,   // station_error
          0.1])  // speed_error
```

Apollo将误差变化率权重**设为0**，实践证明该项不需要。

#### **工程优势**

- ✅ 计算更快（Hessian矩阵更稀疏）
- ✅ 调参更简单（减少一个权重矩阵）
- ✅ 系统反应更快（避免过度平滑）

### 9.3 为什么保留终点加速度？

虽然理论上第3项（全程加速度平滑）+ 第5项（终点速度为0）能保证终点加速度接近0，但**显式约束更可靠**。

#### **工程实践价值**

| 考虑 | 说明 | 效果 |
|------|------|------|
| 数值误差补偿 | OSQP精度eps=1e-4，可能有小误差 | 显式约束消除残余误差 |
| 防止边界抖动 | 到点瞬间的小幅震荡 | **你的关注点⭐** |
| 机械系统友好 | 电机对低速加速度突变敏感 | 确保完全静止 |
| 双保险机制 | 即使第3+5项失效，第6项保底 | 可靠性优先 |

#### **权重策略：适中的10倍**

```python
Ra = 10 * R2  # 而非100倍
```

**原因**：
- 第3项已全程平滑（R2），第6项只是额外强调
- 10倍足够，100倍会让减速过程过于保守
- 可根据距离动态调整（接近时加大）

### 9.4 最终设计对比

| 方案 | 项数 | 特点 | 适用场景 |
|------|------|------|----------|
| 7项（初始） | 7 | 理论完整但冗余 | 理论研究 |
| 5项（我建议） | 5 | 高效，Apollo验证 | 高速路径跟踪 |
| **6项（你决策）** | **6** | **平衡，工程可靠** | **点到点导航⭐** |

### 9.5 核心公式回顾

**目标函数（6项）**：

$$
\boxed{
\begin{align}
J = & \sum_{i=1}^{N} \mathbf{X}_e^\top(k+i) \mathbf{Q}_1 \mathbf{X}_e(k+i) 
    && \text{位姿误差} \\
& + \sum_{i=0}^{N-1} \Delta\mathbf{u}^\top(k+i) \mathbf{R}_1 \Delta\mathbf{u}(k+i) 
    && \text{速度误差} \\
& + \sum_{i=1}^{N-1} \delta\mathbf{u}^\top(k+i) \mathbf{R}_2 \delta\mathbf{u}(k+i) 
    && \text{加速度平滑} \\
& + \mathbf{X}_e^\top(k+N) \mathbf{Q}_f \mathbf{X}_e(k+N) 
    && \text{终点位姿} \\
& + \Delta\mathbf{u}^\top(k+N-1) \mathbf{R}_f \Delta\mathbf{u}(k+N-1) 
    && \text{终点速度} \\
& + \delta\mathbf{u}^\top(k+N-1) \mathbf{R}_a \delta\mathbf{u}(k+N-1) 
    && \text{终点加速度}
\end{align}
}
$$

**Hessian矩阵（简化）**：

$$
\mathbf{H} = 2(\boldsymbol{\Theta}^\top \bar{\mathbf{Q}}_1^{\text{aug}} \boldsymbol{\Theta} + \bar{\mathbf{R}}_1^{\text{aug}} + \mathbf{D}^\top \bar{\mathbf{R}}_2^{\text{aug}} \mathbf{D})
$$

**梯度向量（简化）**：

$$
\mathbf{f} = 2 \boldsymbol{\Theta}^\top \bar{\mathbf{Q}}_1^{\text{aug}} \boldsymbol{\Phi} \mathbf{X}_e(k)
$$

**权重推荐**：

```python
Q1 = diag([100, 100, 10])    # 位姿误差
R1 = diag([0.1, 0.1])        # 速度误差
R2 = diag([1, 1])            # 加速度
Qf = 1000 * Q1               # 终点位姿
Rf = 100 * R1                # 终点速度
Ra = 10 * R2                 # 终点加速度 ⭐关键
```

### 9.6 实施清单

- [x] 连续状态方程（线性化）
- [x] 离散化（欧拉法）
- [x] 预测矩阵 Φ, Θ
- [x] 差分矩阵 D
- [x] ~~位姿变化率矩阵~~ （已去掉）
- [x] 6项目标函数
- [x] Hessian和梯度构造
- [x] 控制和加速度约束
- [x] OSQP格式转换
- [x] Python完整实现

### 9.7 决策总结

**你的决策：理论严谨 + 工程可靠 = 完美平衡** ✅

- ❌ **去掉位姿误差变化率**：Apollo验证，理论冗余，提升效率
- ✅ **保留终点加速度**：工程考虑，防止抖动，确保平稳停止

祝你实现顺利！🚀

---

## 参考文献

1. **Model Predictive Control**: Camacho & Alba (2007)
2. **Nonlinear Model Predictive Control**: Grüne & Pannek (2017)
3. **Apollo MPC**: `modules/common/math/mpc_osqp.cc`
4. **OSQP Documentation**: https://osqp.org/
5. **差速驱动运动学**: Lynch & Park - Modern Robotics (2017)
