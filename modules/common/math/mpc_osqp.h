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

#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Eigen"
#include "osqp/osqp.h"

#include "cyber/common/log.h"

namespace apollo {
namespace common {
namespace math {
class MpcOsqp {
public:
    /**
     * @brief Solver for discrete-time model predictive control problem.
     * @param matrix_a The system dynamic matrix
     * @param matrix_b The control matrix
     * @param matrix_q The cost matrix for control state
     * @param matrix_lower The lower bound control constrain matrix
     * @param matrix_upper The upper bound control constrain matrix
     * @param matrix_initial_state The initial state matrix
     * @param max_iter The maximum iterations
     */
    MpcOsqp(const Eigen::MatrixXd& matrix_a,
            const Eigen::MatrixXd& matrix_b,
            const Eigen::MatrixXd& matrix_q,
            const Eigen::MatrixXd& matrix_r,
            const Eigen::MatrixXd& matrix_initial_x,
            const Eigen::MatrixXd& matrix_u_lower,
            const Eigen::MatrixXd& matrix_u_upper,
            const Eigen::MatrixXd& matrix_x_lower,
            const Eigen::MatrixXd& matrix_x_upper,
            const Eigen::MatrixXd& matrix_x_ref,
            const int max_iter,
            const int horizon,
            const double eps_abs);

    // control vector
    bool Solve(std::vector<double>* control_cmd);

private:
    /**
     * 构造二次规划中的P矩阵
     * P_data: 非零元素的值
     * P_indices: 非零元素的行索引
     * P_indptr: 每列在 P_data 中的起始位置
     */
    void CalculateKernel(std::vector<c_float>* P_data, std::vector<c_int>* P_indices, std::vector<c_int>* P_indptr);
    void CalculateEqualityConstraint(std::vector<c_float>* A_data,
                                     std::vector<c_int>* A_indices,
                                     std::vector<c_int>* A_indptr);
    // 这个函数是为了计算二次规划中的q向量，不过在apollo的实现中，q向量是0
    void CalculateGradient();
    void CalculateConstraintVectors();
    OSQPSettings* Settings();
    OSQPData* Data();
    void FreeData(OSQPData* data);

    // 在堆上分配一块内存，然后将传入的vec复制过来再返回，这样可以延长数据的生命周期
    template <typename T>
    T* CopyData(const std::vector<T>& vec) {
        T* data = new T[vec.size()];
        memcpy(data, vec.data(), sizeof(T) * vec.size());
        return data;
    }

private:
    Eigen::MatrixXd matrix_a_;
    Eigen::MatrixXd matrix_b_;
    Eigen::MatrixXd matrix_q_;  // 状态权重矩阵，6 x 6
    Eigen::MatrixXd matrix_r_;  // 控制权重矩阵
    Eigen::MatrixXd matrix_initial_x_;
    const Eigen::MatrixXd matrix_u_lower_;
    const Eigen::MatrixXd matrix_u_upper_;
    const Eigen::MatrixXd matrix_x_lower_;
    const Eigen::MatrixXd matrix_x_upper_;
    const Eigen::MatrixXd matrix_x_ref_;
    int max_iteration_;
    size_t horizon_;  // 10
    double eps_abs_;
    size_t state_dim_;    // 6
    size_t control_dim_;  // 2
    size_t num_param_;    // 决策变量的总维数
    int num_constraint_;
    Eigen::VectorXd gradient_;
    Eigen::VectorXd lowerBound_;  // 初始状态约束+上下界不等式约束
    Eigen::VectorXd upperBound_;  // 初始状态约束+上下界不等式约束
};
}  // namespace math
}  // namespace common
}  // namespace apollo
