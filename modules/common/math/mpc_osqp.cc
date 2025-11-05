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

#include "modules/common/math/mpc_osqp.h"

namespace apollo {
namespace common {
namespace math {
MpcOsqp::MpcOsqp(const Eigen::MatrixXd& matrix_a,
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
                 const double eps_abs)
    : matrix_a_(matrix_a),
      matrix_b_(matrix_b),
      matrix_q_(matrix_q),
      matrix_r_(matrix_r),
      matrix_initial_x_(matrix_initial_x),
      matrix_u_lower_(matrix_u_lower),
      matrix_u_upper_(matrix_u_upper),
      matrix_x_lower_(matrix_x_lower),
      matrix_x_upper_(matrix_x_upper),
      matrix_x_ref_(matrix_x_ref),
      max_iteration_(max_iter),
      horizon_(horizon),
      eps_abs_(eps_abs) {
    // 状态变量的维度
    state_dim_ = matrix_b.rows();
    // 控制变量的维度
    control_dim_ = matrix_b.cols();
    ADEBUG << "state_dim" << state_dim_;
    ADEBUG << "control_dim_" << control_dim_;
    num_param_ = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
}

void MpcOsqp::CalculateKernel(std::vector<c_float>* P_data,
                              std::vector<c_int>* P_indices,
                              std::vector<c_int>* P_indptr) {
    // col1:(row,val),...; col2:(row,val),....; ...
    // columns是存放所有非零元素的列索引、行索引和数值，用于构建CSC格式的稀疏矩阵
    std::vector<std::vector<std::pair<c_int, c_float>>> columns;
    columns.resize(num_param_);
    size_t value_index = 0;
    // state and terminal state
    for (size_t i = 0; i <= horizon_; ++i) {
        for (size_t j = 0; j < state_dim_; ++j) {
            // (row, val)
            columns[i * state_dim_ + j].emplace_back(i * state_dim_ + j, matrix_q_(j, j));
            ++value_index;
        }
    }
    // control
    const size_t state_total_dim = state_dim_ * (horizon_ + 1);
    for (size_t i = 0; i < horizon_; ++i) {
        for (size_t j = 0; j < control_dim_; ++j) {
            // (row, val)
            columns[i * control_dim_ + j + state_total_dim].emplace_back(state_total_dim + i * control_dim_ + j,
                                                                         matrix_r_(j, j));
            ++value_index;
        }
    }
    /**
     * 假设状态变量(x, y)数量为2，控制状态(a)数量为1，预测周期为2，那么 columns 的大小为5，示例为：
     * columns = [[0, q_x],[1, q_y],[2, q_x],[3, q_y],[4, q_x],[5, q_y],[6, r_a],[7, r_a]]
     */
    CHECK_EQ(value_index, num_param_);

    /**
     * P_data：按列存放所有的非零元素
     * P_indices：所有非零元素所在的行的索引
     * P_indptr：所有元素所在的列在P_data中的索引（非常拗口且TM的抽象）
     *
     * 假设一个矩阵是：
     * [0   0   5   0   0   12   0
     *  11  0   0   0   0   0   77
     *  22  0   6   0   88  0   88
     *  33  0   0   0   99  0   0
     *  0   0   7   0   0   13  0
     *  44  0   0   0   0   0   99
     *  0   0   8   0   0   14  111
     *  55  0   9   0   0   0   0]
     *
     * 那么:
     * P_data =    [11, 22, 33, 44, 55, 5, 6, 7, 8, 9, 88, 99, 12, 13, 14, 77, 88, 99, 111]
     * P_indices = [1, 2, 3, 5, 7, 0, 2, 4, 6, 7, 2, 3, 0, 4, 6, 1, 2, 5, 6]
     * P_indptr =  [0, 5, 5, 10, 10, 12, 15, 19]
     */
    int ind_p = 0;
    for (size_t i = 0; i < num_param_; ++i) {
        // TODO(SHU) Check this
        P_indptr->emplace_back(ind_p);
        for (const auto& row_data_pair : columns[i]) {
            P_data->emplace_back(row_data_pair.second);    // val
            P_indices->emplace_back(row_data_pair.first);  // row
            ++ind_p;
        }
    }
    P_indptr->emplace_back(ind_p);
}

// reference is always zero
// 这个函数是为了计算二次规划中的q向量，不过在apollo的实现中，q向量是0
void MpcOsqp::CalculateGradient() {
    // populate the gradient vector
    gradient_ = Eigen::VectorXd::Zero(state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    for (size_t i = 0; i < horizon_ + 1; i++) {
        gradient_.block(i * state_dim_, 0, state_dim_, 1) = -1.0 * matrix_q_ * matrix_x_ref_;
    }
    ADEBUG << "Gradient_mat";
    ADEBUG << gradient_;
}

// equality constraints x(k+1) = A*x(k)
void MpcOsqp::CalculateEqualityConstraint(std::vector<c_float>* A_data,
                                          std::vector<c_int>* A_indices,
                                          std::vector<c_int>* A_indptr) {
    static constexpr double kEpsilon = 1e-6;
    // block matrix
    Eigen::MatrixXd matrix_constraint =
        Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) + control_dim_ * horizon_,
                              state_dim_ * (horizon_ + 1) + control_dim_ * horizon_);
    Eigen::MatrixXd state_identity_mat =
        Eigen::MatrixXd::Identity(state_dim_ * (horizon_ + 1), state_dim_ * (horizon_ + 1));
    ADEBUG << "state_identity_mat" << state_identity_mat;

    matrix_constraint.block(0, 0, state_dim_ * (horizon_ + 1), state_dim_ * (horizon_ + 1)) = -1 * state_identity_mat;
    ADEBUG << "matrix_constraint";
    ADEBUG << matrix_constraint;

    Eigen::MatrixXd control_identity_mat = Eigen::MatrixXd::Identity(control_dim_, control_dim_);

    for (size_t i = 0; i < horizon_; i++) {
        matrix_constraint.block((i + 1) * state_dim_, i * state_dim_, state_dim_, state_dim_) = matrix_a_;
    }
    ADEBUG << "matrix_constraint with A";
    ADEBUG << matrix_constraint;

    for (size_t i = 0; i < horizon_; i++) {
        matrix_constraint.block((i + 1) * state_dim_, i * control_dim_ + (horizon_ + 1) * state_dim_, state_dim_,
                                control_dim_) = matrix_b_;
    }
    ADEBUG << "matrix_constraint with B";
    ADEBUG << matrix_constraint;

    Eigen::MatrixXd all_identity_mat = Eigen::MatrixXd::Identity(num_param_, num_param_);

    matrix_constraint.block(state_dim_ * (horizon_ + 1), 0, num_param_, num_param_) = all_identity_mat;
    ADEBUG << "matrix_constraint with I";
    ADEBUG << matrix_constraint;

    std::vector<std::vector<std::pair<c_int, c_float>>> columns;
    columns.resize(num_param_ + 1);
    int value_index = 0;
    // state and terminal state
    for (size_t i = 0; i < num_param_; ++i) {                                  // col
        for (size_t j = 0; j < num_param_ + state_dim_ * (horizon_ + 1); ++j)  // row
            if (std::fabs(matrix_constraint(j, i)) > kEpsilon) {
                // (row, val)
                columns[i].emplace_back(j, matrix_constraint(j, i));
                ++value_index;
            }
    }
    ADEBUG << "value_index";
    ADEBUG << value_index;
    int ind_A = 0;
    for (size_t i = 0; i < num_param_; ++i) {
        A_indptr->emplace_back(ind_A);
        for (const auto& row_data_pair : columns[i]) {
            A_data->emplace_back(row_data_pair.second);    // value
            A_indices->emplace_back(row_data_pair.first);  // row
            ++ind_A;
        }
    }
    A_indptr->emplace_back(ind_A);
}

void MpcOsqp::CalculateConstraintVectors() {
    // evaluate the lower and the upper inequality vectors
    // 构造不等式约束的向量
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    for (size_t i = 0; i < horizon_; i++) {
        // 按块初始化
        lowerInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0, control_dim_, 1) = matrix_u_lower_;
        upperInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0, control_dim_, 1) = matrix_u_upper_;
    }
    ADEBUG << " matrix_u_lower_";
    for (size_t i = 0; i < horizon_ + 1; i++) {
        lowerInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_lower_;
        upperInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_upper_;
    }
    ADEBUG << " matrix_x_lower_";

    // evaluate the lower and the upper equality vectors
    // 构造初始状态约束的向量
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1), 1);
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0, 0, state_dim_, 1) = -1 * matrix_initial_x_;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;
    ADEBUG << " matrix_initial_x_";

    // merge inequality and equality vectors
    lowerBound_ = Eigen::MatrixXd::Zero(2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    lowerBound_ << lowerEquality, lowerInequality;
    ADEBUG << " lowerBound_ ";
    upperBound_ = Eigen::MatrixXd::Zero(2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    upperBound_ << upperEquality, upperInequality;
    ADEBUG << " upperBound_";
}

OSQPSettings* MpcOsqp::Settings() {
    // default setting
    OSQPSettings* settings = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
    if (settings == nullptr) {
        return nullptr;
    } else {
        osqp_set_default_settings(settings);
        settings->polish = true;
        settings->scaled_termination = true;
        settings->verbose = false;
        settings->max_iter = max_iteration_;
        settings->eps_abs = eps_abs_;
        return settings;
    }
}

OSQPData* MpcOsqp::Data() {
    OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
    // 决策变量的总维数
    size_t kernel_dim = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
    // 总约束的维数
    size_t num_affine_constraint = 2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
    if (data == nullptr) {
        return nullptr;
    } else {
        data->n = kernel_dim;
        data->m = num_affine_constraint;
        std::vector<c_float> P_data;
        std::vector<c_int> P_indices;
        std::vector<c_int> P_indptr;
        ADEBUG << "before CalculateKernel";
        CalculateKernel(&P_data, &P_indices, &P_indptr);
        ADEBUG << "CalculateKernel done";
        // csc_matrix
        // 中保存的是指针，而P_data是局部变量，如果直接用P_data的地址的话，在整个Data函数返回后P_data就被释放了，就会导致程序崩溃。所以使用CopyData
        // 函数，将局部变量复制到堆上并返回一个指向堆上的指针，以保证在Data函数返回后程序不会崩溃
        data->P = csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data), CopyData(P_indices),
                             CopyData(P_indptr));
        ADEBUG << "Get P matrix";
        data->q = gradient_.data();
        ADEBUG << "before CalculateEqualityConstraint";
        std::vector<c_float> A_data;
        std::vector<c_int> A_indices;
        std::vector<c_int> A_indptr;
        CalculateEqualityConstraint(&A_data, &A_indices, &A_indptr);
        ADEBUG << "CalculateEqualityConstraint done";
        data->A = csc_matrix(state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) + control_dim_ * horizon_,
                             kernel_dim, A_data.size(), CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
        ADEBUG << "Get A matrix";
        data->l = lowerBound_.data();
        data->u = upperBound_.data();
        return data;
    }
}

void MpcOsqp::FreeData(OSQPData* data) {
    c_free(data->A);
    c_free(data->P);
    c_free(data);
}

bool MpcOsqp::Solve(std::vector<double>* control_cmd) {
    ADEBUG << "Before Calc Gradient";
    CalculateGradient();
    ADEBUG << "After Calc Gradient";
    CalculateConstraintVectors();
    ADEBUG << "MPC2Matrix";

    OSQPData* data = Data();
    ADEBUG << "OSQP data done";
    ADEBUG << "OSQP data n" << data->n;
    ADEBUG << "OSQP data m" << data->m;
    for (int i = 0; i < data->n; ++i) {
        ADEBUG << "OSQP data q" << i << ":" << (data->q)[i];
    }
    ADEBUG << "OSQP data l" << data->l;
    for (int i = 0; i < data->m; ++i) {
        ADEBUG << "OSQP data l" << i << ":" << (data->l)[i];
    }
    ADEBUG << "OSQP data u" << data->u;
    for (int i = 0; i < data->m; ++i) {
        ADEBUG << "OSQP data u" << i << ":" << (data->u)[i];
    }

    OSQPSettings* settings = Settings();
    ADEBUG << "OSQP setting done";
    OSQPWorkspace* osqp_workspace = nullptr;
    // osqp_setup(&osqp_workspace, data, settings);
    osqp_workspace = osqp_setup(data, settings);
    ADEBUG << "OSQP workspace ready";
    osqp_solve(osqp_workspace);

    auto status = osqp_workspace->info->status_val;
    ADEBUG << "status:" << status;
    // check status
    if (status < 0 || (status != 1 && status != 2)) {
        AERROR << "failed optimization status:\t" << osqp_workspace->info->status;
        osqp_cleanup(osqp_workspace);
        FreeData(data);
        c_free(settings);
        return false;
    } else if (osqp_workspace->solution == nullptr) {
        AERROR << "The solution from OSQP is nullptr";
        osqp_cleanup(osqp_workspace);
        FreeData(data);
        c_free(settings);
        return false;
    }

    size_t first_control = state_dim_ * (horizon_ + 1);
    for (size_t i = 0; i < control_dim_; ++i) {
        control_cmd->at(i) = osqp_workspace->solution->x[i + first_control];
        ADEBUG << "control_cmd:" << i << ":" << control_cmd->at(i);
    }

    // Cleanup
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return true;
}

}  // namespace math
}  // namespace common
}  // namespace apollo
