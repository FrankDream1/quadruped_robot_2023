#ifndef CONVEXMPC_H
#define CONVEXMPC_H

#define EIGEN_STACK_ALLOCATION_LIMIT 0

#include <vector>
#include <chrono>

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>

#include "CtrlStates.h"
#include "Param.h"
#include "Utils.h"

class ConvexMpc {
public:
    ConvexMpc(Eigen::VectorXd &q_weights_, Eigen::VectorXd &r_weights_);

    // 初始化MPC所用系数
    void reset();

    // 计算Ac矩阵
    void calculate_A_mat_c(Eigen::Vector3d root_euler);

    // 计算Bc矩阵
    void calculate_B_mat_c(double robot_mass, const Eigen::Matrix3d &trunk_inertia, Eigen::Matrix3d root_rot_mat,
                           Eigen::Matrix<double, 3, NUM_LEG> foot_pos);

    // 计算离散时间矩阵
    void state_space_discretization(double dt);

    // 计算QP规划矩阵
    void calculate_qp_mats(CtrlStates &state);

    // 约束参数
    double mu;
    double fz_min;
    double fz_max;

    // MPC权重系数
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> q_weights_mpc;
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> r_weights_mpc;

    // 状态微分权重对角矩阵
    Eigen::DiagonalMatrix<double, MPC_STATE_DIM * PLAN_HORIZON> L;
    // 力度权重对角矩阵
    Eigen::DiagonalMatrix<double, NUM_DOF * PLAN_HORIZON> K;

    // 状态空间系数矩阵
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_mat_c;
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_mat_c;

    // 离散化状态空间系数矩阵
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_mat_d;
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_mat_d;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF> B_mat_d_list;

    // 规划步长内的状态空间系数矩阵
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, MPC_STATE_DIM> A_qp;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON> B_qp;

    // 二次规划所用矩阵
    Eigen::SparseMatrix<double> hessian; // P
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> gradient; // q
    Eigen::SparseMatrix<double> linear_constraints; // Ac
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM * PLAN_HORIZON, 1> lb;
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM * PLAN_HORIZON, 1> ub;
};

#endif //CONVEXMPC_H