#include "../include/ConvexMpc.h"

ConvexMpc::ConvexMpc(Eigen::VectorXd &q_weights_, Eigen::VectorXd &r_weights_) {
    mu = 0.3;
    fz_min = 0.0;
    fz_max = 0.0;

    q_weights_mpc.resize(MPC_STATE_DIM * PLAN_HORIZON);
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        q_weights_mpc.segment(i * MPC_STATE_DIM, MPC_STATE_DIM) = q_weights_;
    }
    L.diagonal() = 2 * q_weights_mpc;

    r_weights_mpc.resize(NUM_DOF * PLAN_HORIZON);
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        r_weights_mpc.segment(i * NUM_DOF, NUM_DOF) = r_weights_;
    }
    K.diagonal() = 2 * r_weights_mpc;

    // 对规划中的一个步长的一条腿的线性系数矩阵
    // Ac.block<5, 3>(5 * i, 3 * i) << 1 0 u
    //                                 1 0 -u
    //                                 0 1 u
    //                                 0 1 -u
    //                                 0 0 1
    linear_constraints.resize(MPC_CONSTRAINT_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);    
    for (int i = 0; i < NUM_LEG * PLAN_HORIZON; ++i) {
        linear_constraints.insert(0 + 5 * i, 0 + 3 * i) = 1;
        linear_constraints.insert(1 + 5 * i, 0 + 3 * i) = 1;
        linear_constraints.insert(2 + 5 * i, 1 + 3 * i) = 1;
        linear_constraints.insert(3 + 5 * i, 1 + 3 * i) = 1;
        linear_constraints.insert(4 + 5 * i, 2 + 3 * i) = 1;

        linear_constraints.insert(0 + 5 * i, 2 + 3 * i) = mu;
        linear_constraints.insert(1 + 5 * i, 2 + 3 * i) = -mu;
        linear_constraints.insert(2 + 5 * i, 2 + 3 * i) = mu;
        linear_constraints.insert(3 + 5 * i, 2 + 3 * i) = -mu;
    }
}

void ConvexMpc::reset() {
    A_mat_c.setZero();
    B_mat_c.setZero();

    A_mat_d.setZero();
    B_mat_d.setZero();
    B_mat_d_list.setZero();

    A_qp.setZero();
    B_qp.setZero();

    gradient.setZero();
    lb.setZero();
    ub.setZero();
}

void ConvexMpc::calculate_A_mat_c(Eigen::Vector3d root_euler) {
    // 欧拉角=[roll滚转(fai) pitch俯仰(theta) yaw偏航(psi)]
    double cos_yaw = cos(root_euler[2]);
    double sin_yaw = sin(root_euler[2]);

    // 世界坐标系角速度转为机体坐标系欧拉角速度的旋转矩阵
    Eigen::Matrix3d ang_vel_to_rpy_rate;

    // ang_vel_to_rpy_rate << cos_yaw / cos_pitch  sin_yaw / cos_pitch  0
    //                        -sin_yaw             cos_yaw              0
    //                        cos_yaw * tan_pitch  sin_yaw * tan_pitch  1
    // 近似于Rz(psi)
    ang_vel_to_rpy_rate << cos_yaw, sin_yaw, 0,
            -sin_yaw, cos_yaw, 0,
            0, 0, 1;

    // A_mat_c << 0*3 0*3 Rz(psi) 0*3 0
    //            0*3 0*3   0*3   1*3 0
    //            0*3 0*3   0*3   0*3 0
    //            0*3 0*3   0*3   0*3 0
    //             0   0     0     0  1
    A_mat_c.block<3, 3>(0, 6) = ang_vel_to_rpy_rate;
    A_mat_c.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    A_mat_c(11, NUM_DOF) = 1;
}

void ConvexMpc::calculate_B_mat_c(double robot_mass, const Eigen::Matrix3d &trunk_inertia, Eigen::Matrix3d root_rot_mat,
                                  Eigen::Matrix<double, 3, NUM_LEG> foot_pos) {
    // 世界坐标系中的惯性矩阵I，机体坐标系中的惯性矩阵Ib
    Eigen::Matrix3d trunk_inertia_world;
    // I = R * Ib * R'
    trunk_inertia_world = root_rot_mat * trunk_inertia * root_rot_mat.transpose();

    // B_mat_c <<   0*3     0*3   ···   0*3
    //              0*3     0*3   ···   0*3
    //            I-1[r1] I-1[r2] ··· I-1[rn]
    //             1*3/m   1*3/m  ···  1*3/m
    //               0       0    ···    0
    for (int i = 0; i < NUM_LEG; ++i) {
        B_mat_c.block<3, 3>(6, 3 * i) =
                trunk_inertia_world.inverse() * Utils::skew(foot_pos.block<3, 1>(0, i));
        B_mat_c.block<3, 3>(9, 3 * i) =
                (1 / robot_mass) * Eigen::Matrix3d::Identity();
    }
}

void ConvexMpc::state_space_discretization(double dt) {
    // 离散时间矩阵A_hat, B_hat
    A_mat_d = Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() + A_mat_c * dt;
    B_mat_d = B_mat_c * dt;
}

void ConvexMpc::calculate_qp_mats(CtrlStates &state) {
    // 标准二次规划的形式
    // minimize 1/2 * x' * P * x + q' * x
    //   subject to lb <= Ac * x <= ub
    //     P: hessian
    //     q: gradient
    //     Ac: linear constraints

    // 计算A_qp和B_qp
    // A_qp << A
    //         A^2
    //         A^3
    //         ...
    //         A^k

    // B_qp <<   A^0*B(0)
    //           A^1*B(0)        B(1)
    //           A^2*B(0)       A*B(1)         B(2)
    //             ...
    //         A^(k-1)*B(0)  A^(k-2)*B(1)  A^(k-3)*B(2) ... B(k-1)
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        if (i == 0) {
            A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, 0) = A_mat_d;
        } else {
            A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, 0) = 
            A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (i - 1), 0)*A_mat_d;
        }
        for (int j = 0; j < i + 1; ++j) {
            if (i - j == 0) {
                B_qp.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM * i, NUM_DOF * j) =
                    B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(j * MPC_STATE_DIM, 0);
            } else {
                B_qp.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM * i, NUM_DOF * j) =
                        A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (i - j - 1), 0) 
                        * B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(j * MPC_STATE_DIM, 0);
            }
        }
    }

    // 计算hessian矩阵
    // H = Bqp'*L*Bqp + K
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON> dense_hessian;
    dense_hessian = (B_qp.transpose() * L * B_qp);
    dense_hessian += K;
    hessian = dense_hessian.sparseView();

    // 计算gradient矩阵
    // g = Bqp'*L*(Aqp*x0 - y)
    Eigen::Matrix<double, 13*PLAN_HORIZON, 1> tmp_vec = A_qp * state.mpc_states;
    tmp_vec -= state.mpc_states_d;
    gradient = B_qp.transpose() * L * tmp_vec;

    fz_min = 0;
    fz_max = 180;

    // 计算上下界ub和lb
    // lb <<   0
    //        -∞
    //         0
    //        -∞
    //       fzmin（有接触的话）

    // ub <<   ∞
    //         0
    //         ∞
    //         0
    //       fzmax（有接触的话）
    Eigen::VectorXd lb_one_horizon(MPC_CONSTRAINT_DIM);
    Eigen::VectorXd ub_one_horizon(MPC_CONSTRAINT_DIM);
    for (int i = 0; i < NUM_LEG; ++i) {
        lb_one_horizon.segment<5>(i * 5) << 0,
                -OsqpEigen::INFTY,
                0,
                -OsqpEigen::INFTY,
                fz_min * state.contacts[i];
        ub_one_horizon.segment<5>(i * 5) << OsqpEigen::INFTY,
                0,
                OsqpEigen::INFTY,
                0,
                fz_max * state.contacts[i];
    }
    
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        lb.segment<MPC_CONSTRAINT_DIM>(i * MPC_CONSTRAINT_DIM) = lb_one_horizon;
        ub.segment<MPC_CONSTRAINT_DIM>(i * MPC_CONSTRAINT_DIM) = ub_one_horizon;
    }
}