#include "../include/RobotControl.h"

RobotControl::RobotControl() {
    std::cout << "init RobotControl" << std::endl;
    
    // 初始二次规划所用参数
    Q.diagonal() << 1.0, 1.0, 1.0, 400.0, 400.0, 100.0;
    R = 1e-3;
    mu = 0.7;
    F_min = 0;
    F_max = 180;
    hessian.resize(3 * NUM_LEG, 3 * NUM_LEG);
    gradient.resize(3 * NUM_LEG);
    linearMatrix.resize(NUM_LEG + 4 * NUM_LEG, 3 * NUM_LEG);
    lowerBound.resize(NUM_LEG + 4 * NUM_LEG);
    lowerBound.setZero();
    upperBound.resize(NUM_LEG + 4 * NUM_LEG);
    upperBound.setZero();

    // 初始化MPC计数器
    mpc_init_counter = 0;

    // 设置线性约束矩阵
    for (int i = 0; i < NUM_LEG; ++i) {
        // F_zi不变
        linearMatrix.insert(i, 2 + i * 3) = 1;
        // 摩擦锥条件
        // 1. F_xi < uF_zi
        linearMatrix.insert(NUM_LEG + i * 4, i * 3) = 1;
        linearMatrix.insert(NUM_LEG + i * 4, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4) = -OsqpEigen::INFTY;
        // 2. F_xi > -uF_zi    ===> -F_xi -uF_zi < 0
        linearMatrix.insert(NUM_LEG + i * 4 + 1, i * 3) = -1;
        linearMatrix.insert(NUM_LEG + i * 4 + 1, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 1) = -OsqpEigen::INFTY;
        // 3. F_yi < uF_zi
        linearMatrix.insert(NUM_LEG + i * 4 + 2, 1 + i * 3) = 1;
        linearMatrix.insert(NUM_LEG + i * 4 + 2, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 2) = -OsqpEigen::INFTY;
        // 4. -F_yi > uF_zi
        linearMatrix.insert(NUM_LEG + i * 4 + 3, 1 + i * 3) = -1;
        linearMatrix.insert(NUM_LEG + i * 4 + 3, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 3) = -OsqpEigen::INFTY;
    }

    // 设置平面角度和接触点的移动窗口滤波窗口大小
    terrain_angle_filter = MovingWindowFilter(100);
    for (int i = 0; i < NUM_LEG; ++i) {
        recent_contact_x_filter[i] = MovingWindowFilter(60);
        recent_contact_y_filter[i] = MovingWindowFilter(60);
        recent_contact_z_filter[i] = MovingWindowFilter(60);
    }
}

RobotControl::RobotControl(ros::NodeHandle &_nh) : RobotControl() {
    std::cout << "init nh" << std::endl;
    nh = _nh;
    _nh.param("use_sim_time", use_sim_time);
    // 发布地形角
    pub_terrain_angle = nh.advertise<std_msgs::Float64>("debug/terrain_angle", 100);
}

void RobotControl::update_plan(CtrlStates &state, double dt) {
    state.counter += 1;
    if (!state.movement_mode) {
        // 当movement_mode == 0时站立，代表所有腿都接触地面，重设步态计数器
        for (bool &plan_contact: state.plan_contacts) 
            plan_contact = true;
        state.gait_counter_reset();
    } else {
        // 当movement_mode == 1时行走
        for (int i = 0; i < NUM_LEG; ++i) {
            state.gait_counter(i) = state.gait_counter(i) + state.gait_counter_speed(i);
            state.gait_counter(i) = std::fmod(state.gait_counter(i), state.counter_per_gait);
            if (state.gait_counter(i) <= state.counter_per_swing) {
                state.plan_contacts[i] = true;
            } else {
                state.plan_contacts[i] = false;
            }
        }
    }

    // 更新足端规划: state.foot_pos_target_world
    Eigen::Vector3d lin_vel_world = state.root_lin_vel; // 世界坐标系中的线速度
    Eigen::Vector3d lin_vel_rel = state.root_rot_mat_z.transpose() * lin_vel_world; // 机体坐标系中的线速度

    // 计算落足点位置，使用Raibert Heuristic算法
    state.foot_pos_target_rel = state.default_foot_pos;
    for (int i = 0; i < NUM_LEG; ++i) {
        double delta_x =
                std::sqrt(std::abs(state.default_foot_pos(2)) / 9.8) * (lin_vel_rel(0) - state.root_lin_vel_d(0)) +
                ((state.counter_per_swing / state.gait_counter_speed(i)) * state.control_dt) / 2.0 *
                state.root_lin_vel_d(0);
        double delta_y =
                std::sqrt(std::abs(state.default_foot_pos(2)) / 9.8) * (lin_vel_rel(1) - state.root_lin_vel_d(1)) +
                ((state.counter_per_swing / state.gait_counter_speed(i)) * state.control_dt) / 2.0 *
                state.root_lin_vel_d(1);

        if (delta_x < -FOOT_DELTA_X_LIMIT) {
            delta_x = -FOOT_DELTA_X_LIMIT;
        }
        if (delta_x > FOOT_DELTA_X_LIMIT) {
            delta_x = FOOT_DELTA_X_LIMIT;
        }
        if (delta_y < -FOOT_DELTA_Y_LIMIT) {
            delta_y = -FOOT_DELTA_Y_LIMIT;
        }
        if (delta_y > FOOT_DELTA_Y_LIMIT) {
            delta_y = FOOT_DELTA_Y_LIMIT;
        }

        state.foot_pos_target_rel(0, i) += delta_x;
        state.foot_pos_target_rel(1, i) += delta_y;

        state.foot_pos_target_abs.block<3, 1>(0, i) = state.root_rot_mat * state.foot_pos_target_rel.block<3, 1>(0, i);
        state.foot_pos_target_world.block<3, 1>(0, i) = state.foot_pos_target_abs.block<3, 1>(0, i) + state.root_pos;
    }
}

void RobotControl::generate_swing_legs_ctrl(CtrlStates &state, double dt) {
    state.joint_torques.setZero();

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;  // 当前足端位置
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;  // 当前足端速度

    Eigen::Matrix<float, 1, NUM_LEG> spline_time;  // 样条曲线时间
    spline_time.setZero();

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;  // 期望足端位置
    foot_pos_target.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;  // 期望足端速度
    foot_vel_target.setZero();

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;   // 落足点位置误差
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;   // 落足点速度误差

    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;  // 摆动腿足端残差力
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;  // 支撑腿足端反作用力

    for (int i = 0; i < NUM_LEG; ++i) {
        foot_pos_cur.block<3, 1>(0, i) = state.root_rot_mat_z.transpose() * state.foot_pos_abs.block<3, 1>(0, i);

        // 使用贝塞尔函数计算中间点
        if (state.gait_counter(i) <= state.counter_per_swing) {
            spline_time(i) = 0.0;
            // 支撑腿，持续刷新foot_pos_start
            state.foot_pos_start.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);
        } else {
            // 摆动腿计算样条曲线时间
            spline_time(i) = float(state.gait_counter(i) - state.counter_per_swing) / float(state.counter_per_swing);
        }

        // 通过贝塞尔曲线计算期望落足点位置
        foot_pos_target.block<3, 1>(0, i) = bezierUtils[i].get_foot_pos_curve(spline_time(i),
                                                                              state.foot_pos_start.block<3, 1>(0, i),
                                                                              state.foot_pos_target_rel.block<3, 1>(0, i),
                                                                              0.0);

        // 计算当前落足点速度
        foot_vel_cur.block<3, 1>(0, i) = (foot_pos_cur.block<3, 1>(0, i) - state.foot_pos_rel_last_time.block<3, 1>(0, i)) / dt;
        
        // 更新机体坐标系中落足点位置
        state.foot_pos_rel_last_time.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);

        // 计算期望落足点速度
        foot_vel_target.block<3, 1>(0, i) = (foot_pos_target.block<3, 1>(0, i) - state.foot_pos_target_last_time.block<3, 1>(0, i)) / dt;
        
        // 更新期望落足点位置
        state.foot_pos_target_last_time.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i);

        // 计算落足点位置误差
        foot_pos_error.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i) - foot_pos_cur.block<3, 1>(0, i);
        
        // 计算落足点速度误差
        foot_vel_error.block<3, 1>(0, i) = foot_vel_target.block<3, 1>(0, i) - foot_vel_cur.block<3, 1>(0, i);
        
        // 计算摆动腿足端残差力
        foot_forces_kin.block<3, 1>(0, i) = foot_pos_error.block<3, 1>(0, i).cwiseProduct(state.kp_foot.block<3, 1>(0, i)) +
                                            foot_vel_error.block<3, 1>(0, i).cwiseProduct(state.kd_foot.block<3, 1>(0, i));
    }
    state.foot_pos_cur = foot_pos_cur;

    // 检测是否有提前接触
    bool last_contacts[NUM_LEG];

    for (int i = 0; i < NUM_LEG; ++i) {
        if (state.gait_counter(i) <= state.counter_per_swing * 1.5) {
            state.early_contacts[i] = false;
        }
        if (!state.plan_contacts[i] && (state.gait_counter(i) > state.counter_per_swing * 1.5) && (state.foot_force(i) > FOOT_FORCE_LOW)) {
            state.early_contacts[i] = true;
        }

        // 更新真实接触状态
        last_contacts[i] = state.contacts[i];
        state.contacts[i] = state.plan_contacts[i] || state.early_contacts[i];

        // 如果接触地面，记录最近的落足点位置
        if (state.contacts[i]) {
            state.foot_pos_recent_contact.block<3, 1>(0, i)
                    << recent_contact_x_filter[i].CalculateAverage(state.foot_pos_abs(0, i)),
                    recent_contact_y_filter[i].CalculateAverage(state.foot_pos_abs(1, i)),
                    recent_contact_z_filter[i].CalculateAverage(state.foot_pos_abs(2, i));
        }
    }

    std::cout << "foot_pos_recent_contact z: " << state.foot_pos_recent_contact.block<1, 4>(2, 0) << std::endl;

    state.foot_forces_kin = foot_forces_kin;
}

void RobotControl::compute_joint_torques(CtrlStates &state) {
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;
    joint_torques.setZero();
    
    mpc_init_counter++;
    // 开始10次，关节力矩设为0
    if (mpc_init_counter < 10) {
        state.joint_torques = joint_torques;
    } else {
        // 对摆动腿(contact[i]为假)使用foot_forces_kin来得到joint_torque
        // 对支撑腿(contact[i]为真)使用foot_forces_grf来得到joint_torque
        for (int i = 0; i < NUM_LEG; ++i) {
            // 雅可比矩阵
            Eigen::Matrix3d jac = state.j_foot.block<3, 3>(3 * i, 3 * i);
            if (state.contacts[i]) {
                // 支撑腿tau[i] = J[i]' * f[i]
                joint_torques.segment<3>(i * 3) = jac.transpose() * -state.foot_forces_grf.block<3, 1>(0, i);
            } else {
                // 摆动腿tau[i] = J[i]^-1 * km * f[i]
                Eigen::Vector3d force_tgt = state.km_foot.cwiseProduct(state.foot_forces_kin.block<3, 1>(0, i));
                joint_torques.segment<3>(i * 3) = jac.lu().solve(force_tgt);   // jac * tau = F
            }
        }

        // 重力补偿
        joint_torques += state.torques_gravity;

        // 防止数据出错
        for (int i = 0; i < 12; ++i) {
            if (!isnan(joint_torques[i]))
                state.joint_torques[i] = joint_torques[i];
        }
    }
}

Eigen::Matrix<double, 3, NUM_LEG> RobotControl::compute_grf(CtrlStates &state, double dt) {
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    // 欧拉误差角
    Eigen::Vector3d euler_error = state.root_euler_d - state.root_euler;

    // 将误差角限制在pi/2内
    if (euler_error(2) > 3.1415926 * 1.5) {
        euler_error(2) = state.root_euler_d(2) - 3.1415926 * 2 - state.root_euler(2);
    } else if (euler_error(2) < -3.1415926 * 1.5) {
        euler_error(2) = state.root_euler_d(2) + 3.1415926 * 2 - state.root_euler(2);
    }

    // 地形适应
    Eigen::Vector3d surf_coef = compute_walking_surface(state); // 当前行走平面
    Eigen::Vector3d flat_ground_coef; // 理想行走平面
    flat_ground_coef << 0, 0, 1;
    double terrain_angle = 0;

    // 只有当机器狗站立超过一定高度时才计算地形角
    if (state.root_pos[2] > 0.1) {
        terrain_angle = terrain_angle_filter.CalculateAverage(Utils::cal_dihedral_angle(flat_ground_coef, surf_coef));
    } else {
        terrain_angle = 0;
    }
    
    if (terrain_angle > 0.5) {
        terrain_angle = 0.5;
    }
    if (terrain_angle < -0.5) {
        terrain_angle = -0.5;
    }

    // 前后腿的足端高度差
    double F_R_diff = state.foot_pos_recent_contact(2, 0) + state.foot_pos_recent_contact(2, 1) - state.foot_pos_recent_contact(2, 2) -
                    state.foot_pos_recent_contact(2, 3); // p0z + p1z - p2z - p3z

    // 当前后腿足端高度差大于某一个值时，俯仰角设为地形角
    if (F_R_diff > 0.05) {
    state.root_euler_d[1] = -terrain_angle;
    } else {
    state.root_euler_d[1] = terrain_angle;
    }

    std_msgs::Float64 terrain_angle_msg;
    terrain_angle_msg.data = terrain_angle * (180 / 3.1415926);
    pub_terrain_angle.publish(terrain_angle_msg); // 用角度发布地形角
    std::cout << "desire pitch in deg: " << state.root_euler_d[1] * (180 / 3.1415926) << std::endl;
    std::cout << "terrain angle: " << terrain_angle << std::endl;

    // 保存计算的地形俯仰角
    state.terrain_pitch_angle = terrain_angle;

    // MPC规划 
    ConvexMpc mpc_solver = ConvexMpc(state.q_weights, state.r_weights);
    mpc_solver.reset();

    // 在第一个时间步长初始化MPC状态函数
    state.mpc_states << state.root_euler[0], state.root_euler[1], state.root_euler[2],
            state.root_pos[0], state.root_pos[1], state.root_pos[2],
            state.root_ang_vel[0], state.root_ang_vel[1], state.root_ang_vel[2],
            state.root_lin_vel[0], state.root_lin_vel[1], state.root_lin_vel[2],
            -9.8;

    // previously we use dt passed by outer thread. It turns out that this dt is not stable on hardware.
    // if the thread is slowed down, dt becomes large, then MPC will output very large force and torque value
    // which will cause over current. Here we use a new mpc_dt, this should be roughly close to the average dt
    // of thread 1 
    double mpc_dt = 0.0025;

    // in simulation, use dt has no problem
    if (use_sim_time == "true") {
        mpc_dt = dt;
    }

    // 初始化一个预测界限内的MPC期望状态函数
    state.root_lin_vel_d_world = state.root_rot_mat * state.root_lin_vel_d;
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        state.mpc_states_d.segment(i * 13, 13)
                <<
                state.root_euler_d[0],
                state.root_euler_d[1],
                state.root_euler[2] + state.root_ang_vel_d[2] * mpc_dt * (i + 1),
                state.root_pos[0] + state.root_lin_vel_d_world[0] * mpc_dt * (i + 1),
                state.root_pos[1] + state.root_lin_vel_d_world[1] * mpc_dt * (i + 1),
                state.root_pos_d[2],
                state.root_ang_vel_d[0],
                state.root_ang_vel_d[1],
                state.root_ang_vel_d[2],
                state.root_lin_vel_d_world[0],
                state.root_lin_vel_d_world[1],
                0,
                -9.8;
    }
    auto t1 = std::chrono::high_resolution_clock::now();

    // 计算Ac矩阵，适用于整个参考轨迹
    mpc_solver.calculate_A_mat_c(state.root_euler);
    auto t2 = std::chrono::high_resolution_clock::now();

    // 对参考轨迹上每一个点计算Bc矩阵
    for (int i = 0; i < PLAN_HORIZON; i++) {
        // 计算当前的Bc矩阵
        mpc_solver.calculate_B_mat_c(state.robot_mass,
                                        state.trunk_inertia,
                                        state.root_rot_mat,
                                        state.foot_pos_abs);
    
        // 状态空间离散化, 计算A_d和当前的B_d
        mpc_solver.state_space_discretization(mpc_dt);

        // 存储当前的B_d矩阵
        mpc_solver.B_mat_d_list.block<13, 12>(i * 13, 0) = mpc_solver.B_mat_d;
    }
    auto t3 = std::chrono::high_resolution_clock::now();

    // 计算QP矩阵
    mpc_solver.calculate_qp_mats(state);
    auto t4 = std::chrono::high_resolution_clock::now();

    // 设置OSQP解算器
    if (!solver.isInitialized()) {
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables(NUM_DOF * PLAN_HORIZON);
        solver.data()->setNumberOfConstraints(MPC_CONSTRAINT_DIM * PLAN_HORIZON);
        solver.data()->setLinearConstraintsMatrix(mpc_solver.linear_constraints);
        solver.data()->setHessianMatrix(mpc_solver.hessian);
        solver.data()->setGradient(mpc_solver.gradient);
        solver.data()->setLowerBound(mpc_solver.lb);
        solver.data()->setUpperBound(mpc_solver.ub);
        solver.initSolver();
    } else {
        solver.updateHessianMatrix(mpc_solver.hessian);
        solver.updateGradient(mpc_solver.gradient);
        solver.updateLowerBound(mpc_solver.lb);
        solver.updateUpperBound(mpc_solver.ub);
    }
    auto t5 = std::chrono::high_resolution_clock::now();

    // OSQP计算结果
    solver.solve();
    auto t6 = std::chrono::high_resolution_clock::now();

    // Ac矩阵计算时间
    std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
    // Bc矩阵计算时间
    std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;
    // QP矩阵计算时间
    std::chrono::duration<double, std::milli> ms_double_3 = t4 - t3;
    // OSQP解算器设置时间
    std::chrono::duration<double, std::milli> ms_double_4 = t5 - t4;
    // OSQP解算时间
    std::chrono::duration<double, std::milli> ms_double_5 = t6 - t5;

    Eigen::VectorXd solution = solver.getSolution();

    // 数据没有错误，将机体坐标系中的地面作用力转化为世界坐标系中
    for (int i = 0; i < NUM_LEG; ++i) {
        if (!isnan(solution.segment<3>(i * 3).norm()))
            foot_forces_grf.block<3, 1>(0, i) = state.root_rot_mat.transpose() * solution.segment<3>(i * 3);
    }

    return foot_forces_grf;
}

Eigen::Vector3d RobotControl::compute_walking_surface(CtrlStates &state) {
    Eigen::Matrix<double, NUM_LEG, 3> W;
    Eigen::VectorXd foot_pos_z; // 落足点z坐标
    Eigen::Vector3d a; // 平面方程系数
    Eigen::Vector3d surf_coef; // 行走平面方程z(x, y) = a0 + a1 * x +a2 * y

    // W << 1 p1x p1y
    //      1 p2x p2y  
    //      1 p3x p3y
    //      1 p4x p4y;
    W.block<NUM_LEG, 1>(0, 0).setOnes();
    W.block<NUM_LEG, 2>(0, 1) = state.foot_pos_recent_contact.block<2, NUM_LEG>(0, 0).transpose();

    foot_pos_z.resize(NUM_LEG);
    foot_pos_z = state.foot_pos_recent_contact.block<1, NUM_LEG>(2, 0).transpose();

    a = Utils::pseudo_inverse(W.transpose() * W) * W.transpose() * foot_pos_z;
    // 平面系数向量[a1, a2, -1]
    surf_coef << a[1], a[2], -1;
    return surf_coef;
}