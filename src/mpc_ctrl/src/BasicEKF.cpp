#include "../include/BasicEKF.h"

BasicEKF::BasicEKF() {
    eye3.setIdentity();
    
    // H表示状态空间变量到观测空间变量的映射
    // H <<  -I3    0     I3    0     0     0 
    //       -I3    0     0     I3    0     0
    //       -I3    0     0     0     I3    0
    //       -I3    0     0     0     0     I3
    //        0     I3    0     0     0     0
    //        0     I3    0     0     0     0
    //        0     I3    0     0     0     0
    //        0     I3    0     0     0     0
    //      0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0
    //      0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0
    //      0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0
    //      0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    H.setZero();
    for (int i = 0; i < NUM_LEG; ++i) {
        H.block<3,3>(i * 3, 0) = -eye3;  // 质心与足端位置之差
        H.block<3,3>(i * 3, 6 + i * 3) = eye3;  
        H.block<3,3>(NUM_LEG * 3 + i * 3, 3) = eye3; // 四条腿的速度
        H(NUM_LEG * 6 + i, 6 + i * 3 + 2) = 1;  // 足端高度
    }

    // Q << I3*noise(ep)      0            0             0             0             0
    //           0       I3*noise(ev)      0             0             0             0
    //           0            0       I3*noise(ep1)      0             0             0
    //           0            0            0        I3*noise(ep2)      0             0
    //           0            0            0             0        I3*noise(ep3)      0
    //           0            0            0             0             0        I3*noise(ep4)
    Q.setIdentity();
    Q.block<3,3>(0,0) = PROCESS_NOISE_PIMU * eye3;  // 位置转移噪声
    Q.block<3,3>(3,3) = PROCESS_NOISE_VIMU * eye3;  // 速度转移噪声
    for (int i = 0; i < NUM_LEG; ++i) {
        Q.block<3,3>(6 + i * 3, 6 + i * 3) = PROCESS_NOISE_PFOOT * eye3;  // 足端位置转移噪声
    }

    // R << I12*noise(epfoot)        0               0
    //             0          I12*noise(evfoot)      0
    //             0                 0          I4*noise(z)
    R.setIdentity();
    for (int i = 0; i < NUM_LEG; ++i) {
        R.block<3,3>(i * 3, i * 3) = SENSOR_NOISE_PIMU_REL_FOOT * eye3;  // IMU位置噪声
        R.block<3,3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = SENSOR_NOISE_VIMU_REL_FOOT * eye3;  // IMU速度噪声
        R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = SENSOR_NOISE_ZFOOT;  // 足端高度噪声
    }

    // F << I18
    F.setIdentity();

    // B << 0
    B.setZero();

    assume_flat_ground = true;
}

BasicEKF::BasicEKF(bool assume_flat_ground_):BasicEKF() {
    assume_flat_ground = assume_flat_ground_;
    if (assume_flat_ground == false) {
        // 如果不在平地上行走，四条腿的足端高度全部不可信，将R中对应项设为最大值
        for (int i = 0; i < NUM_LEG; ++i) {
            R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = 1e5;  
        }
    }
}

void BasicEKF::init_state(CtrlStates& state) {
    filter_initialized = true;
    
    // P << 3 * I18
    P.setIdentity();
    P = P * 3;

    x.setZero();
    // 初始化机器人质心位置
    x.segment<3>(0) = Eigen::Vector3d(0, 0, 0.1);
    for (int i = 0; i < NUM_LEG; ++i) {
        // 机体坐标系中足端位置
        Eigen::Vector3d fk_pos = state.foot_pos_rel.block<3, 1>(0, i);
        // 初始化世界坐标系中足端的位置
        x.segment<3>(6 + i * 3) = state.root_rot_mat * fk_pos + x.segment<3>(0);
    }
}

void BasicEKF::update_estimation(CtrlStates& state, double dt) {
    // 更新状态转移矩阵和控制输入矩阵
    // F << I3 I3*dt 0  0  0  0
    //      0   I3   0  0  0  0
    //      0   0    I3 0  0  0 
    //      0   0    0  I3 0  0 
    //      0   0    0  0  I3 0 
    //      0   0    0  0  0  I3
    F.block<3, 3>(0, 3) = dt * eye3;

    // B <<   0
    //      I3*dt
    //        0 
    //        0
    //        0
    //        0
    B.block<3, 3>(3, 0) = dt * eye3;

    // 加速度作为输入变量，为世界坐标系中质心加速度和重力加速度之和
    Eigen::Vector3d u = state.root_rot_mat * state.imu_acc + Eigen::Vector3d(0, 0, -9.81);

    // 接触估计
    if (state.movement_mode == 0) {  
        // 站立状态，四条腿都是触地的
        for (int i = 0; i < NUM_LEG; ++i) 
            estimated_contacts[i] = 1.0;
    } else {  
        // 行走状态，采用接触状态检测算法
        for (int i = 0; i < NUM_LEG; ++i) {
            // to be continued









        }
    }

    // 根据u和dt更新Q
    // Q << I3*dt*noise(ep)/20           0                0              0             0             0
    //             0          9.8*I3*dt*noise(ev)/20      0              0             0             0
    //             0                     0           I3*noise(ep1)       0             0             0
    //             0                     0                0         I3*noise(ep2)      0             0
    //             0                     0                0              0        I3*noise(ep3)      0
    //             0                     0                0              0             0        I3*noise(ep4)
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * eye3;
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.8 / 20.0 * eye3;

    // 对Q和R中没有触地的部分进行更新
    for (int i = 0; i < NUM_LEG; ++i) {
        // 对没有触地的足端，Q中对应足端位置的项设为最大值，代表不可信
        Q.block<3, 3>(6 + i * 3, 6 + i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * dt * PROCESS_NOISE_PFOOT * eye3;  

        // 对没有触地的足端，R中对应质心位置与足端之差的项设为最大值，代表不可信
        R.block<3, 3>(i * 3, i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_PIMU_REL_FOOT * eye3;                     
        // 对没有触地的足端，R中对应足端速度的项设为最大值，代表不可信
        R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_VIMU_REL_FOOT * eye3;   
        if (assume_flat_ground) {
            // 在平地上走的前提下，如果足端没有触地，R中对应该足端高度的项设为最大值，代表不可信
            R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_ZFOOT;      
        }
    }

    // 预测过程
    xbar = F * x + B * u;               // 状态预测方程
    Pbar = F * P * F.transpose() + Q;   // 先验估计状态协方差方程
    yhat = H * xbar;                    // 测量方程

    // 计算观测值
    for (int i=0; i<NUM_LEG; ++i) {
        Eigen::Vector3d fk_pos = state.foot_pos_rel.block<3, 1>(0, i);
        y.block<3, 1>(i * 3, 0) = state.root_rot_mat * fk_pos;  // 质心与足端位置之差
        
        // leg_v = (-J_rf * av - skew(omega) * p_rf);
        Eigen::Vector3d leg_v = -state.foot_vel_rel.block<3, 1>(0, i) - Utils::skew(state.imu_ang_vel) * fk_pos;
        // r((i - 1) * 3 + 1 : (i - 1) * 3 + 3) = body_v - R_er * leg_v;
        // 摆动腿用质心速度，支撑腿用计算出的足端速度
        y.block<3,1>(NUM_LEG * 3 + i * 3, 0) = (1.0 - estimated_contacts[i]) * x.segment<3>(3) +  estimated_contacts[i] * state.root_rot_mat * leg_v;  
        // 摆动腿用计算出的足端高度，支撑腿足端高度设为0
        y(NUM_LEG * 6 + i) = (1.0 - estimated_contacts[i]) * (x(2) + fk_pos(2)) + estimated_contacts[i] * 0; 
    }

    // 更新过程
    S = H * Pbar * H.transpose() + R;
    S = 0.5 * (S + S.transpose());
    error_y = y - yhat;     // 测量误差
    Serror_y = S.fullPivHouseholderQr().solve(error_y); // K = Pbar * H^T * S^-1 * error_y
    x = xbar + Pbar * H.transpose() * Serror_y;         // 测量更新方程
    SH = S.fullPivHouseholderQr().solve(H);
    P = Pbar - Pbar * H.transpose() * SH * Pbar;    // 后验估计状态协方差方程
    P = 0.5 * (P + P.transpose());

    // 减少位置漂移
    if (P.block<2, 2>(0, 0).determinant() > 1e-6) {
        P.block<2, 16>(0, 2).setZero();
        P.block<16, 2>(2, 0).setZero();
        P.block<2, 2>(0, 0) /= 10.0;
    }

    // 最终将估计值返回CtrlStates& state
    for (int i = 0; i < NUM_LEG; ++i) {
        // 当足端力大于等于50N时认为足端触地
        if (estimated_contacts[i] < 0.5) {
            state.estimated_contacts[i] = false;
        } else {
            state.estimated_contacts[i] = true;
        }
    }
    // 设置估计质心位置和速度为估计值
    state.estimated_root_pos = x.segment<3>(0);
    state.estimated_root_vel = x.segment<3>(3);
    // 设置真实质心位置和线速度为估计值
    state.root_pos = x.segment<3>(0);
    state.root_lin_vel = x.segment<3>(3);
}