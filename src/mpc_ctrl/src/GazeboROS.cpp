#include "../include/GazeboROS.h"

GazeboROS::GazeboROS(ros::NodeHandle &_nh) {
    nh = _nh;

    // ROS publisher
    pub_joint_cmd[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/dog_gazebo/FL_hip_joint_position_controller/command", 1);
    pub_joint_cmd[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/dog_gazebo/FL_thigh_joint_position_controller/command", 1);
    pub_joint_cmd[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/dog_gazebo/FL_calf_joint_position_controller/command", 1);

    pub_joint_cmd[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/dog_gazebo/FR_hip_joint_position_controller/command", 1);
    pub_joint_cmd[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/dog_gazebo/FR_thigh_cjoint_position_ontroller/command", 1);
    pub_joint_cmd[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/dog_gazebo/FR_calf_joint_position_controller/command", 1);

    pub_joint_cmd[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/dog_gazebo/RL_hip_joint_position_controller/command", 1);
    pub_joint_cmd[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/dog_gazebo/RL_thigh_joint_position_controller/command", 1);
    pub_joint_cmd[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/dog_gazebo/RL_calf_joint_position_controller/command", 1);

    pub_joint_cmd[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/dog_gazebo/RR_hip_joint_position_controller/command", 1);
    pub_joint_cmd[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("/dog_gazebo/RR_thigh_joint_position_controller/command", 1);
    pub_joint_cmd[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("/dog_gazebo/RR_calf_joint_position_controller/command", 1);

    // debug estimation
    pub_estimated_pose = nh.advertise<nav_msgs::Odometry>("/dog_gazebo/estimation_body_pose", 100);
    pub_euler_d = nh.advertise<geometry_msgs::PointStamped>("dog_gazebo_debug/euler_d", 100);

    sub_gt_pose_msg = nh.subscribe("/torso_odom", 100, &GazeboROS::gt_pose_callback, this);
    sub_imu_msg = nh.subscribe("/trunk_imu", 100, &GazeboROS::imu_callback, this);

    sub_joint_msg[0] = nh.subscribe("/dog_gazebo/FL_hip_joint_position_controller/state", 2, &GazeboROS::FL_hip_state_callback, this);
    sub_joint_msg[1] = nh.subscribe("/dog_gazebo/FL_thigh_joint_position_controller/state", 2, &GazeboROS::FL_thigh_state_callback, this);
    sub_joint_msg[2] = nh.subscribe("/dog_gazebo/FL_calf_joint_position_controller/state", 2, &GazeboROS::FL_calf_state_callback, this);

    sub_joint_msg[3] = nh.subscribe("/dog_gazebo/FR_hip_joint_position_controller/state", 2, &GazeboROS::FR_hip_state_callback, this);
    sub_joint_msg[4] = nh.subscribe("/dog_gazebo/FR_thigh_joint_position_controller/state", 2, &GazeboROS::FR_thigh_state_callback, this);
    sub_joint_msg[5] = nh.subscribe("/dog_gazebo/FR_calf_joint_position_controller/state", 2, &GazeboROS::FR_calf_state_callback, this);

    sub_joint_msg[6] = nh.subscribe("/dog_gazebo/RL_hip_joint_position_controller/state", 2, &GazeboROS::RL_hip_state_callback, this);
    sub_joint_msg[7] = nh.subscribe("/dog_gazebo/RL_thigh_joint_position_controller/state", 2, &GazeboROS::RL_thigh_state_callback, this);
    sub_joint_msg[8] = nh.subscribe("/dog_gazebo/RL_calf_joint_position_controller/state", 2, &GazeboROS::RL_calf_state_callback, this);

    sub_joint_msg[9] = nh.subscribe("/dog_gazebo/RR_hip_joint_position_controller/state", 2, &GazeboROS::RR_hip_state_callback, this);
    sub_joint_msg[10] = nh.subscribe("/dog_gazebo/RR_thigh_joint_position_controller/state", 2, &GazeboROS::RR_thigh_state_callback, this);
    sub_joint_msg[11] = nh.subscribe("/dog_gazebo/RR_calf_joint_position_controller/state", 2, &GazeboROS::RR_calf_state_callback, this);

    sub_foot_contact_msg[0] = nh.subscribe("/visual/FL_foot_contact/the_force", 2, &GazeboROS::FL_foot_contact_callback, this);
    sub_foot_contact_msg[1] = nh.subscribe("/visual/FR_foot_contact/the_force", 2, &GazeboROS::FR_foot_contact_callback, this);
    sub_foot_contact_msg[2] = nh.subscribe("/visual/RL_foot_contact/the_force", 2, &GazeboROS::RL_foot_contact_callback, this);
    sub_foot_contact_msg[3] = nh.subscribe("/visual/RR_foot_contact/the_force", 2, &GazeboROS::RR_foot_contact_callback, this);

    sub_joy_msg = nh.subscribe("/joy", 1000, &GazeboROS::joy_callback, this);

    joy_cmd_ctrl_state = 0;
    joy_cmd_ctrl_state_change_request = false;
    prev_joy_cmd_ctrl_state = 0;
    joy_cmd_exit = false;

    _root_control = RobotControl(nh);
    dog_ctrl_states.reset();
    dog_ctrl_states.resetFromROSParam(nh);

    // 初始化腿部运动学参数
    p_br = Eigen::Vector3d(-0.2293, 0.0, -0.067);
    R_br = Eigen::Matrix3d::Identity();

    // 腿的顺序: 0-FL  1-FR  2-RL  3-RR
    leg_offset_x[0] = 0.1805;
    leg_offset_x[1] = 0.1805;
    leg_offset_x[2] = -0.1805;
    leg_offset_x[3] = -0.1805;
    leg_offset_y[0] = 0.047;
    leg_offset_y[1] = -0.047;
    leg_offset_y[2] = 0.047;
    leg_offset_y[3] = -0.047;
    motor_offset[0] = 0.0838;
    motor_offset[1] = -0.0838;
    motor_offset[2] = 0.0838;
    motor_offset[3] = -0.0838;
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.21;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = LOWER_LEG_LENGTH;

    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }

    acc_x = MovingWindowFilter(5);
    acc_y = MovingWindowFilter(5);
    acc_z = MovingWindowFilter(5);
    gyro_x = MovingWindowFilter(5);
    gyro_y = MovingWindowFilter(5);
    gyro_z = MovingWindowFilter(5);
    quat_w = MovingWindowFilter(5);
    quat_x = MovingWindowFilter(5);
    quat_y = MovingWindowFilter(5);
    quat_z = MovingWindowFilter(5);
}

bool GazeboROS::update_foot_forces_grf(double dt) {
    // 使用MPC控制计算接触腿足端力
    dog_ctrl_states.foot_forces_grf = _root_control.compute_grf(dog_ctrl_states, dt);
    return true;
}

bool GazeboROS::main_update(double t, double dt) {
    if (joy_cmd_exit) {
        return false;
    }

    joy_cmd_body_height += joy_cmd_velz * dt;
    if (joy_cmd_body_height >= JOY_CMD_BODY_HEIGHT_MAX) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MAX;
    }
    if (joy_cmd_body_height <= JOY_CMD_BODY_HEIGHT_MIN) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MIN;
    }
    
    prev_joy_cmd_ctrl_state = joy_cmd_ctrl_state;

    if (joy_cmd_ctrl_state_change_request) {
        // 切换控制状态，0变成1，1变成0
        joy_cmd_ctrl_state = joy_cmd_ctrl_state + 1;
        joy_cmd_ctrl_state = joy_cmd_ctrl_state % 2;
        joy_cmd_ctrl_state_change_request = false; 
    }

    // 机体坐标系中的期望质心线速度
    dog_ctrl_states.root_lin_vel_d[0] = joy_cmd_velx;
    dog_ctrl_states.root_lin_vel_d[1] = joy_cmd_vely;
    dog_ctrl_states.root_lin_vel_d[2] = joy_cmd_velz;

    // 机体坐标系中的期望质心角速度
    dog_ctrl_states.root_ang_vel_d[0] = joy_cmd_roll_rate;
    dog_ctrl_states.root_ang_vel_d[1] = joy_cmd_pitch_rate;
    dog_ctrl_states.root_ang_vel_d[2] = joy_cmd_yaw_rate;

    // 机体坐标系中的期望姿态
    dog_ctrl_states.root_euler_d[0] += joy_cmd_roll_rate * dt;
    dog_ctrl_states.root_euler_d[1] += joy_cmd_pitch_rate * dt;
    dog_ctrl_states.root_euler_d[2] += joy_cmd_yaw_rate * dt;

    // 机体坐标系中质心高度
    dog_ctrl_states.root_pos_d[2] = joy_cmd_body_height;

    // 运动模式切换
    if (joy_cmd_ctrl_state == 1) {
        // 转换到行走模式
        dog_ctrl_states.movement_mode = 1;
    } else if (joy_cmd_ctrl_state == 0 && prev_joy_cmd_ctrl_state == 1) {
        // 退出行走模式
        dog_ctrl_states.movement_mode = 0;

        // 锁存当前位置和位置增益
        dog_ctrl_states.root_pos_d.segment<2>(0) = dog_ctrl_states.root_pos.segment<2>(0);
        dog_ctrl_states.kp_linear(0) = dog_ctrl_states.kp_linear_lock_x;
        dog_ctrl_states.kp_linear(1) = dog_ctrl_states.kp_linear_lock_y;
    } else {
        // 保持站立模式
        dog_ctrl_states.movement_mode = 0;
    }

    // 在行走模式中，如果没有期望速度输入时对位置进行锁存
    if (dog_ctrl_states.movement_mode == 1) {
        if (dog_ctrl_states.root_lin_vel_d.segment<2>(0).norm() > 0.05) {
            // 此时期望速度不为零，保持更新x和y方向的位置目标
            dog_ctrl_states.root_pos_d.segment<2>(0) = dog_ctrl_states.root_pos.segment<2>(0);
            dog_ctrl_states.kp_linear.segment<2>(0).setZero();
        } else {
            // 没有新的期望速度输入，使用锁存的数据
            dog_ctrl_states.kp_linear(0) = dog_ctrl_states.kp_linear_lock_x;
            dog_ctrl_states.kp_linear(1) = dog_ctrl_states.kp_linear_lock_y;
        }
    }

    // 更新运动状态和足端规划
    _root_control.update_plan(dog_ctrl_states, dt);

    // 根据计算落足点生成摆动腿曲线
    _root_control.generate_swing_legs_ctrl(dog_ctrl_states, dt);

    // 对质心位置和速度估计
    if (!dog_estimate.is_inited()) {
        dog_estimate.init_state(dog_ctrl_states);
    } else {
        dog_estimate.update_estimation(dog_ctrl_states, dt);
    }

    // 发布估计值
    nav_msgs::Odometry estimate_odom;
    estimate_odom.pose.pose.position.x = dog_ctrl_states.estimated_root_pos(0);
    estimate_odom.pose.pose.position.y = dog_ctrl_states.estimated_root_pos(1);
    estimate_odom.pose.pose.position.z = dog_ctrl_states.estimated_root_pos(2);
    estimate_odom.twist.twist.linear.x = dog_ctrl_states.estimated_root_vel(0);
    estimate_odom.twist.twist.linear.y = dog_ctrl_states.estimated_root_vel(1);
    estimate_odom.twist.twist.linear.z = dog_ctrl_states.estimated_root_vel(2);

    pub_estimated_pose.publish(estimate_odom);

    return true;
}

bool GazeboROS::send_cmd() {
    // 计算关节力矩，将关节力矩作为发送命令
    _root_control.compute_joint_torques(dog_ctrl_states);

    unitree_legged_msgs::LowCmd low_cmd;

    // 使用low_cmd消息来发送命令
    for (int i = 0; i < 12; i++) {
        low_cmd.motorCmd[i].mode = 0x0A;
        low_cmd.motorCmd[i].q = 0;
        low_cmd.motorCmd[i].dq = 0;
        low_cmd.motorCmd[i].Kp = 0;
        low_cmd.motorCmd[i].Kd = 0;
        low_cmd.motorCmd[i].tau = dog_ctrl_states.joint_torques(i, 0);
        pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
    }

    return true;
}

void GazeboROS::gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom) {
    // 姿态更新
    dog_ctrl_states.root_quat = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                  odom->pose.pose.orientation.x,
                                                  odom->pose.pose.orientation.y,
                                                  odom->pose.pose.orientation.z);                                              

    // 四元数转化为欧拉角
    dog_ctrl_states.root_rot_mat = dog_ctrl_states.root_quat.toRotationMatrix();
    dog_ctrl_states.root_euler = Utils::quat_to_euler(dog_ctrl_states.root_quat);
    double yaw_angle = dog_ctrl_states.root_euler[2];
    dog_ctrl_states.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    // 足端位置和速度信息更新
    for (int i = 0; i < NUM_LEG; ++i) {
        // 计算机体坐标系中足端位置
        dog_ctrl_states.foot_pos_rel.block<3, 1>(0, i) = dog_kin.fk(
                dog_ctrl_states.joint_pos.segment<3>(3 * i),
                rho_opt_list[i], rho_fix_list[i]);

        // 计算腿部雅可比矩阵
        dog_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i) = dog_kin.jac(
                dog_ctrl_states.joint_pos.segment<3>(3 * i),
                rho_opt_list[i], rho_fix_list[i]);
        
        // 计算足端速度
        Eigen::Matrix3d tmp_mtx = dog_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i);
        Eigen::Vector3d tmp_vec = dog_ctrl_states.joint_vel.segment<3>(3 * i);
        dog_ctrl_states.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

        // 以质心为原点的世界坐标系的足端位置和速度
        dog_ctrl_states.foot_pos_abs.block<3, 1>(0, i) = dog_ctrl_states.root_rot_mat * dog_ctrl_states.foot_pos_rel.block<3, 1>(0, i);
        dog_ctrl_states.foot_vel_abs.block<3, 1>(0, i) = dog_ctrl_states.root_rot_mat * dog_ctrl_states.foot_vel_rel.block<3, 1>(0, i);

        // 世界坐标系中足端位置和速度
        dog_ctrl_states.foot_pos_world.block<3, 1>(0, i) = dog_ctrl_states.foot_pos_abs.block<3, 1>(0, i) + dog_ctrl_states.root_pos;
        dog_ctrl_states.foot_vel_world.block<3, 1>(0, i) = dog_ctrl_states.foot_vel_abs.block<3, 1>(0, i) + dog_ctrl_states.root_lin_vel;
    }
}

void GazeboROS::imu_callback(const sensor_msgs::Imu::ConstPtr &imu) {
    // IMU返回的姿态四元数
    dog_ctrl_states.root_quat = Eigen::Quaterniond(
            quat_w.CalculateAverage(imu->orientation.w),
            quat_x.CalculateAverage(imu->orientation.x),
            quat_y.CalculateAverage(imu->orientation.y),
            quat_z.CalculateAverage(imu->orientation.z)
    );

    // IMU返回的线速度
    dog_ctrl_states.imu_acc = Eigen::Vector3d(
            acc_x.CalculateAverage(imu->linear_acceleration.x),
            acc_y.CalculateAverage(imu->linear_acceleration.y),
            acc_z.CalculateAverage(imu->linear_acceleration.z)
    );

    // IMU返回的角速度
    dog_ctrl_states.imu_ang_vel = Eigen::Vector3d(
            gyro_x.CalculateAverage(imu->angular_velocity.x),
            gyro_y.CalculateAverage(imu->angular_velocity.y),
            gyro_z.CalculateAverage(imu->angular_velocity.z)
    );

    // 世界坐标系中的角速度
    dog_ctrl_states.root_ang_vel = dog_ctrl_states.root_rot_mat * dog_ctrl_states.imu_ang_vel;
}

// FL
void GazeboROS::FL_hip_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state) {
    dog_ctrl_states.joint_pos[0] = dog_joint_state.q;
    dog_ctrl_states.joint_vel[0] = dog_joint_state.dq;
}

void GazeboROS::FL_thigh_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state) {
    dog_ctrl_states.joint_pos[1] = dog_joint_state.q;
    dog_ctrl_states.joint_vel[1] = dog_joint_state.dq;
}

void GazeboROS::FL_calf_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state) {
    dog_ctrl_states.joint_pos[2] = dog_joint_state.q;
    dog_ctrl_states.joint_vel[2] = dog_joint_state.dq;
}

// FR
void GazeboROS::FR_hip_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state) {
    dog_ctrl_states.joint_pos[3] = dog_joint_state.q;
    dog_ctrl_states.joint_vel[3] = dog_joint_state.dq;
}

void GazeboROS::FR_thigh_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state) {
    dog_ctrl_states.joint_pos[4] = dog_joint_state.q;
    dog_ctrl_states.joint_vel[4] = dog_joint_state.dq;
}

void GazeboROS::FR_calf_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state) {
    dog_ctrl_states.joint_pos[5] = dog_joint_state.q;
    dog_ctrl_states.joint_vel[5] = dog_joint_state.dq;
}

// RL
void GazeboROS::RL_hip_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state) {
    dog_ctrl_states.joint_pos[6] = dog_joint_state.q;
    dog_ctrl_states.joint_vel[6] = dog_joint_state.dq;
}

void GazeboROS::RL_thigh_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state) {
    dog_ctrl_states.joint_pos[7] = dog_joint_state.q;
    dog_ctrl_states.joint_vel[7] = dog_joint_state.dq;
}

void GazeboROS::RL_calf_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state) {
    dog_ctrl_states.joint_pos[8] = dog_joint_state.q;
    dog_ctrl_states.joint_vel[8] = dog_joint_state.dq;
}

// RR
void GazeboROS::RR_hip_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state) {
    dog_ctrl_states.joint_pos[9] = dog_joint_state.q;
    dog_ctrl_states.joint_vel[9] = dog_joint_state.dq;
}

void GazeboROS::RR_thigh_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state) {
    dog_ctrl_states.joint_pos[10] = dog_joint_state.q;
    dog_ctrl_states.joint_vel[10] = dog_joint_state.dq;
}

void GazeboROS::RR_calf_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state) {
    dog_ctrl_states.joint_pos[11] = dog_joint_state.q;
    dog_ctrl_states.joint_vel[11] = dog_joint_state.dq;
}

// 足端力
void GazeboROS::FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    dog_ctrl_states.foot_force[0] = force.wrench.force.z;
}

void GazeboROS::FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    dog_ctrl_states.foot_force[1] = force.wrench.force.z;
}

void GazeboROS::RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    dog_ctrl_states.foot_force[2] = force.wrench.force.z;
}

void GazeboROS::RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    dog_ctrl_states.foot_force[3] = force.wrench.force.z;
}

void GazeboROS::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    // left updown
    joy_cmd_velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

    //A
    if (joy_msg->buttons[0] == 1) {
        joy_cmd_ctrl_state_change_request = true;
    }

    // right updown
    joy_cmd_velx = joy_msg->axes[5] * JOY_CMD_VELX_MAX;
    // right horiz
    joy_cmd_vely = joy_msg->axes[2] * JOY_CMD_VELY_MAX;
    // left horiz
    joy_cmd_yaw_rate = joy_msg->axes[0] * JOY_CMD_YAW_MAX;
    // up-down button
    joy_cmd_pitch_rate = joy_msg->axes[7] * JOY_CMD_PITCH_MAX;
    // left-right button
    joy_cmd_roll_rate = joy_msg->axes[6] * JOY_CMD_ROLL_MAX;

    // lb
    if (joy_msg->buttons[4] == 1) {
        std::cout << "You have pressed the exit button!!!!" << std::endl;
        joy_cmd_exit = true;
    }
}