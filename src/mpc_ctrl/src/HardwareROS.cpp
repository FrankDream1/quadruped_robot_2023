#include "../include/HardwareROS.h"

HardwareROS::HardwareROS(ros::NodeHandle &_nh) {
    nh = _nh;

    // 和电机通讯的ROS话题
    motor_cmd = nh.advertise<unitree_legged_msgs::downstream>("/downstream", 100);
    motor_data = nh.subscribe<unitree_legged_msgs::upstream>("/upstream", 1000, &HardwareROS::receive_motor_state, this);
    // 接受IMU数据的ROS话题
    imu_data = nh.subscribe<sensor_msgs::Imu>("/dog_hardware/imu", 1000, &HardwareROS::receive_imu_state, this);
    
    // ！！！！！！！！！！！！！！！！！！！
    usleep(2000000);

    // 接受控制手柄的ROS话题
    sub_joy_msg = nh.subscribe("/joy", 1000, &HardwareROS::joy_callback, this);

    joy_cmd_ctrl_state = 0;
    joy_cmd_ctrl_state_change_request = false;
    prev_joy_cmd_ctrl_state = 0;
    joy_cmd_exit = false;

    _root_control = RobotControl();
    dog_ctrl_states.reset();

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
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.20;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = 0.20;

    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }
}

bool HardwareROS::update_foot_forces_grf(double dt) {
    // 使用MPC控制计算支撑腿足端力
    dog_ctrl_states.foot_forces_grf = _root_control.compute_grf(dog_ctrl_states, dt);
    return true;
}

bool HardwareROS::main_update(double t, double dt) {
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

    // 机体坐标系中的期望质心角速度
    dog_ctrl_states.root_ang_vel_d[0] = joy_cmd_roll_rate;
    dog_ctrl_states.root_ang_vel_d[1] = joy_cmd_pitch_rate;
    dog_ctrl_states.root_ang_vel_d[2] = joy_cmd_yaw_rate;

    // 机体坐标系中的期望姿态
    dog_ctrl_states.root_euler_d[0] = joy_cmd_roll_rate;
    dog_ctrl_states.root_euler_d[1] = joy_cmd_pitch_rate;
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

    return true;
}

void HardwareROS::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    // 左摇杆的上下
    joy_cmd_velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

    // 按钮A
    if (joy_msg->buttons[0] == 1) {
        joy_cmd_ctrl_state_change_request = true;
    }

    // 右摇杆的上下
    joy_cmd_velx = joy_msg->axes[3] * JOY_CMD_VELX_MAX;
    // 右摇杆的左右
    joy_cmd_vely = joy_msg->axes[2] * JOY_CMD_VELY_MAX;
    // 左摇杆的左右
    joy_cmd_yaw_rate  = joy_msg->axes[0] * JOY_CMD_YAW_MAX;
    // 十字按钮的左右
    joy_cmd_roll_rate = joy_msg->axes[6] * JOY_CMD_ROLL_MAX * (-1);
    // 十字按钮的上下
    joy_cmd_pitch_rate = joy_msg->axes[7] * JOY_CMD_PITCH_MAX;

    // 按钮Y
    if (joy_msg->buttons[4] == 1) {
        std::cout << "You have pressed the exit button!!!!" << std::endl;
        joy_cmd_exit = true;
    }
}

bool HardwareROS::send_cmd() {
    // 计算关节力矩
    _root_control.compute_joint_torques(dog_ctrl_states);

    unitree_legged_msgs::downstream motordown;

    // 下发控制命令
    for (int i = 0; i < NUM_DOF; i++) {
        motordown.id[i] = i;        
        motordown.Pos[i] = 0;    // 禁止位置环
        motordown.W[i] = 0;      // 禁止速度环
        motordown.K_P[i] = 0;
        motordown.K_W[i] = 0;
        motordown.T[i] = dog_ctrl_states.joint_torques[i];
    }
    motor_cmd.publish(motordown);

    return true;
}

void HardwareROS::receive_motor_state(const unitree_legged_msgs::upstream::ConstPtr &motorup) {
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration dt(0);

    while (true) {
        // 获取当前时间(s)
        now = ros::Time::now();
        dt = now - prev;
        prev = now;
        double dt_s = dt.toSec();

        // 获取关节状态
        for (int i = 0; i < NUM_DOF; ++i) {
            dog_ctrl_states.joint_vel[i] = motorup->W[i];
            dog_ctrl_states.joint_pos[i] = motorup->Pos[i];
        }

        // 状态估计
        auto t1 = ros::Time::now();
        if (!dog_estimate.is_inited()) {
            dog_estimate.init_state(dog_ctrl_states);
        } else {
            dog_estimate.update_estimation(dog_ctrl_states, dt_s);
        }
        auto t2 = ros::Time::now();
        ros::Duration run_dt = t2 - t1;

        // 使用估计位置和速度来计算世界坐标系中的足端位置和速度
        for (int i = 0; i < NUM_LEG; ++i) {
            dog_ctrl_states.foot_pos_rel.block<3, 1>(0, i) = dog_kin.fk(
                    dog_ctrl_states.joint_pos.segment<3>(3 * i),
                    rho_opt_list[i], rho_fix_list[i]);
            dog_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i) = dog_kin.jac(
                    dog_ctrl_states.joint_pos.segment<3>(3 * i),
                    rho_opt_list[i], rho_fix_list[i]);
            Eigen::Matrix3d tmp_mtx = dog_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i);
            Eigen::Vector3d tmp_vec = dog_ctrl_states.joint_vel.segment<3>(3 * i);
            dog_ctrl_states.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

            dog_ctrl_states.foot_pos_abs.block<3, 1>(0, i) =
                    dog_ctrl_states.root_rot_mat * dog_ctrl_states.foot_pos_rel.block<3, 1>(0, i);
            dog_ctrl_states.foot_vel_abs.block<3, 1>(0, i) =
                    dog_ctrl_states.root_rot_mat * dog_ctrl_states.foot_vel_rel.block<3, 1>(0, i);

            // 世界坐标系中位置为附体坐标系位置加上质心位置
            dog_ctrl_states.foot_pos_world.block<3, 1>(0, i) =
                    dog_ctrl_states.foot_pos_abs.block<3, 1>(0, i) + dog_ctrl_states.root_pos;
            // 世界坐标系中速度为附体坐标系速度加上质心速度
            dog_ctrl_states.foot_vel_world.block<3, 1>(0, i) =
                    dog_ctrl_states.foot_vel_abs.block<3, 1>(0, i) + dog_ctrl_states.root_lin_vel;
        }    

        double interval_ms = HARDWARE_FEEDBACK_FREQUENCY;
        // sleep for interval_ms
        double interval_time = interval_ms / 1000.0;
        if (interval_time > run_dt.toSec()) {
            ros::Duration(interval_time - run_dt.toSec()).sleep();
        }
    };
}

void HardwareROS::receive_imu_state(const sensor_msgs::Imu::ConstPtr &imudata) {
    // 获取IMU数据
    dog_ctrl_states.root_quat = Eigen::Quaterniond(imudata->orientation.w,
                                                   imudata->orientation.x,
                                                   imudata->orientation.y,
                                                   imudata->orientation.z);
    dog_ctrl_states.root_rot_mat = dog_ctrl_states.root_quat.toRotationMatrix();
    dog_ctrl_states.root_euler = Utils::quat_to_euler(dog_ctrl_states.root_quat);
    double yaw_angle = dog_ctrl_states.root_euler[2];

    dog_ctrl_states.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    dog_ctrl_states.imu_acc = Eigen::Vector3d(imudata->linear_acceleration.x, imudata->linear_acceleration.y, imudata->linear_acceleration.z);
    dog_ctrl_states.imu_ang_vel = Eigen::Vector3d(imudata->angular_velocity.x, imudata->angular_velocity.y, imudata->angular_velocity.z);
    dog_ctrl_states.root_ang_vel = dog_ctrl_states.root_rot_mat * dog_ctrl_states.imu_ang_vel;
}