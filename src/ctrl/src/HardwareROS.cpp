#include "../include/HardwareROS.h"

HardwareROS::HardwareROS(ros::NodeHandle &_nh) {
    nh = _nh;

    pub_joint_cmd = nh.advertise<sensor_msgs::JointState>("/dog_hardware/joint_torque_cmd", 100);
    pub_joint_angle = nh.advertise<sensor_msgs::JointState>("/dog_hardware/joint_foot", 100);
    pub_imu = nh.advertise<sensor_msgs::Imu>("/dog_hardware/imu", 100); 

    joint_foot_msg.name = {"FL0", "FL1", "FL2",
                           "FR0", "FR1", "FR2",
                           "RL0", "RL1", "RL2",
                           "RR0", "RR1", "RR2",
                           "FL_foot", "FR_foot", "RL_foot", "RR_foot"};
    joint_foot_msg.position.resize(NUM_DOF + NUM_LEG);
    joint_foot_msg.velocity.resize(NUM_DOF + NUM_LEG);
    joint_foot_msg.effort.resize(NUM_DOF + NUM_LEG);

    pub_estimated_pose = nh.advertise<nav_msgs::Odometry>("/dog_hardware/estimation_body_pose", 100);

    sub_joy_msg = nh.subscribe("/joy", 1000, &HardwareROS::joy_callback, this);

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

    //腿的顺序变换
    swap_joint_indices << 0, 1, 2, 6, 7, 8, 3, 4, 5, 9, 10, 11;
    swap_foot_indices << 1, 0, 3, 2;

    // 启动硬件读取线程
    thread_ = std::thread(&HardwareROS::receive_low_state, this);
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

void HardwareROS::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    // left updown
    joy_cmd_velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

    //A
    if (joy_msg->buttons[0] == 1) {
        joy_cmd_ctrl_state_change_request = true;
    }

    // right updown
    joy_cmd_velx = joy_msg->axes[3] * JOY_CMD_VELX_MAX;
    // right horiz
    joy_cmd_vely = joy_msg->axes[2] * JOY_CMD_VELY_MAX;
    // left horiz
    joy_cmd_yaw_rate  = joy_msg->axes[0] * JOY_CMD_YAW_MAX;
    // cross button, left and right
    joy_cmd_roll_rate = joy_msg->axes[6] * JOY_CMD_ROLL_MAX * (-1);
    // cross button, up and down
    joy_cmd_pitch_rate = joy_msg->axes[7] * JOY_CMD_PITCH_MAX;

    // lb
    if (joy_msg->buttons[4] == 1) {
        std::cout << "You have pressed the exit button!!!!" << std::endl;
        joy_cmd_exit = true;
    }
}

void HardwareROS::receive_low_state() {
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration dt(0);

    while (destruct == false) {

        now = ros::Time::now();
        dt = now - prev;
        prev = now;
        double dt_s = dt.toSec();

        for (int i = 0; i < NUM_DOF; ++i) {
            int swap_i = swap_joint_indices(i);
            dog_ctrl_states.joint_vel[i] = pMotorDriver->MotorData[i].CurVel;
            dog_ctrl_states.joint_pos[i] = pMotorDriver->MotorData[i].CurPos;
        }

        // 发布关节状态
        for (int i = 0; i < NUM_DOF; ++i) {
            joint_foot_msg.position[i] = dog_ctrl_states.joint_pos[i];
            joint_foot_msg.velocity[i] = dog_ctrl_states.joint_vel[i];    
        }
        for (int i = 0; i < NUM_LEG; ++i) {
            joint_foot_msg.velocity[NUM_DOF + i] = dog_ctrl_states.plan_contacts[i]; // 发布规划接触
            joint_foot_msg.effort[NUM_DOF + i] = dog_ctrl_states.foot_force[i];
        }
        joint_foot_msg.header.stamp = ros::Time::now();
        pub_joint_angle.publish(joint_foot_msg);
        
        // 状态估计
        auto t1 = ros::Time::now();
        if (!dog_estimate.is_inited()) {
            dog_estimate.init_state(dog_ctrl_states);
        } else {
            dog_estimate.update_estimation(dog_ctrl_states, dt_s);
        }
        auto t2 = ros::Time::now();
        ros::Duration run_dt = t2 - t1;

        // 使用质心位置和速度估计值计算世界坐标系中的足端位置和速度
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

            dog_ctrl_states.foot_pos_world.block<3, 1>(0, i) =
                    dog_ctrl_states.foot_pos_abs.block<3, 1>(0, i) + dog_ctrl_states.root_pos;
            dog_ctrl_states.foot_vel_world.block<3, 1>(0, i) =
                    dog_ctrl_states.foot_vel_abs.block<3, 1>(0, i) + dog_ctrl_states.root_lin_vel;
        }

        double interval_ms = HARDWARE_FEEDBACK_FREQUENCY;
        double interval_time = interval_ms / 1000.0;
        if (interval_time > run_dt.toSec()) {
            ros::Duration(interval_time - run_dt.toSec()).sleep();
        }
    }
}

void HardwareROS::receive_imu() {
    
}

bool HardwareROS::send_cmd() {
      
}