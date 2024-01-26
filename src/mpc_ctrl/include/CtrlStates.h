#ifndef CTRLSTATES_H
#define CTRLSTATES_H

#include <Eigen/Dense>
#include <ros/ros.h>

#include "Param.h"

class CtrlStates {
public:
    // 初始化
    CtrlStates() {
		reset();
    }

    void reset() {
		movement_mode = 0;
		counter_per_gait = 120 * 2;
		counter_per_swing = 120;
		gait_counter.setZero();
		gait_counter_speed.setZero();
		gait_type = 1;
		gait_type_last = 1;

		// 初始化步态类型
		gait_counter_reset();

		root_pos_d.setZero();
		root_euler_d.setZero();
		root_lin_vel_d.setZero();
		root_ang_vel_d.setZero();

		robot_mass = 20;
		trunk_inertia << 0.0158533, 0.0, 0.0,
			0.0, 0.0377999, 0.0,
			0.0, 0.0, 0.0456542;
		default_foot_pos << 0.20, 0.20, -0.16, -0.16057,
  			0.16, -0.16, 0.16, -0.16,
			-0.035, -0.035, -0.035, -0.035;

		q_weights.resize(13);
		r_weights.resize(12);

		q_weights << 80.0, 80.0, 1.0,
			0.0, 0.0, 270.0,
			1.0, 1.0, 20.0,
			20.0, 20.0, 20.0,
			0.0;
		r_weights << 1e-5, 1e-5, 1e-6,
			1e-5, 1e-5, 1e-6,
			1e-5, 1e-5, 1e-6,
			1e-5, 1e-5, 1e-6;

		root_pos.setZero();
		root_quat.setIdentity();
		root_euler.setZero();
		root_rot_mat.setZero();
		root_rot_mat_z.setZero();
		root_lin_vel.setZero();
		root_ang_vel.setZero();

		foot_force.setZero();

		joint_pos.setZero();
		joint_vel.setZero();

		foot_pos_target_world.setZero();
		foot_pos_target_abs.setZero();
		foot_pos_target_rel.setZero();
		foot_pos_start.setZero();
		foot_pos_world.setZero();
		foot_pos_abs.setZero();
		foot_pos_rel.setZero();
		foot_pos_rel_last_time.setZero();
		foot_pos_target_last_time.setZero();
		foot_pos_cur.setZero();
		foot_pos_recent_contact.setZero();
		foot_vel_world.setZero();
		foot_vel_abs.setZero();
		foot_vel_rel.setZero();
		j_foot.setIdentity();

		for (int i = 0; i < NUM_LEG; ++i) {
			contacts[i] = false;
			plan_contacts[i] = false;
			early_contacts[i] = false;
		}

		gait_counter_speed << 2, 2, 2, 2;

		double kp_foot_x = 300.0;
		double kp_foot_y = 400.0;
		double kp_foot_z = 400.0;

		double kd_foot_x = 8.0;
		double kd_foot_y = 8.0;
		double kd_foot_z = 8.0;

		kp_foot <<
			kp_foot_x, kp_foot_x, kp_foot_x, kp_foot_x,
			kp_foot_y, kp_foot_y, kp_foot_y, kp_foot_y,
			kp_foot_z, kp_foot_z, kp_foot_z, kp_foot_z;
		kd_foot <<
			kd_foot_x, kd_foot_x, kd_foot_x, kd_foot_x,
			kd_foot_y, kd_foot_y, kd_foot_y, kd_foot_y,
			kd_foot_z, kd_foot_z, kd_foot_z, kd_foot_z;

		km_foot = Eigen::Vector3d(0.1, 0.1, 0.1);

		kp_linear = Eigen::Vector3d(1000.0, 1000.0, 1000.0);
		kd_linear = Eigen::Vector3d(200.0, 70.0, 120.0);
		kp_angular = Eigen::Vector3d(650.0, 35.0, 1.0);
		kd_angular = Eigen::Vector3d(4.5, 4.5, 30.0);

		torques_gravity << 0.80, 0, 0, -0.80, 0, 0, 0.80, 0, 0, -0.80, 0, 0;
		joint_torques.setZero();
    }

    void gait_counter_reset() {
		// 对角步态
		if (gait_type == 1) {
			gait_counter << 0, 120, 120, 0;
		}
		// 其他步态
		// to be continued
    }

    int movement_mode;	// 0代表站立，1代表开始移动
    double control_dt = MAIN_UPDATE_FREQUENCY / 1000.0;

    double counter_per_gait; // 步态周期平均计数次数
    double counter_per_swing; // 摆动周期平均计数次数
    int counter;	// 计数周期
	Eigen::Vector4d gait_counter;	// 每个步态周期内计数次数
    Eigen::Vector4d gait_counter_speed;	// 每个步态周期内计数变化率

    int gait_type;	// 步态类型
    int gait_type_last;	// 上阶段的步态类型

    Eigen::Vector3d root_pos_d;	// 期望质心位置	
    Eigen::Vector3d root_euler_d;	// 期望机身姿态
    Eigen::Vector3d root_lin_vel_d; // 期望质心速度
	Eigen::Vector3d root_lin_vel_d_world;	// 期望质心速度（世界坐标系）
    Eigen::Vector3d root_ang_vel_d;	// 期望机体角速度

    Eigen::Matrix<double, MPC_STATE_DIM, 1> mpc_states;	// MPC状态变量
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> mpc_states_d;	// 时间步长内的MPC状态变量

    double robot_mass;	// 机体质量
    Eigen::Matrix3d trunk_inertia;	// 机身转动惯量
    Eigen::Matrix<double, 3, NUM_LEG> default_foot_pos;	// 默认足端位置

    // MPC权重参数
    Eigen::VectorXd q_weights;
    Eigen::VectorXd r_weights;

    double terrain_pitch_angle;	// 地形俯仰角

    Eigen::Vector3d root_pos;	// 机体质心位置
    Eigen::Quaterniond root_quat;	// 机身四元数
    Eigen::Vector3d root_euler;	// 机身姿态角
    Eigen::Matrix3d root_rot_mat;	// 机体坐标系到附体坐标系变换
    Eigen::Matrix3d root_rot_mat_z;	// 机体坐标系到附体坐标系变换（相对z轴）
    Eigen::Vector3d root_lin_vel;	// 质心线速度
    Eigen::Vector3d root_ang_vel;	// 机身角速度

    Eigen::Vector4d foot_force;	// 足端力
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;	// 摆动腿足端残差力 
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;	// 支撑腿足端反作用力

    Eigen::Matrix<double, NUM_DOF, 1> joint_pos;	// 关节位置
    Eigen::Matrix<double, NUM_DOF, 1> joint_vel;	// 关节速度

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_world; // 期望落足点位置（世界坐标系）
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_abs; // 期望落足点位置（附体坐标系）
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_rel; // 期望落足点位置（机体坐标系）
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start;	// 初始落足点位置
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_world;	// 落足点位置（世界坐标系）
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs;	// 落足点位置（附体坐标系）
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel; // 落足点位置（机体坐标系）
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel_last_time;	// 上次的落足点位置（机体坐标系）
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_last_time;	// 上次的期望落足点位置（机体坐标系）
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;	// 当前足端位置
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_recent_contact;	// 最近一次接触地面的足端位置

    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_world;	//足端速度（世界坐标系）
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_abs;	// 足端速度（附体坐标系）
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_rel;	// 足端速度（机体坐标系）
    Eigen::Matrix<double, 12, 12> j_foot;	// 每条腿的雅可比矩阵合并矩阵

    bool contacts[NUM_LEG];	// 用来决定腿部运动模式的flag，1代表接触地面，0代表不接触地面
    bool plan_contacts[NUM_LEG];	// 用来决定腿部期望运动模式的flag，1代表接触地面，0代表不接触地面
    bool early_contacts[NUM_LEG];   // 摆动过程中碰撞则设为真

    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kp_foot;
    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kd_foot;
    Eigen::Matrix<double, NUM_DOF_PER_LEG, 1> km_foot;

    double kp_linear_lock_x, kp_linear_lock_y;
    Eigen::Vector3d kp_linear;
    Eigen::Vector3d kd_linear;
    Eigen::Vector3d kp_angular;
    Eigen::Vector3d kd_angular;

    Eigen::Matrix<double, NUM_DOF, 1> torques_gravity;	// 关节重力补偿

    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;	// 关节扭矩

    Eigen::Vector3d imu_acc;		// IMU加速度
    Eigen::Vector3d imu_ang_vel;	// IMU角速度

    bool estimated_contacts[NUM_LEG];  // 估计有接触则设为真
    Eigen::Vector3d estimated_root_pos;  // 估计质心位置
    Eigen::Vector3d estimated_root_vel;  // 估计质心速度
};

#endif //CTRLSTATES_H