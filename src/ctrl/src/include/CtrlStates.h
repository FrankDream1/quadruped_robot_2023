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
	stance_leg_control_type = 1;
	use_terrain_adapt = 1;
	movement_mode = 0;
	counter_per_gait = 120 * 2;
	counter_per_swing = 120;
	counter = 0;
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

	robot_mass = 15.0;
	trunk_inertia << 0.0158533, 0.0, 0.0,
		0.0, 0.0377999, 0.0,
		0.0, 0.0, 0.0456542;
	default_foot_pos << 0.17, 0.17, -0.17, -0.17,
		0.15, -0.15, 0.15, -0.15,
		-0.35, -0.35, -0.35, -0.35;

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
	root_acc.setZero();

	foot_force.setZero();

	joint_pos.setZero();
	joint_vel.setZero();

	walking_surface_height_tmp = 0;
	walking_surface_height = 0;
	walking_surface_fit_count = 0;

	foot_pos_target_world.setZero();
	foot_pos_target_abs.setZero();
	foot_pos_target_rel.setZero();
	foot_pos_start.setZero();
	foot_pos_world.setZero();
	foot_pos_abs.setZero();
	foot_pos_rel.setZero();
	foot_pos_abs_mpc.setZero();
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

	// 初始化参数服务器
    void resetFromROSParam(ros::NodeHandle &_nh) {
	_nh.param("stance_leg_control_type", stance_leg_control_type, 1);
	_nh.param("use_terrain_adapt", use_terrain_adapt, 1);
	
	_nh.param("robot_mass", robot_mass, 13.0);

	double trunk_inertia_xx;
	double trunk_inertia_xy;
	double trunk_inertia_xz;
	double trunk_inertia_yz;
	double trunk_inertia_yy;
	double trunk_inertia_zz;

	_nh.param("trunk_inertia_xx", trunk_inertia_xx, 0.0158533);
	_nh.param("trunk_inertia_xy", trunk_inertia_xy, 0.0);
	_nh.param("trunk_inertia_xz", trunk_inertia_xz, 0.0);
	_nh.param("trunk_inertia_yz", trunk_inertia_yz, 0.0);
	_nh.param("trunk_inertia_yy", trunk_inertia_yy, 0.0377999);
	_nh.param("trunk_inertia_zz", trunk_inertia_zz, 0.0456542);

	trunk_inertia << trunk_inertia_xx, trunk_inertia_xy, trunk_inertia_xz,
		trunk_inertia_xy, trunk_inertia_yy, trunk_inertia_yz,
		trunk_inertia_xz, trunk_inertia_yz, trunk_inertia_zz;

	double default_foot_pos_FL_x;
	double default_foot_pos_FL_y;
	double default_foot_pos_FL_z;

	double default_foot_pos_FR_x;
	double default_foot_pos_FR_y;
	double default_foot_pos_FR_z;

	double default_foot_pos_RL_x;
	double default_foot_pos_RL_y;
	double default_foot_pos_RL_z;

	double default_foot_pos_RR_x;
	double default_foot_pos_RR_y;
	double default_foot_pos_RR_z;

	_nh.param("default_foot_pos_FL_x", default_foot_pos_FL_x, 0.25);
	_nh.param("default_foot_pos_FL_y", default_foot_pos_FL_y, 0.15);
	_nh.param("default_foot_pos_FL_z", default_foot_pos_FL_z, -0.33);

	_nh.param("default_foot_pos_FR_x", default_foot_pos_FR_x, 0.25);
	_nh.param("default_foot_pos_FR_y", default_foot_pos_FR_y, -0.15);
	_nh.param("default_foot_pos_FR_z", default_foot_pos_FR_z, -0.33);

	_nh.param("default_foot_pos_RL_x", default_foot_pos_RL_x, -0.17);
	_nh.param("default_foot_pos_RL_y", default_foot_pos_RL_y, 0.15);
	_nh.param("default_foot_pos_RL_z", default_foot_pos_RL_z, -0.33);

	_nh.param("default_foot_pos_RR_x", default_foot_pos_RR_x, -0.17);
	_nh.param("default_foot_pos_RR_y", default_foot_pos_RR_y, -0.15);
	_nh.param("default_foot_pos_RR_z", default_foot_pos_RR_z, -0.33);

	default_foot_pos << default_foot_pos_FL_x, default_foot_pos_FR_x, default_foot_pos_RL_x, default_foot_pos_RR_x,
		default_foot_pos_FL_y, default_foot_pos_FR_y, default_foot_pos_RL_y, default_foot_pos_RR_y,
		default_foot_pos_FL_z, default_foot_pos_FR_z, default_foot_pos_RL_z, default_foot_pos_RR_z;

	double q_weights_0, q_weights_1, q_weights_2, q_weights_3, q_weights_4, q_weights_5, q_weights_6, q_weights_7, q_weights_8, q_weights_9, q_weights_10, q_weights_11, q_weights_12;

	_nh.param("q_weights_0", q_weights_0, 80.0);
	_nh.param("q_weights_1", q_weights_1, 80.0);
	_nh.param("q_weights_2", q_weights_2, 1.0);

	_nh.param("q_weights_3", q_weights_3, 0.0);
	_nh.param("q_weights_4", q_weights_4, 0.0);
	_nh.param("q_weights_5", q_weights_5, 270.0);

	_nh.param("q_weights_6", q_weights_6, 1.0);
	_nh.param("q_weights_7", q_weights_7, 1.0);
	_nh.param("q_weights_8", q_weights_8, 20.0);

	_nh.param("q_weights_9", q_weights_9, 20.0);
	_nh.param("q_weights_10", q_weights_10, 20.0);
	_nh.param("q_weights_11", q_weights_11, 20.0);

	_nh.param("q_weights_12", q_weights_12, 0.0);

	q_weights << q_weights_0, q_weights_1, q_weights_2,
		q_weights_3, q_weights_4, q_weights_5,
		q_weights_6, q_weights_7, q_weights_8,
		q_weights_9, q_weights_10, q_weights_11,
		q_weights_12;

	double r_weights_0, r_weights_1, r_weights_2, r_weights_3, r_weights_4, r_weights_5, r_weights_6, r_weights_7, r_weights_8, r_weights_9, r_weights_10, r_weights_11;

	_nh.param("r_weights_0", r_weights_0, 1e-5);
	_nh.param("r_weights_1", r_weights_1, 1e-5);
	_nh.param("r_weights_2", r_weights_2, 1e-6);

	_nh.param("r_weights_3", r_weights_3, 1e-5);
	_nh.param("r_weights_4", r_weights_4, 1e-5);
	_nh.param("r_weights_5", r_weights_5, 1e-6);

	_nh.param("r_weights_6", r_weights_6, 1e-5);
	_nh.param("r_weights_7", r_weights_7, 1e-5);
	_nh.param("r_weights_8", r_weights_8, 1e-6);

	_nh.param("r_weights_9", r_weights_9, 1e-5);
	_nh.param("r_weights_10", r_weights_10, 1e-5);
	_nh.param("r_weights_11", r_weights_11, 1e-6);

	r_weights << r_weights_0, r_weights_1, r_weights_2,
		r_weights_3, r_weights_4, r_weights_5,
		r_weights_6, r_weights_7, r_weights_8,
		r_weights_9, r_weights_10, r_weights_11;

	double kp_foot_x, kp_foot_y, kp_foot_z, kd_foot_x, kd_foot_y, kd_foot_z, km_foot_x, km_foot_y, km_foot_z;

	_nh.param("kp_foot_x", kp_foot_x, 150.0);
	_nh.param("kp_foot_y", kp_foot_y, 150.0);
	_nh.param("kp_foot_z", kp_foot_z, 200.0);

	_nh.param("kd_foot_x", kd_foot_x, 0.0);
	_nh.param("kd_foot_y", kd_foot_y, 0.0);
	_nh.param("kd_foot_z", kd_foot_z, 0.0);

	_nh.param("km_foot_x", km_foot_x, 0.1);
	_nh.param("km_foot_y", km_foot_y, 0.1);
	_nh.param("km_foot_z", km_foot_z, 0.04);

	kp_foot <<
		kp_foot_x, kp_foot_x, kp_foot_x, kp_foot_x,
		kp_foot_y, kp_foot_y, kp_foot_y, kp_foot_y,
		kp_foot_z, kp_foot_z, kp_foot_z, kp_foot_z;
	kd_foot <<
		kd_foot_x, kd_foot_x, kd_foot_x, kd_foot_x,
		kd_foot_y, kd_foot_y, kd_foot_y, kd_foot_y,
		kd_foot_z, kd_foot_z, kd_foot_z, kd_foot_z;

	km_foot = Eigen::Vector3d(km_foot_x, km_foot_y, km_foot_z);

	double kp_linear_x;
	double kp_linear_y;
	double kp_linear_z;
	_nh.param("kp_linear_x", kp_linear_x, 120.0);
	_nh.param("kp_linear_y", kp_linear_y, 120.0);
	_nh.param("kp_linear_z", kp_linear_z, 500.0);

	double kd_linear_x;
	double kd_linear_y;
	double kd_linear_z;
	_nh.param("kd_linear_x", kd_linear_x, 70.0);
	_nh.param("kd_linear_y", kd_linear_y, 70.0);
	_nh.param("kd_linear_z", kd_linear_z, 120.0);

	double kp_angular_x;
	double kp_angular_y;
	double kp_angular_z;
	_nh.param("kp_angular_x", kp_angular_x, 250.0);
	_nh.param("kp_angular_y", kp_angular_y, 35.0);
	_nh.param("kp_angular_z", kp_angular_z, 1.0);

	double kd_angular_x;
	double kd_angular_y;
	double kd_angular_z;
	_nh.param("kd_angular_x", kd_angular_x, 1.5);
	_nh.param("kd_angular_y", kd_angular_y, 1.5);
	_nh.param("kd_angular_z", kd_angular_z, 30.0);

	kp_linear_lock_x = kp_linear_x;
	kp_linear_lock_y = kp_linear_y;

	kp_linear = Eigen::Vector3d(kp_linear_x, kp_linear_y, kp_linear_z);
	kd_linear = Eigen::Vector3d(kd_linear_x, kd_linear_y, kd_linear_z);
	kp_angular = Eigen::Vector3d(kp_angular_x, kp_angular_y, kp_angular_z);
	kd_angular = Eigen::Vector3d(kd_angular_x, kd_angular_y, kd_angular_z);

	double gait_counter_speed_FL;
	double gait_counter_speed_FR;
	double gait_counter_speed_RL;
	double gait_counter_speed_RR;

	_nh.param("gait_counter_speed_FL", gait_counter_speed_FL, 2.0);
	_nh.param("gait_counter_speed_FR", gait_counter_speed_FR, 2.0);
	_nh.param("gait_counter_speed_RL", gait_counter_speed_RL, 2.0);
	_nh.param("gait_counter_speed_RR", gait_counter_speed_RR, 2.0);

	gait_counter_speed
		<< gait_counter_speed_FL, gait_counter_speed_FR, gait_counter_speed_RL, gait_counter_speed_RR;
    }

    void gait_counter_reset() {
	if (gait_type == 1) {
	    gait_counter << 0, 120, 120, 0;
	}
    }

    // 行动变量
    int stance_leg_control_type;  // 0代表QP规划, 1代表MPC规划
    int movement_mode;  // 0代表站立, 1代表开始移动
    int use_terrain_adapt;  // 是否使用地形适应
    double control_dt = MAIN_UPDATE_FREQUENCY / 1000.0;

    // 步态变量
    double plan_dt;
    int counter_per_plan; // 每个规划周期计数次数
    double counter_per_gait; // 每个步态周期计数次数
    double counter_per_swing; // 每个摆动周期计数次数
    int counter;
    Eigen::Vector4d gait_counter;
    Eigen::Vector4d gait_counter_speed;

    // 步态类型
    int gait_type;
    int gait_type_last;

    // 期望质心控制量：位置、姿态、速度（机体坐标系和世界坐标系）、角速度（机体坐标系和世界坐标系）
    Eigen::Vector3d root_pos_d;
    Eigen::Vector3d root_euler_d;
    Eigen::Vector3d root_lin_vel_d;
    Eigen::Vector3d root_lin_vel_d_world;
    Eigen::Vector3d root_ang_vel_d;
    Eigen::Vector3d root_ang_vel_d_world;

	// MPC控制状态量
    Eigen::Matrix<double, MPC_STATE_DIM, 1> mpc_states;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> mpc_states_d;

    // 运动学参数
    double robot_mass;
    Eigen::Matrix3d trunk_inertia;
    Eigen::Matrix<double, 3, NUM_LEG> default_foot_pos;

    // MPC权重参数
    Eigen::VectorXd q_weights;
    Eigen::VectorXd r_weights;

    // 地形俯仰角
    double terrain_pitch_angle; 

    // 运动学变量
    Eigen::Vector3d root_pos;
    Eigen::Quaterniond root_quat;
    Eigen::Vector3d root_euler;
    Eigen::Matrix3d root_rot_mat;
    Eigen::Matrix3d root_rot_mat_z;
    Eigen::Vector3d root_lin_vel;
    Eigen::Vector3d root_ang_vel;
    Eigen::Vector3d root_acc;

    Eigen::Vector4d foot_force;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;

    Eigen::Matrix<double, NUM_DOF, 1> joint_pos;
    Eigen::Matrix<double, NUM_DOF, 1> joint_vel;

    double walking_surface_height_tmp;
    double walking_surface_height;
    int walking_surface_fit_count;

    // 期望落足点位置
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_world; // 世界坐标系中
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_abs; // 以质心为原点的世界坐标系中
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_rel; // 机体坐标系中
    
	// 初始落足点位置
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start; 

	// 落足点位置
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_world; // 世界坐标系中
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs; // 以质心为原点的世界坐标系中
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel; // 机体坐标系中
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs_mpc;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_recent_contact;

	// 足部速度
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_world;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_abs;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_rel;
    Eigen::Matrix<double, 12, 12> j_foot;

    bool contacts[NUM_LEG];         // 用来决定腿部运动模式的flag
    bool plan_contacts[NUM_LEG];    // 用来决定腿部期望运动模式的flag
    bool early_contacts[NUM_LEG];   // 摆动过程中碰撞则设为真

    // PD控制系数
    double kp_lin_x;
    double kd_lin_x;
    double kf_lin_x;
    double kp_lin_y;
    double kd_lin_y;
    double kf_lin_y;

    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kp_foot;
    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kd_foot;
    Eigen::Matrix<double, NUM_DOF_PER_LEG, 1> km_foot;

    double kp_linear_lock_x, kp_linear_lock_y;
    Eigen::Vector3d kp_linear;
    Eigen::Vector3d kd_linear;
    Eigen::Vector3d kp_angular;
    Eigen::Vector3d kd_angular;

	// 关节重力补偿
    Eigen::Matrix<double, NUM_DOF, 1> torques_gravity;

	// 关节扭矩
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;

    // IMU数据
    Eigen::Vector3d imu_acc;  // 加速度
    Eigen::Vector3d imu_ang_vel;  // 角速度

    // 状态估计
    bool estimated_contacts[NUM_LEG];  // 估计有接触则设为真
    Eigen::Vector3d estimated_root_pos;  // 估计质心位置
    Eigen::Vector3d estimated_root_vel;  // 估计质心速度
};

#endif //CTRLSTATES_H