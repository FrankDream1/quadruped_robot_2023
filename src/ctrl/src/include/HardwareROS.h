#ifndef HARDWAREROS_H
#define HARDWAREROS_H

// std
#include <Eigen/Dense>
#include <memory>
#include <set>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>

// control parameters
#include "Param.h"
#include "CtrlStates.h"
#include "RobotControl.h"
#include "BasicEKF.h"
#include "Kinematics.h"
#include "Utils.h"

#define FOOT_FILTER_WINDOW_SIZE 5

class HardwareROS {
public:
    HardwareROS(ros::NodeHandle &_nh);

    ~HardwareROS() {}

    // 更新接触腿的足端力
    bool update_foot_forces_grf(double dt);

    // 主要状态更新
    bool main_update(double t, double dt);

    // 操作杆控制信息回调函数
    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

    // 下位机向上位机发送反馈信息
    void receive_low_state();

    // 上位机向下位机发送控制指令
    bool send_cmd();

private:
    ros::NodeHandle nh;

    ros::Publisher pub_joint_cmd; // 关节力矩命令
    ros::Publisher pub_joint_angle; // 关节角度和足端位置
    ros::Publisher pub_imu; // IMU数据

    sensor_msgs::JointState joint_foot_msg;
    sensor_msgs::Imu imu_msg;
    
    ros::Subscriber sub_joy_msg;
    ros::Subscriber sub_downstream; // 订阅

    // 估计状态发布者
    ros::Publisher pub_estimated_pose;

    // 电机数据存储指针
    static UnitreeDriver *pMotorDriver;

    // 硬件读取指针
    std::thread thread_;
    bool destruct = false;    

    // 腿的顺序
    Eigen::Matrix<int, NUM_DOF, 1> swap_joint_indices;
    Eigen::Matrix<int, NUM_LEG, 1> swap_foot_indices;

    // dog hardware foot force filter
    Eigen::Matrix<double, NUM_LEG, FOOT_FILTER_WINDOW_SIZE> foot_force_filters;
    Eigen::Matrix<int, NUM_LEG, 1> foot_force_filters_idx;
    Eigen::Matrix<double, NUM_LEG, 1> foot_force_filters_sum;

    // 操作杆命令
    double joy_cmd_velx = 0.0;
    double joy_cmd_vely = 0.0;
    double joy_cmd_velz = 0.0;
    double joy_cmd_roll_rate = 0.0;
    double joy_cmd_pitch_rate = 0.0;
    double joy_cmd_yaw_rate = 0.0;
    double joy_cmd_pitch_ang = 0.0;
    double joy_cmd_roll_ang = 0.0;
    double joy_cmd_body_height = 0.12;

    int joy_cmd_ctrl_state = 0; // 操作杆控杆变量，0代表站立, 1代表行走
    bool joy_cmd_ctrl_state_change_request = false;
    int prev_joy_cmd_ctrl_state = 0;
    bool joy_cmd_exit = false;

    // 机体坐标系(r)到世界坐标系(b)平移向量
    Eigen::Vector3d p_br;
    // 机体坐标系(r)到世界坐标系(b)旋转矩阵
    Eigen::Matrix3d R_br;

    // 腿部偏置：hip关节相对于质心
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    // 电机偏置
    double motor_offset[4] = {};
    // 大腿和小腿长度列表
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};
    // 运动学常数列表
    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;

    // 正运动学变量
    Kinematics dog_kin;

    // 控制状态变量，包含机器狗运动的状态
    CtrlStates dog_ctrl_states;

    // 控制过程变量，包含运动过程所需的函数
    RobotControl _root_control;

    // 状态估计变量
    BasicEKF dog_estimate;
};

#endif //HARDWAREROS_H
