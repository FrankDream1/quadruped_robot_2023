#ifndef GAZEBOROS_H
#define GAZEBOROS_H

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
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowCmd.h>

// control parameters
#include "Param.h"
#include "CtrlStates.h"
#include "RobotControl.h"
#include "BasicEKF.h"
#include "Kinematics.h"
#include "Utils.h"

#include "filter.hpp"

class GazeboROS {
public:
    GazeboROS(ros::NodeHandle &_nh);

    // 更新接触腿的足端力
    bool update_foot_forces_grf(double dt);

    // 主要状态更新
    bool main_update(double t, double dt);

    // 发送关节力矩的控制命令
    bool send_cmd();

    // 里程计数据(姿态和足端)信息回调函数
    void gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom);

    // IMU数据回调函数
    void imu_callback(const sensor_msgs::Imu::ConstPtr &imu);

    // 操作杆控制信息回调函数
    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

    // 腿部关节状态回调函数
    void FL_hip_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state);
    void FL_thigh_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state);
    void FL_calf_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state);
    void FR_hip_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state);
    void FR_thigh_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state);
    void FR_calf_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state);
    void RL_hip_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state);
    void RL_thigh_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state);
    void RL_calf_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state);
    void RR_hip_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state);
    void RR_thigh_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state);
    void RR_calf_state_callback(const unitree_legged_msgs::MotorState &dog_joint_state);
    void FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);
    void FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);
    void RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);
    void RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

private:
    ros::NodeHandle nh;

    // 0,  1,  2: FL_hip, FL_thigh, FL_calf
    // 3,  4,  5: FR_hip, FR_thigh, FR_calf
    // 6,  7,  8: RL_hip, RL_thigh, RL_calf
    // 9, 10, 11: RR_hip, RR_thigh, RR_calf
    ros::Publisher pub_joint_cmd[12];
    ros::Subscriber sub_joint_msg[12];
    ros::Publisher pub_euler_d;

    // 0, 1, 2, 3: FL, FR, RL, RR
    ros::Subscriber sub_foot_contact_msg[4];
    ros::Subscriber sub_gt_pose_msg;
    ros::Subscriber sub_imu_msg;
    ros::Subscriber sub_joy_msg;

    // 估计状态发布者
    ros::Publisher pub_estimated_pose;

    // 操作杆命令
    double joy_cmd_velx = 0.0;
    double joy_cmd_vely = 0.0;
    double joy_cmd_velz = 0.0;
    double joy_cmd_pitch_rate = 0.0;
    double joy_cmd_roll_rate = 0.0;
    double joy_cmd_yaw_rate = 0.0;
    double joy_cmd_body_height = 0.3;

    int joy_cmd_ctrl_state = 0;  // 操作杆控杆变量，0代表站立, 1代表行走
    bool joy_cmd_ctrl_state_change_request = false;
    int prev_joy_cmd_ctrl_state = 0;
    bool joy_cmd_exit = false;

    Eigen::Vector3d p_br;   // 机体坐标系(r)到世界坐标系(b)平移向量
    Eigen::Matrix3d R_br;   // 机体坐标系(r)到世界坐标系(b)旋转矩阵

    double leg_offset_x[4] = {};    // 腿部偏置：hip关节相对于质心
    double leg_offset_y[4] = {};    // 腿部偏置：hip关节相对于质心
    double motor_offset[4] = {};    // 电机偏置
    double upper_leg_length[4] = {};    // 大腿长度列表 
    double lower_leg_length[4] = {};    // 小腿长度列表

    std::vector<Eigen::VectorXd> rho_fix_list;  // 机体偏置x和y, thigh关节偏置, 大腿长度, 小腿长度
    std::vector<Eigen::VectorXd> rho_opt_list;  // 接触的偏置cx, cy, cz

    Kinematics dog_kin; // 运动学变量
    
    CtrlStates dog_ctrl_states; // 控制状态变量，包含机器狗运动的状态
   
    RobotControl _root_control; // 控制过程变量，包含运动过程所需的函数

    BasicEKF dog_estimate;  // 状态估计变量

    // 移动窗口滤波后的运动量
    MovingWindowFilter acc_x;  // 加速度
    MovingWindowFilter acc_y;
    MovingWindowFilter acc_z;
    MovingWindowFilter gyro_x;  // 陀螺仪
    MovingWindowFilter gyro_y;
    MovingWindowFilter gyro_z;
    MovingWindowFilter quat_w;  // 四元数
    MovingWindowFilter quat_x;
    MovingWindowFilter quat_y;
    MovingWindowFilter quat_z;
};


#endif //GAZEBOROS_H