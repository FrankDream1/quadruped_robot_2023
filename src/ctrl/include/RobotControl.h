#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <iostream>
#include <string>
#include <chrono>

// to debug
#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>

#include "Param.h"
#include "CtrlStates.h"
#include "Utils.h"
#include "ConvexMpc.h"

#include "filter.hpp"


class RobotControl {
public:
    RobotControl();

    // 更新腿部运动模式和计算落足点位置
    void update_plan(CtrlStates &state, double dt);

    // 计算期望落足点和摆动腿关节力矩
    void generate_swing_legs_ctrl(CtrlStates &state, double dt);

    // 计算关节扭矩
    void compute_joint_torques(CtrlStates &state);

    // 使用MPC计算足端接触力
    Eigen::Matrix<double, 3, NUM_LEG> compute_grf(CtrlStates &state, double dt);

    // 计算行走平面
    Eigen::Vector3d compute_walking_surface(CtrlStates &state);

private:
    // 摆动腿生成贝塞尔曲线
    BezierUtils bezierUtils[NUM_LEG];

    // MPC权重矩阵
    Eigen::DiagonalMatrix<double, 6> Q;
    double R;

    double mu;  // 地面摩擦系数
    double F_min;   // 最小摩擦力
    double F_max;   // 最大摩擦力

    // 设置二次规划中的常数
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // 实例化解算器
    OsqpEigen::Solver solver;

    // ROS debug话题
    ros::NodeHandle nh;
    ros::Publisher pub_foot_start[NUM_LEG];
    ros::Publisher pub_foot_end[NUM_LEG];
    ros::Publisher pub_foot_path[NUM_LEG];
    visualization_msgs::Marker foot_start_marker[NUM_LEG];
    visualization_msgs::Marker foot_end_marker[NUM_LEG];
    visualization_msgs::Marker foot_path_marker[NUM_LEG];

    // debug的话题
    ros::Publisher pub_terrain_angle;

    ros::Publisher pub_foot_pose_target_FL;
    ros::Publisher pub_foot_pose_target_FR;
    ros::Publisher pub_foot_pose_target_RL;
    ros::Publisher pub_foot_pose_target_RR;

    ros::Publisher pub_foot_pose_target_rel_FL;
    ros::Publisher pub_foot_pose_target_rel_FR;
    ros::Publisher pub_foot_pose_target_rel_RL;
    ros::Publisher pub_foot_pose_target_rel_RR;

    ros::Publisher pub_foot_pose_error_FL;
    ros::Publisher pub_foot_pose_error_FR;
    ros::Publisher pub_foot_pose_error_RL;
    ros::Publisher pub_foot_pose_error_RR;

    ros::Publisher pub_euler;

    //MPC最开始10个计数单位不启动
    int mpc_init_counter;

    // 使用仿真时间
    std::string use_sim_time;

    // 对地面角度和接触点滤波
    MovingWindowFilter terrain_angle_filter;
    MovingWindowFilter recent_contact_x_filter[NUM_LEG];
    MovingWindowFilter recent_contact_y_filter[NUM_LEG];
    MovingWindowFilter recent_contact_z_filter[NUM_LEG];
};

#endif //ROBOTCONTROL_H