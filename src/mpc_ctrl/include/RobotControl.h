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

    RobotControl(ros::NodeHandle &_nh);

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
    ros::Publisher pub_terrain_angle;

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