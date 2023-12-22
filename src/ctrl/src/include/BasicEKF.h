#ifndef BASICEKF_H
#define BASICEKF_H

#include "Param.h"
#include "CtrlStates.h"
#include "Utils.h"

// 状态估计参数
#define STATE_SIZE 18                       // 状态变量个数
#define MEAS_SIZE 28                        // 观测变量个数
#define PROCESS_NOISE_PIMU 0.01             // 质心位置的过程噪声
#define PROCESS_NOISE_VIMU 0.01             // 质心速度的过程噪声
#define PROCESS_NOISE_PFOOT 0.01            // 足端位置的过程噪声
#define SENSOR_NOISE_PIMU_REL_FOOT 0.001    // 质心位置的观测噪声
#define SENSOR_NOISE_VIMU_REL_FOOT 0.1      // 质心速度的观测噪声
#define SENSOR_NOISE_ZFOOT 0.001            // 足端高度的观测噪声

// 状态估计仅估计机器人质心位置和速度和足端位置，机器人姿态假设已从IMU中获取精确值(state.root_rot_mat)
class BasicEKF {
public:
    BasicEKF();

    // 如果不是平地，观测噪声协方差R中对应足端高度的项设置为最大值，代表足端高度估计不可信
    BasicEKF(bool assume_flat_ground_);

    // 初始化质心位置和足端位置
    void init_state(CtrlStates& state);

    // 估计下一步长的足端接触状态，质心位置和线速度
    void update_estimation(CtrlStates& state, double dt);

    bool is_inited() {
        return filter_initialized;
    }

private:
    bool filter_initialized = false;

    // 状态变量定义如下
    // 0 1 2  质心位置 
    // 3 4 5  质心速度 
    // 世界坐标系中四条腿足端位置
    // 6 7 8     FL 
    // 9 10 11   FR 
    // 12 13 14  RL 
    // 15 16 17  RR
    Eigen::Matrix<double, STATE_SIZE, 1> x;             // 先验估计状态
    Eigen::Matrix<double, STATE_SIZE, 1> xbar;          // 后验估计状态
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P;    // 先验估计状态协方差
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; // 后验估计状态协方差
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F;    // 状态转移矩阵
    Eigen::Matrix<double, STATE_SIZE, 3> B;             // 控制输入矩阵
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q;    // 状态转移噪声协方差矩阵

    // 观测变量定义如下
    // 世界坐标系中质心与足端位置之差
    // 0 1 2    FL
    // 3 4 5    FR
    // 6 7 8    RL 
    // 9 10 11  RR
    // 世界坐标系中四条腿的速度
    // 12 13 14  FL
    // 15 16 17  FR
    // 18 19 20  RL
    // 21 22 23  RR
    // 24 25 26 27  四条腿足端高度
    Eigen::Matrix<double, MEAS_SIZE, 1> y;              // 观测值
    Eigen::Matrix<double, MEAS_SIZE, 1> yhat;           // 估计观测值
    Eigen::Matrix<double, MEAS_SIZE, 1> error_y;        // 观测误差
    Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y;       // S^-1 * error_y
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> H;     // 观测矩阵
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SH;    // S^-1 * H
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R;      // 观测噪声协方差

    Eigen::Matrix<double, 3, 3> eye3;                   // 3x3单位矩阵
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S;      // 更新协方差

    bool assume_flat_ground = false;

    // 判断足端是否触地
    double estimated_contacts[4];
};

#endif //BASICEKF_H
