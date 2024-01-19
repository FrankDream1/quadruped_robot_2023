/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
// #ifdef COMPILE_WITH_ROS

#ifndef IOROS_H
#define IOROS_H

#include "ros/ros.h"
// #include "interface/IOInterface.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <sensor_msgs/Imu.h>
#include <string>
#include <unitree_legged_msgs/upstream.h>
#include <unitree_legged_msgs/downstream.h>

class IOROS {
public:
    IOROS();
    ~IOROS();
    void sendCmd();
private:
    ros::NodeHandle _nm;
    ros::Subscriber _servo_sub[12], _imu_sub,our_imu_sub;
    ros::Subscriber  _our_motor;   //mzx
    ros::Publisher _servo_pub[12];
    ros::Publisher pub_imu;
    //gyk
    ros::Publisher servo_pub;//硬件通讯话题
    unitree_legged_msgs::upstream motor_upstream;
    sensor_msgs::Imu imudata;
    unitree_legged_msgs::LowCmd _lowCmd;
    unitree_legged_msgs::LowState _lowState;
    std::string _robot_name;

    //repeated functions for multi-thread
    void initRecv();
    void initSend();

    //Callback functions for ROS
    void imuCallback(const sensor_msgs::Imu & msg);
    //our upstream callback functions
    void motorDataCallback(const unitree_legged_msgs::downstream& msg);

    void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
    void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
    void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);

    void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
    void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
    void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);

    void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
    void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
    void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);

    void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
    void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
    void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);
};

#endif  // IOROS_H

// #endif  // COMPILE_WITH_ROS