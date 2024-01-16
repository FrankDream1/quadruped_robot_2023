#include "ros/ros.h"
#include "serialPort/SerialPort.h"
#include <unistd.h>
// #include <unitree_legged_msgs/motordata.h>
// #include <unitree_legged_msgs/motorcmd.h>
#include <unitree_legged_msgs/downstream.h>
#include <unitree_legged_msgs/upstream.h>
#include "yaml-cpp/yaml.h"

#define SENDRATE 1000   // 节点通过串口下发的频率
MotorCmd cmd[12];       // 12个电机的控制命令
MotorData data[12];     // 12个电机的返回数据
double derta_pos[12] = {0};
double kp[12];
int stand_time = 200;  // 站立所用时间，单位0.1s
double tt;

void motorCallback(const unitree_legged_msgs::upstream::ConstPtr& motor_cmd);

int main(int argc, char** argv) {
  	ros::init(argc, argv, "standup");
  	ros::NodeHandle nh;
  	ros::Rate r(5);

  //ros::Publisher motorctrl = nh.advertise<unitree_legged_msgs::motorcmd>("/dog_hardware/motorcmd", 100);
    // ros::Publisher motorctrl = nh.advertise<unitree_legged_msgs::motorcmd>("/downstream", 100);
  	// ros::Subscriber motorcmd = nh.subscribe<unitree_legged_msgs::motordata>("/upstream", 100, motorCallback);
    //ros::Subscriber motorcmd = nh.subscribe<unitree_legged_msgs::motordata>("/dog_hardware/motordata", 100, motorCallback);

    ros::Publisher motorctrl = nh.advertise<unitree_legged_msgs::downstream>("/downstream", 100);
  	ros::Subscriber motorcmd = nh.subscribe<unitree_legged_msgs::upstream>("/upstream", 100, motorCallback);
    // unitree_legged_msgs::motorcmd motcmd;
    unitree_legged_msgs::downstream motcmd;

    ros::Rate loop_rate(SENDRATE);

    // YAML::Node joint = YAML::LoadFile("/home/guoyunkai/quadruped_robot_2023/src/hardware_ctrl/config/standup.yaml");
    YAML::Node joint = YAML::LoadFile("/home/mzx/Desktop/dog/src/hardware_ctrl/config/standup.yaml");


    if (joint["motorcmd"]) {    // 验证yaml中是否有名为motorcmd的节点
        // 从yaml中读取关节电机数据
        const YAML::Node& dataNode = joint["motorcmd"];
        const std::size_t arraySize = dataNode.size();

        // 将yaml中的数据存入发布控制命令中
        for (std::size_t i = 0; i < arraySize; ++i) {
            motcmd.id[i] = dataNode[i]["info"]["id"].as<int>();
            motcmd.mode[i] = dataNode[i]["info"]["mode"].as<int>();
            motcmd.K_P[i] = 0;
            kp[i] = dataNode[i]["info"]["K_P"].as<float>();
            //motorcmd[i].K_P = dataNode[i]["info"]["K_P"].as<float>();
            motcmd.K_W[i] = dataNode[i]["info"]["K_W"].as<float>();
            motcmd.Pos[i] = dataNode[i]["info"]["Pos"].as<float>();
            motcmd.W[i] = dataNode[i]["info"]["W"].as<float>();
            motcmd.T[i] = dataNode[i]["info"]["T"].as<float>();
        }
    } else {
        std::cerr << "YAML file does not contain 'motorcmd' key." << std::endl;
        return -1;
    }  
    // 获取电机当前位置并计算与目标位置的差值
    // motorctrl.publish(motcmd);
    // 延迟3s
    // usleep(3000000);
    // 先静默3s，监听此时的电机状态
    for (int i = 0; i <= 10; i++) {
        ros::spinOnce();
        usleep(300000);
    }
    for (int i = 1; i <= 12; i++) {
        //data[i-1].Pos=1;
        derta_pos[i-1]=motcmd.Pos[i-1]-data[i-1].Pos;
    }
    for(int j = 1; j <= stand_time; j++) {
        tt = (double) j / stand_time;
        // 填充数据到发布的控制命令中
        for (int i = 1; i <= 12; i++) {
            motcmd.Pos[i-1] = data[i-1].Pos + tt * derta_pos[i-1];
            // motcmd.Pos[i-1] = data[i-1].Pos + tt * derta_pos[i-1];
            motcmd.K_P[i-1] = kp[i-1];
            // 下发控制信息并接收电机回传数据                                                                                                                                                                              
            // std::cout << std::endl;
            // std::cout << "ID: " << i-1 << std::endl;
            // std::cout << "motor.Pos: " << data[i-1].Pos << std::endl;
            // std::cout << "motor.Temp: " << data[i-1].Temp << std::endl;
            // std::cout << "motor.W: " << data[i-1].W << std::endl;
            // std::cout << "motor.T: " << data[i-1].T << std::endl;
            // std::cout << "motor.MError: " << data[i-1].MError << std::endl;
            // std::cout << std::endl
        }
        motorctrl.publish(motcmd);
        //延时0.1s
        usleep(100000);
    }
    return 0;
}
void motorCallback(const unitree_legged_msgs::upstream::ConstPtr& motor_cmd){
    //接收到控制数据后将其填入下发结构体中
    for (int j = 1; j <= 12; j++) {
    //  cmd[j-1].motorType = MotorType::Go2;
    //  cmd[j-1].id    = motor_cmd->id[j-1];
    //  cmd[j-1].mode  = motor_cmd->mode[j-1];
    //  cmd[j-1].K_P   = motor_cmd->K_P[j-1];
    //  cmd[j-1].K_W   = motor_cmd->K_W[j-1];
    data[j-1].Pos = motor_cmd->Pos[j-1];
    //  cmd[j-1].W     = motor_cmd->W[j-1];
    //  cmd[j-1].T     = motor_cmd->T[j-1];
    }
}