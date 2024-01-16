#include "ros/ros.h"
#include "serialPort/SerialPort.h"
#include <unistd.h>
// #include <unitree_legged_msgs/motordata.h>
// #include <unitree_legged_msgs/motorcmd.h>
#include <unitree_legged_msgs/downstream.h>
#include <unitree_legged_msgs/upstream.h>
#include "yaml-cpp/yaml.h"

#define SENDRATE 500    // 节点通过串口下发的频率
MotorCmd cmd[12];       // 12个电机的控制命令
MotorData data[12];     // 12个电机的返回数据
int maxstep=1;          // 电机最大步长
double maxkp=0.05;      // 电机最大刚度

void motorCallback(const unitree_legged_msgs::downstream::ConstPtr& motor_cmd);

int main(int argc, char** argv) {
  	ros::init(argc, argv, "motor_communication");
  	ros::NodeHandle nh;

  	// ros::Publisher motorpub = nh.advertise<unitree_legged_msgs::motordata>("/dog_hardware/motordata", 100);
  	// ros::Subscriber motorsub = nh.subscribe<unitree_legged_msgs::motorcmd>("/dog_hardware/motorcmd", 100, motorCallback);
    ros::Publisher motorpub = nh.advertise<unitree_legged_msgs::upstream>("/dog_hardware/motordata", 100);
  	ros::Subscriber motorsub = nh.subscribe<unitree_legged_msgs::downstream>("/dog_hardware/motorcmd", 100, motorCallback);
        
  	//unitree_legged_msgs::motordata motorback;
  	unitree_legged_msgs::upstream motorback;

    // 定义串口
  	SerialPort serial("/dev/ttyUSB0");

    ros::Rate loop_rate(SENDRATE);

    // 预先填充一组kp，kw均为零的安全命令
    for (int i = 1; i <= 12; i++) {
        cmd[i-1].K_P = 0;
        cmd[i-1].K_W = 0;
        cmd[i-1].id = i-1;
    }

    // 下发安全命令，获取电机当前的状态信息
    for (int i = 1; i <= 12; i++) {
        serial.sendRecv(&cmd[i-1], &data[i-1]);
        motorback.Pos[i-1] = data[i-1].Pos;
        motorback.Temp[i-1] = data[i-1].Temp;
        motorback.W[i-1] = data[i-1].W;
        motorback.T[i-1] = data[i-1].T;
        motorback.MError[i-1] = data[i-1].MError;
        motorback.id[i-1] = data[i-1].motor_id;
        motorback.mode[i-1] = data[i-1].mode;
    }
        
    motorpub.publish(motorback);

    while (ros::ok) {  
        // 查询执行一次回调函数
        ros::spinOnce();    

        for (int i = 1; i <= 12; i++) {
            //对下发的控制命令进行限制，防止出现危险
            if (cmd[i-1].Pos - data[i-1].Pos > maxstep) {
                std::cout << "步长超限：" << cmd[i-1].Pos - data[i-1].Pos << std::endl;
                cmd[i-1].Pos = data[i-1].Pos;
            }
            if (cmd[i-1].K_P > maxkp) {
                std::cout << "位置刚度超限：" << cmd[i-1].K_P << std::endl;
                cmd[i-1].K_P = maxkp;
            }
            // 下发控制信息并接收电机回传数据                                                                                                                                                                              
            serial.sendRecv(&cmd[i-1], &data[i-1]);

            std::cout << "下发命令电机ID: " << cmd[i-1].id << std::endl;
            std::cout << "下发命令Pos: " << cmd[i-1].Pos << std::endl;

            // 将回传数据放入要发布的消息体里
            motorback.Pos[i-1] = data[i-1].Pos;
            motorback.Temp[i-1] = data[i-1].Temp;
            motorback.W[i-1] = data[i-1].W;
            motorback.T[i-1] = data[i-1].T;
            motorback.MError[i-1] = data[i-1].MError;
            motorback.id[i-1] = data[i-1].motor_id;
            motorback.mode[i-1] = data[i-1].mode;
        }

        // 发布电机回传消息
        motorpub.publish(motorback);
        loop_rate.sleep();
    }
    return 0;
}

// 回调函数，用来将订阅的控制信息储存在下发消息体里
void motorCallback(const unitree_legged_msgs::downstream::ConstPtr& motor_cmd) {
    //接收到控制数据后将其填入下发结构体中
    for (int j = 1; j <= 12; j++) {
        cmd[j-1].motorType = MotorType::Go2;
        cmd[j-1].id = motor_cmd->id[j-1];
        cmd[j-1].mode = motor_cmd->mode[j-1];
        cmd[j-1].K_P = motor_cmd->K_P[j-1];
        cmd[j-1].K_W = motor_cmd->K_W[j-1];
        cmd[j-1].Pos = motor_cmd->Pos[j-1];
        cmd[j-1].W = motor_cmd->W[j-1];
        cmd[j-1].T = motor_cmd->T[j-1];
    }
}     