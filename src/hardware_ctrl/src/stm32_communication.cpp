#include "UnitreeDriver.h"
#include <serial/serial.h>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <unitree_legged_msgs/downstream.h>
#include <unitree_legged_msgs/upstream.h>
#include <thread>
using namespace std;

#define SENDRATE        100    // 节点通过串口下发的频率
#define UPQUEUESIZE     10     // UpQueue长度
#define DOWNQUEUESIZE   200    // DownQueue长度
// 可供参考的KP KD
#define MOTOR0KP            0.05f    //   0号KP
#define MOTOR0KD            20.0f
#define MOTOR0_SWING_KD     4.0f
#define MOTOR1KP            0.05f    // 1号KP
#define MOTOR1KD            5.0f
#define MOTOR2KP            0.1f    // 2号KP
#define MOTOR2KD            5.0f
            
//坐标系变换
const double transformation[12] = 
    {-0.795, -0.837, 2.990, 0.818, -0.863, 3.033, 0.761, 0.842, -3.001, -0.769, 0.859, -3.106};
static UnitreeDriver *pMotorDriver = nullptr;       // 发送指针
static UnitreeDriver *pMotorDriver_rec = nullptr;   // 接受指针
void DownStreamCallback(const unitree_legged_msgs::downstream::ConstPtr& DownStreamMsg);
void Map_PublishMotorData(ros::Publisher& Pub);

int main(int argc, char **argv) {
    ros::init(argc, argv, "STM32_Node");
    ros::NodeHandle nh;

    ros::Publisher UpStreamPub = nh.advertise<unitree_legged_msgs::upstream>("/upstream", UPQUEUESIZE);
    ros::Subscriber DownStreamSub = nh.subscribe<unitree_legged_msgs::downstream>("/downstream", DOWNQUEUESIZE, DownStreamCallback);
    
    //could change any time
    pMotorDriver = new UnitreeDriver("/dev/ttyUSB1");
    pMotorDriver_rec = new UnitreeDriver("/dev/ttyUSB0");

    ros::Rate loop_rate(SENDRATE);

    while (ros::ok()) {
        // 更新电机状态
        pMotorDriver_rec->UpdateMotorData();

        //变换坐标系
        for (int i = 0; i < 12; i++) {
            pMotorDriver_rec->MotorData[i].CurPos=pMotorDriver_rec->MotorData[i].CurPos-transformation[i];
        }
        // 填充数据到发送消息中
        Map_PublishMotorData(UpStreamPub);

        // 刷新控制数据(调用本函数之后，会直接调用CallBack函数，所以应该是不用担心数据还没来得及刷新的问题的)    
        ros::spinOnce();

        // 下发新的数据到STM32
        pMotorDriver->SendControlDataToSTM32(); 
        
        loop_rate.sleep();
    }
}

void DownStreamCallback(const unitree_legged_msgs::downstream::ConstPtr& DownStreamMsg){
    for (uint8_t MotorCount = 0; MotorCount < 12; MotorCount++) {
        int id;
        id = DownStreamMsg->id[MotorCount];
        UnitreeMotorData_t_sendToStm32* pMotorData = &(pMotorDriver->data_to_stm32[id]);
        pMotorData->TarTor = DownStreamMsg->T[MotorCount];
        pMotorData->TarVel = DownStreamMsg->W[MotorCount];
        pMotorData->TarPos = DownStreamMsg->Pos[MotorCount] + transformation[id];//变换坐标系并加上初始值
        pMotorData->KP = DownStreamMsg->K_P[MotorCount];
        pMotorData->KD = DownStreamMsg->K_W[MotorCount];
    }
}

void Map_PublishMotorData(ros::Publisher& Pub){ 
    unitree_legged_msgs::upstream motorback;

    // 腿的顺序: 0-FL  1-RL  2-FR  3-RR
    for (uint8_t Count = 0; Count < 12; Count++) {
        memcpy(&(motorback.Pos[Count]), &(pMotorDriver_rec->MotorData[Count].CurPos), 4);
        memcpy(&(motorback.W[Count]), &(pMotorDriver_rec->MotorData[Count].CurVel), 4);
        memcpy(&(motorback.T[Count]), &(pMotorDriver_rec->MotorData[Count].CurTor), 4);
        memcpy(&(motorback.id[Count]), &Count, 1);
    }

    Pub.publish(motorback);
}