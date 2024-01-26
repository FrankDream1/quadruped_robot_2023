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
            
//坐标系变换（从初始上电位置到竖直位置）
const double transformation[12]={-0.795, -0.837, 2.990, 0.818, -0.863, 3.033, 0.761, 0.842, -3.001, -0.769, 0.859, -3.106};
static UnitreeDriver *pMotorDriver = nullptr;     //
static UnitreeDriver *pMotorDriver_rec = nullptr;     //
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

    // //build multiple thread
    // PubThread (pMotorDriver->SendControlDataToSTM32());
    // PubThread.join();
    while (ros::ok()) {
        // 更新电机状态
        pMotorDriver_rec->UpdateMotorData();

        //变换坐标系
        // std::cout <<pMotorDriver_rec->MotorData[7].CurPos<< std::endl;
        for(int i=0;i<12;i++)
        {
            pMotorDriver_rec->MotorData[i].CurPos=pMotorDriver_rec->MotorData[i].CurPos-transformation[i];
        }
        // 填充数据到发送消息中
        Map_PublishMotorData(UpStreamPub);

        // 显示下位机上发的数据
        // std::cout << "MotorData_Pos:";
        // for (int i = 0; i < 12; i++) {
        //     std::cout << pMotorDriver_rec->MotorData[i].CurPos << " ";
        // }
        // std::cout << std::endl;
        // std::cout << "MotorData_Vel:";
        // for (int i = 0; i < 12; i++) {
        //     std::cout << pMotorDriver_rec->MotorData[i].CurVel << " ";
        // }
        // std::cout << std::endl;
        // std::cout << "MotorData_Tor:";
        // for (int i = 0; i < 12; i++) {
        //     std::cout << pMotorDriver_rec->MotorData[i].CurTor << " ";
        // }
        // std::cout << std::endl;
        // std::cout << std::endl;
        
        ros::spinOnce(); // 刷新控制数据(调用本函数之后，会直接调用CallBack函数，所以应该是不用担心数据还没来得及刷新的问题的)
        //Send message to stm32
        // UpdateSwingLegMotor0KD();
        pMotorDriver->SendControlDataToSTM32(); // 下发新的数据到STM32 

        // 下发新的数据到STM32
        pMotorDriver->SendControlDataToSTM32(); 
        
        loop_rate.sleep();
    }
}

void DownStreamCallback(const unitree_legged_msgs::downstream::ConstPtr& DownStreamMsg){
    for (uint8_t MotorCount = 0; MotorCount < 12; MotorCount++) {
        //UnitreeMotorData_t* pMotorData = &(pMotorDriver->MotorData[MotorCount]);
        //int id_t;
        //id_t=DownStreamMsg->id[MotorCount];
        //int swap_indices[12] = {0, 1, 2, 6, 7, 8, 3, 4, 5, 9, 10, 11};//mpc_ctrl转换矩阵
        //int swap_indices[12] = {6, 7, 8, 0, 1, 2, 9, 10, 11, 3, 4, 5};//unitree_guide转换矩阵
        //int id = swap_indices[id_t];
        int id=DownStreamMsg->id[MotorCount];
        UnitreeMotorData_t_sendToStm32* pMotorData = &(pMotorDriver->data_to_stm32[id]);
        //DownStreamMsg->id[MotorCount];3
        pMotorData->TarTor=DownStreamMsg->T[MotorCount];
        pMotorData->TarVel=DownStreamMsg->W[MotorCount];
        //pMotorData->TarPos=DownStreamMsg->Pos[MotorCount]+transformation[id]+initial_pose[id];//变换坐标系并加上初始值
        pMotorData->TarPos=DownStreamMsg->Pos[MotorCount]+transformation[id];//变换坐标系并加上初始值
        pMotorData->KP = DownStreamMsg->K_P[MotorCount];
        pMotorData->KD = DownStreamMsg->K_W[MotorCount];
    }
}

void Map_PublishMotorData(ros::Publisher& Pub){ 
    unitree_legged_msgs::upstream motorback;
    // MotorDataArray.data.clear();
    // MotorDataArray.data.resize(24); // 12位置 + 12速度
    for(uint8_t Count = 0;Count < 12;Count ++){ // 左前后 右前后
        /* 实际->逻辑 */
        // int swap_indices[12] = {0, 1, 2, 6, 7, 8, 3, 4, 5, 9, 10, 11};//mpc_ctrl转换矩阵
        //int swap_indices[12] = {6, 7, 8, 0, 1, 2, 9, 10, 11, 3, 4, 5};//unitree_guide转换矩阵
        // int id = swap_indices[Count];
        int id=Count;
       memcpy(&(motorback.Pos[id]), &(pMotorDriver_rec->MotorData[Count].CurPos), 4);
       memcpy(&(motorback.W[id]), &(pMotorDriver_rec->MotorData[Count].CurVel), 4);
       memcpy(&(motorback.T[id]), &(pMotorDriver_rec->MotorData[Count].CurTor), 4);
       memcpy(&(motorback.id[id]), &Count, 1);
        // MotorDataArray.data[Count] = (pMotorDriver->MotorData[Count].CurPos - LogicZeroPosArray[Count]) / MotorRatioArray[Count];
        // MotorDataArray.data[12 + Count] = pMotorDriver->MotorData[Count].CurVel / MotorRatioArray[Count];
    }

    Pub.publish(motorback);
}