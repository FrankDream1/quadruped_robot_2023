#include "UnitreeDriver.h"
#include <serial/serial.h>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32MultiArray.h>
// #include <unitree_legged_msgs/motordata.h>
// #include <unitree_legged_msgs/motorcmd.h>
#include <unitree_legged_msgs/downstream.h>
#include <unitree_legged_msgs/upstream.h>
#include <thread>
using namespace std;


#define SENDRATE        100    // 节点通过串口下发的频率
#define UPQUEUESIZE     10     // UpQueue长度
#define DOWNQUEUESIZE   200    // DownQueue长度
// KP KD
#define MOTOR0KP            0.05f    //   0号KP
#define MOTOR0KD            20.0f
#define MOTOR0_SWING_KD     4.0f
#define MOTOR1KP            0.05f    // 1号KP
#define MOTOR1KD            5.0f
#define MOTOR2KP            0.1f    // 2号KP
#define MOTOR2KD            5.0f
// 逻辑零点实际位置数组：下发命令的时候，加上本数组；接收的时候，减掉本数组
#define LOGIZZEROPOSARRAY   {0.f, -0.f, 0.f, 0.f, 0.f, 0.f, 0.f,0.f, 0.f, 0.f, 0.f, 0.f}
// 正反 + 减速比数组：下发命令时乘本数组，接收的时候除以本数组
#define MOTORDIRARRAY       {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}                  

static UnitreeDriver *pMotorDriver = nullptr;     //
static UnitreeDriver *pMotorDriver_rec = nullptr;     //
static const float LogicZeroPosArray[12] = LOGIZZEROPOSARRAY; // 逻辑零位实际位置数组
static const float MotorRatioArray[12] = MOTORDIRARRAY;         // 电机正反减速比数组
void DownStreamCallback(const unitree_legged_msgs::downstream::ConstPtr& DownStreamMsg);
void Map_PublishMotorData(ros::Publisher& Pub);
void NodeUserInit();
void UpdateSwingLegMotor0KD(void);
void LowerTimercallback(const ros::TimerEvent&);


int main(int argc, char **argv) {
    ros::init(argc, argv, "STM32_Node");
    ros::NodeHandle nh;

    // ros::Publisher UpStreamPub = nh.advertise<unitree_legged_msgs::motordata>("/upstream", UPQUEUESIZE);
    // ros::Subscriber DownStreamSub = nh.subscribe<unitree_legged_msgs::motorcmd>("/downstream", DOWNQUEUESIZE, DownStreamCallback);
    ros::Publisher UpStreamPub = nh.advertise<unitree_legged_msgs::upstream>("/upstream", UPQUEUESIZE);
    ros::Subscriber DownStreamSub = nh.subscribe<unitree_legged_msgs::downstream>("/downstream", DOWNQUEUESIZE, DownStreamCallback);
    
    //could change any time
    pMotorDriver = new UnitreeDriver("/dev/ttyUSB1");
    pMotorDriver_rec = new UnitreeDriver("/dev/ttyUSB0");
    //NodeUserInit(); // 参数初始化
    ros::Rate loop_rate(SENDRATE);

    // //build multiple thread
    // PubThread (pMotorDriver->SendControlDataToSTM32());
    // PubThread.join();

    while (ros::ok()) {
        // 更新电机状态
        pMotorDriver_rec->UpdateMotorData();
        // 填充数据到发送消息中
        Map_PublishMotorData(UpStreamPub);                      // 发布电机当前数据

        // 显示下位机上发的数据
        std::cout << "MotorData_Pos:";
        for (int i = 0; i < 12; i++) {
            std::cout << pMotorDriver_rec->MotorData[i].CurPos << " ";
        }
        std::cout << std::endl;
        std::cout << "MotorData_Vel:";
        for (int i = 0; i < 12; i++) {
            std::cout << pMotorDriver_rec->MotorData[i].CurVel << " ";
        }
        std::cout << std::endl;
        std::cout << "MotorData_Tor:";
        for (int i = 0; i < 12; i++) {
            std::cout << pMotorDriver_rec->MotorData[i].CurTor << " ";
        }
        std::cout << std::endl;
        std::cout << std::endl;
        
        ros::spinOnce(); // 刷新控制数据(调用本函数之后，会直接调用CallBack函数，所以应该是不用担心数据还没来得及刷新的问题的)

        printf("Send message\n");
        //Send message to stm32
        // UpdateSwingLegMotor0KD();
        pMotorDriver->SendControlDataToSTM32(); // 下发新的数据到STM32 

        printf("\n\n\n");
        
        loop_rate.sleep();
        
    }
}
        



// 节点部分数据初始化函数
void NodeUserInit(void){
    pMotorDriver->SetKPKD(0, MOTOR0KP, MOTOR0KD);
    pMotorDriver->SetKPKD(1, MOTOR1KP, MOTOR1KD);
    pMotorDriver->SetKPKD(2, MOTOR2KP, MOTOR2KD);
    pMotorDriver->SetKPKD(3, MOTOR0KP, MOTOR0KD);
    pMotorDriver->SetKPKD(4, MOTOR1KP, MOTOR1KD);
    pMotorDriver->SetKPKD(5, MOTOR2KP, MOTOR2KD);
    pMotorDriver->SetKPKD(6, MOTOR0KP, MOTOR0KD);
    pMotorDriver->SetKPKD(7, MOTOR1KP, MOTOR1KD);
    pMotorDriver->SetKPKD(8, MOTOR2KP, MOTOR2KD);
    pMotorDriver->SetKPKD(9, MOTOR0KP, MOTOR0KD);
    pMotorDriver->SetKPKD(10, MOTOR1KP, MOTOR1KD);
    pMotorDriver->SetKPKD(11, MOTOR2KP, MOTOR2KD);
}

// 动态调整摆动腿0号电机的阻尼
void UpdateSwingLegMotor0KD(void){
    if(pMotorDriver->MotorData[0].MotionMode == SWING_TORMODE){
        pMotorDriver->SetKPKD(0, MOTOR0KP, MOTOR0_SWING_KD);
    }else{
        pMotorDriver->SetKPKD(0, MOTOR0KP, MOTOR0KD);
    }
        
    if(pMotorDriver->MotorData[3].MotionMode == SWING_TORMODE){
        pMotorDriver->SetKPKD(3, MOTOR0KP, MOTOR0_SWING_KD);
    }else{
        pMotorDriver->SetKPKD(3, MOTOR0KP, MOTOR0KD);
    }

    if(pMotorDriver->MotorData[6].MotionMode == SWING_TORMODE){
        pMotorDriver->SetKPKD(6, MOTOR0KP, MOTOR0_SWING_KD);
    }else{
        pMotorDriver->SetKPKD(6, MOTOR0KP, MOTOR0KD);
    }

    if(pMotorDriver->MotorData[9].MotionMode == SWING_TORMODE){
        pMotorDriver->SetKPKD(9, MOTOR0KP, MOTOR0_SWING_KD);
    }else{
        pMotorDriver->SetKPKD(9, MOTOR0KP, MOTOR0KD);
    }
}


void DownStreamCallback(const unitree_legged_msgs::downstream::ConstPtr& DownStreamMsg){
    for (uint8_t MotorCount = 0; MotorCount < 12; MotorCount++) {
        //UnitreeMotorData_t* pMotorData = &(pMotorDriver->MotorData[MotorCount]);
        int id;
        id=DownStreamMsg->id[MotorCount];
        UnitreeMotorData_t_sendToStm32* pMotorData = &(pMotorDriver->data_to_stm32[id]);
        //DownStreamMsg->id[MotorCount];
        pMotorData->TarTor=DownStreamMsg->T[MotorCount];
        pMotorData->TarVel=DownStreamMsg->W[MotorCount];
        pMotorData->TarPos=DownStreamMsg->Pos[MotorCount];
        pMotorData->KP = DownStreamMsg->K_P[MotorCount];
        pMotorData->KD = DownStreamMsg->K_W[MotorCount];
        // pMotorData->KP=DownStreamMsg.K_P[MotorCount];
        // pMotorData->KD=DownStreamMsg.K_W[MotorCount];
    }
}


void Map_PublishMotorData(ros::Publisher& Pub){
    // std_msgs::Float32MultiArray MotorDataArray;
    //unitree_legged_msgs::motordata motorback;   
    unitree_legged_msgs::upstream motorback;
    // MotorDataArray.data.clear();
    // MotorDataArray.data.resize(24); // 12位置 + 12速度
    for(uint8_t Count = 0;Count < 12;Count ++){ // 左前后 右前后
        /* 实际->逻辑 */
       memcpy(&(motorback.Pos[Count]), &(pMotorDriver_rec->MotorData[Count].CurPos), 4);
       memcpy(&(motorback.W[Count]), &(pMotorDriver_rec->MotorData[Count].CurVel), 4);
       memcpy(&(motorback.T[Count]), &(pMotorDriver_rec->MotorData[Count].CurTor), 4);
       memcpy(&(motorback.id[Count]), &Count, 1);
        // MotorDataArray.data[Count] = (pMotorDriver->MotorData[Count].CurPos - LogicZeroPosArray[Count]) / MotorRatioArray[Count];
        // MotorDataArray.data[12 + Count] = pMotorDriver->MotorData[Count].CurVel / MotorRatioArray[Count];
    }
    Pub.publish(motorback);
}