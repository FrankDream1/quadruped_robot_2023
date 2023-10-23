#include "include/UnitreeDriver.h"
#include <serial/serial.h>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32MultiArray.h>

#define SENDRATE 100        // Hz,节点通过串口下发的频率
#define UPQUEUESIZE 10      // UpQueue长度
#define DOWNQUEUESIZE 20    // DownQueue长度

// 设置KP和KD
#define MOTOR0KP 0.05f      // 0号KP
#define MOTOR0KD 20.0f
#define MOTOR0_SWING_KD 4.0f
#define MOTOR1KP 0.05f      // 1号KP
#define MOTOR1KD 5.0f
#define MOTOR2KP 0.1f       // 2号KP
#define MOTOR2KD 5.0f

// 逻辑零点实际位置数组：下发命令的时候，加上本数组；接收的时候，减掉本数组
#define LOGIZZEROPOSARRAY {0.f, -0.f, 0.f, 0.f, 0.f, 0.f, 0.f,0.f, 0.f, 0.f, 0.f, 0.f}
// 正反+减速比数组：下发命令时乘本数组，接收的时候除以本数组
#define MOTORDIRARRAY {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}                  

static UnitreeDriver *pMotorDriver = nullptr; // 电机数据类
static const float LogicZeroPosArray[12] = LOGIZZEROPOSARRAY; // 逻辑零位实际位置数组
static const float MotorRatioArray[12] = MOTORDIRARRAY; // 电机正反减速比数组
        
// 初始化KP KD
void KPKDInit(void){
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
    if (pMotorDriver->MotorData[0].MotionMode == SWING_TORMODE) {
        pMotorDriver->SetKPKD(0, MOTOR0KP, MOTOR0_SWING_KD);
    } else {
        pMotorDriver->SetKPKD(0, MOTOR0KP, MOTOR0KD);
    }
         
    if (pMotorDriver->MotorData[3].MotionMode == SWING_TORMODE) {
        pMotorDriver->SetKPKD(3, MOTOR0KP, MOTOR0_SWING_KD);
    } else {
        pMotorDriver->SetKPKD(3, MOTOR0KP, MOTOR0KD);
    }

    if (pMotorDriver->MotorData[6].MotionMode == SWING_TORMODE) {
        pMotorDriver->SetKPKD(6, MOTOR0KP, MOTOR0_SWING_KD);
    } else {
        pMotorDriver->SetKPKD(6, MOTOR0KP, MOTOR0KD);
    }

    if (pMotorDriver->MotorData[9].MotionMode == SWING_TORMODE) {
        pMotorDriver->SetKPKD(9, MOTOR0KP, MOTOR0_SWING_KD);
    } else {
        pMotorDriver->SetKPKD(9, MOTOR0KP, MOTOR0KD);
    }
}

// 订阅回调函数
// DownStreamMsg：DownStream数据，前12个字节表示运动模式，对应的后12个字节表示数据(位置模式就是位置数据,速度模式就是速度数据)
// 左前 左后 右前 右后
void DownStreamCallback(const std_msgs::Float32MultiArray::ConstPtr& DownStreamMsg){
    for (int MotorCount = 0; MotorCount < 12; MotorCount++) {
        UnitreeMotorData_t* pMotorData = &(pMotorDriver->MotorData[MotorCount]);
        pMotorData->MotionMode = MotionMode_t(DownStreamMsg->data.at(MotorCount));
        switch(pMotorData->MotionMode) {
            case DISABLE:
                break;
            case TORMODE: {
                float LogTarTor = DownStreamMsg->data.at(12 + MotorCount);
                pMotorData->TarTor = LogTarTor / MotorRatioArray[MotorCount];
            }
                break;
            case SWING_TORMODE:{
                float LogTarTor = DownStreamMsg->data.at(12 + MotorCount);
                pMotorData->TarTor = LogTarTor / MotorRatioArray[MotorCount];
            }
                break;
            case VELMODE:{
                float LogTarVel = DownStreamMsg->data.at(12 + MotorCount);
                pMotorData->TarVel = LogTarVel * MotorRatioArray[MotorCount];
            }
                break;
            case POSMODE:{
                float LogTarPos = DownStreamMsg->data.at(12 + MotorCount);
                pMotorData->TarPos = LogTarPos * MotorRatioArray[MotorCount] + LogicZeroPosArray[MotorCount];
            }
                break;   
        }
    }
}

// 依据原始数据做映射，并且发布电机数据
void Map_PublishMotorData(ros::Publisher& Pub){
    std_msgs::Float32MultiArray MotorDataArray;
    MotorDataArray.data.clear();
    MotorDataArray.data.resize(24); // 12位置 + 12速度
    for(int Count = 0; Count < 12; Count++){ // 左前后 右前后
        // 实际->逻辑
        MotorDataArray.data[Count] = (pMotorDriver->MotorData[Count].CurPos - LogicZeroPosArray[Count]) / MotorRatioArray[Count];
        MotorDataArray.data[12 + Count] = pMotorDriver->MotorData[Count].CurVel / MotorRatioArray[Count];
    }
    Pub.publish(MotorDataArray); // 发布
}

// 下位机看门狗回调函数
void LowerTimercallback(const ros::TimerEvent&){
    ROS_ERROR_STREAM("Lower Disconnected!");
    // 后续操作
}