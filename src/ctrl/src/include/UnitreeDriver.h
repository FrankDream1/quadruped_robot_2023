#ifndef UNITREEDRIVER_H
#define UNITREEDRIVER_H

#include <serial/serial.h>
#include <string.h>
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>

#define FRAMEHEAD           "0x7E"    // 帧头
#define BACKFRAMELENGTH     11
#define CONTROLFRAMELENGTH  7
#define DriverBaudRate      921600    // 驱动板使用的串口波特率
#define USERPASSWORD        "XJPisNo1"

/*  上位机下发控制帧：(7字节)
        帧头(1bytes) + 电机ID(4bits) + 控制帧类型(4bits) + 数据内容(4bytes) + 校验码(1bytes)
            其中数据内容：
            0为使能帧(bool)
            1为参数帧(uint16_t + uint16_t)
            2为力矩帧(float)
            3为速度帧(float)
            4为位置帧(float)
    下位机上发反馈帧：(11字节)
        帧头(1bytes) + 电机ID(4bits) + 反馈帧类型(4bits) + 当前速度(4bytes) + 当前位置(4bytes) + 校验码(1bytes)
*/

typedef enum {
    DISABLE = -1,       // 未启动
    POSMODE = 0,        // 位置模式
    VELMODE = 1,        // 速度模式
    TORMODE = 2,        // 支撑相力矩模式
    SWING_TORMODE = 3   // 摆动相力矩模式
}MotionMode_t;

typedef struct {
    MotionMode_t MotionMode = DISABLE;  // 运动模式
    float TarTor = 0.0f;
    float TarPos = 0.0f;
    float TarVel = 0.0f;
    float KP = 0.0f;
    float KD = 0.0f;
    float CurVel = 0.0f;
    float CurPos = 0.0f;
}UnitreeMotorData_t;    // 存储控制信息和反馈信息的结构体

class UnitreeDriver {
public:
    UnitreeDriver(const std::string PortName = "Null");
    ~UnitreeDriver();

    void SendControlDataToSTM32(void);
    bool UpdateMotorData(void);
    void SetKPKD(uint8_t MotorID, float KP, float KD);
    UnitreeMotorData_t MotorData[12]; // 0 1 2对应左前；3 4 5对应左后；6 7 8对应右前；9 10 11对应右后

private:
    serial::Serial prvSerial;
    uint8_t prvCRCCalculate(uint8_t *pStr, uint8_t Len);

    void EncodeAbleFrame(uint8_t *pData, uint8_t MotorID, bool Able);
    void EncodeParaFrame(uint8_t *pData, uint8_t MotorID, float KP, float KD);
    void EncodeTorFrame(uint8_t *pData, uint8_t MotorID, float Torque);
    void EncodeVelFrame(uint8_t *pData, uint8_t MotorID, float Velocity);
    void EncodePosFrame(uint8_t *pData, uint8_t MotorID, float Position);

    std::string RxBuffer = {};          // 接收缓冲区，默认为空的
    void DecodeFrame(uint8_t *pData);
};

#endif
