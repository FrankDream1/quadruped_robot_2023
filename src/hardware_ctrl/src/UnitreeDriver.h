#ifndef UNITREEDRIVER_H
#define UNITREEDRIVER_H

#include <serial/serial.h>
#include <string.h>

typedef enum{
    DISABLE = -1,       // 未启动
    POSMODE = 0,        // 位置模式
    VELMODE = 1,        // 速度模式
    TORMODE = 2,        // 支撑相力矩模式
    SWING_TORMODE = 3   // 摆动相力矩模式
}MotionMode_t;

typedef struct{
    MotionMode_t MotionMode = DISABLE;  // 运动模式
    float TarTor = 0.0f;
    float TarPos = 0.0f;
    float TarVel = 0.0f;
    float KP = 0.0f;
    float KD = 0.0f;
    float CurVel = 0.0f;
    float CurPos = 0.0f;
    float CurTor = 0.0f;
}UnitreeMotorData_t;    // 存储控制信息和反馈信息的结构体

typedef struct{
    uint8_t fe = 0x7e;
    uint8_t id;
    float TarTor = 0.0f;
    float TarVel = 0.0f;
    float TarPos = 0.0f;
    float KP = 0.0f;
    float KD = 0.0f;
    uint8_t crc8;
}UnitreeMotorData_t_sendToStm32;    // 存储控制信息和反馈信息的结构体

class UnitreeDriver
{
    public:
        UnitreeDriver(const std::string PortName = "Null");
        ~UnitreeDriver();
        void SendControlDataToSTM32(void);
        bool UpdateMotorData(void);
        void SetKPKD(uint8_t MotorID, float KP, float KD);
        uint8_t prvCRCCalculate(uint8_t *pStr, uint8_t Len);
        UnitreeMotorData_t MotorData[12];            // 0 1 2对应左前；3 4 5对应左后；6 7 8对应右前；9 10 11对应右后
        UnitreeMotorData_t_sendToStm32 data_to_stm32[12];
    private:

        serial::Serial prvSerial;
        
        /* Tx */
        void EncodeAbleFrame(uint8_t *pData, uint8_t MotorID, bool Able);
        void EncodeParaFrame(uint8_t *pData, uint8_t MotorID, float KP, float KD);
        void EncodeTorFrame(uint8_t *pData, uint8_t MotorID, float Torque);
        void EncodeVelFrame(uint8_t *pData, uint8_t MotorID, float Velocity);
        void EncodePosFrame(uint8_t *pData, uint8_t MotorID, float Position);
        /* Rx */
        std::string RxBuffer = {};          // 接收缓冲区，默认为空的
        void DecodeFrame(uint8_t *pData);
};

#endif
