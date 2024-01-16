#include "UnitreeDriver.h"
#include <ros/ros.h>
#include<iostream>
#include<stdlib.h>
#include<string>
#include <thread>

#define FRAMEHEAD           "~"    // 帧头
#define BACKFRAMELENGTH     15
#define CONTROLFRAMELENGTH  7
#define DriverBaudRate      921600  // 驱动板使用的串口波特率
#define USERPASSWORD        "081811gyk2002"
using namespace std;

std::string ByteStream2String(const uint8_t *pByteStream, size_t iStreamLen)
{
    std::string sRet = "";
    char curr_byte[3];

    for (size_t idx = 0; idx < iStreamLen; ++idx)
    {
    memset(curr_byte, 0, 3);
    sprintf(curr_byte, "%02hhX", pByteStream[idx]);

    sRet += curr_byte;
    sRet += " ";
    }

    return sRet;
}


static double getCurrentTime()
{
    ros::Time CurrentTime = ros::Time::now();
    return double(CurrentTime.toSec());
}


UnitreeDriver::UnitreeDriver(const std::string PortName)
{
    if(PortName == "Null"){
        ROS_INFO_STREAM("Null!");
    }
    else{
        try{
            std::string sudoPassword = USERPASSWORD;
            std::string SerialCommand = "sudo chmod 777 " + PortName;
            std::string FinalCommand = "echo " + sudoPassword + "|sudo -S " + SerialCommand;
            system(FinalCommand.data());
            prvSerial.setPort(PortName);
            prvSerial.setBaudrate(DriverBaudRate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            prvSerial.setTimeout(to);
            prvSerial.open();
        }
        catch(serial::IOException& e){
            ROS_ERROR_STREAM("Open Serial Failed!");
            return ;
        }

        if(prvSerial.isOpen()){
            // double StartTime = getCurrentTime();
            ROS_INFO_STREAM(PortName << "Open Serial Succeed!");
            // double EndTime = getCurrentTime();
            // ROS_INFO_STREAM(EndTime - StartTime);
        }
        else
            return ;
    }
}

float cnt_c=3;

// 下发控制信息给STM32
void UnitreeDriver::SendControlDataToSTM32(){
    std::vector<uint8_t> SendBuffer;
    SendBuffer.resize(23 * 12);
    /* 发送 */
    int pos = 23;
    for(int i = 0; i < 12; i++)
    {
        
        // data_to_stm32[i].TarTor = 0.4;
        // data_to_stm32[i].TarVel = 0;
        // data_to_stm32[i].TarPos = 3.14;
        // data_to_stm32[i].KP = 0.0;
        // data_to_stm32[i].KD = 0.0;
        SendBuffer.data()[0+i*pos] = 0x7e;
        SendBuffer.data()[1+i*pos] = i << 4;
        memcpy(&SendBuffer.data()[2+i*pos], &data_to_stm32[i].TarTor, 4);
        memcpy(&SendBuffer.data()[6+i*pos], &data_to_stm32[i].TarVel, 4);
        memcpy(&SendBuffer.data()[10+i*pos], &data_to_stm32[i].TarPos, 4);
        memcpy(&SendBuffer.data()[14+i*pos], &data_to_stm32[i].KP, 4);
        memcpy(&SendBuffer.data()[18+i*pos], &data_to_stm32[i].KD, 4);
        data_to_stm32[i].crc8 = prvCRCCalculate(&SendBuffer.data()[0+i*pos], 22);
        memcpy(&SendBuffer.data()[22+i*pos], &data_to_stm32[i].crc8, 1);

        cout << "data_to_stm32[i].id: " << i << endl;
        cout << "data_to_stm32[i].TarTor: " << data_to_stm32[i].TarTor << endl;
        cout << "data_to_stm32[i].TarVel: " << data_to_stm32[i].TarVel << endl;
        cout << "data_to_stm32[i].TarPos: " << data_to_stm32[i].TarPos << endl;
        cout << "data_to_stm32[i].KP: " << data_to_stm32[i].KP << endl;
        cout << "data_to_stm32[i].KD: " << data_to_stm32[i].KD << endl;
    }
    
    // printf("SendBuffer: %s\n\n\n", ByteStream2String(SendBuffer.data(), 23*12).data()); //


    if(prvSerial.isOpen()){
            prvSerial.write(SendBuffer);
            //std::cout << SendBuffer.data() <<std::endl;
            // printf("\n\ni = %d, crc8 = %d\n", i, data_to_stm32[i].crc8);
            // printf("SendBuffer: %s\n\n\n", ByteStream2String(SendBuffer.data(), 23*12).data()); //
            SendBuffer.clear();
        }
        else{
            ROS_ERROR_STREAM("Serial is not open!");
        }

    
}

// 刷新电机数据，如果有数据则返回1，没有数据则返回0
bool UnitreeDriver::UpdateMotorData() { 
    if (prvSerial.available()) {  // 如果缓冲区有数据
        std::string NewData = prvSerial.read(prvSerial.available());
        RxBuffer.append(NewData);

        int FrameHeadIndex = RxBuffer.find(FRAMEHEAD);

        while (FrameHeadIndex != RxBuffer.npos) {
            if (RxBuffer.length() - FrameHeadIndex >= BACKFRAMELENGTH) {
                uint8_t* pFrameStart = (uint8_t*)(RxBuffer.data()) + FrameHeadIndex;
                
                if (prvCRCCalculate(pFrameStart, BACKFRAMELENGTH) == 0x00) { // 校验通过
                    DecodeFrame(pFrameStart);
                }
                RxBuffer.erase(0, FrameHeadIndex + BACKFRAMELENGTH);    // “出队”
                FrameHeadIndex = RxBuffer.find(FRAMEHEAD);
            } else {   // 可能没接收完毕，不选择丢弃，暂时保留
                break;
            }
        }
        return true;
    } else {   // 如果缓冲区没有数据，说明可能是跟下位机失联了
        std::cout << "No Data!" <<std::endl;
        return false;
    }
}

/**
 * @brief   KP KD设置函数
 * @param   MotorID: 电机ID
            KP KD：如所示
 */
void UnitreeDriver::SetKPKD(uint8_t MotorID, float KP, float KD){
    // std::vector<uint8_t> KPKDSendBuffer;
    // KPKDSendBuffer.resize(7);
    // EncodeParaFrame(KPKDSendBuffer.data(), MotorID, KP, KD);
    // if(prvSerial.isOpen())
    //     prvSerial.write(KPKDSendBuffer);
    // else{
    //     // ROS_ERROR_STREAM("Serial is not open!");
    // }
    data_to_stm32[MotorID].KP = KP;
    data_to_stm32[MotorID].KD = KD;
}

/**
 * @brief   CRC8计算函数
 * @param   pStr: 要校验的数据的起始指针
            Len: 要校验的数据长度
 */
uint8_t UnitreeDriver::prvCRCCalculate(uint8_t *pStr, uint8_t Len){
    uint8_t crc = 0xAA, polynomial=0x7D;
    for(uint8_t i = 0; i < Len; i++){
        crc = crc ^ (*pStr++);
        for (uint8_t j = 0; j < 8; j++){
            if (crc & 0x01){
                crc = (crc >> 1) ^ polynomial;
            }
            else{
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief   编码使能帧
 * @param   pData: 发送缓冲区起始指针
 *          MotorID: 电机ID
            Able: 使能与否？
 */
void UnitreeDriver::EncodeAbleFrame(uint8_t *pData, uint8_t MotorID, bool Able){
    pData[0] = 0x7E;
    pData[1] = (uint8_t)((MotorID & 0x0F) << 4) | (0x00);
    pData[5] = Able;
    pData[6] = prvCRCCalculate(pData, 6);   // 包括CRC校验码总共7个字节
}

/**
 * @brief   编码参数帧
 * @param   pData: 发送缓冲区起始指针
 *          MotorID: 电机ID
            KP,KD: 如同字面意思
 */
void UnitreeDriver::EncodeParaFrame(uint8_t *pData, uint8_t MotorID, float KP, float KD){
    pData[0] = 0x7E;
    pData[1] = (uint8_t)((MotorID & 0x0F) << 4) | (0x01);
    /* KP KD */
    uint16_t KPKDInt[2];
    KPKDInt[0] = KP * 2048;
    KPKDInt[1] = KD * 1024;
    memcpy(&pData[2], KPKDInt, 4);
    pData[6] = prvCRCCalculate(pData, 6);   // 包括CRC校验码总共7个字节
}

/**
 * @brief   编码力矩帧
 * @param   pData: 发送缓冲区起始指针
 *          MotorID: 电机ID
            Torque: 目标力矩
 */
void UnitreeDriver::EncodeTorFrame(uint8_t *pData, uint8_t MotorID, float Torque){
    pData[0] = 0x7E;
    pData[1] = (uint8_t)((MotorID & 0x0F) << 4) | (0x02);
    memcpy(&pData[2], &Torque, 4);
    pData[6] = prvCRCCalculate(pData, 6);   // 包括CRC校验码总共7个字节
}

/**
 * @brief   编码速度帧
 * @param   pData: 发送缓冲区起始指针
 *          MotorID: 电机ID
            Velocity: 目标速度
 */
void UnitreeDriver::EncodeVelFrame(uint8_t *pData, uint8_t MotorID, float Velocity){
    pData[0] = 0x7E;
    Velocity=2.35;
    //std::cout << "12312321" <<std::endl;
    pData[1] = (uint8_t)((MotorID & 0x0F) << 4) | (0x03);
    memcpy(&pData[2], &Velocity, 4);
    pData[6] = prvCRCCalculate(pData, 6);   // 包括CRC校验码总共7个字节
}

/**
 * @brief   编码位置帧
 * @param   pData: 发送缓冲区起始指针
 *          MotorID: 电机ID
            Position: 目标位置
 */
void UnitreeDriver::EncodePosFrame(uint8_t *pData, uint8_t MotorID, float Position){
    pData[0] = 0x7E;
    pData[1] = (uint8_t)((MotorID & 0x0F) << 4) | (0x04);
    memcpy(&pData[2], &Position, 4);
    pData[6] = prvCRCCalculate(pData, 6);   // 包括CRC校验码总共7个字节
}

// 解码反馈信息
void UnitreeDriver::DecodeFrame(uint8_t *pData){
    uint8_t ErrorCode = pData[1] & 0x0F;

    if (ErrorCode) {    // 电机发生了错误
        std::cout << "Error Decoding!" <<std::endl;
    } else {
        int8_t MotorID =pData[1]>> 4;
        int num = MotorID;
        //int8_t MotorID = (pData[1] & 0xF0) >> 4;
        //std::cout << num <<std::endl;
        memcpy(&(MotorData[MotorID].CurVel), &pData[2], 8);   // 先速度
        memcpy(&(MotorData[MotorID].CurPos), &pData[6], 8);   //位置
        memcpy(&(MotorData[MotorID].CurTor), &pData[10], 8);   //力矩
    }
}