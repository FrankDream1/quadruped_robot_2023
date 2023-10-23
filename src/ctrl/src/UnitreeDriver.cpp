#include "include/UnitreeDriver.h"

UnitreeDriver::UnitreeDriver(const std::string PortName) {
    if (PortName == "Null") {
        ROS_INFO_STREAM("Null!");
    } else {
        try {
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

        catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Open Serial Failed!");
            return ;
        }

        if (prvSerial.isOpen()) {
            ROS_INFO_STREAM(PortName << "Open Serial Succeed!");
        } else
            return ;
    }
}

UnitreeDriver::~UnitreeDriver() {}

// 下发控制信息给STM32
void UnitreeDriver::SendControlDataToSTM32() {
    std::vector<uint8_t> SendBuffer;
    SendBuffer.resize(7 * 12);

    // 编码
    for (uint8_t count = 0; count < 12; count ++) {
        switch(MotorData[count].MotionMode) {
            case DISABLE:
                EncodeAbleFrame(SendBuffer.data() + 7 * count, count, false);
                break;
            case TORMODE:
                EncodeTorFrame(SendBuffer.data() + 7 * count, count, MotorData[count].TarTor);
                break;
            case SWING_TORMODE:
                EncodeTorFrame(SendBuffer.data() + 7 * count, count, MotorData[count].TarTor);
                break;
            case VELMODE:
                EncodeVelFrame(SendBuffer.data() + 7 * count, count, MotorData[count].TarVel);
                break;
            case POSMODE:
                EncodePosFrame(SendBuffer.data() + 7 * count, count, MotorData[count].TarPos);
                break;
        }
    }
    // 发送
    if (prvSerial.isOpen()) {
        prvSerial.write(SendBuffer);
    }
}

// 刷新电机数据，如果有数据则返回1，没有数据则返回0
bool UnitreeDriver::UpdateMotorData(){
    if (prvSerial.available()) { // 如果串口有数据
        // 读取串口数据
        std::string NewData = prvSerial.read(prvSerial.available());
        // 将读取的数据放到缓冲区
        RxBuffer.append(NewData);
        // 查找帧头
        int FrameHeadIndex = RxBuffer.find(FRAMEHEAD);

        while (FrameHeadIndex != RxBuffer.npos) { // 能找到帧头
            if (RxBuffer.length() - FrameHeadIndex >= BACKFRAMELENGTH) { // 缓冲区数据超过一帧的长度
                // 确定数据起始位置
                uint8_t* pFrameStart = (uint8_t*)(RxBuffer.data()) + FrameHeadIndex;

                if (prvCRCCalculate(pFrameStart, BACKFRAMELENGTH) == 0x00) { // 校验通过
                    // 解码数据帧
                    DecodeFrame(pFrameStart);
                }

                RxBuffer.erase(0, FrameHeadIndex + BACKFRAMELENGTH); // 清除已解码数据
                FrameHeadIndex = RxBuffer.find(FRAMEHEAD); // 继续查找
            } else { // 可能没接收完毕，不选择丢弃，暂时保留
                break;
            }
        }
        return true;
    } else { // 如果缓冲区没有数据，说明可能是跟下位机失联了
        std::cout << "No Data!" <<std::endl;
        return false;
    }
}

// KP KD设置函数
void UnitreeDriver::SetKPKD(uint8_t MotorID, float KP, float KD) {
    std::vector<uint8_t> KPKDSendBuffer;
    KPKDSendBuffer.resize(7);
    EncodeParaFrame(KPKDSendBuffer.data(), MotorID, KP, KD);
    if (prvSerial.isOpen()) {
        prvSerial.write(KPKDSendBuffer);
    } else {
        ROS_ERROR_STREAM("Serial is not open!");
    }
}

//CRC8计算函数
uint8_t UnitreeDriver::prvCRCCalculate(uint8_t *pStr, uint8_t Len){
    uint8_t crc = 0xAA, polynomial = 0x7D;
    for (uint8_t i = 0; i < Len; i++) {
        crc = crc ^ (*pStr++);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ polynomial;
            }
            else{
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 编码使能帧
void UnitreeDriver::EncodeAbleFrame(uint8_t *pData, uint8_t MotorID, bool Able) {
    pData[0] = 0x7E;
    pData[1] = (uint8_t)((MotorID & 0x0F) << 4) | (0x00);
    pData[5] = Able;
    pData[6] = prvCRCCalculate(pData, 6);
}

//编码参数帧
void UnitreeDriver::EncodeParaFrame(uint8_t *pData, uint8_t MotorID, float KP, float KD){
    pData[0] = 0x7E;
    pData[1] = (uint8_t)((MotorID & 0x0F) << 4) | (0x01);
    /* KP KD */
    uint16_t KPKDInt[2];
    KPKDInt[0] = KP * 2048;
    KPKDInt[1] = KD * 1024;
    memcpy(&pData[2], KPKDInt, 4);
    pData[6] = prvCRCCalculate(pData, 6);
}

// 编码力矩帧
void UnitreeDriver::EncodeTorFrame(uint8_t *pData, uint8_t MotorID, float Torque){
    pData[0] = 0x7E;
    pData[1] = (uint8_t)((MotorID & 0x0F) << 4) | (0x02);
    memcpy(&pData[2], &Torque, 4);
    pData[6] = prvCRCCalculate(pData, 6);
}

// 编码速度帧
void UnitreeDriver::EncodeVelFrame(uint8_t *pData, uint8_t MotorID, float Velocity){
    pData[0] = 0x7E;
    pData[1] = (uint8_t)((MotorID & 0x0F) << 4) | (0x03);
    memcpy(&pData[2], &Velocity, 4);
    pData[6] = prvCRCCalculate(pData, 6);
}

// 编码位置帧
void UnitreeDriver::EncodePosFrame(uint8_t *pData, uint8_t MotorID, float Position){
    pData[0] = 0x7E;
    pData[1] = (uint8_t)((MotorID & 0x0F) << 4) | (0x04);
    memcpy(&pData[2], &Position, 4);
    pData[6] = prvCRCCalculate(pData, 6);
}

// 解码反馈信息
void UnitreeDriver::DecodeFrame(uint8_t *pData){
    uint8_t ErrorCode = pData[1] & 0x0F;
    if (ErrorCode) { // 电机发生了错误
        ROS_ERROR_STREAM("Code error!");
    } else {
        int8_t MotorID = (pData[1] & 0xF0) >> 4;
        memcpy(&(MotorData[MotorID].CurVel), &pData[2], 8);   // 先速度后位置
    }
}