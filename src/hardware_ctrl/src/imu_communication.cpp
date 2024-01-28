// 数据帧格式：以0X7E开头 + 0x01 + 角度(float*3）+角速速(float* 3)+ 加速度(float * 3) + 四元数(float*4) + CRC8 (500Hz)
//
#include <iostream>
#include <vector>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <serial/serial.h>
#include <iomanip>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

using namespace std;
using namespace serial;
#define USERPASSWORD "XJPisNo1"
#define DriverBaudRate 115200   // 驱动板使用的串口波特率
std::string PortName = "/dev/ttyACM0";

struct DataFrame {
    float angle[3];         // 角度
    float angularVel[3];    // 角速度
    float acceleration[3];  // 加速度
    float quaternion[4];    // 四元数
};

// CRC8校验函数，如果校验成功返回true，否则返回false
bool checkCRC(const std::vector<uint8_t>& data) {
    uint8_t crc = 0xAA; // 初始值
    uint8_t polynomial = 0x7D; // CRC-8多项式

    for (size_t n = 0; n < 54; n++) {
        crc ^= data[n]; // XOR操作

        for (int i = 0; i < 8; i++) {
            if (crc & 0x01) { // 如果最高位为1
                crc = (crc >> 1) ^ polynomial; // 右移一位并进行XOR操作
            } else {
                crc >>= 1; // 否则只右移一位
            }
        }
    }
    
    return crc == data.back();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_communication");
    ros::NodeHandle nh;

    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("/dog_hardware/imu", 100);

    sensor_msgs::Imu imudata;

    serial::Serial ser; // 声明串口对象
    try {
        std::string sudoPassword = USERPASSWORD;
        std::string SerialCommand = "sudo chmod 777 " + PortName;
        std::string FinalCommand = "echo " + sudoPassword + "|sudo -S " + SerialCommand;
        system(FinalCommand.data());
        ser.setPort(PortName);
        ser.setBaudrate(DriverBaudRate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);  //设置读取串口数据到缓存区的时间
        ser.setTimeout(to);
        ser.open(); 
    }
    catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Open Serial Failed!");
    }
    if (ser.isOpen()) {
        ROS_INFO_STREAM(PortName << " Open Serial Succeed!");
    }
    Serial my_serial(PortName, DriverBaudRate, Timeout::simpleTimeout(1000));

    string rec_data;    // 接受到的数据
    std::vector<uint8_t> buffer;    // 缓冲区

    while (ros::ok) {
        // 读取数据
        size_t bytes_read = my_serial.read(buffer, 2);

        // 检查帧头
        if (bytes_read > 0 && buffer[0] == 0x7E && buffer[1] == 0x01) {
            // 继续读取剩余的数据
            size_t frame_size = 2 + 13 * sizeof(float) + 1;
            buffer.resize(frame_size);
            bytes_read += my_serial.read(buffer.data() + 2, frame_size - 2);

            if (bytes_read == frame_size) {
                if (checkCRC(buffer) == true) {
                    DataFrame c_data;
                    // 拷贝有用数据
                    memcpy(&c_data, &buffer[2], 13 * sizeof(float));

                    // 将数据存入IMU消息中
                    imudata.header.stamp = ros::Time::now();
                    imudata.angular_velocity.x = c_data.angularVel[0];
                    imudata.angular_velocity.y = c_data.angularVel[1];
                    imudata.angular_velocity.z = c_data.angularVel[2];
                    imudata.linear_acceleration.x = c_data.acceleration[0];
                    imudata.linear_acceleration.y = c_data.acceleration[1];
                    imudata.linear_acceleration.z = c_data.acceleration[2];
                    imudata.orientation.w = c_data.quaternion[0];
                    imudata.orientation.x = c_data.quaternion[1];
                    imudata.orientation.y = c_data.quaternion[2];
                    imudata.orientation.z = c_data.quaternion[3];

                    // 发布IMU的ROS话题
                    pub_imu.publish(imudata);
                    ROS_INFO_STREAM("Read Succeed!");
                } else {
                    cerr << "CRC校验失败" << endl;
                }
            }
        }
    }
}