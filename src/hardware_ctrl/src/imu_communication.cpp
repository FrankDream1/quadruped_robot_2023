//数据帧格式：以0X7E开头 + 0x01 + 角度(float*3）+角速速(float* 3)+ 加速度(float * 3) + CRC (500Hz)）
#include <iostream>
#include <vector>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <serial/serial.h>
#include <iomanip>
using namespace std;
using namespace serial;

const char *portName = "/dev/ttyUSB0"; // 串口名称
const int baudRate = 9600; // 波特率

struct DataFrame {
    float angle[3]; // 角度
    float angularVel[3];    // 角速度
    float acceleration[3];  // 加速度
};

bool checkCRC(const std::vector<uint8_t>& data) {
    // 这里添加CRC校验的实现
    // 返回true如果校验成功，否则返回false
    return true;
}

void print_data(DataFrame data)
{
    // 处理数据...
    // 比如打印数据
    for (int i = 0; i < 3; ++i) {
        std::cout << "Angle[" << i << "]: " << data.angle[i] << std::endl;
    }
    // 类似地，打印角速度和加速度
    for (int i = 0; i < 3; ++i) {
        std::cout << "Ang_vel[" << i << "]: " << data.angularVel[i] << std::endl;
    }
}

void processData(vector<uint8_t>& buffer) {
    if (buffer.size() < sizeof(DataFrame) + 2 || buffer[0] != 0x7E || buffer[1] != 0x01) {
        std::cerr << "Invalid data frame" << std::endl;
        return;
    }

    if (!checkCRC(buffer)) {
        std::cerr << "CRC check failed" << std::endl;
        return;
    }
    DataFrame data;
    cout << 11111;
    return;
    while(buffer[0]==0x7e && buffer.size() >= sizeof(DataFrame) + 2)
    {
        memcpy(&data, buffer.data() + 2, sizeof(DataFrame));
        
        buffer.erase(buffer.begin(), buffer.begin()+39);
        print_data(data);
    }
    

}

void hexStringToBytes(vector<uint8_t>& buffer, const std::string& hex) {
    for (size_t i = 0; i < hex.length(); i += 2) 
    {
        if(i+2 >= hex.length())
            return;
        std::string byteString = hex.substr(i, 2);
        // cout << (int)byteString << " " << i << endl;
        uint8_t byte = static_cast<uint8_t>(std::stoul(byteString, nullptr, 16));
        buffer.push_back(byte);
    }

}

void printHex(const std::string& data) {
    for (unsigned char c : data) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << " ";
    }
    std::cout << std::endl;
}

void aa(vector<int>& v)
{
    cout << v.front();
}

int main() {
    serial::Serial ser; // 声明串口对象
    ser.setPort("/dev/ttyACM0");    // 串口设备
    ser.setBaudrate(115200);    // 设置波特率

    //设置读取串口数据到缓存区的时间
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    
    ser.open(); // 打开串口
    string rec_data;    // 接受到的数据
    std::vector<uint8_t> buffer;    // 缓冲区

    Serial my_serial("/dev/ttyACM0", 115200, Timeout::simpleTimeout(1000));
    while (true) {
        // 读取数据
        size_t bytes_read = my_serial.read(buffer, 1);

        // 检查帧头
        if (bytes_read > 0 && buffer[0] == 0x7E) {
            // 继续读取剩余的数据
            size_t frame_size = 1 + 3 * sizeof(float) * 3 + 1;
            buffer.resize(frame_size);
            bytes_read += my_serial.read(buffer.data() + 1, frame_size - 1);

            if (bytes_read == frame_size) {
                // CRC8校验
                if (1) {
                    DataFrame c_data;
                    memcpy(&c_data, &buffer[2], 9 * sizeof(float));

                    float angle[3], angular_velocity[3], acceleration[3];
                    memcpy(angle, &buffer[2], 3 * sizeof(float));
                    memcpy(angular_velocity, &buffer[2 + 3 * sizeof(float)], 3 * sizeof(float));
                    memcpy(acceleration, &buffer[2 + 6 * sizeof(float)], 3 * sizeof(float));

                    // 打印数据
                    for (int i = 0; i < 3; ++i) {
                        if(abs(c_data.acceleration[i]) > 0.1)
                            cout << "角速度[" << i << "]: " << c_data.acceleration[i] << ", " << endl;
                        // cout << "角速度[" << i << "]: " << angular_velocity[i] << ", ";
                        // cout << "加速度[" << i << "]: " << acceleration[i] << endl;
                    }
                    // cout << endl;
                } else {
                    cerr << "CRC 校验失败" << endl;
                }
            }
        }
    }




























    // while(1)
    // {
    //     if(ser.available())//读取到缓存区数据的字节数
    //     {
    //         rec_data += ser.read(ser.available());//读出缓存区缓存的数据
    //         string toFind = "\x7E\x01"; // 要查找的子字符串
    //         size_t rpos = rec_data.rfind(toFind);
    //         size_t lpos = rec_data.find(toFind);
    //         //cout << lpos << " " << rpos << endl;
    //         if (rpos != lpos && rpos != 0) //找到最右边的7e01 开始输出
    //         {
    //             string rec_data1 = rec_data.substr(0,rpos);
    //             rec_data = rec_data.substr(rpos);
    //             if(rec_data1.empty())
    //                 continue;

    //             printf("size=%d\n",rec_data.size());    
    //             hexStringToBytes(buffer, rec_data1);
                
    //              cout << buffer.size() << endl;
    //              for(int i = 0; i < buffer.size(); i++)
    //                  cout << buffer[i] << endl;
    //             processData(buffer);
    //         }
            
    //     }
        
    // }
    





    /*
    struct sp_port *port;
    sp_return error = sp_get_port_by_name(portName, &port);
    if (error != SP_OK) {
        std::cerr << "Unable to find serial port." << std::endl;
        return -1;
    }

    error = sp_open(port, SP_MODE_READ);
    if (error != SP_OK) {
        std::cerr << "Unable to open serial port." << std::endl;
        sp_close(port);
        return -1;
    }

    sp_set_baudrate(port, baudRate);

    while (true) {
        uint8_t byte;
        error = sp_blocking_read(port, &byte, 1, 1000); // 读取1字节数据，超时时间1000毫秒
        if (error > 0) {
            buffer.push_back(byte);

            if (byte == 0x7E && buffer.size() > 1) {
                
                buffer.clear();
            }
        } else if (error < 0) {
            std::cerr << "Error reading from serial port." << std::endl;
            break;
        }
    }

    sp_close(port);
    sp_free_port(port);
    */
    return 0;
}
