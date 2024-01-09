#ifndef _UNITREE_LEGGED_COMM_H_
#define _UNITREE_LEGGED_COMM_H_

#include <stdint.h>
#include <array>

namespace UNITREE_LEGGED_SDK {

    constexpr double PosStopF = (2.146E+9f);
    constexpr double VelStopF = (16000.0f);

#pragma pack(1)

    typedef struct {
        std::array<float, 4> quaternion;    // quaternion, normalized, (w,x,y,z)
        std::array<float, 3> gyroscope;     // angular velocity （unit: rad/s)    (raw data)
        std::array<float, 3> accelerometer; // acceleration （unit: m/(s2))       (raw data)
        std::array<float, 3> rpy;           // euler angle（unit: rad)
        int8_t temperature;                 // the temperature of imu (unit: °C)
    } IMU;                                // when under accelerated motion, the attitude of the robot calculated by IMU will drift.

    typedef struct {
        uint8_t mode;       // motor working mode. Servo : 0x0A, Damping : 0x00，Overheat ： 0x08.
        float q;            // current angle (unit: radian)
        float dq;           // current velocity (unit: radian/second)
        float ddq;          // current acc (unit: radian/second*second)
        float tauEst;       // current estimated output torque (unit: N.m)
        float q_raw;        // reserve
        float dq_raw;       // reserve
        float ddq_raw;      // reserve
        int8_t temperature; // current temperature (temperature conduction is slow that leads to lag)
        std::array<uint32_t, 2> reserve;
    } MotorState; // motor feedback

    typedef struct {
        uint8_t mode; // desired working mode. Servo : 0x0A, Damping : 0x00.
        float q;      // desired angle (unit: radian)
        float dq;     // desired velocity (unit: radian/second)
        float tau;    // desired output torque (unit: N.m)
        float Kp;     // desired position stiffness (unit: N.m/rad )
        float Kd;     // desired velocity stiffness (unit: N.m/(rad/s) )
    } MotorCmd; // motor control

    typedef struct {
        IMU imu;
        std::array<MotorState, 20> motorState;
        uint32_t crc;
    } LowState; // low level feedback

#pragma pack()

} // namespace UNITREE_LEGGED_SDK

#endif