#ifndef PARAM_H
#define PARAM_H

#define GRF_UPDATE_FREQUENCY 2.5        // MPC控制器更新频率(ms)
#define MAIN_UPDATE_FREQUENCY 2.5       // 主控制器更新频率(ms)
#define HARDWARE_FEEDBACK_FREQUENCY 2.0 // 硬件反馈频率(ms)

// 控制手柄参数
#define JOY_CMD_BODY_HEIGHT_MAX 0.32    // 站立最大高度(m)
#define JOY_CMD_BODY_HEIGHT_MIN 0.1     // 站立最低高度(m)
#define JOY_CMD_BODY_HEIGHT_VEL 0.04    // 站立速度(m/s)
#define JOY_CMD_VELX_MAX 0.6            // x方向最大速度(m/s)
#define JOY_CMD_VELY_MAX 0.3            // y方向最大速度(m/s)
#define JOY_CMD_YAW_MAX 0.8             // 最大偏航角度(rad)
#define JOY_CMD_PITCH_MAX 0.4           // 最大俯仰角度(rad)
#define JOY_CMD_ROLL_MAX 0.4            // 最大滚转角度(rad)

// MPC常数
#define PLAN_HORIZON 10         // 规划步长
#define MPC_STATE_DIM 13        // 状态维度
#define MPC_CONSTRAINT_DIM 20   // 约束维度

// 机构参数
#define NUM_LEG 4           // 腿的数量
#define NUM_DOF_PER_LEG 3   // 每条腿的自由度
#define DIM_GRF 12          // 所有腿的地面反作用力维度
#define NUM_DOF 12          // 所有腿的自由度
#define LOWER_LEG_LENGTH 0.21   // 小腿长度(m)

// 足端参数
#define FOOT_FORCE_LOW 30.0     // 最小足端力(N)
#define FOOT_FORCE_HIGH 80.0    // 最大足端力(N)

#define FOOT_SWING_CLEARANCE1 0.0f  // 足端摆动间隙1
#define FOOT_SWING_CLEARANCE2 0.4f  // 足端摆动间隙2

#define FOOT_DELTA_X_LIMIT 0.1  // 足端x方向最大差值    
#define FOOT_DELTA_Y_LIMIT 0.1  // 足端y方向最大差值

#endif //PARAM_H