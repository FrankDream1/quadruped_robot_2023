#ifndef PARAM_H
#define PARAM_H

// 控制时间
#define GRF_UPDATE_FREQUENCY 2.5        // ms
#define MAIN_UPDATE_FREQUENCY 2.5       // ms
#define HARDWARE_FEEDBACK_FREQUENCY 2.0 // ms

// 控制器常数
#define JOY_CMD_BODY_HEIGHT_MAX 0.32    // m
#define JOY_CMD_BODY_HEIGHT_MIN 0.1     // m
#define JOY_CMD_BODY_HEIGHT_VEL 0.04    // m/s
#define JOY_CMD_VELX_MAX 0.6            // m/s
#define JOY_CMD_VELY_MAX 0.3            // m/s
#define JOY_CMD_YAW_MAX 0.8             // rad
#define JOY_CMD_PITCH_MAX 0.4           // rad
#define JOY_CMD_ROLL_MAX 0.4            // rad

// MPC常数
#define PLAN_HORIZON 10
#define MPC_STATE_DIM 13
#define MPC_CONSTRAINT_DIM 20

// 机器狗参数
#define NUM_LEG 4   
#define NUM_DOF_PER_LEG 3
#define DIM_GRF 12
#define NUM_DOF 12

#define LOWER_LEG_LENGTH 0.21

#define FOOT_FORCE_LOW 30.0     
#define FOOT_FORCE_HIGH 80.0    

#define FOOT_SWING_CLEARANCE1 0.0f  
#define FOOT_SWING_CLEARANCE2 0.4f

#define FOOT_DELTA_X_LIMIT 0.1
#define FOOT_DELTA_Y_LIMIT 0.1

#endif //PARAM_H