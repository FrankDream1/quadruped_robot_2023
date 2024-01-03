#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

#include "../include/Param.h"
#include "../include/HardwareROS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "hardware_dog_mpc_ctrl");
    ros::NodeHandle nh;

    // 改变ros日志显示输出的级别设置
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // 确保ROS不使用仿真时间
    std::string use_sim_time;
    if (ros::param::get("/use_sim_time", use_sim_time)) {
        if (use_sim_time != "false") {
            std::cout << "hardware must have real time in order to use this program!" << std::endl;
            return -1;
        }
    }

    // 创建专属指针dog控制器
    std::unique_ptr<HardwareROS> dog = std::make_unique<HardwareROS>(nh);

    // 创建控制执行过程中的原子变量防止两个线程冲突
    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);

    // 线程1: 计算期望地面作用力
    std::cout << "Enter thread 1: compute desired ground forces" << std::endl;
    std::thread compute_foot_forces_grf_thread([&]() {
        // 控制过程的时间变量
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();  
        ros::Duration dt(0);
        ros::Duration dt_solver_time(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            // 获取t和dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;

            // 计算期望地面作用力
            bool running = dog->update_foot_forces_grf(dt.toSec());

            // 计算时间
            dt_solver_time = ros::Time::now() - now;
            
            if (!running) {
                std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }

            if (dt_solver_time.toSec() < GRF_UPDATE_FREQUENCY / 1000) {
                ros::Duration( GRF_UPDATE_FREQUENCY / 1000 - dt_solver_time.toSec() ).sleep();
            }
        }
    });

    // 线程2: 更新机器狗状态, 计算摆动腿期望力, 计算期望关节力矩, 发送控制命令
    std::cout << "Enter thread 2: update robot states, compute desired swing legs forces, compute desired joint torques, and send commands"
            << std::endl;
    std::thread main_thread([&]() {
        // 控制过程的时间变量
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();
        ros::Duration dt(0);
        ros::Duration dt_solver_time(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            // 获取t和dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start;

            // 计算地面期望反作用力
            bool main_update_running = dog->main_update(elapsed.toSec(), dt.toSec());
            bool send_cmd_running = dog->send_cmd();

            if (!main_update_running || !send_cmd_running) {
                std::cout << "Thread 2 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }

            dt_solver_time = ros::Time::now() - now;
            if (dt_solver_time.toSec() < MAIN_UPDATE_FREQUENCY / 1000) {
                ros::Duration( MAIN_UPDATE_FREQUENCY / 1000 - dt_solver_time.toSec() ).sleep();
            }
        }
    });

    ros::AsyncSpinner spinner(12);
    spinner.start();

    compute_foot_forces_grf_thread.join();
    main_thread.join();

    return 0;
}
