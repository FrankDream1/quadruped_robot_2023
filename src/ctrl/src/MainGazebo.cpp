#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

// control parameters
#include "include/Param.h"
// dog control
#include "include/GazeboROS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_dog_mpc_ctrl");
    ros::NodeHandle nh;

    // 更改ros logger
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // 确保ROS使用的是仿真时间use_sim_time
    std::string use_sim_time;
    if (ros::param::get("/use_sim_time", use_sim_time)) {
        if (use_sim_time != "true") {
            std::cout << "ROS must set use_sim_time in order to use this program!" << std::endl;
            return -1;
        }
    }

    // 创建专属指针dog控制器
    std::unique_ptr<GazeboROS> dog = std::make_unique<GazeboROS>(nh);

    // 创建控制执行原子数据
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

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            ros::Duration(GRF_UPDATE_FREQUENCY / 1000).sleep();

            // 获取t和dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start; // 运行时间
            
            // 更新前的精准时间
            auto t1 = std::chrono::high_resolution_clock::now();

            // 计算期望地面作用力
            bool running = dog->update_foot_forces_grf(dt.toSec());

            // 更新后的精准时间
            auto t2 = std::chrono::high_resolution_clock::now();

            // 更新时间
            std::chrono::duration<double, std::milli> ms_double = t2 - t1;
            std::cout << "MPC solution is updated in " << ms_double.count() << "ms" << std::endl;

            if (!running) {
                std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }
        }
    });

    // 线程2: 更新机器狗状态, 计算摆动腿期望力, 计算期望关节力矩, 发送控制命令
    std::cout << "Enter thread 2: update robot states, compute desired swing legs forces, compute desired joint torques, and send commands" << std::endl;
    std::thread main_thread([&]() {
        // 控制过程的时间变量
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();  
        ros::Duration dt(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            // 更新前的精准时间
            auto t3 = std::chrono::high_resolution_clock::now();

            ros::Duration(MAIN_UPDATE_FREQUENCY / 1000).sleep();

            // 获取t和dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start; // 运行时间

            // 更新状态，发送命令
            bool main_update_running = dog->main_update(elapsed.toSec(), dt.toSec());
            bool send_cmd_running = dog->send_cmd();

            // 更新后的精准时间
            auto t4 = std::chrono::high_resolution_clock::now();

            // 更新时间
            std::chrono::duration<double, std::milli> ms_double = t4 - t3;
            std::cout << "Thread 2 is updated in " << ms_double.count() << "ms" << std::endl;

            if (!main_update_running || !send_cmd_running) {
                std::cout << "Thread 2 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }
        }
    });

    // 接受多线程订阅话题
    ros::AsyncSpinner spinner(12);
    spinner.start();

    // 结束线程
    compute_foot_forces_grf_thread.join();
    main_thread.join();

    return 0;
}