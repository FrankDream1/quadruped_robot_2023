#include "IOROS.h"
#include <ros/ros.h>
#include <string>
#include <unistd.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "IOROS");
    // ros::NodeHandle nh;
    IOROS _IOROS;
    //NodeUserInit(); // 参数初始化
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        _IOROS.sendCmd();
        loop_rate.sleep(); 
    }
}