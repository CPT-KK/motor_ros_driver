#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <numeric>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <ros/ros.h>

#include "USVMotor.h"

int main(int argc, char** argv){
    // 初始化 ROS 节点
    ros::init(argc, argv, "usv_motor_driver_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);

    std::string can_interface;
    nh.getParam("can_interface", can_interface);

    try {
        ROS_INFO("Starting USV Motor CAN<->ROS driver on %s...", can_interface.c_str());
        USVMotor motor(&nh, can_interface);
        ROS_INFO("All done. USV Motor CAN<->ROS driver started.");

        while(nh.ok()) {
            rate.sleep();
            ros::spinOnce();
        }
    } catch(const std::exception& e) {
        std::cerr << "[Exception detected] " << e.what() << std::endl;
    }

    return 0;
}