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

#include "USVPod.h"

int main(int argc, char** argv){
    // 初始化 ROS 节点
    ros::init(argc, argv, "usv_pod_driver_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);

    try {
        // 启动 USVPod CAN-ROS driver
        USVPod usv_pod(&nh, 1000.0);

        while(nh.ok()) {
            rate.sleep();
            ros::spinOnce();
        }
    } catch(const std::exception& e) {
        std::cerr << "[Exception detected] " << e.what() << std::endl;
    }

    return 0;
}