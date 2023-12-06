#include <iostream>
#include <array>
#include <thread>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include "CANDevice.h"

class USVBattery {
 public:
    USVBattery(ros::NodeHandle* nodehandle, std::string can_interface);
    ~USVBattery();

 private:
    std::array<std::shared_ptr<CANDevice>, 4> battery_overall_state;
    std::array<std::shared_ptr<CANDevice>, 4> battery_cell_state;

    ros::NodeHandle _nh;

    ros::Publisher battery_soc_pub;
    ros::Publisher battery_min_cell_volt_pub;

    std_msgs::Float32MultiArray battery_soc;
    std_msgs::Float32MultiArray battery_min_cell_volt;

    void battery_polling_can();
    void overall_state_can_to_ros(const can_frame& frame, int batt_idx);
    void cell_state_can_to_ros(const can_frame& frame, int batt_idx);

    void decode_overall_state(const can_frame& frame, double& cumulative_voltage, double& collecting_voltage, double& current, double& soc);
    void decode_cell_state(const can_frame& frame, double& max_volt, double& min_volt, int& max_volt_id, int& min_volt_id);

    bool stopThread_;
    std::thread polling_thread_;
};

USVBattery::USVBattery(ros::NodeHandle* nodehandle, std::string can_interface) : _nh(*nodehandle), stopThread_(false) {
    for (int i = 0; i < 4; i++) {
        battery_overall_state[i] = std::make_shared<CANDevice>(can_interface.c_str(), 0x18904001 + i, boost::bind(&USVBattery::overall_state_can_to_ros, this, _1, i), 0x18900140 + i);
        battery_cell_state[i] = std::make_shared<CANDevice>(can_interface.c_str(), 0x18914001 + i, boost::bind(&USVBattery::cell_state_can_to_ros, this, _1, i), 0x18910140 + i);

        battery_overall_state[i]->read();
        battery_cell_state[i]->read();
    }

    polling_thread_ = std::thread(&USVBattery::battery_polling_can, this);

    ROS_INFO("USV battery CAN-ROS driver stared.");
}

USVBattery::~USVBattery() {
    printf("Stopping battery polling thread, please wait...\n");
    stopThread_ = true;
    if (polling_thread_.joinable()) {
        polling_thread_.join();
    }
    printf("Battery polling thread stopped.\n");
    return;
}

void USVBattery::overall_state_can_to_ros(const can_frame& frame, int batt_idx) {
    double cumulative_voltage, collecting_voltage, current, soc;
    decode_overall_state(frame, cumulative_voltage, collecting_voltage, current, soc);

    battery_soc.data.clear();
    battery_soc.data.push_back(batt_idx + 1.0);
    battery_soc.data.push_back(soc);
    battery_soc_pub.publish(battery_soc);
    battery_soc.data.clear();

    return;

}

void USVBattery::cell_state_can_to_ros(const can_frame& frame, int batt_idx) {
    double max_volt, min_volt;
    int max_volt_id, min_volt_id;
    decode_cell_state(frame, max_volt, min_volt, max_volt_id, min_volt_id);

    battery_min_cell_volt.data.clear();
    battery_min_cell_volt.data.push_back(batt_idx + 1.0);
    battery_min_cell_volt.data.push_back(min_volt);
    battery_min_cell_volt_pub.publish(battery_min_cell_volt);
    battery_min_cell_volt.data.clear();

    return;
    
}

void USVBattery::battery_polling_can() {
    int i = 0;
    ros::Rate _rate(5);
    uint8_t payload[8];
    memset(payload, 0, sizeof(payload));

    ROS_INFO("Start to polling USV battery CAN frame.");
    while (_nh.ok() && !stopThread_) {
        // 发送 CAN 帧
        battery_overall_state[i]->send(payload, 8); 
        _rate.sleep();
        battery_cell_state[i]->send(payload, 8); 
        _rate.sleep();
        i = (i + 1) % 4;
    }
    
}

static inline uint64_t reverse_byte_order(uint64_t x) {
	x = (x & 0x00000000FFFFFFFF) << 32 | (x & 0xFFFFFFFF00000000) >> 32;
	x = (x & 0x0000FFFF0000FFFF) << 16 | (x & 0xFFFF0000FFFF0000) >> 16;
	x = (x & 0x00FF00FF00FF00FF) << 8  | (x & 0xFF00FF00FF00FF00) >> 8;
	return x;
}

void USVBattery::decode_overall_state(const can_frame& frame, double& cumulative_voltage, double& collecting_voltage, double& current, double& soc) {
    register uint64_t x;
	register uint64_t m = reverse_byte_order(*(uint64_t *)(frame.data));

	/* cumulative_voltage: start-bit 7, length 16, endianess motorola, scaling 0.1, offset 0 */
	x = (m >> 48) & 0xffff;
    cumulative_voltage = (double)(x);
	cumulative_voltage *= 0.1;
	if (cumulative_voltage > 60) {
		cumulative_voltage = 0.0;
	}

	/* collecting_voltage: start-bit 23, length 16, endianess motorola, scaling 0.1, offset 0 */
	x = (m >> 32) & 0xffff;
    collecting_voltage = (double)(x);
	collecting_voltage *= 0.1;
	if (collecting_voltage > 60) {
		collecting_voltage = 0.0;
	}

	/* current: start-bit 39, length 16, endianess motorola, scaling 0.1, offset -3000 */
	x = (m >> 16) & 0xffff;
    current = (double)(x);
	current *= 0.1;
	current += -3000;
	if (current > 60000) {
		current = 0.0;
	}

	/* soc: start-bit 55, length 16, endianess motorola, scaling 0.1, offset 0 */
	x = m & 0xffff;
	soc = (double)(x);
	soc *= 0.1;
	if (soc > 1000) {
		soc = 0.0;
	}

    return;
}

void USVBattery::decode_cell_state(const can_frame& frame, double& max_volt, double& min_volt, int& max_volt_id, int& min_volt_id) {
	register uint64_t x;
	register uint64_t m = reverse_byte_order(*(uint64_t *)(frame.data));

	/* max_volt: start-bit 7, length 16, endianess motorola, scaling 0.001, offset 0 */
	x = (m >> 48) & 0xffff;
    max_volt = (double)(x);
	max_volt *= 0.001;
	if (max_volt > 60000.0) {
		max_volt = 0.0;
	}

	/* min_volt: start-bit 31, length 16, endianess motorola, scaling 0.001, offset 0 */
	x = (m >> 24) & 0xffff;
    min_volt = (double)(x);
	min_volt *= 0.001;
	if (min_volt > 60000.0) {
		min_volt = 0.0;
	}

	/* max_volt_id: start-bit 23, length 8, endianess motorola, scaling 1, offset 0 */
	x = (m >> 40) & 0xff;
    max_volt_id = (int)(uint8_t)(x);
	if (max_volt_id > 63) {
		max_volt_id = 0;
	}
    
	/* min_volt_id: start-bit 47, length 8, endianess motorola, scaling 1, offset 0 */
	x = (m >> 16) & 0xff;
    min_volt_id = (int)(uint8_t)(x);
	if (min_volt_id > 63) {
		min_volt_id = 0;
	}

	return;
}
