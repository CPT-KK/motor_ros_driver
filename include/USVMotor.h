#include <iostream>

#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

#include "CANDevice.h"
#include "angle_operation.h"

class USVMotor {
 public:
    USVMotor(ros::NodeHandle* nodehandle, std::string can_interface);
    ~USVMotor();

 private:
    CANDevice torq_both, pod_left, pod_right;
    // static const int torq_left_id = 0x02;
    // static const int torq_right_id = 0x01;
    
    // static const int pod_left_send_id = 0x384;
    // static const int pod_left_recv_id = 0x604;
    // static const int pod_right_send_id = 0x383;
    // static const int pod_right_recv_id = 0x603;

    ros::NodeHandle _nh;

    ros::Publisher pod_left_estimate_pub;
    ros::Publisher pod_right_estimate_pub;
    ros::Subscriber pod_left_setpoint_sub;
    ros::Subscriber pod_right_setpoint_sub;
    std_msgs::Float32 left_pod_angle;
    std_msgs::Float32 right_pod_angle;

    ros::Publisher torq_left_estimate_pub;
    ros::Publisher torq_right_estimate_pub;
    ros::Publisher torq_left_status_pub;
    ros::Publisher torq_right_status_pub;
    ros::Subscriber torq_left_setpoint_sub;
    ros::Subscriber torq_right_setpoint_sub;
    std_msgs::Int16 torq_left_rpm;
    std_msgs::Int16 torq_right_rpm;
    std_msgs::Int16 torq_left_status;
    std_msgs::Int16 torq_right_status;

    void torqeedo_estimate_can_to_ros(const can_frame& frame);
    void torqeedo_setpoint_ros_to_can(const std_msgs::Int16::ConstPtr& msg, const unsigned int idx);

    void pod_angle_estimate_can_to_ros(const can_frame& frame, const unsigned int idx);
    void pod_angle_setpoint_ros_to_can(const std_msgs::Float32::ConstPtr& msg, const unsigned int idx);

    void decode_motor_state(const can_frame& frame, double& engine_speed, int& motor_index, int& motor_status, int& motor_gear);
    void decode_pod_angle(const can_frame& frame, double& current_position);

};

USVMotor::USVMotor(ros::NodeHandle* nodehandle, std::string can_interface) : _nh(*nodehandle), 
    torq_both(can_interface.c_str(), 0x00000A05, boost::bind(&USVMotor::torqeedo_estimate_can_to_ros, this, _1), 0x00000A02), 
    pod_left(can_interface.c_str(), 0x384, boost::bind(&USVMotor::pod_angle_estimate_can_to_ros, this, _1, 0x04), 0x604), 
    pod_right(can_interface.c_str(), 0x383, boost::bind(&USVMotor::pod_angle_estimate_can_to_ros, this, _1, 0x03), 0x603) {

    // Create ROS publishers and subscribers
    torq_left_estimate_pub = _nh.advertise<std_msgs::Int16>("/usv/torqeedo/left/estimate", 1);
    torq_right_estimate_pub = _nh.advertise<std_msgs::Int16>("/usv/torqeedo/right/estimate", 1);
    torq_left_status_pub = _nh.advertise<std_msgs::Int16>("/usv/torqeedo/left/status", 1);
    torq_right_status_pub = _nh.advertise<std_msgs::Int16>("/usv/torqeedo/right/status", 1);
    torq_left_setpoint_sub = _nh.subscribe<std_msgs::Int16>("/usv/torqeedo/left/setpoint", 10, boost::bind(&USVMotor::torqeedo_setpoint_ros_to_can, this, _1, 0x02));
    torq_right_setpoint_sub = _nh.subscribe<std_msgs::Int16>("/usv/torqeedo/right/setpoint", 10, boost::bind(&USVMotor::torqeedo_setpoint_ros_to_can, this, _1, 0x01));

    pod_left_estimate_pub = _nh.advertise<std_msgs::Float32>("/usv/pod/left/estimate", 1);
    pod_right_estimate_pub = _nh.advertise<std_msgs::Float32>("/usv/pod/right/estimate", 1);
    pod_left_setpoint_sub = _nh.subscribe<std_msgs::Float32>("/usv/pod/left/setpoint", 10, boost::bind(&USVMotor::pod_angle_setpoint_ros_to_can, this, _1, 0x04));
    pod_right_setpoint_sub = _nh.subscribe<std_msgs::Float32>("/usv/pod/right/setpoint", 10, boost::bind(&USVMotor::pod_angle_setpoint_ros_to_can, this, _1, 0x03));

    // Start to read from devices
    ROS_INFO("Starting CAN<->ROS instances for torqeedos...");
    torq_both.read();

    ROS_INFO("Starting CAN<->ROS instances for pods...");
    pod_left.read();
    pod_right.read();

    return;
}

USVMotor::~USVMotor() {
    ROS_INFO("Closing CAN<->ROS instances...");
    return;
}

void USVMotor::torqeedo_estimate_can_to_ros(const can_frame& frame) {
    // Decode CAN frame from torqeedo
    int motor_index, motor_status, motor_gear;
    double rpm_data;
    decode_motor_state(frame, rpm_data, motor_index, motor_status, motor_gear);
    rpm_data = rpm_data * static_cast<double>(motor_gear);
    // See this CAN frame is for which motor, and publish the rpm data
    switch (motor_index) {
        case 0x02:
            torq_left_rpm.data = static_cast<int16_t>(rpm_data);
            torq_left_estimate_pub.publish(torq_left_rpm);
            torq_left_status.data = static_cast<int16_t>(motor_status);
            torq_left_status_pub.publish(torq_left_status);
            break;
        case 0x01:
            torq_right_rpm.data = static_cast<int16_t>(rpm_data);
            torq_right_estimate_pub.publish(torq_right_rpm);
            torq_right_status.data = static_cast<int16_t>(motor_status);
            torq_right_status_pub.publish(torq_right_status);
            break;
        default:
            break;
    }

    return;
}

void USVMotor::torqeedo_setpoint_ros_to_can(const std_msgs::Int16::ConstPtr& msg, const unsigned int idx) {
    // The input rpm value
    int16_t raw_rpm = msg->data;

    // Input RPM limitation
    if (raw_rpm > 1000 || raw_rpm < -1000) {
        ROS_WARN("Setpoint %d rpm for torqeedo 0x%X not in [-1000, 1000] rpm.", raw_rpm, idx);
        raw_rpm = raw_rpm / abs(raw_rpm) * 1000;
    }

    if (raw_rpm > -50 && raw_rpm < 50) {
        ROS_WARN("Setpoint %d rpm for torqeedo 0x%X is too small to trigger an action.", raw_rpm, idx);
        if (abs(raw_rpm) > 5) {
            raw_rpm = raw_rpm / abs(raw_rpm) * 50;
        } else {
            raw_rpm = 0;
        }
    }

    // CAN frame payload for torqeedo
    uint8_t payload[3];

    // Set the 1st byte of CAN frame payload to motor idx
    payload[0] = (uint8_t)idx;

    // Set the 2nd and 3rd byte of CAN frame payload to motor rpm
    // Note: RPM needs to be divided by 4 to [-250, 250]
    // If the motor status is not E5, send full zero command to try to restart the motor
    *((int16_t*)(payload + 1)) = raw_rpm / 4;  

    // Send CAN frame
    torq_both.send(payload, 3);

    return;
}

void USVMotor::pod_angle_estimate_can_to_ros(const can_frame& frame, const unsigned int idx) {
    // Decode CAN frame from left or right pod
    double current_position;
    decode_pod_angle(frame, current_position);

    // See which pod is this CAN frame from, and publish the angle data
    switch (idx) {
        case 0x04:
            left_pod_angle.data = static_cast<float>(current_position);
            pod_left_estimate_pub.publish(left_pod_angle);
            break;
        case 0x03:
            right_pod_angle.data = static_cast<float>(current_position);
            pod_right_estimate_pub.publish(right_pod_angle);
            break;
        default:
            break;
    }

    return;
}

void USVMotor::pod_angle_setpoint_ros_to_can(const std_msgs::Float32::ConstPtr& msg, const unsigned int idx) {
    // Input angle must multiply 500 to get the raw angle value for the pod
    int32_t raw_angle = (int32_t)(msg->data * 500.0);

    // Angle limitation [-44, 140] -> [-22000, 70000]
    if (raw_angle > 22000 || raw_angle < -70000) {
        ROS_WARN("Setpoint %d deg for pod 0x%X not in [-44, 140] deg.", (int32_t)msg->data, idx);
        raw_angle = (raw_angle >= 0) ? 22000 : -70000;
    }

    // CAN frame payload for pod
    uint8_t payload[8];

    // Set the first 4 bytes of the CAN payload to 0x01240023
    // Set the remaining 4 bytes to the raw angle value
    // int32_t is 4 bytes, so we use pointer to assign values. 
    // Add 1 to a pointer with type int32_t will make the pointer point the next 4 bytes.
    *((int32_t*)payload) = 0x01240023;
    *((int32_t*)(payload) + 1) = raw_angle;

    // See which pod is this CAN frame for, and send the CAN frame
    switch (idx) {
        case 0x04:
            pod_left.send(payload, 8);
            break;
        case 0x03:
            pod_right.send(payload, 8);
            break;
        default:
            break;
    }

    return;
}

void USVMotor::decode_motor_state(const can_frame& frame, double& engine_speed, int& motor_index, int& motor_status, int& motor_gear) {
	register uint64_t x;
	register uint64_t i = *(uint64_t *)(frame.data);
	if (frame.can_dlc < 8)
		throw std::runtime_error("CAN frame data length is not enough.");

    /* engine_speed: start-bit 24, length 16, endianess intel, scaling 0.25, offset 0 */
	x = (i >> 24) & 0xffff;
    engine_speed = (double)(x) * 0.25;

	/* motor_index: start-bit 0, length 8, endianess intel, scaling 1, offset 0 */
	x = i & 0xff;
	motor_index = (int)x;

	/* valid_check: start-bit 8, length 8, endianess intel, scaling 1, offset 0 */
	// x = (i >> 8) & 0xff;
	// uint8_t valid_check = (uint8_t)x;

	/* motor_status: start-bit 16, length 8, endianess intel, scaling 1, offset 0 */
	x = (i >> 16) & 0xff;
	motor_status = (int)x;
    // int drive_enable = (int)x & 0b00000001;
    // int charger_present = (int)x & 0b00000010;
    // int no_kill_switch = (int)x & 0b00001100;
    // int battery_connected = (int)x & 0b00010000;
    // int throttle_ready = (int)x & 0b00100000;
    // int master_ready = (int)x & 0b01000000;
    // int drive_ready  = (int)x & 0b10000000;

	/* motor_torque: start-bit 40, length 8, endianess intel, scaling 1, offset 0 */
	// x = (i >> 40) & 0xff;
	// uint8_t motor_torque = (uint8_t)x;

	/* motor_throttle: start-bit 48, length 8, endianess intel, scaling 1, offset 0 */
	// x = (i >> 48) & 0xff;
	// uint8_t motor_throttle = (uint8_t)x;

	/* motor_gear: start-bit 56, length 8, endianess intel, scaling 1, offset -125 */
	x = (i >> 56) & 0xff;
	motor_gear = (int)x - 0x7D;

    return;
}

void USVMotor::decode_pod_angle(const can_frame& frame, double& current_position) {
	register uint64_t x;
	register uint64_t i = *(uint64_t *)(frame.data);
	if (frame.can_dlc < 6)
		throw std::runtime_error("CAN frame data length is not enough.");

	/* current_position: start-bit 0, length 32, endianess intel, scaling 0.002, offset 0 */
	x = i & 0xffffffff;
	current_position = (double)(int32_t)x * 0.002;

	/* analog_input: start-bit 32, length 16, endianess intel, scaling 1, offset 0 */
	// x = (i >> 32) & 0xffff;
	// analog_input = (int16_t)x;

	return;
}