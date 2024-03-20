# motor_ros_driver

The **motor_ros_driver** is a ROS package for reading and sending messages with two CAN torqeedo thrusters.

The package uses [can_device_interface](https://github.com/CPT-KK/can_device_interface) to interact with the CAN bus.

## Prerequisite

- Ubuntu 20.04 with ROS noetic
- can-utils: Linux-CAN / SocketCAN user space applications. Can be installed via `sudo apt install can-utils`.
- cmake

## Available ROS topics

After building and running this package, the following ROS topics are available.

- `/usv/torqeedo/left/estimate`: From CAN to ROS. Giving the current RPM of the left torqeedo.
- `/usv/torqeedo/right/estimate`: From CAN to ROS. Giving the current RPM of the right torqeedo.
- `/usv/torqeedo/left/status`: From CAN to ROS. Giving the current left torqeedo state.
- `/usv/torqeedo/right/status`: From CAN to ROS. Giving the current right torqeedo state.
- `/usv/torqeedo/left/setpoint`: From ROS to CAN. Sending the desired RPM of the left torqeedo.
- `/usv/torqeedo/right/setpoint`: From ROS to CAN. Sending the desired RPM of the right torqeedo.
- `/usv/pod/left/estimate`: From CAN to ROS. Giving the current angle of the left torqeedo.
- `/usv/pod/right/estimate`: From CAN to ROS. Giving the current angle of the right torqeedo.
- `/usv/pod/left/setpoint`: From ROS to CAN. Sending the desired angle of the left torqeedo.
- `/usv/pod/right/setpoint`: From ROS to CAN. Sending the desired angle of the right torqeedo.

The package also monitors the status of the battery on the USV and sends them to ROS topics. Details are omitted.
