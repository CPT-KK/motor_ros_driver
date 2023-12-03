cmake_minimum_required(VERSION 3.8)
project(motor_ros_driver VERSION 1.0.0 LANGUAGES CXX)

add_compile_options(-std=c++11)

# Set the build type
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy std_msgs
)

aux_source_directory(src DIR_SRCS)
add_executable(${PROJECT_NAME} ${DIR_SRCS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  CANDevice
)

add_subdirectory(external/can_device_interface)