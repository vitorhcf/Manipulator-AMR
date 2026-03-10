#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS 2 subscription for /cmd_vel
extern rcl_subscription_t cmd_vel_sub;
extern geometry_msgs__msg__Twist cmd_vel_msg;

// Setup hardware and pins
void motors_setup();

// ROS 2 callback for cmd_vel
void motors_cmd_vel_callback(const void *msgin);

#endif
