#ifndef SONAR_SENSOR_H
#define SONAR_SENSOR_H

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>

// Publisher handle
extern rcl_publisher_t sonar_pub;

// Setup hardware
void sonar_setup();

// Initialize ROS publisher
void sonar_publisher_init(rcl_node_t *node);

void sonar_update(std_msgs__msg__Float32 *msg);

// ROS timer callback
void sonar_timer_callback(rcl_timer_t * timer, int64_t last_call_time);

#endif // SONAR_SENSOR_H
