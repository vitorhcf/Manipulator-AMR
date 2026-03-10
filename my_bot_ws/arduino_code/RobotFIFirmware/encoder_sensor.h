#ifndef ENCODER_SENSOR_H
#define ENCODER_SENSOR_H

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32_multi_array.h>

// Publisher for encoder angles
extern rcl_publisher_t encoder_publisher;

// Function prototypes
void encoder_init();
float encoder_read_degrees_right(float &voltage);
float encoder_read_degrees_left(float &voltage);
void encoder_publish_angles(rcl_publisher_t *publisher);
void encoder_timer_callback(rcl_timer_t * timer, int64_t last_call_time);

#endif
