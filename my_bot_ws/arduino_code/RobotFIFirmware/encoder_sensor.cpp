#include "encoder_sensor.h"
#include <Arduino.h>
#include <math.h>
#include <rosidl_runtime_c/string_functions.h> 
#include <rcl/time.h>
#include <rcl/allocator.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32_multi_array.h>

#define DEG_TO_RAD (M_PI / 180.0)

const int analogPin_right = 35;
const int analogPin_left = 34;

std_msgs__msg__Float32MultiArray encoder_msg;

void encoder_init() {
  // Initialize message
  encoder_msg.data.capacity = 2;
  encoder_msg.data.size = 2;
  encoder_msg.data.data = (float*)malloc(sizeof(float) * 2);
}

float encoder_read_degrees_right(float &voltage) {
  int rawValue = analogRead(analogPin_right);
  voltage = rawValue * (3.3 / 4095.0); // Convert to volts
  float degrees = (rawValue / 4095.0) * 360.0; // Convert to degrees
  return degrees;
}

float encoder_read_degrees_left(float &voltage) {
  int rawValue = analogRead(analogPin_left);
  voltage = rawValue * (3.3 / 4095.0); // Convert to volts
  float degrees = 360.0 - (rawValue / 4095.0) * 360.0; // Convert to degrees
  return degrees;
}

void encoder_publish_angles(rcl_publisher_t *publisher) {
  float voltage; // not used
  float left_angle = encoder_read_degrees_left(voltage);
  float right_angle = encoder_read_degrees_right(voltage);

  // Debug
  Serial.print("\n[ENC] Left: ");
  Serial.print(left_angle, 2);
  Serial.print(" deg  Right: ");
  Serial.println(right_angle, 2);

  // Fill message
  encoder_msg.data.data[0] = left_angle;
  encoder_msg.data.data[1] = right_angle;

  // Publish
  rcl_publish(publisher, &encoder_msg, NULL);
}

void encoder_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    if (timer == NULL) {
        return;
    }

    // Just publish encoder angles
    encoder_publish_angles(&encoder_publisher);
}
