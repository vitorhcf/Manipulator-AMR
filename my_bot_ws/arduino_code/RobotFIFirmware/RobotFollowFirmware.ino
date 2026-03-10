#include <Arduino.h>
#include <Wire.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float32.h>

#include "motors.h"
#include "encoder_sensor.h"
#include "sonar_sensor.h"

// === Wi-Fi Config ===
#define WIFI_SSID     "FI Robotics_(Workstation)"
#define WIFI_PASSWORD "248@F1nim-01"
#define AGENT_IP      "192.168.1.29"
#define AGENT_PORT    8888

// Micro-ROS objects
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t sonar_timer;
rcl_timer_t odom_timer;

// Subscriptions and publishers
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_publisher_t encoder_publisher;
rcl_publisher_t sonar_pub;


// Convenience macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.print("Failed status on line "); Serial.println(__LINE__); return;} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.print("Soft fail on line "); Serial.println(__LINE__);} }


// ================= Setup =================
void setup() {
  Serial.begin(115200);
  delay(2000);
  // Connect to Micro-ROS agent
  set_microros_transports();
  delay(2000);

  // Initialize actuators and sensors
  motors_setup();
  sonar_setup();
  encoder_init();

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

  // Subscription
  RCCHECK(rclc_subscription_init_best_effort(&cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_publisher_init_best_effort(&sonar_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "sonar/range"));

  RCCHECK(rclc_publisher_init_best_effort(&encoder_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "encoders/angles"));

  // Timers
  RCCHECK(rclc_timer_init_default2(&sonar_timer, &support, RCL_MS_TO_NS(100), sonar_timer_callback, true));
  RCCHECK(rclc_timer_init_default2(&odom_timer, &support, RCL_MS_TO_NS(20), encoder_timer_callback, true));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &motors_cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &sonar_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
}

// ================= Loop =================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

}
