#include "sonar_sensor.h"

std_msgs__msg__Float32 sonar_msg;

#define TRIG_PIN 5
#define ECHO_PIN 18

void sonar_setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void sonar_publisher_init(rcl_node_t *node) {
  rclc_publisher_init_default(
    &sonar_pub,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "sonar_distance"
  );
}

float read_distance_cm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.0343 / 2.0;
}

void sonar_update(std_msgs__msg__Float32 *msg) {
    msg->data = read_distance_cm();
}

void sonar_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    sonar_msg.data = read_distance_cm();
    rcl_publish(&sonar_pub, &sonar_msg, NULL);
  }
}
