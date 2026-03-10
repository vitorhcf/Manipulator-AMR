#include "motors.h"
#include <Arduino.h>
#include <geometry_msgs/msg/twist.h>

#define ENA 14
#define IN1 27
#define IN2 26
#define ENB 25
#define IN3 33
#define IN4 32
#define MAX_PWM 180
#define PWM_LEFT_CHANNEL 0
#define PWM_RIGHT_CHANNEL 1
#define PWM_FREQ 20000
#define PWM_RES 8
const float WHEEL_BASE = 0.13;
const float MAX_LINEAR_SPEED = 0.225;

int velocityToPWM(float velocity) {
  int pwm = abs(velocity) * MAX_PWM / MAX_LINEAR_SPEED;
  const int MIN_WORKING_PWM = 75;
  if (pwm > 0 && pwm < MIN_WORKING_PWM) pwm = MIN_WORKING_PWM;
  return constrain(pwm, 0, MAX_PWM);
}

void setMotor(int in1, int in2, int pwm_channel, float velocity) {
  int pwm_value = velocityToPWM(velocity);
  digitalWrite(in1, velocity >= 0 ? HIGH : LOW);
  digitalWrite(in2, velocity >= 0 ? LOW : HIGH);
  ledcWrite(pwm_channel, pwm_value);
}

void motors_setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  ledcSetup(PWM_LEFT_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_LEFT_CHANNEL);
  ledcSetup(PWM_RIGHT_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, PWM_RIGHT_CHANNEL);

  ledcWrite(PWM_LEFT_CHANNEL, 0);
  ledcWrite(PWM_RIGHT_CHANNEL, 0);
}

void motors_cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *twist = (const geometry_msgs__msg__Twist *)msgin;
  float linear = twist->linear.x;
  float angular = twist->angular.z;
  float left_speed = linear - (angular * WHEEL_BASE / 2.0);
  float right_speed = linear + (angular * WHEEL_BASE / 2.0);

  setMotor(IN1, IN2, PWM_LEFT_CHANNEL, left_speed);
  setMotor(IN3, IN4, PWM_RIGHT_CHANNEL, right_speed);
}
