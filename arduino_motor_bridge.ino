/*
 * ROS Arduino Motor Bridge for L293D
 * Subscribes to /cmd_vel and drives motors
 * No encoders - open loop control
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>

// L293D Motor Driver Pins
#define ENA 3   // PWM speed control Motor A
#define IN1 5
#define IN2 6
#define ENB 9   // PWM speed control Motor B
#define IN3 7
#define IN4 8

// Robot parameters (adjust based on your bot)
#define WHEEL_SEPARATION 0.15  // meters (distance between wheels)
#define MAX_PWM 255

ros::NodeHandle nh;

// Motor control function
void setMotorSpeed(int motor, int speed) {
  // motor: 0 = left (A), 1 = right (B)
  // speed: -255 to 255
  
  int in1, in2, en;
  
  if (motor == 0) {  // Left motor (A)
    in1 = IN1;
    in2 = IN2;
    en = ENA;
  } else {           // Right motor (B)
    in1 = IN3;
    in2 = IN4;
    en = ENB;
  }
  
  if (speed > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (speed < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  
  analogWrite(en, abs(speed));
}

// Callback for /cmd_vel
void cmdVelCallback(const geometry_msgs::Twist& cmd_msg) {
  float linear = cmd_msg.linear.x;   // m/s
  float angular = cmd_msg.angular.z; // rad/s
  
  // Differential drive kinematics
  // v_left = linear - angular * (wheel_sep / 2)
  // v_right = linear + angular * (wheel_sep / 2)
  
  float v_left = linear - angular * (WHEEL_SEPARATION / 2.0);
  float v_right = linear + angular * (WHEEL_SEPARATION / 2.0);
  
  // Convert to PWM (-255 to 255) - simple linear mapping
  // Assuming max linear speed is ~0.5 m/s for scaling
  int pwm_left = (int)(v_left * 510);   // scale factor
  int pwm_right = (int)(v_right * 510);
  
  // Clamp values
  pwm_left = constrain(pwm_left, -MAX_PWM, MAX_PWM);
  pwm_right = constrain(pwm_right, -MAX_PWM, MAX_PWM);
  
  setMotorSpeed(0, pwm_left);
  setMotorSpeed(1, pwm_right);
  
  // Debug via serial (optional)
  // nh.loginfo("Motor PWM set");
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCallback);

void setup() {
  // Pin setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Stop motors initially
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  
  // ROS setup
  nh.initNode();
  nh.subscribe(sub);
  
  // Wait for serial (optional, for debugging)
  // Serial.begin(57600);
}

void loop() {
  nh.spinOnce();
  delay(10);  // ~100Hz update rate
}
