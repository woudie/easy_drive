/* Standard Imports */
#include <Arduino.h>
#include <Sabertooth.h>
#include <SoftwareSerial.h>

/* ROS imports */
#include <ros.h>
#include <geometry_msgs/Twist.h>

SoftwareSerial SWSerial(NOT_A_PIN, 14);
SoftwareSerial pinROS(NOT_A_PIN, 1);
void drive_callback(const geometry_msgs::Twist &drive_msg);

#define FR 1
#define FL 2
#define BL 1
#define BR 2

Sabertooth FrontST(128, SWSerial); //Address 128 Dip Switches (000111)
Sabertooth RearST(129, SWSerial);  //Address 130 Dip Switches (000101) Address 129 did not work for some reason

// PWM specs of Sabertooth Motor Controller (adjust accordingly for your motors)
#define controllerMax 70  // Default full-reverse input pulse
#define controllerMin -70 // Default full-forward input pulse

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", drive_callback);

void setWheelVelocity(int left, int right)
{
  int theLeft = map(left, -100, 100, controllerMin, controllerMax);
  int theRight = map(right, -100, 100, controllerMin, controllerMax);

  FrontST.motor(FL, theLeft);
  RearST.motor(BL, theLeft);
  FrontST.motor(FR, theRight);
  RearST.motor(BR, theRight);
}

void drive_callback(const geometry_msgs::Twist &drive_msg)
{

  double lin = drive_msg.linear.x;
  double ang = drive_msg.angular.z;

  setWheelVelocity((int)((lin + ang) * 100), (int)((lin - ang) * 100));
}

void setup()
{
  SWSerial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
  //pinROS.begin(57600);
  nh.getHardware()->setBaud(115200);
  FrontST.autobaud();
  RearST.autobaud();
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}