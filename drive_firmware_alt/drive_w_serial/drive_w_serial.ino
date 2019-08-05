/* Standard Imports */
#include <Arduino.h>
#include <Sabertooth.h>
#include <SoftwareSerial.h>

/* ROS imports */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>


/**
 * Arduino Mega Pin Definitions
 * ================================================
 * A0 -> Battery 1 Cell 1
 * A1 -> Battery 1 Cell 2
 * A2 -> Battery 1 Cell 3
 * A3 -> Battery 1 Cell 4
 * A4 -> Battery 1 Cell 5
 * A5 -> Battery 1 Cell 6
 * A6 -> Battery 2 Cell 1
 * A7 -> Battery 2 Cell 2
 * A8 -> Battery 2 Cell 3
 * A9 -> Battery 2 Cell 4
 * A10 -> Battery 2 Cell 5
 * A11 -> Battery 2 Cell 6
 * Pin 48 -> High Beam Lights output
 * Pin 49 -> Low Beam Lights output
 * Pin 50 -> Reverse Beam Lights output
 */

/**
 * ===================================
 *              NOTES
 * ===================================
 * Battery Monitor is untested, set use_bat to 0 to disable function
 * Light Control is untested, set use_lights to 0 to disable function
 */

SoftwareSerial SWSerial(NOT_A_PIN, 14);
SoftwareSerial pinROS(NOT_A_PIN, 1);
void drive_callback(const geometry_msgs::Twist &drive_msg);

#define FR 1
#define FL 2
#define BL 1
#define BR 2

int use_batt = 1, use_lights = 1;

Sabertooth FrontST(128, SWSerial); //Address 128 Dip Switches (000111)
Sabertooth RearST(129, SWSerial);  //Address 130 Dip Switches (000101) Address 129 did not work for some reason

// PWM specs of Sabertooth Motor Controller (adjust accordingly for your motors)
#define controllerMax 70  // Default full-reverse input pulse
#define controllerMin -70 // Default full-forward input pulse

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", drive_callback);
ros::Subscriber<std_msgs::Int16> sub1("high_beams", h_callback);
ros::Subscriber<std_msgs::Int16> sub2("low_beams", l_callback);
ros::Subscriber<std_msgs::Int16> sub3("reverse_beams", r_callback);

std_msg::Float32MultiArray bat1[6];
ros::Publisher pub1("battery_monitor/bat1", &bat1);

std_msg::Float32MultiArray bat2[6];
ros::Publisher pub2("battery_monitor/bat2", &bat2);

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

  void h_callback(const std_msgs::Int16 &hmsg){
    if(hmsg.data == 1){
      digitalWrite(48, HIGH);
    }else{
      digitalWrite(48, LOW);
    }
  }
  void l_callback(const std_msgs::Int16 &lmsg){
    if(lmsg.data == 1){
      digitalWrite(49, HIGH);
    }else{
      digitalWrite(49, LOW);
    }
  }
  void r_callback(const std_msgs::Int16 &rmsg){
    if(rmsg.data == 1){
      digitalWrite(50, HIGH);
    }else{
      digitalWrite(50, LOW);
    }
  }


void setup()
{
  SWSerial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
  pinROS.begin(57600);
  //nh.getHardware()->setBaud(115200);
  FrontST.autobaud();
  RearST.autobaud();
  
  pinMode(48, OUTPUT);
  pinMode(49, OUTPUT);
  pinMode(50, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  if(use_lights == 1){
    nh.subscribe(sub1);
    nh.subscribe(sub2);
    nh.subscribe(sub3);
  }

  if( use_batt == 1){  
    nh.advertise(pub1);
    nh.advertise(pub2);
  }
}

void battery_read(){
    bat1[0] = analogRead(A0)
    bat1[1] = analogRead(A1)
    bat1[2] = analogRead(A2)
    bat1[3] = analogRead(A3)
    bat1[4] = analogRead(A4)
    bat1[5] = analogRead(A5)
    
    bat2[0] = analogRead(A6)
    bat2[1] = analogRead(A7)
    bat2[2] = analogRead(A8)
    bat2[3] = analogRead(A9)
    bat2[4] = analogRead(A10)
    bat2[5] = analogRead(A11)
}

void loop()
{
  if(use_batt == 1){
    battery_read();
  }
  nh.spinOnce();
  delay(1);
}