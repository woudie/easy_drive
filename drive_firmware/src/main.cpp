/* Standard Imports */
#include <Arduino.h>
#include <Servo.h>

/* ROS imports */
#include <ros.h>
#include <geometry_msgs/Twist.h>

void drive_callback(const geometry_msgs::Twist& drive_msg);

#define pinL1 5
#define pinL2 6
#define pinR1 10
#define pinR2 11


Servo leftRear;
Servo leftFront;
Servo rightRear;
Servo rightFront;

// PWM specs of Sabertooth Motor Controller (adjust accordingly for your motors)
#define controllerMax 1000   // Default full-reverse input pulse
#define controllerMin 2000 // Default full-forward input pulse

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", drive_callback);

void setWheelVelocity(int left, int right)
{
    int theLeft = map(left, -100, 100, controllerMin, controllerMax);
    int theRight = map(right, -100, 100, controllerMin, controllerMax);

    leftRear.writeMicroseconds(theLeft);
    leftFront.writeMicroseconds(theLeft);
    rightRear.writeMicroseconds(theRight);
    rightFront.writeMicroseconds(theRight);
}

void drive_callback(const geometry_msgs::Twist &drive_msg)
{
    
    double lin = drive_msg.linear.x;
    double ang = drive_msg.angular.z;
        
    setWheelVelocity((int)((lin + ang) * 100), (int)((lin - ang) * 100));
}

void setup(){
    Serial.begin(57600);

    leftRear.attach(pinL1);
    leftFront.attach(pinL2);
    rightRear.attach(pinR1);
    rightFront.attach(pinR2);

    nh.initNode();
    nh.subscribe(sub);
}

void loop(){
    nh.spinOnce();
    delay(1);
}