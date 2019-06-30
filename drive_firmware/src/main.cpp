/* Standard Imports */
#include <Arduino.h>
#include <Servo.h>

/* ROS imports */
#include <ros.h>
#include <geometry_msgs/Twist.h>

void drive_callback(const geometry_msgs::Twist& drive_msg);

#define pinL 5
#define pinR 6

Servo leftMotors;
Servo rightMotors;

// PWM specs of L298N H-Bridge (Change according to your motor specs)
#define controllerMax 1000   // Default full-reverse input pulse
#define controllerMin 2000 // Default full-forward input pulse

ros::NodeHandle nh;
double lin, ang;

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", drive_callback);

void setWheelVelocity(int left, int right)
{
    int theLeft = map(left, -100, 100, controllerMin, controllerMax);
    int theRight = map(right, -100, 100, controllerMin, controllerMax);

    leftMotors.writeMicroseconds(theLeft);
    rightMotors.writeMicroseconds(theRight);

}

void drive_callback(const geometry_msgs::Twist &drive_msg)
{
    
    lin = drive_msg.linear.x;
    ang = drive_msg.angular.z;
        
    setWheelVelocity((int)((lin + ang) * 100), (int)((lin - ang) * 100)*(-1));
}

void setup(){
    Serial.begin(57600);

    leftMotors.attach(pinL);
    rightMotors.attach(pinR);

    nh.initNode();
    nh.subscribe(sub);
}

void loop(){
    nh.spinOnce();
    delay(1);
}