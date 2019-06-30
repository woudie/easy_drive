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

void motor_set(int num){
    switch(num){
        case 1: // Forward
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);
            break;
        case 2: // Backward
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            break;
        case 3: // Turn Right
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            break;
        case 4: // Turn Left
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);
            break;
        default: // Stop
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, LOW);
            break;
    }
}

void setWheelVelocity(int left, int right)
{
    int theLeft = map(left, -100, 100, controllerMin, controllerMax);
    int theRight = map(right, -100, 100, controllerMin, controllerMax);

    leftMotors.writeMicrosecond(theLeft);
    rightMotors.writeMicrosecond(theRight);

}

void drive_callback(const geometry_msgs::Twist &drive_msg)
{
    
    lin = drive_msg.linear.x;
    ang = drive_msg.angular.z;
        
    setWheelVelocity((int)((lin + ang) * 100), (int)((lin - ang) * 100));
}

void setup(){
    Serial.begin(57600);

    Servo.attach(pinL);
    Servo.attach(pinR);

    nh.initNode();
    nh.subscribe(sub);
}

void loop(){
    nh.spinOnce();
    delay(1);
}