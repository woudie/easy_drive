/* Standard Imports */
#include <Arduino.h>
#include <Servo.h>

/* ROS imports */
#include <ros.h>
#include <geometry_msgs/Twist.h>

void drive_callback(const geometry_msgs::Twist& drive_msg);

// Left Motor Control
#define enA 5
#define in1 7
#define in2 8

// Right Motor Control
#define enB 6
#define in3 9
#define in4 11

// PWM specs of L298N H-Bridge (Change according to your motor specs)
#define controllerMax 0   // Default full-reverse input pulse
#define controllerMin 255 // Default full-forward input pulse

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

    if (left > 0.0 && right > 0.0)
    { // Going Forwards
        motor_set(1);
        analogWrite(enA, 255 - theLeft);
        analogWrite(enB, 255 - theRight);
    }
    else if (left < 0 && right < 0)
    { // Going Backwards
        motor_set(2);
        analogWrite(enA, theLeft);
        analogWrite(enB, theRight);
    }
    else if (left >= 0 && right <= 0)
    { // Turning Right
        motor_set(3);
        analogWrite(enA, 255 - theLeft);
        analogWrite(enB, 255 - theRight);
    }
    else if (left <= 0 && right >= 0)
    { // Turning Left
        motor_set(4);
        analogWrite(enA, 255 - theLeft);
        analogWrite(enB, 255 - theRight);
    }
    else
    {
        motor_set(0);
        analogWrite(enA, 0);
        analogWrite(enB, 0);
    }
}

void drive_callback(const geometry_msgs::Twist &drive_msg)
{

    lin = -1.0 * drive_msg.linear.x;
    ang = -1.0*drive_msg.angular.z;

    if (lin != 0.0 || ang != 0.0)
    {
        setWheelVelocity((int)((lin + ang) * 100), (int)((lin - ang) * 100));
    }
    else
    {
        motor_set(0);
    }
}

void setup(){
    Serial.begin(57600);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    nh.initNode();
    nh.subscribe(sub);
}

void loop(){
    nh.spinOnce();
    delay(1);
}