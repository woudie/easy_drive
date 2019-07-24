#!/usr/bin/python

import rospy
from dynamic_reconfigure.client import Client

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class DriveTeleop:
    def __init__(self):
        self.clt = Client("drive_param_server", timeout=30, config_callback=self.d_callback)
        self.drive_state = 1
        self.control_mode = 0
        self.speed_setting = 2  # default to medium speed
        self.cmd_vel_pub = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=10)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)

    def d_callback(self, config):
        self.drive_state = config['drive_state']
        self.control_mode = config['control_state']
        
    def on_joy(self, data):
        if data.buttons[6] == 1:
            self.drive_state = 1 - self.drive_state
            self.clt.update_configuration({"drive_state": self.drive_state})
        if data.buttons[7] == 1:
            self.control_mode = 1 - self.control_mode
            self.clt.update_configuration({"control_state": self.drive_state})
            
        if self.drive_state == 1 and self.control_mode == 0:
            # Set speed ratio using d-pad
            if data.axes[7] == 1:  # full speed (d-pad up)
                self.speed_setting = 1
                self.clt.update_configuration({"speed_setting":1})
            if data.axes[6] != 0:  # medium speed (d-pad left or right)
                self.speed_setting = 2
                self.clt.update_configuration({"speed_setting": 2})
            if data.axes[7] == -1:  # low speed (d-pad down)
                self.speed_setting = 3
                self.clt.update_configuration({"speed_setting": 3})

            # Drive sticks
            left_speed = data.axes[1] / self.speed_setting  # left stick
            right_speed = data.axes[4] / self.speed_setting  # right stick

            # Convert skid steering speeds to twist speeds
            linear_vel = (left_speed + right_speed) / 2.0  # (m/s)
            angular_vel = (right_speed - left_speed) / 2.0  # (rad/s)

            # Publish Twist
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist)


def main():
    rospy.init_node("drive_teleop")
    controller = DriveTeleop()
    rospy.spin()
