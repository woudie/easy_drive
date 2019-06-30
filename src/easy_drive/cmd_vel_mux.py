#!/usr/bin/python

import rospy
import time

from geometry_msgs.msg import Twist

class CmdVelMux:
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("teleop/cmd_vel", Twist, callback=self.on_teleop_cmd)

    def on_teleop_cmd(self, twist):
        self.pub.publish(twist)

def main():
    rospy.init_node("cmd_vel_mux")
    muxer = CmdVelMux()
    rospy.spin()
