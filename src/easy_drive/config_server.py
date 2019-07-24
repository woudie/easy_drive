#!/usr/bin/env python
import rospy

from dynamic_reconfigure.server import Server
from easy_drive.cfg import drive_teleopConfig

def srv_callback(config, level):
    rospy.loginfo(config)
    return config

def main():
    rospy.init_node("drive_param_server_node")
    srv = Server(drive_teleopConfig, srv_callback)
    rospy.spin()
    
if __name__ == '__main__':    
    main()