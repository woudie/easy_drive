# Easy Drive

A easy to use teleop robot drive system that utilizes skid steering technique to move.

##  Usage
First and foremost, identify the device names, typically the controller shows up as `/dev/input/js0` and the microcontroller as `/dev/ttyUSB0`. 
You can use `ls /dev | grep tty` and `ls /dev/input` to check. 
## Basestation
On the basestation/controller side, do the following:
```
$ roslaunch easy_drive drive_teleop.launch joy_id:=/dev/input/js0
$ roslaunch easy_drive cmd_vel_mux.launch
```
OR all-at-once:
```
$ roslaunch easy_drive drive.launch joy_id:=/dev/input/js0
```
## Robot
On the robot side, Make sure you have compiled and uploaded the firmware provided in the `drive_firmware` or `drive_firmware_alt` to your microcontroller. 

Afterwards do the following:
```
roslaunch easy_drive drive_robot.launch serial_id:=/dev/ttyUSB0
```
## Future Improvements
- Add features for autonomous mode to make it a full drive system

For any questions or concerns please email [woudie](https://github.com/woudie) @ stwoudie@gmail.com
