# Easy Drive

A easy to use teleop robot drive system that utilizes skid steering technique to move.

##  Usage
```
$ roslaunch easy_drive drive_teleop.launch joy_id:=/dev/input/js0 serial_id:=/dev/ttyACM0
$ roslaunch easy_drive cmd_vel_mux.launch
```
OR all-at-once:
```
$ roslaunch easy_drive drive.launch joy_id:=/dev/input/js0 serial_id:=/dev/ttyACM0
```

## Future Improvements
- Add features for autonomous mode to make it a full druve system

For any questions or concerns please email [woudie](https://github.com/woudie) @ stwoudie@gmail.com