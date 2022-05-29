# odrive_motor_control
odrive_motor_control

## Environment
OS : Raspbian(buster)  
ROS : noetic

# Execution
## STEP1
```
roscore
```

## STEP2
```
lsusb
Bus 001 Device 009: ID 1209:0d32 Generic ODrive Robotics ODrive v3
```

```
ls -al /dev/bus//usb//001/009
crw-rw-r-- 1 root root 189, 8 5月 29 14:49 /dev/bus//usb//001/009
```

```
sudo chmod 666 /dev/bus/usb/001/009
```

```
ls -al /dev/bus//usb//001/009
crw-rw-rw- 1 root root 189, 8 5月 29 14:49 /dev/bus//usb//001/009
```

## STEP3
```
rosrun odrive_motor_control key_teleop.py
```
## STEP4
```
rosrun odrive_motor_control odrive_ros_control.py
```
If all goes well, the following message will appear on the terminal
```
Connect to Odrive...
Connect to Odrive Success!!!
```
