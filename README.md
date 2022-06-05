# odrive_motor_control
## scripts

odrive_config.py       : Odrive for BLDC motor Automatic configuration.  
odrive_twist_driver.py : Odrive subscrive /cmd_vel, then calculate /odom_buf.  
odom_publisher.py      : subscribe /odom_buf, then publish /odom.  
keyboard_teleop.py     : keyborad input, then publish /cmd_vel.  

## Environment
OS : Raspbian(buster)  
ROS : noetic

# motor configuration
```
cd ~/catkin_ws/src/odrive_motor_control/script
sudo python3 odrive_config.py
```

# motor operation check(Keyboard)
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
source ~/catkin_ws/devel/setup.bash
rosrun odrive_motor_control key_teleop.py
```
## STEP4
```
source ~/catkin_ws/devel/setup.bash
rosrun odrive_motor_control odrive_ros_control.py
```
If all goes well, the following message will appear on the terminal
```
Connect to Odrive...
Connect to Odrive Success!!!
```

## roslaunch version
```
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
roslaunch odrive_motor_control odrive_keyboard_control.launch
```

# odom publish
```
source ~/catkin_ws/devel/setup.bash
roslaunch odrive_motor_control odrive_control.launch
```

# motor operation check(joystick)
## STEP1
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
# STEP2
```
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
roslaunch odrive_motor_control odrive_joystick_control.launch
```
