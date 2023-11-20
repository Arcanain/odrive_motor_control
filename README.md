# odrive_motor_control
## scripts

odrive_config.py       : Odrive for BLDC motor Automatic configuration.  
odrive_twist_driver.py : Odrive subscrive /cmd_vel, then calculate /odom and /odom_path  
keyboard_teleop.py     : keyborad input, then publish /cmd_vel.  

## Environment
OS : Ubuntu Mate  
ROS : noetic

## composition
![Untitled diagram-2023-11-20-053818](https://github.com/Arcanain/odrive_motor_control/assets/52307432/7855f467-9c50-4226-a4b3-aa8d1887d376)

# motor configuration
```
cd ~/catkin_ws/src/odrive_motor_control/script
sudo python3 odrive_config.py
```

# motor operation check(Keyboard)

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

## STEP2
```
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
roslaunch odrive_motor_control odrive_keyboard_control.launch
```

If all goes well, the following message will appear on the terminal
```
Connect to Odrive...
Connect to Odrive Success!!!
```

## rviz
![Screenshot at 2022-09-25 04-22-02](https://user-images.githubusercontent.com/52307432/192115079-4b98a837-ac96-4961-88ec-0cbac8609a34.png)

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
## STEP2
```
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
roslaunch odrive_motor_control odrive_joystick_control.launch
```
