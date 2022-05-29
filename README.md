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
rosrun odrive_motor_control key_teleop.py
```
## STEP3
```
rosrun odrive_motor_control odrive_ros_control.py
```
If all goes well, the following message will appear on the terminal
```
Connect to Odrive...
Connect to Odrive Success!!!
```
