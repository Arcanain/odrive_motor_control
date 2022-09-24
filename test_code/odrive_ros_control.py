#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

import sys
import time
import odrive
from odrive.enums import *
import fibre.libfibre
import math as m

class OdriveMotorControl:
    def __init__(self):
        # Connect to Odrive
        self.find_odrive()

        # subscriber
        rospy.Subscriber("cmd_vel", Twist, self.callback_vel)
        
        # setup parameter
        self.tread              = 0.4
        self.target_linear_vel  = 0.0
        self.target_angular_vel = 0.0
        self.right_wheel_radius = 0.08
        self.left_wheel_radius  = 0.08
        
    def find_odrive(self):
        while True:
            print("Connect to Odrive...")
            self.odrv0 = odrive.find_any()
            if self.odrv0 is not None:
                print("Connect to Odrive Success!!!")
                break
            else:
                print("Disconnect to Odrive...")
    
    def odrive_setup(self):
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.input_vel = 0
        self.odrv0.axis1.controller.input_vel = 0
    
    def odrive_control(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # calculate relative velocity
            right_vel, left_vel = self.calc_relative_vel(self.target_linear_vel, self.target_angular_vel)

            # convert to pulse
            right_pulse = right_vel/(m.pi*self.right_wheel_radius)*900.
            left_pulse  = left_vel/(m.pi*self.left_wheel_radius)*900.
            
            try:
                # Set velocity
                self.odrv0.axis0.controller.input_vel = right_pulse
                self.odrv0.axis1.controller.input_vel = -left_pulse
                
                rate.sleep()
            except AttributeError as error:
                # Output expected AttributeErrors.
                rospy.signal_shutdown(error)
            except KeyboardInterrupt:
                self.odrv0.axis0.controller.input_vel = 0
                self.odrv0.axis1.controller.input_vel = 0
                rospy.signal_shutdown("KeyboardInterrupt")
                
    def calc_relative_vel(self, target_linear_vel, target_angular_vel):
        # Convert to each vel
        right_vel = target_linear_vel + (self.tread / 2.0) * target_angular_vel
        left_vel  = target_linear_vel - (self.tread / 2.0) * target_angular_vel
        
        return right_vel, left_vel
    
    def callback_vel(self, msg):
        self.target_linear_vel = msg.linear.x
        self.target_angular_vel = msg.angular.z
        
if __name__ == "__main__":
    rospy.init_node("odrive_motor_control", disable_signals=True)
    Odrive_motor_control = OdriveMotorControl()
    Odrive_motor_control.odrive_setup()
    Odrive_motor_control.odrive_control()
    rospy.spin()