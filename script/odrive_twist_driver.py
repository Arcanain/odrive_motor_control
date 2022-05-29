#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

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

        # subscriber cmd_vel
        rospy.Subscriber("cmd_vel", Twist, self.callback_vel)
        
        # publisher odom
        self.odom_pub = rospy.Publisher("odom_buf", Vector3, queue_size=1)
        self.odom = Vector3()
        self.odom.x = 0.0
        self.odom.y = 0.0
        self.odom.z = 0.0
        
        # setup parameter
        self.tread              = 0.4
        self.target_linear_vel  = 0.0
        self.target_angular_vel = 0.0
        self.right_wheel_radius = 0.08
        self.left_wheel_radius  = 0.08
        # <axis>.encoder.pos_estimate [turns]
        # https://docs.odriverobotics.com/v/latest/commands.html
        self.right_pos          = self.odrv0.axis0.encoder.pos_estimate/900.0*m.pi*self.right_wheel_radius
        self.left_pos           = -self.odrv0.axis1.encoder.pos_estimate/900.0*m.pi*self.left_wheel_radius
        self.last_right_pos     = self.right_pos
        self.last_left_pos      = self.left_pos

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
    
    def calcodom(self, right_delta_dist, left_delta_dist):
        delta_linear = (right_delta_dist + left_delta_dist)/2.0
        delta_yaw    = (right_delta_dist - left_delta_dist)/self.tread
        approximate_delta_linear = 0.0
        turn_rad = 0.0
        
        if abs(delta_yaw) < 245e-3:
            self.odom.x = self.odom.x + delta_linear * m.cos(self.odom.z + (delta_yaw/2.)) 
            self.odom.y = self.odom.y + delta_linear * m.sin(self.odom.z + (delta_yaw/2.))
            self.odom.z = self.odom.z + delta_yaw
        else:
            turn_rad = delta_linear/delta_yaw
            approximate_delta_linear = 2.0*turn_rad*m.sin((delta_yaw/2.))
            self.odom.x = self.odom.x +  approximate_delta_linear * m.cos(self.odom.z + (delta_yaw/2.))
            self.odom.y = self.odom.y + approximate_delta_linear * m.sin(self.odom.z + (delta_yaw/2.))
            self.odom.z = self.odom.z + delta_yaw
            
        self.odom_pub.publish(self.odom)
        
    def odrive_control(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # calculate relative velocity
            right_vel, left_vel = self.calc_relative_vel(self.target_linear_vel, self.target_angular_vel)

            # convert to pulse
            right_pulse = right_vel/(m.pi*self.right_wheel_radius)*900.0
            left_pulse  = left_vel/(m.pi*self.left_wheel_radius)*900.0
            
            try:
                # Get current position
                self.right_pos = self.odrv0.axis0.encoder.pos_estimate/900.0*m.pi*self.right_wheel_radius
                self.left_pos  = -self.odrv0.axis1.encoder.pos_estimate/900.0*m.pi*self.left_wheel_radius
                right_pos_diff = self.right_pos - self.last_right_pos
                left_pos_diff  = self.left_pos - self.last_left_pos
                self.calcodom(right_pos_diff, left_pos_diff)
                
                print(self.odrv0.axis0.encoder.pos_estimate)

                # Set velocity
                self.odrv0.axis0.controller.input_vel = right_pulse
                self.odrv0.axis1.controller.input_vel = -left_pulse
                
                # Update last pos
                self.last_right_pos = self.right_pos
                self.last_left_pos = self.left_pos
                
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