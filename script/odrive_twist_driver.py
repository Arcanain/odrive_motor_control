#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

import sys
import time
import odrive
from odrive.enums import *
import fibre.libfibre
import math

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
        self.tire_tread         = 0.32                      #[m]
        self.target_linear_vel  = 0.0                       #[m/s]
        self.target_angular_vel = 0.0                       #[rad/s]
        self.tire_diameter      = 0.165                     #[m]
        self.right_wheel_radius = self.tire_diameter        #[m]
        self.left_wheel_radius  = self.tire_diameter        #[m]
        self.encoder_cpr        = 90.0                      #[count]
        self.tire_circumference = math.pi * self.tire_diameter #[m]

        # <axis>.encoder.pos_estimate [turns]
        # https://docs.odriverobotics.com/v/latest/commands.html
        #self.right_pos          = self.odrv0.axis0.encoder.pos_estimate/900.0*m.pi*self.right_wheel_radius
        #self.left_pos           = -self.odrv0.axis1.encoder.pos_estimate/900.0*m.pi*self.left_wheel_radius
        self.right_pos          = self.odrv0.axis0.encoder.pos_estimate/self.encoder_cpr*math.pi*self.right_wheel_radius
        self.left_pos           = -self.odrv0.axis1.encoder.pos_estimate/self.encoder_cpr*math.pi*self.left_wheel_radius
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
        delta_yaw    = (right_delta_dist - left_delta_dist)/self.tire_tread
        approximate_delta_linear = 0.0
        turn_rad = 0.0
        
        if abs(delta_yaw) < 245e-3:
            self.odom.x = self.odom.x + delta_linear * math.cos(self.odom.z + (delta_yaw/2.)) 
            self.odom.y = self.odom.y + delta_linear * math.sin(self.odom.z + (delta_yaw/2.))
            self.odom.z = self.odom.z + delta_yaw
        else:
            turn_rad = delta_linear/delta_yaw
            approximate_delta_linear = 2.0*turn_rad*math.sin((delta_yaw/2.))
            self.odom.x = self.odom.x +  approximate_delta_linear * math.cos(self.odom.z + (delta_yaw/2.))
            self.odom.y = self.odom.y + approximate_delta_linear * math.sin(self.odom.z + (delta_yaw/2.))
            self.odom.z = self.odom.z + delta_yaw
            
        self.odom_pub.publish(self.odom)
        
    def odrive_control(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # calculate relative velocity
            right_vel, left_vel = self.calc_relative_vel(self.target_linear_vel, self.target_angular_vel)

            # convert to pulse
            #right_pulse = right_vel/(m.pi*self.right_wheel_radius)*900.0
            #left_pulse  = left_vel/(m.pi*self.left_wheel_radius)*900.0

            #right_pulse = right_vel/(m.pi*self.right_wheel_radius)*self.encoder_cpr
            #left_pulse  = left_vel/(m.pi*self.left_wheel_radius)*self.encoder_cpr
            
            try:
                # Get current position
                #self.right_pos = self.odrv0.axis0.encoder.pos_estimate/900.0*m.pi*self.right_wheel_radius
                #self.left_pos  = -self.odrv0.axis1.encoder.pos_estimate/900.0*m.pi*self.left_wheel_radius
                self.right_pos = self.odrv0.axis0.encoder.pos_estimate/self.encoder_cpr*math.pi*self.right_wheel_radius
                self.left_pos  = -self.odrv0.axis1.encoder.pos_estimate/self.encoder_cpr*math.pi*self.left_wheel_radius
                right_pos_diff = self.right_pos - self.last_right_pos
                left_pos_diff  = self.left_pos - self.last_left_pos
                self.calcodom(right_pos_diff, left_pos_diff)
                
                #print(self.odrv0.axis0.encoder.pos_estimate)

                # Set velocity
                self.odrv0.axis0.controller.input_vel = right_vel
                self.odrv0.axis1.controller.input_vel = -left_vel
                
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
        #right_vel = target_linear_vel + (self.tire_tread / 2.0) * target_angular_vel
        #left_vel  = target_linear_vel - (self.tire_tread / 2.0) * target_angular_vel
        
        # Convert to each circumferential velocity
        circumferential_right_vel = target_linear_vel + (self.tire_tread / 2.0) * target_angular_vel #[m/s]
        circumferential_left_vel  = target_linear_vel - (self.tire_tread / 2.0) * target_angular_vel #[m/s]

        # Convert to each rotational velocity
        right_vel = circumferential_right_vel / self.tire_circumference #[turn/s]
        left_vel  = circumferential_left_vel / self.tire_circumference  #[turn/s]

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