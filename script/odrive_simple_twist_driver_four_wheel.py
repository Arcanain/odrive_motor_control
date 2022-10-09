#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import tf.transformations
import tf_conversions
import tf2_ros

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

        # setup parameter
        self.tire_tread         = 0.32                                         #[m] distance between wheel centres
        self.target_linear_vel  = 0.0                                          #[m/s]
        self.target_angular_vel = 0.0                                          #[rad/s]
        self.tire_diameter      = 0.165                                        #[m]
        self.encoder_cpr        = 90.0                                         #[count]
        self.tire_circumference = math.pi * self.tire_diameter                 #[m]

        # subscriber cmd_vel
        rospy.Subscriber("cmd_vel", Twist, self.callback_vel)

    def find_odrive(self):
        while True:
            print("Connect to Odrive...")
            self.odrv0 = odrive.find_any(serial_number="208434855748")
            self.odrv1 = odrive.find_any(serial_number="207B346D5748")

            if self.odrv0 is not None and self.odrv1 is not None:
                print("Connect to Odrive Success!!!")
                break
            else:
                print("Disconnect to Odrive...")
    
    def odrive_setup(self):
        # odrv0
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.input_vel = 0
        self.odrv0.axis1.controller.input_vel = 0

        # odrv1
        self.odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv1.axis0.controller.input_vel = 0
        self.odrv1.axis1.controller.input_vel = 0

    def odrive_control(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # current time
            current_time = rospy.Time.now()
        
            # calculate relative velocity
            right_vel, left_vel = self.calc_relative_vel(self.target_linear_vel, self.target_angular_vel)
            
            try:
                # Set velocity
                self.odrv0.axis0.controller.input_vel = right_vel
                self.odrv0.axis1.controller.input_vel = -left_vel
                self.odrv1.axis0.controller.input_vel = right_vel
                self.odrv1.axis1.controller.input_vel = -left_vel

                # Time sleep
                rate.sleep()
            except AttributeError as error:
                # Output expected AttributeErrors.
                rospy.signal_shutdown(error)
            except KeyboardInterrupt:
                self.odrv0.axis0.controller.input_vel = 0
                self.odrv0.axis1.controller.input_vel = 0
                self.odrv1.axis0.controller.input_vel = 0
                self.odrv1.axis1.controller.input_vel = 0

                rospy.signal_shutdown("KeyboardInterrupt")
                
    def calc_relative_vel(self, target_linear_vel, target_angular_vel):
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