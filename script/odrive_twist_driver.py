#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

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
        self.tire_tread         = 0.32                      #[m] distance between wheel centres
        self.target_linear_vel  = 0.0                       #[m/s]
        self.target_angular_vel = 0.0                       #[rad/s]
        self.tire_diameter      = 0.165                     #[m]
        self.right_wheel_radius = self.tire_diameter        #[m]
        self.left_wheel_radius  = self.tire_diameter        #[m]
        self.encoder_cpr        = 90.0                      #[count]
        self.tire_circumference = math.pi * self.tire_diameter #[m]
        self.m_t_to_value       = 1.0 / (self.tire_circumference) #[turns/s]
        self.m_s_to_value       = self.encoder_cpr / (self.tire_circumference) #[count/s]
        self.vel_l = 0.0
        self.vel_r = 0.0
        self.new_pos_l = 0.0
        self.new_pos_r = 0.0
        self.old_pos_l = 0.0
        self.old_pos_r = 0.0
        # store current location to be updated. 
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # https://qiita.com/honeytrap15/items/550c757f2964b575883c
        self.odom_frame         = rospy.get_param('~odom_frame', "odom")
        self.base_frame         = rospy.get_param('~base_frame', "base_link")

        # <axis>.encoder.pos_estimate [turns]
        # https://docs.odriverobotics.com/v/latest/commands.html
        #self.right_pos          = self.odrv0.axis0.encoder.pos_estimate/900.0*m.pi*self.right_wheel_radius
        #self.left_pos           = -self.odrv0.axis1.encoder.pos_estimate/900.0*m.pi*self.left_wheel_radius
        self.right_pos          = self.odrv0.axis0.encoder.pos_estimate/self.encoder_cpr*math.pi*self.right_wheel_radius
        self.left_pos           = -self.odrv0.axis1.encoder.pos_estimate/self.encoder_cpr*math.pi*self.left_wheel_radius
        self.last_right_pos     = self.right_pos
        self.last_left_pos      = self.left_pos

        # subscriber cmd_vel
        rospy.Subscriber("cmd_vel", Twist, self.callback_vel)
        
        # publisher odom
        self.odom_pub = rospy.Publisher("odom_buf", Vector3, queue_size=1)
        self.odom = Vector3()
        self.odom.x = 0.0
        self.odom.y = 0.0
        self.odom.z = 0.0

        # publish odom
        self.odom_publisher = rospy.Publisher("odom", Odometry, tcp_nodelay=True, queue_size=2)
        # setup message
        self.odom_msg = Odometry()
        #print(dir(self.odom_msg))
        self.odom_msg.header.frame_id = self.odom_frame
        self.odom_msg.child_frame_id  = self.base_frame
        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0
        self.odom_msg.pose.pose.position.z = 0.0    # always on the ground, we hope
        self.odom_msg.pose.pose.orientation.x = 0.0 # always vertical
        self.odom_msg.pose.pose.orientation.y = 0.0 # always vertical
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0
        self.odom_msg.twist.twist.linear.x = 0.0
        self.odom_msg.twist.twist.linear.y = 0.0  # no sideways
        self.odom_msg.twist.twist.linear.z = 0.0  # or upwards... only forward
        self.odom_msg.twist.twist.angular.x = 0.0 # or roll
        self.odom_msg.twist.twist.angular.y = 0.0 # or pitch... only yaw
        self.odom_msg.twist.twist.angular.z = 0.0

        # setup transform
        self.tf_publisher = tf2_ros.TransformBroadcaster()
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = self.odom_frame
        self.tf_msg.child_frame_id  = self.base_frame
        self.tf_msg.transform.translation.x = 0.0
        self.tf_msg.transform.translation.y = 0.0
        self.tf_msg.transform.translation.z = 0.0
        self.tf_msg.transform.rotation.x = 0.0
        self.tf_msg.transform.rotation.y = 0.0
        self.tf_msg.transform.rotation.w = 0.0
        self.tf_msg.transform.rotation.z = 1.0

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
    
    """
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
    """

    def calcodom(self, current_time):
        # Calcurate Position
        #self.new_pos_r = self.odrv0.axis0.encoder.pos_est_counts
        #self.new_pos_l = self.odrv0.axis1.encoder.pos_est_counts
        #self.new_pos_r = self.odrv0.axis0.encoder.pos_estimate
        #self.new_pos_l = self.odrv0.axis1.encoder.pos_estimate
        self.new_pos_r = self.encoder_cpr * self.odrv0.axis0.encoder.pos_estimate
        self.new_pos_l = self.encoder_cpr * self.odrv0.axis1.encoder.pos_estimate

        delta_pos_r = self.new_pos_r - self.old_pos_r
        delta_pos_l = self.new_pos_l - self.old_pos_l
        
        self.old_pos_r = self.new_pos_r
        self.old_pos_l = self.new_pos_l
        
        # Check for overflow. Assume we can't move more than half a circumference in a single timestep. 
        half_cpr = self.encoder_cpr / 2.0
        if delta_pos_r >  half_cpr: 
            delta_pos_r = delta_pos_r - self.encoder_cpr
        elif delta_pos_r < -half_cpr: 
            delta_pos_r = delta_pos_r + self.encoder_cpr
        if delta_pos_l >  half_cpr: 
            delta_pos_l = delta_pos_l - self.encoder_cpr
        elif delta_pos_l < -half_cpr: 
            delta_pos_l = delta_pos_l + self.encoder_cpr
        
        # convert [turns] into [m]
        delta_pos_r_m = delta_pos_r / self.m_s_to_value
        delta_pos_l_m = delta_pos_l / self.m_s_to_value * (-1)
        
        # Distance travelled
        d = (delta_pos_r_m + delta_pos_l_m) / 2.0  # delta_ps
        th = (delta_pos_r_m - delta_pos_l_m) / self.tire_tread # works for small angles
    
        xd = math.cos(th)*d
        yd = -math.sin(th)*d

        # Pose: updated from previous pose + position delta
        #self.x = self.x + math.cos(self.theta)*xd - math.sin(self.theta)*yd
        #self.y = self.y + math.sin(self.theta)*xd + math.cos(self.theta)*yd
        self.x += math.cos(self.theta)*xd - math.sin(self.theta)*yd
        self.y += math.sin(self.theta)*xd + math.cos(self.theta)*yd
        self.theta = (self.theta + th) % (2*math.pi)

        # Twist/velocity: calculated from motor values only
        self.vel_r = self.encoder_cpr * self.odrv0.axis0.encoder.vel_estimate
        self.vel_l = self.encoder_cpr * self.odrv0.axis1.encoder.vel_estimate * (-1)
        v = self.tire_circumference * (self.vel_r + self.vel_l) / (2.0*self.encoder_cpr)
        w = self.tire_circumference * (self.vel_r - self.vel_l) / (self.tire_tread * self.encoder_cpr) # angle: vel_r*tyre_radius - vel_l*tyre_radius

        #print(delta_pos_r)
        #print(delta_pos_l)
        #print(delta_pos_r_m)
        #print(delta_pos_l_m)
        #print(d)
        #print(th)
        #print(xd)
        #print(yd)
        #print(self.x)
        #print(self.y)
        #print(self.theta)
        #print(self.odrv0.axis0.encoder.pos_estimate * 90.0)
        #print(self.vel_r)
        #print(self.vel_l)
        #print(self.vel_l)
        #print(v)
        #print(w)
        #print(math.cos(self.theta)*xd - math.sin(self.theta)*yd)
        #print(math.sin(self.theta)*xd + math.cos(self.theta)*yd)

    def odrive_control(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # current time
            current_time = rospy.Time.now()
        
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
                #self.right_pos = self.odrv0.axis0.encoder.pos_estimate/self.encoder_cpr*math.pi*self.right_wheel_radius
                #self.left_pos  = -self.odrv0.axis1.encoder.pos_estimate/self.encoder_cpr*math.pi*self.left_wheel_radius
                #right_pos_diff = self.right_pos - self.last_right_pos
                #left_pos_diff  = self.left_pos - self.last_left_pos
                #self.calcodom(right_pos_diff, left_pos_diff)
                self.calcodom(current_time)
                
                #print(self.odrv0.axis0.encoder.pos_estimate)

                # Set velocity
                self.odrv0.axis0.controller.input_vel = right_vel
                self.odrv0.axis1.controller.input_vel = -left_vel
                
                # Update last pos
                #self.last_right_pos = self.right_pos
                #self.last_left_pos = self.left_pos
                
                rate.sleep()
            except AttributeError as error:
                # Output expected AttributeErrors.
                rospy.signal_shutdown(error)
            except KeyboardInterrupt:
                self.odrv0.axis0.controller.input_vel = 0
                self.odrv0.axis1.controller.input_vel = 0
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