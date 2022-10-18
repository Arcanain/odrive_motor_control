#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
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
        self.tire_tread         = 0.32                                         #[m] distance between wheel centres
        self.target_linear_vel  = 0.0                                          #[m/s]
        self.target_angular_vel = 0.0                                          #[rad/s]
        self.tire_diameter      = 0.165                                        #[m]
        self.right_wheel_radius = self.tire_diameter                           #[m]
        self.left_wheel_radius  = self.tire_diameter                           #[m]
        self.encoder_cpr        = 90.0                                         #[count]
        self.tire_circumference = math.pi * self.tire_diameter                 #[m]
        self.m_t_to_value       = 1.0 / (self.tire_circumference)              #[turns/s]
        self.m_s_to_value       = self.encoder_cpr / (self.tire_circumference) #[count/s]
        self.vel_l = 0.0     #[count/s]
        self.vel_r = 0.0     #[count/s]
        self.new_pos_l = 0.0 #[count]
        self.new_pos_r = 0.0 #[count]
        self.old_pos_l = 0.0 #[count]
        self.old_pos_r = 0.0 #[count]

        # store current location to be updated. 
        self.x = 0.0     #[m]
        self.y = 0.0     #[m]
        self.theta = 0.0 #[rad]

        # subscriber cmd_vel
        rospy.Subscriber("cmd_vel", Twist, self.callback_vel)
        rospy.Subscriber("imu/data", Imu, self.imu_callback)
        self.imu_msg = Imu()

        # publish odom
        self.odom_publisher = rospy.Publisher("odom", Odometry, tcp_nodelay=True, queue_size=10)

        # publish odom_path
        self.odom_path_publisher = rospy.Publisher("odom_path", Path, tcp_nodelay=True, queue_size=10)
        self.poses_list = []

        # setup message
        # https://qiita.com/honeytrap15/items/550c757f2964b575883c
        #self.odom_frame = rospy.get_param('~odom_frame', "odom")
        #self.base_frame = rospy.get_param('~base_frame', "base_link")

        self.odom_frame = "map"
        self.base_frame = "base_link"

        self.odom_msg = Odometry()
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
        self.map_broadcaster  = tf.TransformBroadcaster()
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.map_to_odom_msg = TransformStamped()
        self.map_to_odom_msg.header.frame_id = "map"
        self.map_to_odom_msg.child_frame_id  = "odom"
        self.map_to_odom_msg.transform.translation.x = 0.0
        self.map_to_odom_msg.transform.translation.y = 0.0
        self.map_to_odom_msg.transform.translation.z = 0.0
        self.map_to_odom_msg.transform.rotation.x = 0.0
        self.map_to_odom_msg.transform.rotation.y = 0.0
        self.map_to_odom_msg.transform.rotation.w = 0.0
        self.map_to_odom_msg.transform.rotation.z = 1.0

        self.odom_to_baselink_msg = TransformStamped()
        self.odom_to_baselink_msg.header.frame_id = "map"
        self.odom_to_baselink_msg.child_frame_id  = "base_link"
        self.odom_to_baselink_msg.transform.translation.x = 0.0
        self.odom_to_baselink_msg.transform.translation.y = 0.0
        self.odom_to_baselink_msg.transform.translation.z = 0.0
        self.odom_to_baselink_msg.transform.rotation.x = 0.0
        self.odom_to_baselink_msg.transform.rotation.y = 0.0
        self.odom_to_baselink_msg.transform.rotation.w = 0.0
        self.odom_to_baselink_msg.transform.rotation.z = 1.0

        # timer
        self.pre_current_time = rospy.Time.now()

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
    
    def calcodom(self, current_time):
        dt = (current_time - self.pre_current_time).to_sec()
        self.pre_current_time = current_time
        #print(dt)

        # https://docs.odriverobotics.com/v/latest/commands.html
        ######################
        # Calcurate Position #
        ######################
        # convert [turn] into [count]
        self.new_pos_r = self.encoder_cpr * self.odrv0.axis0.encoder.pos_estimate #[count]
        self.new_pos_l = self.encoder_cpr * self.odrv0.axis1.encoder.pos_estimate #[count]

        delta_pos_r = self.new_pos_r - self.old_pos_r #[count]
        delta_pos_l = self.new_pos_l - self.old_pos_l #[count]
        
        self.old_pos_r = self.new_pos_r #[count]
        self.old_pos_l = self.new_pos_l #[count]
        
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
        #th = (delta_pos_r_m - delta_pos_l_m) / self.tire_tread # works for small angles

        e = tf.transformations.euler_from_quaternion((self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w))
        th = self.imu_msg.angular_velocity.z
        dth = (th * dt) % (2*math.pi)
        xd = math.cos(dth)*d
        yd = -math.sin(dth)*d

        # Pose: updated from previous pose + position delta
        """
        self.x += math.cos(self.theta)*xd - math.sin(self.theta)*yd
        self.y += math.sin(self.theta)*xd + math.cos(self.theta)*yd
        self.theta = (self.theta + th) % (2*math.pi)
        """
        self.theta = e[2] % (2*math.pi)
        self.x += math.cos(self.theta)*xd - math.sin(self.theta)*yd
        self.y += math.sin(self.theta)*xd + math.cos(self.theta)*yd

        # Twist/velocity: calculated from motor values only
        self.vel_r = self.encoder_cpr * self.odrv0.axis0.encoder.vel_estimate
        self.vel_l = self.encoder_cpr * self.odrv0.axis1.encoder.vel_estimate * (-1)
        v = self.tire_circumference * (self.vel_r + self.vel_l) / (2.0*self.encoder_cpr)
        w = self.tire_circumference * (self.vel_r - self.vel_l) / (self.tire_tread * self.encoder_cpr) # angle: vel_r*tyre_radius - vel_l*tyre_radius
        #print(v)
        #sprint(w)
        
        ####################
        # Publish Odometry #
        ####################
        self.odom_msg.header.stamp = current_time
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        self.odom_msg.pose.pose.orientation.z = q[2] # math.sin(self.theta)/2
        self.odom_msg.pose.pose.orientation.w = q[3] # math.cos(self.theta)/2
        self.odom_msg.twist.twist.linear.x  = v
        self.odom_msg.twist.twist.angular.z = w
        self.odom_publisher.publish(self.odom_msg)

        ######################################
        # transform odom_frame to base_frame #
        ######################################
        #self.map_broadcaster.sendTransform(self.map_to_odom_msg)
        
        odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        self.map_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            odom_quat,
            rospy.Time.now(),
            "odom",
            "map"
        )

        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            q,
            rospy.Time.now(),
            "base_link",
            "odom"
        )

        #########################
        # Publish Odometry Path #
        #########################
        temp_pose = PoseStamped()
        temp_pose.header.stamp = rospy.Time.now()
        temp_pose.header.frame_id = "map"
        temp_pose.pose.position.x = self.x
        temp_pose.pose.position.y = self.y
        temp_pose.pose.orientation.z = q[2]
        temp_pose.pose.orientation.w = q[3]

        self.poses_list.append(temp_pose)

        # creat path data
        self.path = Path()
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "map"
        self.path.poses = self.poses_list

        self.odom_path_publisher.publish(self.path)

    def odrive_control(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # current time
            current_time = rospy.Time.now()
        
            # calculate relative velocity
            right_vel, left_vel = self.calc_relative_vel(self.target_linear_vel, self.target_angular_vel)
            
            try:
                # Get current position
                self.calcodom(current_time)

                # Set velocity
                self.odrv0.axis0.controller.input_vel = right_vel
                self.odrv0.axis1.controller.input_vel = -left_vel
                
                # Time sleep
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
    
    def imu_callback(self, msg):
        self.imu_msg.orientation.x = msg.orientation.x
        self.imu_msg.orientation.y = msg.orientation.y
        self.imu_msg.orientation.z = msg.orientation.z
        self.imu_msg.orientation.w = msg.orientation.w	
        
        self.imu_msg.angular_velocity.x = msg.angular_velocity.x
        self.imu_msg.angular_velocity.y = msg.angular_velocity.y
        self.imu_msg.angular_velocity.z = msg.angular_velocity.z
        
        self.imu_msg.linear_acceleration.x = msg.linear_acceleration.x
        self.imu_msg.linear_acceleration.y = msg.linear_acceleration.y
        self.imu_msg.linear_acceleration.z = msg.linear_acceleration.z

if __name__ == "__main__":
    rospy.init_node("odrive_motor_control", disable_signals=True)
    Odrive_motor_control = OdriveMotorControl()
    Odrive_motor_control.odrive_setup()
    Odrive_motor_control.odrive_control()
    rospy.spin()