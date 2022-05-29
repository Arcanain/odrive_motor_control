#! /usr/bin/env python3
import sys
import time
import odrive
from odrive.enums import *
import fibre.libfibre

class OdriveConfiguration:
    def __init__(self):
        # Connect to Odrive
        self.find_odrive()
    
    def find_odrive(self):
        while True:
            print("Connect to Odrive...")
            self.odrv0 = odrive.find_any()
            if self.odrv0 is not None:
                print("Connect to Odrive Success!!!")
                break
            else:
                print("Disconnect to Odrive...")
                
    def set_odrive_parameters(self):
        # Connect to Odrive
        self.find_odrive()
        
        self.odrv0.axis0.motor.config.pole_pairs = 15
        self.odrv0.axis0.motor.config.resistance_calib_max_voltage = 4
        self.odrv0.axis0.motor.config.requested_current_range = 25 
        self.odrv0.axis0.motor.config.current_control_bandwidth = 100

        self.odrv0.axis1.motor.config.pole_pairs = 15
        self.odrv0.axis1.motor.config.resistance_calib_max_voltage = 4
        self.odrv0.axis1.motor.config.requested_current_range = 25 
        self.odrv0.axis1.motor.config.current_control_bandwidth = 100

        self.odrv0.axis0.motor.config.torque_constant = 8.27 / 16
        self.odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
        self.odrv0.axis0.encoder.config.cpr = 90
        self.odrv0.axis0.encoder.config.calib_scan_distance = 150

        self.odrv0.axis1.motor.config.torque_constant = 8.27 / 16
        self.odrv0.axis1.encoder.config.mode = ENCODER_MODE_HALL
        self.odrv0.axis1.encoder.config.cpr = 90
        self.odrv0.axis1.encoder.config.calib_scan_distance = 150

        self.odrv0.axis0.encoder.config.bandwidth = 100
        self.odrv0.axis0.controller.config.pos_gain = 1
        self.odrv0.axis0.controller.config.vel_gain = 0.02 * self.odrv0.axis0.motor.config.torque_constant * self.odrv0.axis0.encoder.config.cpr
        self.odrv0.axis0.controller.config.vel_integrator_gain = 0.1 * self.odrv0.axis0.motor.config.torque_constant * self.odrv0.axis0.encoder.config.cpr
        self.odrv0.axis0.controller.config.vel_limit = 10

        self.odrv0.axis1.encoder.config.bandwidth = 100
        self.odrv0.axis1.controller.config.pos_gain = 1
        self.odrv0.axis1.controller.config.vel_gain = 0.02 * self.odrv0.axis0.motor.config.torque_constant * self.odrv0.axis1.encoder.config.cpr
        self.odrv0.axis1.controller.config.vel_integrator_gain = 0.1 * self.odrv0.axis0.motor.config.torque_constant * self.odrv0.axis1.encoder.config.cpr
        self.odrv0.axis1.controller.config.vel_limit = 10

        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        self.odrv0.config.enable_brake_resistor = True
        
        self.odrv0.axis0.motor.config.current_lim = 5
        self.odrv0.axis0.motor.config.calibration_current = 5
        self.odrv0.axis0.encoder.config.hall_polarity_calibrated = True 

        self.odrv0.axis1.motor.config.current_lim = 5
        self.odrv0.axis1.motor.config.calibration_current = 5
        self.odrv0.axis1.encoder.config.hall_polarity_calibrated = True 
        
        # https://discourse.odriverobotics.com/t/problem-with-save-configuration-using-python/7581
        print("Saving manual configuration and rebooting...")
        try:
            print("Save Configuration")
            self.odrv0.save_configuration()
        except fibre.libfibre.ObjectLostError:
            pass # Saving configuration makes the device reboot
        
        try:
            print("Odrive reboot")
            self.odrv0.reboot()
        except:
            pass
        
    def motor_calibration(self):
        # Connect to Odrive
        self.find_odrive()
    
        self.odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        # Wait for calibration to take place
        time.sleep(10)
        self.odrv0.axis0.motor.config.pre_calibrated = True

        self.odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        # Wait for calibration to take place
        time.sleep(10)
        self.odrv0.axis1.motor.config.pre_calibrated = True
    
    def encoder_calibration(self):
        self.odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        # Wait for calibration to take place
        time.sleep(30)
        self.odrv0.axis0.encoder.config.pre_calibrated = True
        
        self.odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        # Wait for calibration to take place
        time.sleep(30)
        self.odrv0.axis1.encoder.config.pre_calibrated = True
        
        # https://discourse.odriverobotics.com/t/problem-with-save-configuration-using-python/7581
        print("Saving manual configuration and rebooting...")
        try:
            print("Save Configuration")
            self.odrv0.save_configuration()
        except fibre.libfibre.ObjectLostError:
            pass # Saving configuration makes the device reboot
        
        try:
            print("Odrive reboot")
            self.odrv0.reboot()
        except:
            pass
    
    def config(self):
        print("Start set_odrive_parameters")
        self.set_odrive_parameters()
        print("Finish set_odrive_parameters")
        
        print("Start motor_calibration")
        self.motor_calibration()
        print("Finish motor_calibration")
        
        print("Start encoder_calibration")
        self.encoder_calibration()
        print("Finish encoder_calibration")

    def operation_check(self):
        # Connect to Odrive
        self.find_odrive()
        
        print("Motor operation check...")
        
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.input_vel = 2
        self.odrv0.axis1.controller.input_vel = -2
        # Wait for calibration to take place
        time.sleep(10)
        self.odrv0.axis0.controller.input_vel = 0
        self.odrv0.axis1.controller.input_vel = 0
        
        print("Motor operation check Success!!!")
        
if __name__ == "__main__":
    Odrive_motor_config = OdriveConfiguration()
    Odrive_motor_config.config()
    Odrive_motor_config.operation_check()