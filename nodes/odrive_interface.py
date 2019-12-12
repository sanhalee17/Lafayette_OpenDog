import serial
from serial.serialutil import SerialException

import sys
import time
import logging
import traceback
import rospy

import odrive
from odrive.enums import *
from odrive.utils import start_liveplotter, dump_errors

import fibre

default_logger = logging.getLogger(__name__)
default_logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

default_logger.addHandler(ch)

class ODriveFailure(Exception):
    pass

class ODriveInterfaceAPI(object):
    driver = None
    # encoder_cpr = 4096
    encoder_cpr = 8192   # Edit by GGC on June 14
    right_axis = None
    left_axis = None
    connected = False
    # Edit by GGC on June 13
    _preroll_started = False
    _preroll_completed = False
    # _preroll_started = True
    # _preroll_completed = True
    #engaged = False

    #Added Nov.13.2019 by SL:
    home_position = AXIS_STATE_ENCODER_INDEX_SEARCH
    initial_position = home_position #going fifteen revolutions back from the limit switch location 
    
    def __init__(self, logger=None):
        # Edit by GGC on June 14: 
        # Initializes log
        self.logger = logger if logger else default_logger
                
    def __del__(self):
        # Edit by GGC on June 14: 
        # Deletes by disconnecting
        self.disconnect()
                    
    def connect(self, port=None, right_axis=0, timeout=30,snum='56501121856822'): # port=/dev/ttyACM0

        # Edit by GGC on June 14: 
        # Checks if driver is connected. If it is, reports that it's reconnecting.
        # Tries to find an ODrive to connect to with find_any within timeout time (seconds???)
        # Assigns motors to axes array (I think...)
        # If fails, logs error and returns False
        # Sets encoder counts/revolution based on info from ODrive and encoder
        # Sets connected to True, and logs some info about the ODrive
        # Sets preroll info to False...

        if self.driver:
            self.logger.info("Already connected. Disconnecting and reconnecting.")
        try:
            self.driver = odrive.find_any(timeout=timeout, logger=self.logger, serial_number=snum)  #, port_number = port
            self.axes = (self.driver.axis0, self.driver.axis1)
        except:
            self.logger.error("No ODrive found. Is device powered?")
            return False
                        
        # save some parameters for easy access
        self.right_axis = self.driver.axis0 if right_axis == 0 else self.driver.axis1
        self.left_axis  = self.driver.axis1 if right_axis == 0 else self.driver.axis0
        
        # check for no errors
        for axis in [self.right_axis, self.left_axis]:
            if axis.error != 0:
                error_str = "Had error on startup, rebooting. Axis error 0x%x, motor error 0x%x, encoder error 0x%x. Rebooting." % (axis.error, axis.motor.error, axis.encoder.error)
                self.driver.reboot()
                return False
        
        self.encoder_cpr = self.driver.axis0.encoder.config.cpr
        
        self.connected = True
        self.logger.info("Connected to ODrive. Hardware v%d.%d-%d, firmware v%d.%d.%d%s" % (
                        self.driver.hw_version_major, self.driver.hw_version_minor, self.driver.hw_version_variant,
                        self.driver.fw_version_major, self.driver.fw_version_minor, self.driver.fw_version_revision,
                        "-dev" if self.driver.fw_version_unreleased else ""
                        ))
        
        # Edit by GGC on June 13               
        self._preroll_started = False
        self._preroll_completed = False
        # self._preroll_started = True
        # self._preroll_completed = True
        
        return True
        
    def disconnect(self):
        # Edit by GGC on June 14: 
        # Checks if driver is connected. If not, throws error.
        # Clears the values of connected, right_axis, and left_axis
        # Tries to release motors (see release function)
        # If it fails, logs error and returns false
        # Successful or not, will disconnect ODrive with "driver = None"  *******Check if this is true

        self.connected = False
        self.right_axis = None
        self.left_axis = None
        
        #self.engaged = False
        
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        
        try:
            self.release()
        except:
            self.logger.error("Error in timer: " + traceback.format_exc())
            return False
        finally:
            self.driver = None
        return True
        
    def reboot(self):
        # Edit by GGC on June 14: 
        # Checks if driver is connected. If not, throws error.
        # Tries to reboot using <odrv>.reboot()
        # If it fails, logs error and returns false
        # Successful or not, will disconnect ODrive with "driver = None"  *******Check if this is true

        if not self.driver:
            self.logger.error("Not connected.")
            return False
        
        try:
            self.driver.reboot()
        except:
            self.logger.error("Failed to reboot: " + traceback.format_exc())
            return False
        finally:
            self.driver = None
        return True
        
    def calibrate(self):
        # Edit by GGC on June 14: 
        # Checks if driver is connected. If not, throws error.
        # Calibrates each axis (motor) with AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        # Pauses to make sure sequence is done (waits until it is in IDLE)
        # If there is an error, it is reported and returns False (success status)

        if not self.driver:
            self.logger.error("Not connected.")
            return False
        
        self.logger.info("Vbus %.2fV" % self.driver.vbus_voltage)
        
        # Edit by GGC on June 28: Temporarily get rid of for loop since we are using 1 motor right now
        for i, axis in enumerate(self.axes):
            self.logger.info("Calibrating axis %d..." % i)
            axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            time.sleep(1)
            while axis.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            if axis.error != 0:
                self.logger.error("Failed calibration with axis error 0x%x, motor error 0x%x" % (axis.error, axis.motor.error))
                return False
        
        
        # self.logger.info("Calibrating axis %d..." % 0)
        # self.axes[0].requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        # time.sleep(1)
        
        # for i, axis in enumerate(self.axes):
        #     while axis.current_state != AXIS_STATE_IDLE:
        #         time.sleep(0.1)
        #     if axis.error != 0:
        #         self.logger.error("Failed calibration with axis error 0x%x, motor error 0x%x" % (axis.error, axis.motor.error))
        #         return False

        return True
        
    def preroll(self, wait=True):
        # Edit by GGC on June 14: 
        # Checks if driver is connected. If not, throws error.
        # Checks if preroll was started. If so, returns False.
        # For each axis (motor), calibrates encoder using AXIS_STATE_ENCODER_INDEX_SEARCH
        # Sets _preroll_started to True
        # Waits until both axes's encoders have finished index search by checking it is idling
        # If there is an error for one of the axes, logs it
        # Sets _preroll_completed to True
        # If wait = False, returns False

        if not self.driver:
            self.logger.error("Not connected.")
            return False
            
        if self._preroll_started: # must be prerolling or already prerolled
            self.logger.error("Already prerolling")  # Edit by GGC on June 14: print to find where it's failing
            return False
            
        #self.logger.info("Vbus %.2fV" % self.driver.vbus_voltage)

        # Edit by GGC on June 14: 
        # Only look at axis0 (right motor) since we are working with one motor
        for i, axis in enumerate(self.axes):
            self.logger.info("Index search preroll axis %d..." % i)
            axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        # self.logger.info("Index search preroll axis %d..." % 0)
        # self.axes[0].requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            
        self._preroll_started = True
        
        if wait:
            # Edit by GGC on July 1
            for i, axis in enumerate(self.axes):
                while axis.current_state != AXIS_STATE_IDLE:
                    time.sleep(0.1)
            for i, axis in enumerate(self.axes):
                if axis.error != 0:
                    self.logger.error("Failed preroll with axis error 0x%x, motor error 0x%x" % (axis.error, axis.motor.error))
                    return False
            self._preroll_completed = True

            
            # while self.axes[0].current_state != AXIS_STATE_IDLE:
            #     time.sleep(0.1)
            
            # if self.axes[0].error != 0:
            #     self.logger.error("Failed preroll with axis error 0x%x, motor error 0x%x" % (self.axes[0].error, self.axes[0].motor.error))
            #     return False
            # self._preroll_completed = True
        else:
            self.logger.error("Wait was false")  # Edit by GGC on June 14: print to find where it's failing
            return False

        return True  # Edit by GGC on June 14: this function was never told to be true if it was successful!
        
    # def prerolling(self):
    #     return self.axes[0].current_state == AXIS_STATE_ENCODER_INDEX_SEARCH or self.axes[1].current_state == AXIS_STATE_ENCODER_INDEX_SEARCH
    #
    # def prerolled(self): #
    #     return self._prerolled and not self.prerolling()
        
    def ensure_prerolled(self):
        # Edit by GGC on June 14: 
        # Checks if preroll was completed. If so, returns True
        # If it started but did not complete, checks if it is still index searching
            # If not, checks if there are errors for each axis (motor). If there are errors, logs it and attempts to reboot
            # If there are no errors, it was successful, so set _preroll_completed to True
        # If it is still index searching, the motor is still prerolling, so return False
        # If neither _preroll_started nor _preroll_completed are True, call preroll function and return False

        # preroll success
        if self._preroll_completed:
            return True
        # started, not completed
        elif self._preroll_started:
            #self.logger.info("Checking for preroll complete.")
            if self.axes[0].current_state != AXIS_STATE_ENCODER_INDEX_SEARCH and self.axes[1].current_state != AXIS_STATE_ENCODER_INDEX_SEARCH:
                # completed, check for errors before marking complete
                for i, axis in enumerate(self.axes):
                    if axis.error != 0:
                        error_str = "Failed preroll with axis error 0x%x, motor error 0x%x, encoder error 0x%x. Rebooting." % (axis.error, axis.motor.error, axis.encoder.error)
                        self.driver.reboot()
                        self.logger.error(error_str)
                        raise Exception(error_str)
                # no errors, success
                self._preroll_completed = True
                self.logger.info("Preroll complete.")
                return True
            else:
                # still prerolling
                return False
        else: # start preroll
            #self.logger.info("Preroll started.")
            self.preroll(wait=False)
            return False
    
    def engaged(self):
        # Edit by GGC on June 14: 
        # Checks if both axes (motors) are engaged with AXIS_STATE_CLOSED_LOOP_CONTROL
        return self.axes[0].current_state == AXIS_STATE_CLOSED_LOOP_CONTROL or self.axes[1].current_state == AXIS_STATE_CLOSED_LOOP_CONTROL
    
    def idle(self):
        # Edit by GGC on June 14: 
        # Checks if both axes (motors) are released/idled with AXIS_STATE_IDLE
        return self.axes[0].current_state == AXIS_STATE_IDLE and self.axes[1].current_state == AXIS_STATE_IDLE
    
    # Edit by GGC on June 28: make separate velocity and position engage functions    
    def engage_vel(self):
        # Edit by GGC on June 14: 
        # Checks if driver is connected. If not, throws error.
        # Engages motors with AXIS_STATE_CLOSED_LOOP_CONTROL.
        # Then enters velocity mode with CTRL_MODE_VELOCITY_CONTROL

        if not self.driver:
            self.logger.error("Not connected.")
            return False

        #self.logger.debug("Setting drive mode.")
        for axis in self.axes:
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
            axis.controller.config.vel_limit = 200000.0
            axis.controller.vel_setpoint = 0

        #self.engaged = True
        return True
    
    def engage_pos(self):
        # Checks if driver is connected. If not, throws error.
        # Engages motors with AXIS_STATE_CLOSED_LOOP_CONTROL.
        # Then enters position mode with CTRL_MODE_POSITION_CONTROL

        if not self.driver:
            self.logger.error("Not connected.")
            return False
        # for axis in self.axes:
        #     axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #     axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        #     axis.controller.config.vel_limit = 200000.0
        #     axis.controller.pos_setpoint = 0

        self.axes[0].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axes[0].controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.axes[0].controller.config.vel_limit = 200000.0
        # self.axes[0].controller.pos_setpoint = 0
        # time.sleep(2)
        # self.axes[0].controller.pos_setpoint = 10000
        # time.sleep(1)
        self.axes[0].controller.pos_setpoint = 0
        self.current_state_0 = self.axes[0].current_state

        
        #added Nov.13.2019 by SL:
        # self.axes[0].encoder.config.offset = -49152

        self.axes[1].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axes[1].controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.axes[1].controller.config.vel_limit = 200000.0
        self.axes[1].controller.pos_setpoint = 0
        # time.sleep(2)
        # self.axes[1].controller.pos_setpoint = 10000
        # time.sleep(1)
        # self.axes[1].controller.pos_setpoint = 0
        self.current_state_1 = self.axes[1].current_state

        # time.sleep(10)

        #added Nov.13.2019 by SL:
        # self.axes[0].encoder.config.offset = -49152

        return (True, self.current_state_0, self.current_state_1)


    def release(self):
        # Edit by GGC on June 14: 
        # Checks if driver is connected. If not, throws error.
        # Executes AXIS_STATE_IDLE on each available axis (motor) to disengage them

        if not self.driver:
            self.logger.error("Not connected.")
            return False
        #self.logger.debug("Releasing.")
        for axis in self.axes: 
            axis.requested_state = AXIS_STATE_IDLE

        #self.engaged = False
        return True
    
    def clearE(self):
        # Edit by GGC on June 28
        # An attempt to clear odrive errors
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        else:
            print(dump_errors(self.driver))
            dump_errors(self.driver, True)
            return True


    #added Nov.13.2019 by SL:
    #this definition is adding an offset to encoders
    def home(self):
        if not self.driver:
            self.logger.errr("Not connected.")
            return False
        else:
            print(reset_encoders(self.driver))
            home_encoders(self.driver, True)
            home_position = self.AXIS_STATE_ENCODER_INDEX_SEARCH

            pos_setpoint = home_position - initial_position







    def drive_vel(self, left_motor_val, right_motor_val):
        # Edit by GGC on June 14: 
        # Checks if driver is connected. If not, throws error.
        # Executes axis.controller.vel_setpoint for each axis (motor) to make them move!
        # For wheeled bot, one motor must spin the opposite direction as the other to go straight

        if not self.driver:
            self.logger.error("Not connected.")
            return
        #try:
        self.left_axis.controller.vel_setpoint = left_motor_val
        #self.right_axis.controller.vel_setpoint = -right_motor_val
        self.right_axis.controller.vel_setpoint = right_motor_val
        #except (fibre.protocol.ChannelBrokenException, AttributeError) as e:
        #    raise ODriveFailure(str(e))
    
    def drive_pos(self, left_motor_val, right_motor_val):
        # Edit by GGC on June 28: 
        # Checks if driver is connected. If not, throws error.
        # Executes axis.controller.pos_setpoint for each axis (motor) to make them move!
        # For wheeled bot, one motor must spin the opposite direction as the other to go straight

        if not self.driver:
            self.logger.error("Not connected.")
            return
        
        self.left_axis.controller.pos_setpoint = left_motor_val
        self.right_axis.controller.pos_setpoint = right_motor_val
        

    def get_errors(self, clear=True):
        # Edit by GGC on June 14: 
        # Checks if driver is connected. If not, returns "none".
        # Checks if there are errors.  
        # If clear is True, sets all errors to 0. If axis_error is true, returns "error"

        # TODO: add error parsing, see: https://github.com/madcowswe/ODrive/blob/master/tools/odrive/utils.py#L34
        if not self.driver:
            return None
            
        axis_error = self.axes[0].error or self.axes[1].error
        
        if clear:
            for axis in self.axes:
                axis.error = 0
                axis.motor.error = 0
                axis.encoder.error = 0
                #axis.controller.error = 0
        
        if axis_error:
            return "error"
        
        
        