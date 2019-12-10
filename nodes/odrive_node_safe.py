#!/usr/bin/env python
from __future__ import print_function

#import roslib; roslib.load_manifest('BINCADDY')
import rospy
import tf.transformations
import tf_conversions
import tf2_ros
import odrive
# Edit by GGC on June 14: 
# Imports message types and services from several libraries
from std_msgs.msg import Float64, Int32,Bool
from geometry_msgs.msg import Twist, TransformStamped, Pose
from nav_msgs.msg import Odometry
import std_srvs.srv

import time
import math
import traceback
import Queue

# Edit by GGC on June 14: 
# From odrive_interface.py, imports 2 classes
# Note: ODriveFailure only contains "pass"
from odrive_interface import ODriveInterfaceAPI, ODriveFailure

class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg)  #  print(msg) #
    def info(self, msg):     rospy.loginfo(msg)   #  print(msg) #
    def warn(self, msg):     rospy.logwarn(msg)   #  print(msg) #
    def error(self, msg):    rospy.logerr(msg)    #  print(msg) #
    def critical(self, msg): rospy.logfatal(msg)  #  print(msg) #

    # use_index = False (bool)
    # offset_float = 0.590887010098 (float)
    # calib_range = 0.019999999553 (float)
    # mode = 0 (int)
    # offset = 1809 (int)
    # cpr = 4096 (int)
    # idx_search_speed = 10.0 (float)
    # pre_calibrated = False (bool)

#m_s_to_rpm = 60.0/tyre_circumference
#m_s_to_erpm = 10 * m_s_to_rpm 

# 4096 counts / rev, so 4096 == 1 rev/s


# 1 m/s = 3.6 km/hr

class ODriveNode(object):
    last_speed = 0.0
    driver = None
    prerolling = False   # Edit by GGC on June 14: Not used anywhere

    # Robot wheel_track params for velocity -> motor speed conversion
    wheel_track = None
    tyre_circumference = None
    encoder_counts_per_rev = None
    m_s_to_value = 1.0
    axis_for_right = 0
    #encoder_cpr = 4096   # Edit by GGC on June 21: assigned in fast_timer() as value from odrive_interface?
    encoder_cpr = 8192

    # Startup parameters
    connect_on_startup = False
    calibrate_on_startup = False
    engage_on_startup = False

    #limit switch global variables
    # lim1low = False
    # lim1high = False
    # lim2low = False
    # lim2high = False


    def __init__(self):
        self.axis_for_right = float(rospy.get_param('~axis_for_right', 0)) # if right calibrates first, this should be 0, else 1
        self.wheel_track = float(rospy.get_param('~wheel_track', 0.285)) # m, distance between wheel centres
        self.tyre_circumference = float(rospy.get_param('~tyre_circumference', 0.341)) # used to translate velocity commands in m/s into motor rpm

        self.connect_on_startup   = rospy.get_param('~connect_on_startup', False)  # Edit by GGC on June 14: Does not automatically connect
        #self.calibrate_on_startup = rospy.get_param('~calibrate_on_startup', False)
        #self.engage_on_startup    = rospy.get_param('~engage_on_startup', False)

        self.has_preroll     = rospy.get_param('~use_preroll', False)  # GGC on July 11: PREROLL IS NOT WORKING
        # Specifically, when axis1 is put in Encoder Index Search, it throws the error: ERROR_INVALID_STATE

        self.publish_current = rospy.get_param('~publish_current', True)
        self.publish_raw_odom =rospy.get_param('~publish_raw_odom', True)

        self.publish_odom    = rospy.get_param('~publish_odom', True)
        self.publish_tf      = rospy.get_param('~publish_odom_tf', False)
        self.odom_topic      = rospy.get_param('~odom_topic', "odom")
        self.odom_frame      = rospy.get_param('~odom_frame', "odom")
        self.base_frame      = rospy.get_param('~base_frame', "base_link")
        self.odom_calc_hz    = rospy.get_param('~odom_calc_hz', 100)  # Edit by GGC on June 20
        self.pos_cmd_topic_name = rospy.get_param('~pos_cmd_topic',"/cmd_pos") 

        self.mode            = rospy.get_param('~control_mode', "position")
        self.lim1low_topic   = rospy.get_param('~lim1low_topic', "odrive1_low_tib")
        self.lim1high_topic   = rospy.get_param('~lim1high_topic', "odrive1_high_tib")
        self.lim2low_topic   = rospy.get_param('~lim2low_topic', "odrive1_low_fem")
        self.lim2high_topic   = rospy.get_param('~lim2high_topic', "odrive1_high_fem")
        self.serial_number   = rospy.get_param('~odrive_serial', "3365314F3536")
        # self.port_nunber = rospy.get_param('~odrive_port', "/dev/ttyACM0")

        print(self.mode)

        # rospy.on_shutdown(self.terminate)

        rospy.Service('connect_driver',    std_srvs.srv.Trigger, self.connect_driver)
        rospy.Service('disconnect_driver', std_srvs.srv.Trigger, self.disconnect_driver)
        rospy.Service('stop_motor', std_srvs.srv.Trigger, self.stop_motor)
        rospy.Service('home_encoder', std_srvs.srv.Trigger, self.home_encoder)

        rospy.Service('calibrate_motors',  std_srvs.srv.Trigger, self.calibrate_motor)
        rospy.Service('engage_motors',     std_srvs.srv.Trigger, self.engage_motor)
        rospy.Service('release_motors',    std_srvs.srv.Trigger, self.release_motor)

        rospy.Service('clear_errors',    std_srvs.srv.Trigger, self.clear_errors)

        self.command_queue = Queue.Queue(maxsize=5)

        # Edit by GGC on June 28: Determine subscribed topic based on control mode
        # Edit by GGC on July 4: Changing "is" to "==" allows the if-else block to work properly
        if self.mode == "position":
            self.pos_subscribe = rospy.Subscriber(self.pos_cmd_topic_name, Pose, self.cmd_pos_callback, queue_size=2)
            print("Subscribed to /cmd_pos")
        elif self.mode == "velocity":
            self.vel_subscribe = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=2)
            print("Subscribed to /cmd_vel")
        else:
            # Edit by GGC on July 4:
            # Debugging line to see if something went wrong with self.mode assignment...
            # ...or the if statement syntax
            print("Can't understand you, launch file")
        self.lim1low_sub = rospy.Subscriber(self.lim1low_topic ,Bool,self.lim1lowcallback)
        self.lim1high_sub = rospy.Subscriber(self.lim1high_topic ,Bool,self.lim1highcallback)
        self.lim2low_sub = rospy.Subscriber(self.lim2low_topic ,Bool,self.lim2lowcallback)
        self.lim2high_sub = rospy.Subscriber(self.lim2high_topic ,Bool,self.lim2highcallback)


        if self.publish_current:
            self.current_loop_count = 0
            self.left_current_accumulator  = 0.0
            self.right_current_accumulator = 0.0
            self.current_publisher_left  = rospy.Publisher('odrive/left_current', Float64, queue_size=2)
            self.current_publisher_right = rospy.Publisher('odrive/right_current', Float64, queue_size=2)
            rospy.loginfo("ODrive will publish motor currents.")

        self.last_cmd_vel_time = rospy.Time.now()

        if self.publish_raw_odom:
            self.raw_odom_publisher_encoder_left  = rospy.Publisher('odrive/raw_odom/encoder_left',   Int32, queue_size=2) if self.publish_raw_odom else None
            # Temporary Edit by GGC on June 25: commented this so I could test pos_control with rostopic pub
            # REMEMBER TO UNCOMMENT THIS WHEN WE USE THE MOTOR!
            self.raw_odom_publisher_encoder_right = rospy.Publisher('odrive/raw_odom/encoder_right',  Int32, queue_size=2) if self.publish_raw_odom else None
            self.raw_odom_publisher_vel_left      = rospy.Publisher('odrive/raw_odom/velocity_left',  Int32, queue_size=2) if self.publish_raw_odom else None
            self.raw_odom_publisher_vel_right     = rospy.Publisher('odrive/raw_odom/velocity_right', Int32, queue_size=2) if self.publish_raw_odom else None

        if self.publish_odom:
            rospy.Service('reset_odometry',    std_srvs.srv.Trigger, self.reset_odometry)
            self.old_pos_l = 0
            self.old_pos_r = 0

            self.odom_publisher  = rospy.Publisher(self.odom_topic, Odometry, tcp_nodelay=True, queue_size=2)
            # setup message
            self.odom_msg = Odometry()
            #print(dir(self.odom_msg))
            self.odom_msg.header.frame_id = self.odom_frame
            self.odom_msg.child_frame_id = self.base_frame
            self.odom_msg.pose.pose.position.x = 0.0
            self.odom_msg.pose.pose.position.y = 0.0
            self.odom_msg.pose.pose.position.z = 0.0    # always on the ground, we hope
            self.odom_msg.pose.pose.orientation.x = 0.0 # always vertical
            self.odom_msg.pose.pose.orientation.y = 0.0 # always vertical
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0  # Edit by GGC on June 14: What is w???
            self.odom_msg.twist.twist.linear.x = 0.0
            self.odom_msg.twist.twist.linear.y = 0.0  # no sideways
            self.odom_msg.twist.twist.linear.z = 0.0  # or upwards... only forward
            self.odom_msg.twist.twist.angular.x = 0.0 # or roll
            self.odom_msg.twist.twist.angular.y = 0.0 # or pitch... only yaw
            self.odom_msg.twist.twist.angular.z = 0.0

            # store current location to be updated. 
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0

            # setup transform
            self.tf_publisher = tf2_ros.TransformBroadcaster()
            self.tf_msg = TransformStamped()
            self.tf_msg.header.frame_id = self.odom_frame
            # self.tf_msg.child_frame_id  = self.base_frames
            self.tf_msg.transform.translation.x = 0.0
            self.tf_msg.transform.translation.y = 0.0
            self.tf_msg.transform.translation.z = 0.0
            self.tf_msg.transform.rotation.x = 0.0
            self.tf_msg.transform.rotation.y = 0.0
            self.tf_msg.transform.rotation.w = 0.0
            self.tf_msg.transform.rotation.z = 1.0

        #Added Nov.5.2019 by SL:
        #set up vars to hold limit switch states
        self.lim1low = False
        self.lim1high = False
        self.lim2low = False
        self.lim2high = False

        self.lim1low_old = False
        self.lim1high_old = False
        self.lim2low_old = False
        self.lim2high_old = False


        #modified Nov.21
        #SL
    def lim1lowcallback(self,data):

        self.lim1low = data.data
        if self.lim1low and not self.lim1low_old:
                self.left_current_accumulator = left_current_accumulator-194088
                rospy.logwarn(data)
        self.lim1low_old = self.lim1low

    def lim1highcallback(self,data):

        self.lim1high = data.data
        if self.lim1high and not self.lim1high_old:
                self.right_current_accumulator=0
                rospy.logwarn(self.right_current_accumulator)
        self.lim1high_old = self.lim1high

    def lim2lowcallback(self,data):

        self.lim2low = data.data
        if self.lim2low and not self.lim2low_old:
                self.right_current_accumulator=0
                rospy.logwarn(data)
        self.lim2low_old = self.lim2low

    def lim2highcallback(self,data):

        self.lim2high = data.data
        if self.lim2high and not self.lim2high_old:
                self.left_current_accumulator = left_current_accumulator-194088
                rospy.logwarn(data)
        self.lim2high_old = self.lim2high


    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        main_rate = rospy.Rate(1) # hz
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.odom_calc_hz)), self.fast_timer)

        self.fast_timer_comms_active = False

        while not rospy.is_shutdown():
            try:
                main_rate.sleep()
            except rospy.ROSInterruptException: # shutdown / stop ODrive??
                break;

            # fast timer running, so do nothing and wait for any errors
            if self.fast_timer_comms_active:
                continue

            # check for errors
            if self.driver:
                try:
                    # driver connected, but fast_comms not active -> must be an error?
                    if self.driver.get_errors(clear=True):
                        rospy.logerr("Had errors, disconnecting and retrying connection.")
                        self.driver.disconnect()
                        self.driver = None
                    else:
                        # must have called connect service from another node
                        self.fast_timer_comms_active = True
                except:
                    rospy.logerr("Errors accessing ODrive:" + traceback.format_exc())
                    self.driver = None

            if not self.driver:
                if not self.connect_on_startup:
                    #rospy.loginfo("ODrive node started, but not connected.")
                    continue

                if not self.connect_driver(None)[0]:
                    rospy.logerr("Failed to connect.") # TODO: can we check for timeout here?
                    continue

            else:
                pass # loop around and try again

    def fast_timer(self, timer_event):
        time_now = rospy.Time.now()
        # in case of failure, assume some values are zero
        self.vel_l = 0
        self.vel_r = 0
        self.new_pos_l = 0
        self.new_pos_r = 0
        self.current_l = 0
        self.current_r = 0

        # Handle reading from Odrive and sending odometry
        if self.fast_timer_comms_active:
            try:

                # read all required values from ODrive for odometry
                self.encoder_cpr = self.driver.encoder_cpr
                self.m_s_to_value = self.encoder_cpr/self.tyre_circumference # calculated

                self.vel_l = self.driver.left_axis.encoder.vel_estimate  # units: encoder counts/s
                self.vel_r = -self.driver.right_axis.encoder.vel_estimate # neg is forward for right
                self.new_pos_l = self.driver.left_axis.encoder.pos_cpr    # units: encoder counts
                self.new_pos_r = -self.driver.right_axis.encoder.pos_cpr  # sign!

                # for current
                self.current_l = self.driver.left_axis.motor.current_control.Ibus
                self.current_r = self.driver.right_axis.motor.current_control.Ibus

                #for encoders
                #Added Nov.13.2019 by SL:
                #self.requested_state = self.AXIS_STATE_ENCODER_INDEX_SEARCH


            except:
                rospy.logerr("Fast timer exception reading:" + traceback.format_exc())
                self.fast_timer_comms_active = False

        # odometry is published regardless of ODrive connection or failure (but assumed zero for those)
        # as required by SLAM
        if self.publish_odom:
            self.publish_odometry(time_now)
        if self.publish_current:
            self.pub_current()


        try:
            # check and stop motor if no vel command has been received in > 1s
            if self.fast_timer_comms_active:
                if (time_now - self.last_cmd_vel_time).to_sec() > 0.5 and self.last_speed > 0:
                    self.driver.drive_vel(0,0)  # *******CHECK THIS*******************************
                    self.last_speed = 0
                    self.last_cmd_vel_time = time_now
                # release motor after 10s stopped
                if (time_now - self.last_cmd_vel_time).to_sec() > 10.0 and self.driver.engaged():
                    self.driver.release() # and release            
        except:
            rospy.logerr("Fast timer exception on cmd_vel timeout:" + traceback.format_exc())
            self.fast_timer_comms_active = False

        # handle sending drive commands.
        # from here, any errors return to get out
        if self.fast_timer_comms_active and not self.command_queue.empty():
            # check to see if we're initialised and engaged motor
            # try:
            #     if not self.driver.ensure_prerolled():
            #         return
            # except:
            #     rospy.logerr("Fast timer exception on preroll." + traceback.format_exc())
            #     self.fast_timer_comms_active = False                
            try:
                motor_command = self.command_queue.get_nowait()
            except Queue.Empty:
                rospy.logerr("Queue was empty??" + traceback.format_exc())
                return

            if motor_command[0] == 'drive':
                try:
                    # Edit by GGC on June 28
                    # if not self.driver.engaged():
                    #     self.driver.engage()

                    left_linear_val, right_linear_val = motor_command[1]

                    # Edit by GGC on June 28:
                    # Check mode

                    # Edit by GGC on July 3: Switch if statement to check position first?
                    # For some reason, when it checks velocity first, it does not get into that if block
                    # if self.mode == "velocity":
                    #     if not self.driver.engaged():
                    #         self.driver.engage_vel()

                    #     self.driver.drive_vel(left_linear_val, right_linear_val)
                    #     self.last_speed = max(abs(left_linear_val), abs(right_linear_val))
                    #     self.last_cmd_vel_time = time_now
                    # else:
                    #     if not self.driver.engaged():
                    #         self.driver.engage_pos()

                    #     self.driver.drive_pos(left_linear_val, right_linear_val)
                    #     self.last_cmd_vel_time = time_now   # change to be last_cmd_pos_time????

                    # Edit by GGC on July 4: Changing "is" to "==" allows the if-else block to work properly
                    if self.mode == "position":
                        if not self.driver.engaged():
                            self.driver.engage_pos()
                        self.driver.drive_pos(left_linear_val, right_linear_val)
                        self.last_cmd_vel_time = time_now   # change to be last_cmd_pos_time????
                    else:
                        if not self.driver.engaged():
                            self.driver.engage_vel()

                        self.driver.drive_vel(left_linear_val, right_linear_val)
                        self.last_speed = max(abs(left_linear_val), abs(right_linear_val))
                        self.last_cmd_vel_time = time_now


                    print("attempted to write to Odrive")

                except:
                    rospy.logerr("Fast timer exception on drive cmd:" + traceback.format_exc())
                    self.fast_timer_comms_active = False
            elif motor_command[0] == 'release':
                pass
            # ?
            else:
                pass


    # def terminate(self):
    #     self.fast_timer.shutdown()
    #     if self.driver:
    #         self.driver.release()

    # ROS services
    def connect_driver(self, request):
        # Edit by GGC on June 14: 
        # Checks if driver is connected.  If it is, returns False and says so
        # Initializes driver log
        # Calls connect() function and passes it what right_axis is
        # If it fails to connect, sets driver to None again and returns False
        # Calculates conversion from m/s input to count value
        # Initializes positions of left and right axes (motors)
        # Starts Fast Timer Communications
        # If all goes well, returns True and reports that the ODrive connected

        if self.driver:
            return (False, "Already connected.")

        self.driver = ODriveInterfaceAPI(logger=ROSLogger())
        rospy.loginfo("Connecting to ODrive...")
        if not self.driver.connect(right_axis=self.axis_for_right,snum=self.serial_number):   #port=self.port_nunber
            self.driver = None
            #rospy.logerr("Failed to connect.")
            return (False, "Failed to connect.")

        #rospy.loginfo("ODrive connected.")

        # okay, connected, 
        self.m_s_to_value = self.driver.encoder_cpr/self.tyre_circumference

        if self.publish_odom:
            self.old_pos_l = self.driver.left_axis.encoder.pos_cpr
            self.old_pos_r = self.driver.right_axis.encoder.pos_cpr

        self.fast_timer_comms_active = True

        return (True, "ODrive connected successfully")

    def disconnect_driver(self, request):
        # Edit by GGC on June 14: 
        # Checks if driver is connected. If not, throws error.
        # Tries to call disconnect() function.  If it fails, returns False and error message
        # If that fails, throw error
        # Successful or not, sets driver to None
        # If it was successful, returns True

        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        try:
            if not self.driver.disconnect():
                return (False, "Failed disconnection, but try reconnecting.")
        except:
            rospy.logerr('Error while disconnecting: {}'.format(traceback.format_exc()))
        finally:
            self.driver = None
        return (True, "Disconnection success.")

    def calibrate_motor(self, request):
        # Edit by GGC on June 14: 
        # Checks if driver is connected. If not, throws error.
        # If using preroll (has_preroll = True), then call preroll() function
            # This will run AXIS_STATE_ENCODER_INDEX_SEARCH
            # If this fails, returns False and error message
        # Otherwise, calls calibrate() function to run AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            # If this fails, returns False and error message
        # If successful, returns True and success message

        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")

        if self.has_preroll:
            if not self.driver.preroll():
                return (False, "Failed preroll.")        
        else:
            if not self.driver.calibrate():
                return (False, "Failed calibration.")

        return (True, "Calibration success.")

    def home_encoder(self, request):
        if not self.home_encoder:
            rospy.logger("Not homed")
            return (False, "Not homed")

        if self.lim1high == True:
            self.right_current_accumulator=0

        if self.lim2low == True:
            self.left_current_accumulator =0

    def stop_motor(self, request):
        if lim1low_topic == True:
            self.left_current_accumulator = left_current_accumulator-194088
            #rospy.logwarn(self.left_current_accumulator)

        if lim2high_topic == True:
            self.right_current_accumulator = right_current_accumulator+194088
            #rospy.logwarn(self.right_current_accumulator)

    def engage_motor(self, request):
        # Edit by GGC on June 14: 
        # Checks if driver is connected. If not, throws error.
        # Calls engage() function to execute AXIS_STATE_CLOSED_LOOP_CONTROL and CTRL_MODE_VELOCITY_CONTROL
            # If this fails, returns False and error message
        # If successful, returns True and success message

        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")

        # Edit by GGC on June 28
        # Add position control engage

        # Edit by GGC on July 3: Switch if statement to check position first?
        # For some reason, when it checks velocity first, it does not get into that if block
        # if self.mode == "velocity":
        #     if not self.driver.engage_vel():
        #         return (False, "Failed to engage_vel motor.")
        #     return (True, "Engage_vel motor success.")
        # else:
        #     if not self.driver.engage_pos():
        #         return (False, "Failed to engage_pos motor.")
        #     return (True, "Engage_pos motor success.")

        # Edit by GGC on July 4: Changing "is" to "==" allows the if-else block to work properly
        if self.mode == "position":
            if not self.driver.engage_pos():
                return (False, "Failed to engage_pos motor.")
            return (True, "Engage_pos motor success.")
        else:
            if not self.driver.engage_vel():
                return (False, "Failed to engage_vel motor.")
            return (True, "Engage_vel motor success.")

    def release_motor(self, request):
        # Edit by GGC on June 14: 
        # Checks if driver is connected. If not, throws error.
        # Calls release() function to execute AXIS_STATE_IDLE
            # If this fails, returns False and error message
        # If successful, returns True and success message

        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.release():
            return (False, "Failed to release motor.")
        return (True, "Release motor success.")

    def clear_errors(self, request):
        # Edit by GGC on June 14: 
        # Checks if driver is connected. If not, throws error.
        # Calls release() function to execute AXIS_STATE_IDLE
            # If this fails, returns False and error message
        # If successful, returns True and success message

        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.clearE():
            return (False, "Failed to dump errors.")
        return (True, "Dumped errors success.")


    def reset_odometry(self, request):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        return(True, "Odometry reset.")

    # Helpers and callbacks

    def convert(self, forward, ccw):
        angular_to_linear = ccw * (self.wheel_track/2.0) 
        left_linear_val  = int((forward - angular_to_linear) * self.m_s_to_value)
        right_linear_val = int((forward + angular_to_linear) * self.m_s_to_value)

        return left_linear_val, right_linear_val

    def cmd_vel_callback(self, msg):
        #rospy.loginfo("Received a /cmd_vel message!")
        #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

        # rostopic pub -r 1 /commands/motor/current std_msgs/Float64 -- -1.0

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands

        # 3600 ERPM = 360 RPM ~= 6 km/hr

        #angular_to_linear = msg.angular.z * (wheel_track/2.0) 
        #left_linear_rpm  = (msg.linear.x - angular_to_linear) * m_s_to_erpm
        #right_linear_rpm = (msg.linear.x + angular_to_linear) * m_s_to_erpm

        #AAB CHANGED JUNE 11:
        # left_linear_val, right_linear_val = self.convert(msg.linear.x, msg.angular.z)
        #left_linear_val, right_linear_val = msg.linear.x*10000, msg.angular.z*10000
        # print (left_linear_val,right_linear_val)

        # Editted by GGC on June 21:
        # rad_to_count = self.encoder_cpr / (2 * math.pi)
        # # left_linear_val, right_linear_val = msg.linear.x*rad_to_count, msg.angular.z*rad_to_count   # Edit by GGC on June 28
        # left_linear_val, right_linear_val = msg.linear.x*10, msg.angular.z*10   # Edit by GGC, July 3: rqt sliders limited to +-10000

        # if wheel speed = 0, stop publishing after sending 0 once. #TODO add error term, work out why VESC turns on for 0 rpm

        # Then set your wheel speeds (using wheel_left and wheel_right as examples)
        #self.left_motor_pub.publish(left_linear_rpm)
        #self.right_motor_pub.publish(right_linear_rpm)
        #wheel_left.set_speed(v_l)
        #wheel_right.set_speed(v_r)


        rad_to_count = self.encoder_cpr / (2 * math.pi)
        right_linear_val = msg.angular.z*10
        left_linear_val = msg.linear.x*10    # Edit by GGC, July 3: rqt sliders limited to +-10000

        #rospy.logdebug("Driving left: %d, right: %d, from linear.x %.2f and angular.z %.2f" % (left_linear_val, right_linear_val, msg.linear.x, msg.angular.z))
        try:
            drive_command = ('drive', (left_linear_val, right_linear_val))
            self.command_queue.put_nowait(drive_command)
        except Queue.Full:
            pass

        self.last_cmd_vel_time = rospy.Time.now()

    def cmd_pos_callback(self, msg):
        deg_to_rad = math.pi / 180
        rad_to_count = 8192 / (2 * math.pi)

        # Edit by GGC on July 15: If it is receiving degrees...
        # left_linear_val, right_linear_val = msg.position.y * deg_to_rad * rad_to_count, msg.position.x * deg_to_rad * rad_to_count

        # If it is receiving counts...
        # left = femur (axis 1), right = tibia (axis 0)
        left_linear_val, right_linear_val = msg.position.y, msg.position.x  

        rospy.logwarn(str(left_linear_val) + ", " + str(right_linear_val))

        try:
            drive_command = ('drive', (left_linear_val, right_linear_val))
            self.command_queue.put_nowait(drive_command)
        except Queue.Full:
            pass

        self.last_cmd_vel_time = rospy.Time.now()   # change to be last_cmd_pos_time????

    def pub_current(self):
        current_quantizer = 5

        self.left_current_accumulator += self.current_l
        self.right_current_accumulator += self.current_r

        self.current_loop_count += 1
        if self.current_loop_count >= current_quantizer:
            self.current_publisher_left.publish(float(self.left_current_accumulator) / current_quantizer)
            self.current_publisher_right.publish(float(self.right_current_accumulator) / current_quantizer)

            self.current_loop_count = 0
            self.left_current_accumulator = 0.0
            self.right_current_accumulator = 0.0

    def publish_odometry(self, time_now):
        # Edit by GGC on June 14: 
        # This is where all the math happens!
        # Calculates things like position, speed, etc.
        # Specific to neomanic's wheeled robot right now
        # For us, this is where our inverse kinematics would go (unless we make separate nodes)

        now = time_now
        self.odom_msg.header.stamp = now
        self.tf_msg.header.stamp = now

        wheel_track = self.wheel_track   # check these. Values in m
        tyre_circumference = self.tyre_circumference
        # self.m_s_to_value = encoder_cpr/tyre_circumference set earlier

        # Twist/velocity: calculated from motor values only
        s = tyre_circumference * (self.vel_l+self.vel_r) / (2.0*self.encoder_cpr)
        w = tyre_circumference * (self.vel_r-self.vel_l) / (wheel_track * self.encoder_cpr) # angle: vel_r*tyre_radius - vel_l*tyre_radius
        self.odom_msg.twist.twist.linear.x = s
        self.odom_msg.twist.twist.angular.z = w

        #rospy.loginfo("vel_l: % 2.2f  vel_r: % 2.2f  vel_l: % 2.2f  vel_r: % 2.2f  x: % 2.2f  th: % 2.2f  pos_l: % 5.1f pos_r: % 5.1f " % (
        #                vel_l, -vel_r,
        #                vel_l/encoder_cpr, vel_r/encoder_cpr, self.odom_msg.twist.twist.linear.x, self.odom_msg.twist.twist.angular.z,
        #                self.driver.left_axis.encoder.pos_cpr, self.driver.right_axis.encoder.pos_cpr))

        # Position
        delta_pos_l = self.new_pos_l - self.old_pos_l
        delta_pos_r = self.new_pos_r - self.old_pos_r

        self.old_pos_l = self.new_pos_l
        self.old_pos_r = self.new_pos_r

        # Check for overflow. Assume we can't move more than half a circumference in a single timestep. 
        half_cpr = self.encoder_cpr/2.0
        if   delta_pos_l >  half_cpr: delta_pos_l = delta_pos_l - self.encoder_cpr
        elif delta_pos_l < -half_cpr: delta_pos_l = delta_pos_l + self.encoder_cpr
        if   delta_pos_r >  half_cpr: delta_pos_r = delta_pos_r - self.encoder_cpr
        elif delta_pos_r < -half_cpr: delta_pos_r = delta_pos_r + self.encoder_cpr

        # counts to metres
        delta_pos_l_m = delta_pos_l / self.m_s_to_value
        delta_pos_r_m = delta_pos_r / self.m_s_to_value

        # Distance travelled
        d = (delta_pos_l_m+delta_pos_r_m)/2.0  # delta_ps
        th = (delta_pos_r_m-delta_pos_l_m)/wheel_track # works for small angles

        xd = math.cos(th)*d
        yd = -math.sin(th)*d

        # elapsed time = event.last_real, event.current_real
        #elapsed = (event.current_real-event.last_real).to_sec()
        # calc_vel: d/elapsed, th/elapsed

        # Pose: updated from previous pose + position delta
        self.x += math.cos(self.theta)*xd - math.sin(self.theta)*yd
        self.y += math.sin(self.theta)*xd + math.cos(self.theta)*yd
        self.theta = (self.theta + th) % (2*math.pi)

        #rospy.loginfo("dl_m: % 2.2f  dr_m: % 2.2f  d: % 2.2f  th: % 2.2f  xd: % 2.2f  yd: % 2.2f  x: % 5.1f y: % 5.1f  th: % 5.1f" % (
        #                delta_pos_l_m, delta_pos_r_m,
        #                d, th, xd, yd,
        #                self.x, self.y, self.theta
        #                ))

        # fill odom message and publish

        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        self.odom_msg.pose.pose.orientation.z = q[2] # math.sin(self.theta)/2
        self.odom_msg.pose.pose.orientation.w = q[3] # math.cos(self.theta)/2

        #rospy.loginfo("theta: % 2.2f  z_m: % 2.2f  w_m: % 2.2f  q[2]: % 2.2f  q[3]: % 2.2f (q[0]: %2.2f  q[1]: %2.2f)" % (
        #                        self.theta,
        #                        math.sin(self.theta)/2, math.cos(self.theta)/2,
        #                        q[2],q[3],q[0],q[1]
        #                        ))

        #self.odom_msg.pose.covariance
         # x y z
         # x y z

        self.tf_msg.transform.translation.x = self.x
        self.tf_msg.transform.translation.y = self.y
        #self.tf_msg.transform.rotation.x
        #self.tf_msg.transform.rotation.x
        self.tf_msg.transform.rotation.z = q[2]
        self.tf_msg.transform.rotation.w = q[3]

        if self.publish_raw_odom:
            self.raw_odom_publisher_encoder_left.publish(self.new_pos_l)
            # Temporary Edit by GGC on June 25: commented this so I could test pos_control with rostopic pub
            # REMEMBER TO UNCOMMENT THIS WHEN WE USE THE MOTOR!
            self.raw_odom_publisher_encoder_right.publish(self.new_pos_r)    
            self.raw_odom_publisher_vel_left.publish(self.vel_l)
            self.raw_odom_publisher_vel_right.publish(self.vel_r)

        # ... and publish!
        self.odom_publisher.publish(self.odom_msg)
        if self.publish_tf:
            self.tf_publisher.sendTransform(self.tf_msg)            


def start_odrive():
    rospy.init_node('odrive')
    odrive_node = ODriveNode()
    odrive_node.main_loop()
    #rospy.spin() 

if __name__ == '__main__':
    try:
        start_odrive()
    except rospy.ROSInterruptException:
        pass