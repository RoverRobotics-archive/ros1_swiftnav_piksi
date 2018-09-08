#!/usr/bin/env python

import subprocess
import os
import struct
import time
import sys
import rospy
import numpy as np
import math
from sbp.client.drivers.network_drivers import TCPDriver
from sbp.client import Handler, Framer
from sbp.settings import SBP_MSG_SETTINGS_READ_RESP, MsgSettingsWrite, MsgSettingsReadReq
from sbp.imu import SBP_MSG_IMU_RAW
from sbp.navigation import SBP_MSG_BASELINE_HEADING_DEP_A, SBP_MSG_POS_LLH, SBP_MSG_BASELINE_NED
from datetime import datetime
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import TwistStamped

class SwiftNavDriver(object):

    def __init__(self):
        rospy.loginfo("[RR_SWIFTNAV_PIKSI] Initializing")

        # Initialize message and publisher structures
        self.drive_direction = "forward"
        self.comms_enabled = False
        self.ncat_process = None
        self.previous_x = 0
        self.previous_y = 0

        # TOPIC: swift_gps/llh/fix_mode
        # This topic reports the fix_mode of llh position
        # 0 - Invalid 
	# 1 - Single Point Position (SSP)
	# 2 - Differential GNSS (DGNSS)
	# 3 - Float RTK
	# 4 - Fixed RTK
	# 5 - Dead Reckoning
        # 6 - Satellite-based Augmentation System (SBAS)

	# ROS Publishers
	self.pub_imu = rospy.Publisher('/swift_gps/imu/raw', Imu, queue_size=10)
	self.pub_llh = rospy.Publisher('/swift_gps/llh/position', NavSatFix, queue_size=3)
	self.pub_llh_n_sats = rospy.Publisher('/swift_gps/llh/n_sats', Int32, queue_size=3)
	self.pub_llh_fix_mode = rospy.Publisher('/swift_gps/llh/fix_mode', Int32, queue_size=3)
	self.pub_ecef_odom = rospy.Publisher('/swift_gps/baseline/ecef/position', Odometry, queue_size=3)

	# ROS Subscriber
	self.sub_rtk_cmd = rospy.Subscriber("/swift_gps/enable_comms", Bool, self.enable_comms_cb)
        self.sub_cmd_vel = rospy.Subscriber("/cmd_vel/managed", TwistStamped, self.cmd_vel_cb)            

        # ROS Parameters
        rospy.loginfo("[RR_SWIFTNAV_PIKSI] Loading ROS Parameters")

        path_piksi_ip_address = rospy.search_param('piksi_ip_address')
        self.piksi_ip_address = rospy.get_param(path_piksi_ip_address, '1.2.3.10')
        rospy.loginfo("[RR_SWIFTNAV_PIKSI] Piksi IP address: %s", self.piksi_ip_address)

        path_piksi_port = rospy.search_param('piksi_port')
        self.piksi_port = rospy.get_param(path_piksi_port, '55555')
        rospy.loginfo("[RR_SWIFTNAV_PIKSI] Piksi Port: %s", self.piksi_port)

        path_base_station_ip_address = rospy.search_param('base_station_ip_address')
        self.base_station_ip_address = rospy.get_param(path_base_station_ip_address, '111.111.111.111')
        rospy.loginfo("[RR_SWIFTNAV_PIKSI] Base Station IP address: %s", self.base_station_ip_address)

        path_base_station_port = rospy.search_param('base_station_port')
        self.base_station_port = rospy.get_param(path_base_station_port, '55555')
        rospy.loginfo("[RR_SWIFTNAV_PIKSI] Base Station Port: %s", self.base_station_port)

        path_computer_ip_address = rospy.search_param('computer_ip_address')
        self.computer_ip_address = rospy.get_param(path_computer_ip_address, '1.2.3.55')
        rospy.loginfo("[RR_SWIFTNAV_PIKSI] Computer IP address: %s", self.computer_ip_address)

        # Create SwiftNav Callbacks
        with TCPDriver(self.piksi_ip_address, self.piksi_port) as driver:
            with Handler(Framer(driver.read, driver.write)) as source:
                driver.flush()
                time.sleep(2)
                source.add_callback(self.publish_baseline_msg, SBP_MSG_BASELINE_NED)
                source.add_callback(self.publish_imu_msg,SBP_MSG_IMU_RAW)
                source.add_callback(self.publish_llh_msg,SBP_MSG_POS_LLH)
                source.start

                rospy.spin()


    def cmd_vel_cb(self, cmd_vel):
        if cmd_vel.twist.linear.x > 0:
            self.drive_direction = "forward"
        if cmd_vel.twist.linear.x < 0:
            self.drive_direction = "reverse"


    def enable_comms_cb(self, msg):
        if (msg.data == True):
	    # Note: ncat is the linux networking tool used to tunnel the RTK data through the main PC
	    if (self.ncat_process is None):
                str_cmd = '/usr/bin/ncat -l ' + str(self.computer_ip_address) + ' ' + str(self.piksi_port) + ' --sh-exec "/usr/bin/ncat ' + str(self.base_station_ip_address) + ' ' + str(self.base_station_port) + '"'
                rospy.loginfo(str_cmd)
	        self.ncat_process = subprocess.Popen(str_cmd, shell=True)
                self.comms_enabled = True
                rospy.loginfo("[RR_SWIFNAV_PIKSI] GPS comms enabled, ncat started")
	    else:
	        rospy.logwarn("[RR_SWIFTNAV_PIKSI] GPS comms already enabled, ignoring request")
        if (msg.data == False):
	    if (self.ncat_process is not None):
	        subprocess.call(["kill", "-9", "%d" % self.ncat_process.pid])
	        self.ncat_process.wait()
	        os.system('killall ncat')
                rospy.loginfo("[RR_SWIFT_NAV_PIKSI] GPS comms disables, ncat stopped")
                self.comms_enabled = False
	        self.ncat_process=None
	    else:
	        rospy.logwarn("[RR_SWIFTNAV_PIKSI] RTK GPS already disabled, ignoring request")

    def publish_baseline_msg(self, msg, **metadata):
        if not self.comms_enabled:
            return

        # Obtain position and accuracies and convert from mm to m
        x_pos = float(msg.e)/1000
        y_pos = float(msg.n)/1000
        z_pos = float(msg.d)/1000
        h_accuracy = float(msg.h_accuracy)/1000
        v_accuracy = float(msg.v_accuracy)/1000

        if (x_pos,y_pos) == (0.0,0.0):
            rospy.logwarn_throttle(10,"SwiftNav GPS baseline reported x=0 y=0. Message not published")
            return

        # Build the ROS Odometry message
        ecef_odom_msg = Odometry()
        ecef_odom_msg.child_frame_id = 'gps_link'
        ecef_odom_msg.header.stamp = rospy.Time.now()
        ecef_odom_msg.header.frame_id = 'map'
        ecef_odom_msg.pose.pose.position.x = x_pos
        ecef_odom_msg.pose.pose.position.y = y_pos
        ecef_odom_msg.pose.pose.position.z = 0

        # Calculate distance travelled since last RTK measurement
        if self.drive_direction=="forward":
            delta_x = x_pos - self.previous_x
            delta_y = y_pos - self.previous_y
        if self.drive_direction=="reverse":
            delta_x = self.previous_x - x_pos
            delta_y = self.previous_y - y_pos
        distance_travelled = np.sqrt(np.power(delta_x,2) + np.power(delta_y,2))

        # Normalize the orientation vector
        if (distance_travelled==0):
            delta_x_hat = 0
            delta_y_hat = 0
        else:
            delta_x_hat = delta_x / distance_travelled
            delta_y_hat = delta_y / distance_travelled

        if (distance_travelled>0.04):
            angle = np.arctan2(delta_y_hat, delta_x_hat)
            ecef_odom_msg.pose.pose.orientation.z = 1*np.sin(angle/2)
            ecef_odom_msg.pose.pose.orientation.w = np.cos(angle/2)

        # Update the old positions
        self.previous_x = x_pos
        self.previous_y = y_pos
            
        # Calculate the position covariances using the accuracy reported by the Piksi
        cov_x = cov_y = h_accuracy
            
        # Calculate the orientation covariance, the further we have moved the more accurate orientation is
        if (0<=distance_travelled and distance_travelled<=0.04):
            theta_accuracy = 1000
        elif(0.04<distance_travelled and distance_travelled<=0.01):
            theta_accuracy = 0.348
        elif(0.01<distance_travelled and distance_travelled<=0.4):
            theta_accuracy = 0.174
        elif(0.4<distance_travelled):
            theta_accuracy = 0.14
        else:
            theta_accuracy = -1
            rospy.logerr_throttle(5,"distance travelled was negative")

        cov_theta = theta_accuracy
        ecef_odom_msg.pose.covariance = [cov_x, 0, 0, 0, 0, 0,
                                            0, cov_y, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, cov_theta]
        # Publish earth-centered-earth-fixed message
        self.pub_ecef_odom.publish(ecef_odom_msg)


    def publish_imu_msg(self, msg, **metadata):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'gps_link'
        # acc_range scale settings to +- 4g (8192 LSB/g), gyro_range to +-500 (65.6 LSB/deg/s)
        ascale=9.8/8192.0 # output in meters per second-squared
        gscale=3.14159/180/65.6 # output in radians per second
        imu_msg.angular_velocity.x = msg.gyr_x*gscale
        imu_msg.angular_velocity.y = msg.gyr_y*gscale
        imu_msg.angular_velocity.z = msg.gyr_z*gscale
        imu_msg.linear_acceleration.x = msg.acc_x*ascale
        imu_msg.linear_acceleration.y = msg.acc_y*ascale
        imu_msg.linear_acceleration.z = msg.acc_z*ascale
        imu_msg.orientation_covariance = [0,0,0,
                                          0,0,0,
                                          0,0,0]
        imu_msg.angular_velocity_covariance= [0,0,0,
                                              0,0,0,
                                              0,0,0.01]
        imu_msg.linear_acceleration_covariance= [0.01,0,0,
                                                 0,0.01,0,
                                                 0,0,0.01]
        # Publish to /gps/imu/raw
        self.pub_imu.publish(imu_msg)


    def publish_llh_msg(self, msg, **metadata):
        llh_msg = NavSatFix()
        llh_msg.latitude = msg.lat
        llh_msg.longitude = msg.lon
        llh_msg.altitude = msg.height
        llh_msg.position_covariance_type = 2
        llh_msg.position_covariance = [9,0,0,
                                       0,9,0,
                                       0,0,9]
        # Publish ROS messages
        self.pub_llh.publish(llh_msg)
        self.pub_llh_n_sats.publish(Int32(msg.n_sats))
        self.pub_llh_fix_mode.publish(Int32(msg.flags))

if __name__ == "__main__":
    rospy.init_node('rr_swiftnav_gps_node')
    swift_nav_driver = SwiftNavDriver()



