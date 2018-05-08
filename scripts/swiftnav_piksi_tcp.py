#!/usr/bin/env python

from sbp.client.drivers.network_drivers import TCPDriver
from sbp.client import Handler, Framer
from sbp.settings import MsgSettingsWrite, MsgSettingsReadReq
from sbp.settings import SBP_MSG_SETTINGS_READ_RESP

from sbp.navigation import SBP_MSG_BASELINE_NED
from sbp.imu import SBP_MSG_IMU_RAW
from sbp.mag import SBP_MSG_MAG_RAW
from sbp.navigation import SBP_MSG_BASELINE_HEADING_DEP_A
from sbp.navigation import SBP_MSG_POS_LLH

from datetime import datetime
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField 
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool

import subprocess
import os

import struct
import time
import sys
import rospy

# Initialize message and publisher structures

DEFAULT_COMM_STATUS = False
NCAT_PROC = None
RTK_ENABLE = False
RTK_DISABLE = False

# TOPIC: swift_gps/imu/raw
# This topic publishes the current raw IMU output of the swift nav gps
imu_msg = Imu()
imu_pub = rospy.Publisher('swift_gps/imu/raw', Imu, queue_size=10)

# TOPIC: swift_gps/imu/mag
# This topic puslished the current magnotometer reading of the swift nav gps
heading_msg = MagneticField()
heading_pub = rospy.Publisher('swift_gps/imu/mag', MagneticField, queue_size=10)

# TOPIC: swift_gps/llh/position
# This position solution message reports the absolute geodetic coordinates and the status (single point
# vs pseudo-absolute RTK) of the position solution. If the rover receiver knows the surveyed position of
# the base station and has an RTK solution, this reports a pseudo-absolute position solution using the
# base station position and the rovers RTK baseline vector. The full GPS time is given by the preceding
# MSG GPS TIME with the matching time-of-week (tow).

# TOPIC: swift_gps/llh/n_sats
# This topic reports the number of satelites used to obtain llh position 

# TOPIC: swift_gps/llh/fix_mode
# This topic reports the fix_mode of llh position
# 0 - Invalid 
# 1 - Single Point Position (SSP)
# 2 - Differential GNSS (DGNSS)
# 3 - Float RTK
# 4 - Fixed RTK
# 5 - Dead Reckoning

llh_msg = NavSatFix()
llh_pub = rospy.Publisher('swift_gps/llh/position', NavSatFix, queue_size=3)

llh_n_sats_msg = Int32()
llh_n_sats_pub = rospy.Publisher('swift_gps/llh/n_sats', Int32, queue_size=3)

llh_fix_mode_msg = Int32()
llh_fix_mode_pub = rospy.Publisher('swift_gps/llh/fix_mode', Int32, queue_size=3)

# TOPIC: swift_nav/baseline/odom
# This topic provides an X Y Z position in ECEF (earth centered earth fixed) coordinates of the swift_nav gps with respect to the basestation. This is the 
# topic that should be used for navigation purposes and can be accurate to cm level when RTK fix is achieved

# TOPIC: swif_gps/baseline/n_sats
# This topic provides the number of sattelites currently being used for the RTK fix algorythm

# TOPIC: swift_nav/baseline/fix_mode
# This topic  publishes the current fix mode of baseline/ecef/position topic when RTK float or fix is achieved
# 0 - Invalid
# 1 - Reserved
# 2 - Differential GNSS (DGNSS)
# 3 - Float RTK
# 4 - Fixed RTk

ecef_odom_msg = Odometry()
ecef_odom_pub = rospy.Publisher('swift_gps/baseline/ecef/position', Odometry, queue_size=3)

ecef_n_sats_msg = Int32()
ecef_n_sats_pub = rospy.Publisher('swift_gps/baseline/ecef/n_sats', Int32, queue_size=3, latch=True)

ecef_fix_mode_msg = Int32()
ecef_fix_mode_pub = rospy.Publisher('swift_gps/baseline/ecef/fix_mode', Int32, queue_size=3, latch=True)



# swift_gps/comms_disabled provides feedback as to whether comms has been disabled or not  
comms_disabled_msg = Bool()
comms_disabled_pub = rospy.Publisher('swift_gps/comms_disabled', Bool, queue_size=3, latch=True)

# Variable to track is ncat process is running
# note: ncat if the linux networking tool used to tunnel RTK data through the main PC
NCAT_PROC=None

# Initialize ROS node 
rospy.init_node('swift_gps_node')

def disable_comms_cb(msg):
    global RTK_ENABLE, RTK_DISABLE, NCAT_PROC
    # determine what message was - do we need high accuracy or not?
    if (msg.data == False and NCAT_PROC is None):
        RTK_ENABLE = True
    elif (msg.data == True and NCAT_PROC is not None):
        RTK_DISABLE = True

# swift_gps/disable_comms provides a way of stopping the SwiftNav GPS from communicating with the base station during times when RTK fix is not needed
# in order to save network bandwidth
rtk_cmd_sub = rospy.Subscriber("swift_gps/disable_comms", Bool, disable_comms_cb)

def publish_baseline_msg(msg, **metadata):
    global comms_disabled_msg

    # Publish n_sats and fix_mode
    ecef_n_sats_msg.data = msg.n_sats
    ecef_fix_mode_msg.data = msg.flags
    ecef_n_sats_pub.publish(ecef_n_sats_msg)
    ecef_fix_mode_pub.publish(ecef_fix_mode_msg)
    

    # Obtain position and covariance and convert from mm to m
    x_pos = float(msg.e)/1000
    y_pos = float(msg.n)/1000
    z_pos = float(msg.d)/1000
    h_accuracy = float(msg.h_accuracy)/1000
    v_accuracy = float(msg.v_accuracy)/1000

    if comms_disabled_msg == True:
        rospy.logwarn_throttle(60, "SfiwtNav GPS comms are disabled")
        return
    elif comms_disabled_msg == False:
        if x_pos < 5 and y_pos < 5 and x_pos > -5 and y_pos > -5:
            rospy.logwarn_throttle(10,"SwiftNav GPS baseline reported x=0 y=0. Message not published")
            return

    # Calculate the covariance from accuracy
    cov_x = h_accuracy * h_accuracy
    #cov_x = 9*9
    cov_y = cov_x
    cov_z = v_accuracy * v_accuracy
    #cov_x = 9
    #cov_y = 9
    #cov_z = 9
   
    # Build the ROS Odometry message
    ecef_odom_msg.child_frame_id = 'gps_link'
    ecef_odom_msg.header.stamp = rospy.Time.now()
    ecef_odom_msg.header.frame_id = 'map'
    ecef_odom_msg.pose.pose.position.x = x_pos
    ecef_odom_msg.pose.pose.position.y = y_pos
    ecef_odom_msg.pose.pose.position.z = z_pos
    ecef_odom_msg.pose.covariance = [cov_x, 0, 0, 0, 0, 0,
                                    0, cov_y, 0, 0, 0, 0,
                                    0, 0, cov_z, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0]
    # odom_msg.pose.quaternion.w =
    # odom_msg.pose.quaternion.z =
    

    # Publish topics
    ecef_odom_pub.publish(ecef_odom_msg)


def publish_imu_msg(msg, **metadata):
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'gps_link'
    # acc_range scale settings to +- 8g (4096 LSB/g), gyro_range to +-1000 (32.8 LSB/deg/s)
    #ascale=1.0/4096.0
    #gscale=1.0/32.8
    # acc_range scale settings to +- 4g (8192 LSB/g), gyro_range to +-500 (65.6 LSB/deg/s)
    # output in meters per second-squared
    ascale=9.8/8192.0
    # output in radians per second
    gscale=3.14159/180/65.6
    imu_msg.angular_velocity.x = msg.gyr_x*gscale
    imu_msg.angular_velocity.y = msg.gyr_y*gscale
    imu_msg.angular_velocity.z = msg.gyr_z*gscale
    imu_msg.linear_acceleration.x = msg.acc_x*ascale
    imu_msg.linear_acceleration.y = msg.acc_y*ascale
    imu_msg.linear_acceleration.z = msg.acc_z*ascale
    imu_msg.orientation_covariance = [0,0,0,
                                      0,0,0,
                                      0,0,0]
    imu_msg.angular_velocity_covariance= [1,0,0,
                                      0,1,0,
                                      0,0,1]
    imu_msg.linear_acceleration_covariance= [1,0,0,
                                      0,1,0,
                                      0,0,1]

    # Publish to /gps/imu/raw
    imu_pub.publish(imu_msg)



def publish_llh_msg(msg, **metadata):
    llh_msg.latitude = msg.lat
    llh_msg.longitude = msg.lon
    llh_msg.altitude = msg.height
    llh_msg.position_covariance_type = 2
    #llh_msg.position_covariance = [pow(float(msg.h_accuracy)/1000,2),0,0,
    #                                  0,pow(float(msg.h_accuracy)/1000,2),0,
    #                                  0,0,pow(float(msg.v_accuracy)/1000,2)]
    llh_msg.position_covariance = [pow(3,2),0,0,
                                      0,pow(3,2),0,
                                      0,0,pow(3,2)]
    llh_n_sats_msg = msg.n_sats
    llh_fix_mode_msg = msg.flags

    # Publish ROS messages
    llh_pub.publish(llh_msg)
    llh_n_sats_pub.publish(llh_n_sats_msg)
    llh_fix_mode_pub.publish(llh_fix_mode_msg)



def publish_heading_msg(msg, **metadata):
    heading_msg.header.stamp = rospy.Time.now()
    heading_msg.header.frame_id = 'gps_link'
    # should be in Tesla - chip stores data in two's complement 13-bits data
    # according to /usr/local/lib/python2.7/dist-packages/sbp/mag.py swiftnav does not do anything special to it
    # according to Bosch's datasheet, output is scaled by 16, so divide by 16 to get to uTesla
    # and divide further by 100 so it is in Gauss
    mscale=0.01/16
    # -0.15833344 0.06957173 0.15392401
    # 24.84406845 22.90925231 149.02582974
    #heading_msg.magnetic_field.x = (msg.mag_x*mscale + 0.15833344)*24.84406845
    #heading_msg.magnetic_field.y = (msg.mag_y*mscale - 0.06957173)*22.90925231
    #heading_msg.magnetic_field.z = (msg.mag_z*mscale - 0.15392401)*20
    heading_msg.magnetic_field.x = (msg.mag_x*mscale)
    heading_msg.magnetic_field.y = (msg.mag_y*mscale)
    heading_msg.magnetic_field.z = (msg.mag_z*mscale)
    heading_msg.magnetic_field_covariance= [1,0,0,
                                      0,1,0,
                                      0,0,1]

    # Publish to /gps/imu/mag
    heading_pub.publish(heading_msg)

class SettingMonitor(object):
    """Class to monitor Settings via SBP messages
    Parameters
    ----------
    None
    """

    def __init__(self):
        self.settings = []

    def capture_setting(self, sbp_msg, **metadata):
        """Callback to extract and store setting values from
        SBP_MSG_SETTINGS_READ_RESP
        Messages of any type other than SBP_MSG_SETTINGS_READ_RESP are ignored
        """
        if sbp_msg.msg_type == SBP_MSG_SETTINGS_READ_RESP:
            section, setting, value = sbp_msg.payload.split('\0')[:3]
            self.settings.append((section, setting, value))

    def wait_for_setting_value(self, section, setting, value, wait_time=5.0):
        """Function to wait wait_time seconds to see a
        SBP_MSG_SETTINGS_READ_RESP message with a user-specified value
        """
        expire = time.time() + wait_time
        ok = False
        while not ok and time.time() < expire:
            settings = filter(lambda x: (x[0], x[1]) == (section, setting),
                              self.settings)
            # Check to see if the last setting has the value we want
            if len(settings) > 0:
                ok = settings[-1][2] == value

            time.sleep(0.1)
        return ok

    def clear(self, section=None, setting=None, value=None):
        """Clear settings"""
        match = map(lambda (x,y,z): all((section is None or x == section,
                                         setting is None or y == setting,
                                         value is None or z == value)),
                    self.settings)

        keep = filter(lambda (setting,remove): not remove,
                      zip(self.settings,match))

        self.settings[:] = map(lambda x: x[0], keep)

def sbp_print_setting(sbp_msg, **metadata):
    print sbp_msg

def main():
    global RTK_ENABLE, RTK_DISABLE, NCAT_PROC, DEFAULT_COMM_STATUS
    global comms_disabled_msg

    ipaddr = rospy.get_param('default_param', '1.2.3.10')
    tcp_port = rospy.get_param('default_param', '55555')

    monitor = SettingMonitor()

    with TCPDriver(ipaddr, tcp_port) as driver:
        with Handler(Framer(driver.read, driver.write)) as source:
            driver.flush()
            time.sleep(2)
            # Capture setting messages
            source.add_callback(monitor.capture_setting,SBP_MSG_SETTINGS_READ_RESP)
            source.add_callback(sbp_print_setting, SBP_MSG_SETTINGS_READ_RESP)
            source.add_callback(publish_baseline_msg, SBP_MSG_BASELINE_NED)
            source.add_callback(publish_imu_msg,SBP_MSG_IMU_RAW)
            source.add_callback(publish_heading_msg,SBP_MSG_MAG_RAW)
            source.add_callback(publish_llh_msg,SBP_MSG_POS_LLH)
            source.start

            
            # we leave the swiftnav tcp_client0 on all the time, but pointing to 1.2.3.55:55555 and only start the ncat forwarder when it is needed
            comms_disabled_msg.data = DEFAULT_COMM_STATUS
            comms_disabled_pub.publish(comms_disabled_msg)
            os.system('killall ncat > /dev/null 2>&1')

            while not rospy.is_shutdown():
                # Enable basestation communication if rtk_enable_global is TRUE 
                # Note: ncat is the linux networking tool used to tunnel the RTK data through the main PC 
                if (RTK_ENABLE):
                    if (NCAT_PROC is None):
                        NCAT_PROC = subprocess.Popen('/usr/bin/ncat -l 1.2.3.55 55555 --sh-exec "/usr/bin/ncat 65.132.94.146 55555"', shell=True)
                        rospy.loginfo("GPS tunnel running...")
                        # done setting, set the rtk_enable boolean to False
                        RTK_ENABLE=False
                        comms_disabled_msg.data = True
                        comms_disabled_pub.publish(comms_disabled_msg)
                # Disable basestation communication if rtf_disable_global is TRUE
                # Note: ncat is the linux networking tool used to tunnel the RTK data through the main PC
                if (RTK_DISABLE):
                    if (NCAT_PROC is not None):
                        subprocess.call(["kill", "-9", "%d" % NCAT_PROC.pid])
                        NCAT_PROC.wait()
                        os.system('killall ncat')
                        rospy.loginfo("GPS tunnel killed...")
                        NCAT_PROC=None
                        # done setting, set the rtk_disable boolean to False
                        RTK_DISABLE=False
                        comms_disabled_msg.data = False
                        comms_disabled_pub.publish(comms_disabled_msg)
            	time.sleep(0.1)
            sys.exit()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        # kill the ncat process
        if (NCAT_PROC is not None):
            subprocess.call(["kill", "-9", "%d" % NCAT_PROC.pid])
            NCAT_PROC.wait()
        pass

    #except KeyboardInterrupt:
