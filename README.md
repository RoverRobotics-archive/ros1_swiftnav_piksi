# openrover_ros_swift_gps
ROS Package for using SwiftNav Piksi GPS  
Currently only supports TCP (SwiftNav Piksi must be connected via ethernet cable)  

## Installation
cd ~/catkin_ws/src  
git clone https://github.com/RoverRobotics/openrover_ros_swift_gps.git  
cd ~/catkin_ws  
catkin_make  
source devel/setup.bash 

## Setup 
Use the SwiftNav Console to set the IP address and TCP port  
Edit tcp_example.launch to reflect the IP address and TCP port  

## Run Example Code
roslaunch openrover_ros_swift_gps tcp_example.launch  


## Published Topics:

* `/swift_gps/llh/position`:
  Publishes `sensor_msgs/NavSatFix`, useful for plotting location on a map

* `/swift_gps/llh/fix_mode`:
  Publishes `std_msgs/Int32` value meanings are listed in code, and in SwiftNav SBP documentation 

* `/swift_gps/llh/n_sats`:
  Publishes `std_msgs/Int32` number of sattelites used to obtain llh position

* `/swift_gps/baseline/ecef/position`:
  Publishes `nav_msgs/Odometry` useful as odometry input to kalman filter

* `/swift_gps/baseline/ecef/fix_mode`:
  Publishes `std_msgs/Int32` its recomended to use /swift_gps/llh/fix_mode over this topic

* `/swift_gps/baseline/ecef/n_sats`:
  Publishes `std_msgs/Int32` number of sattelites used to obtain baseline position

* `/swift_gps/imu/mag`:
  Publishes `sensor_msgs/MagneticField` with jpeg from the camera module.

* `/swift_gps/imu/raw`:
  Publishes `sensor_msgs/Imu` camera info for each frame.

* `/swift_gps/comms_disabled`:
  Publishes `std_msgs/Bool` provides feedback for if communication is disabled

## Subcribed Topics:

* `/swift_gps/disable_comms`:
  Subscribes to `std_msgs/Bool` msg that can disable communication to save network bandwidth

## Parameters:

* `ip_address` : IP Address of SwiftNav Piksi, use the SwiftNAv console to check what this is set to 

* `tcp_port`: The tcp port of SwiftNav Piksi, use the SwiftNav console to check what this is set to


