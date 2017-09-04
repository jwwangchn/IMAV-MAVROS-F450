# IMAV-MAVROS-F450

### 1. Run the check program
`roslaunch mavros apm.launch`

### 2. Modify the radio frequency
`rosservice call /mavros/set_stream_rate 0 50 1`

### 3. Arm
`rosrun mavros mavsafety arm`

### 4. Run the self program
`rosrun ros_erle_takeoff_land`

### 5. Tips
- Use `rosrun mavros mavsys mode -c AUTO` modify the mode of UAV.
- Use `rostopic echo -n1 /diagnostics | tee /tmp/diag.yml` watch the state of system.
- Use `catkin_create_pkg offboard roscpp mavros geometry_msgs` create ROS package

### Reference 
[MAVROS Package Summary](http://wiki.ros.org/mavros#Usage)