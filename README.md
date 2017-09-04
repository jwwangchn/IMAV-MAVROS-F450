# IMAV-MAVROS-F450

1. Create ROS Package
catkin_create_pkg offboard roscpp mavros geometry_msgs

2. Run the check program
roslaunch mavros apm.launch

3. Modify the radio frequency
rosservice call /mavros/set_stream_rate 0 50 1

4. Arm
rosrun mavros mavsafety arm

5. Run the self program
rosrun ros_erle_takeoff_land

6. Tips
- Use `rosrun mavros mavsys mode -c AUTO` modify the mode of UAV.
- Use `rostopic echo -n1 /diagnostics | tee /tmp/diag.yml` watch the state of system.

## Reference 
[MAVROS Package Summary](http://wiki.ros.org/mavros#Usage)