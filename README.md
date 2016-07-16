# About:

This is ROS package dedicated for simple visual marker recognition and follow.

# Installation:

	cd your_ros_ws/src
  	git clone https://github.com/dkoguciuk/jetson_follower.git
  	cd .. && catkin_make
	
# Usage:

First of all you'll have to calibrate HSV params for your marker with jetson_calibration:

	rosrun jetson_follower jetson_calibration
  
The resulted HSV values will be stored in config directory. Then you can run robot following node:

	rosrun jetson_follower jetson_follower
