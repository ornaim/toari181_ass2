# toari181_ass2
ass2
## assignment running instructions:
	⋅⋅* Robot ip = 192.168.0.100
	⋅⋅* Computer ip = 192.168.0.101	
1) run the following command in terminal .
	1.1) ssh komodo@192.168.0.100  #when 192.168.0.100 is the robot ip
	1.2) export ROS_IP="192.168.0.100" #define ROS_IP and check you defined it right with echo.
	1.3) run the following command in the ros.
		"roslaunch robotican_komodo komodo.launch lidar:=true front_camera:=true robot_localization:=true asus_camera:=true"
2)run the following command in terminal .
	source /opt/ros/indigo/setup.bash
    source $HOME/catkin_ws/devel/setup.bash
    source /usr/share/gazebo/setup.sh
	export ROS_MASTER_URI="http://192.168.0.100:11311"
	echo $ROS_MASTER_URI
	export ROS_IP="192.168.0.101"  #my computer ip. ifconfig
	echo $ROS_IP
	export PS1="[ROS-ROBOT]${PS1}"

	Another Option is to define the following in ~/.bashrc
	############################################################
"	# Setup ROS Lunar environment
	ros-env() {
	  export LD_LIBRARY_PATH="/usr/oldlib:${LD_LIBRARY_PATH}"
	    source /opt/ros/indigo/setup.bash
	      source $HOME/catkin_ws/devel/setup.bash
	        source /usr/share/gazebo/setup.sh
	          alias catkin_make="catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python2"
	    	export PS1="[ROS-${ROS_DISTRO}]${PS1}"
	}
	ros-robot-ip(){
		source /opt/ros/indigo/setup.bash
	    source $HOME/catkin_ws/devel/setup.bash
	    source /usr/share/gazebo/setup.sh
		export ROS_MASTER_URI="http://192.168.0.100:11311"
		echo $ROS_MASTER_URI
		export ROS_IP="192.168.0.101"  #my computer ip. ifconfig
		echo $ROS_IP
		export PS1="[ROS-ROBOT]${PS1}"

	}"
	############################################################	
3) after we defined the env-variables . run in terminal the following " rosrun ass2 demoSpin.py ". 

4)enter a command number between 0-4:
0 - exit the program
1 - move the robot 50cm straight if there is no obsticle within 50cm of the robot
2 - turn the robot X degrees
3 - check if there is a red obsticle in front of the robot
4 - rotate the robot until it locates a red obsticle

writen by :
			Linoy Barel			
			Or Naimark			
			Inon Ben-David		
