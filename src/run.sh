source /opt/ros/melodic/setup.bash
source ~/blamlight_ws/devel/setup.bash
#source ~/catkin_ws/devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sudo chmod a+rw /dev/ttyUSB0; sudo chmod 777 /dev/ttyTHS0; sleep 1; rosrun serial_port serial_port.cpp; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch rplidar_ros rplidar.launch; exec bash"' \
--tab -e 'bash -c "roslaunch mbot_gazebo mbot_laser_nav_gazebo.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch mbot_navigation blamlight_slam.launch; exec bash"' \

