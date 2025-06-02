# sourcing
source /opt/ros/noetic/setup.bash
source /usr/share/gazebo/setup.sh
source ~/workspace/devel/setup.bash

# launch file
roslaunch my_command_arm my_launch.launch usb_port:=/dev/ttyACM0 baud_rate:=1000000
