# sourcing
source /opt/ros/noetic/setup.bash
source /usr/share/gazebo/setup.sh
source devel/setup.bash

# open port access
sudo chmod 777 /dev/ttyACM0

# launch file
roslaunch my_command_arm controller_custom.launch usb_port:=/dev/ttyACM0 baud_rate:=1000000
