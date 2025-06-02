# sourcing
source /opt/ros/noetic/setup.bash
source /usr/share/gazebo/setup.sh
source ~/workspace/devel/setup.bash

# launch file
roslaunch open_manipulator_controller open_manipulator_controller.launch usb_port:=/dev/ttyACM0 baud_rate:=1000000
