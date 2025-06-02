# sourcing
source /opt/ros/noetic/setup.bash
source /usr/share/gazebo/setup.sh
source devel/setup.bash

# launch file
roslaunch open_manipulator_controllers joint_trajectory_controller.launch sim:=false usb_port:=/dev/ttyACM0