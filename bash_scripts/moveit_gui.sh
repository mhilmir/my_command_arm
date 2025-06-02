# sourcing
source /opt/ros/noetic/setup.bash
source /usr/share/gazebo/setup.sh
source devel/setup.bash

# open port access
sudo chmod 777 /dev/ttyACM0

# launch file
roslaunch open_manipulator_controllers joint_trajectory_controller.launch sim:=false usb_port:=/dev/ttyACM0