# sourcing
source /opt/ros/noetic/setup.bash
source /usr/share/gazebo/setup.sh
source ~/workspace/devel/setup.bash

# launch file
roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch
