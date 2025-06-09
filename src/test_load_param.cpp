#include <ros/ros.h>
#include <iostream>
#include <map>
#include <vector>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_load_param_node");
    ros::NodeHandle nh;

    std::map<std::string, double> joint_states;
    if (nh.getParam("before_grasp_pose/joint_states", joint_states)) {
        std::cout << "Joint States:" << std::endl;
        for (const auto& pair : joint_states) {
            std::cout << "  " << pair.first << ": " << pair.second << std::endl;
        }
    }

    std::map<std::string, double> position;
    if (nh.getParam("before_grasp_pose/position", position)) {
        std::cout << "Position:" << std::endl;
        for (const auto& pair : position) {
            std::cout << "  " << pair.first << ": " << pair.second << std::endl;
        }
    }

    std::map<std::string, double> quaternion;
    if (nh.getParam("before_grasp_pose/quaternion", quaternion)) {
        std::cout << "Quaternion:" << std::endl;
        for (const auto& pair : quaternion) {
            std::cout << "  " << pair.first << ": " << pair.second << std::endl;
        }
    }

    std::map<std::string, double> euler;
    if (nh.getParam("before_grasp_pose/euler", euler)) {
        std::cout << "Euler:" << std::endl;
        for (const auto& pair : euler) {
            std::cout << "  " << pair.first << ": " << pair.second << std::endl;
        }
    }

  return 0;
}
