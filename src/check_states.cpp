#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
// #include <tf/transform_datatypes.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define NUM_OF_JOINT 4

std::vector<double> present_joint_angle_;
open_manipulator_msgs::KinematicsPose current_pose;

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "check_states_node");
    ros::NodeHandle nh;

    ros::Rate rate(10);

    present_joint_angle_.resize(NUM_OF_JOINT);

    ros::Subscriber joint_states_sub_ = nh.subscribe("joint_states", 10, jointStatesCallback);
    ros::Subscriber kinematics_pose_sub_ = nh.subscribe("/gripper/kinematics_pose", 10, kinematicsPoseCallback);;
    
    while(ros::ok()){

        // double roll, pitch, yaw;
        // tf::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        tf2::Quaternion q(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w
        );

        // Convert Quaternion to RPY (roll, pitch, yaw)
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Convert Quaternion to Rotation Matrix
        tf2::Matrix3x3 rotation_matrix(q);
        double r00 = rotation_matrix[0][0];
        double r01 = rotation_matrix[0][1];
        double r02 = rotation_matrix[0][2];
        double r10 = rotation_matrix[1][0];
        double r11 = rotation_matrix[1][1];
        double r12 = rotation_matrix[1][2];
        double r20 = rotation_matrix[2][0];
        double r21 = rotation_matrix[2][1];
        double r22 = rotation_matrix[2][2];

        ROS_INFO("Joint States:\n  joint1: %f, joint2: %f, joint3: %f, joint4: %f", present_joint_angle_.at(0), present_joint_angle_.at(1), present_joint_angle_.at(2), present_joint_angle_.at(3));

        ROS_INFO("Position :\n  x: %f, y: %f, z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
        
        ROS_INFO("Orientation Quaternion :\n  x: %f, y: %f, z: %f, w: %f", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        
        ROS_INFO("Orientation Euler :\n  roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

        ROS_INFO("Orientation Matrix Rotation :");
        std::cout << "[" << r00 << ", " << r01 << ", " << r02 << "]" << std::endl;
        std::cout << "[" << r10 << ", " << r11 << ", " << r12 << "]" << std::endl;
        std::cout << "[" << r20 << ", " << r21 << ", " << r22 << "]" << std::endl;
        std::cout << std::endl;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
