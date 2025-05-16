#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include <tf/transform_datatypes.h>

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

        // Convert to RPY (roll, pitch, yaw)
        double roll, pitch, yaw;
        tf::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        ROS_INFO("Joint States:\njoint1: %f, joint2: %f, joint3: %f, joint4: %f", present_joint_angle_.at(0), present_joint_angle_.at(1), present_joint_angle_.at(2), present_joint_angle_.at(3));

        ROS_INFO("Pose :\nx: %f, y: %f, z: %f\nroll: %f, pitch: %f, yaw: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, roll, pitch, yaw);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
