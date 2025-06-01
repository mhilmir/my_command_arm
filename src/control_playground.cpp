#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include <tf/transform_datatypes.h>

#define NUM_OF_JOINT 4

ros::ServiceClient goal_joint_space_path_client_ ;
ros::ServiceClient goal_tool_control_client_;
ros::ServiceClient goal_task_space_path_from_present_client_;
open_manipulator_msgs::KinematicsPose current_pose;

bool setToolControl(ros::NodeHandle& nh, std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(nh.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool setTaskSpacePathFromPresent(ros::NodeHandle& nh, std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = nh.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  tf::Quaternion q = tf::createQuaternionFromRPY(kinematics_pose.at(3), kinematics_pose.at(4), kinematics_pose.at(5));

  srv.request.kinematics_pose.pose.orientation.x = q.x();
  srv.request.kinematics_pose.pose.orientation.y = q.y();
  srv.request.kinematics_pose.pose.orientation.z = q.z();
  srv.request.kinematics_pose.pose.orientation.w = q.w();
  srv.request.path_time = path_time;

  if (goal_task_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_control_node");
    ros::NodeHandle nh;

    ros::Rate rate(10);

    goal_joint_space_path_client_ = nh.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
    goal_tool_control_client_ = nh.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
    goal_task_space_path_from_present_client_ = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present");

    ros::Subscriber kinematics_pose_sub_ = nh.subscribe("/gripper/kinematics_pose", 10, kinematicsPoseCallback);;
    
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    std::vector<double> goalPose;  goalPose.resize(3, 0.0);
    std::vector<double> gripper_joint;
    double delta = 0.01;

    ros::Duration(1.0).sleep();

    // Get Current Pose
    double roll, pitch, yaw;
    tf::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // ROS_INFO("Pose :\nx: %f, y: %f, z: %f\nroll: %f, pitch: %f, yaw: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, roll, pitch, yaw);

    // target pose objek di kiri
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.219665;
    target_pose.position.y = 0.101296;
    target_pose.position.z = 0.0918007;
    // orientation: 
    target_pose.orientation.x = -0.125005;
    target_pose.orientation.y = 0.548743;
    target_pose.orientation.z = 0.0369283;
    target_pose.orientation.w = 0.825767;
    // roll: -0.425102, pitch: 1.156749, yaw: -0.190560


    // Get the Error
    // double x_error = 



    // Kontrol Pose from present

    double path_time3 = 3.0;
    
    goalPose.clear();  goalPose.resize(6, 0.0);
    goalPose.at(0) = delta * 5;  // x
    goalPose.at(1) = 0;  // y
    goalPose.at(2) = -delta * 1;  // z
    goalPose.at(3) = 0;  // roll
    goalPose.at(4) = 0.3;  // pitch
    goalPose.at(5) = 0;  // yaw
    setTaskSpacePathFromPresent(nh, goalPose, path_time3);
    ros::Duration(2.0).sleep();
    


    
    


    
    return 0;
}
