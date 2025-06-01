#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include <tf/transform_datatypes.h>

#define NUM_OF_JOINT 4

ros::ServiceClient goal_joint_space_path_client_ ;
ros::ServiceClient goal_joint_space_path_from_present_client_ ;
ros::ServiceClient goal_task_space_path_from_present_position_only_client_;
ros::ServiceClient goal_tool_control_client_;
ros::ServiceClient goal_task_space_path_from_present_client_;
ros::ServiceClient goal_task_space_path_client_;

bool setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_from_present_client_.call(srv))
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

bool setTaskSpacePathFromPresentPositionOnly(ros::NodeHandle& nh, std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = nh.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if (goal_task_space_path_from_present_position_only_client_.call(srv))
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

bool setTaskSpacePath(ros::NodeHandle& nh, std::vector<double> kinematics_pose, double path_time)
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

  if (goal_task_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_control_node");
    ros::NodeHandle nh;

    ros::Rate rate(10);

    goal_joint_space_path_client_ = nh.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
    goal_joint_space_path_from_present_client_ = nh.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
    goal_task_space_path_from_present_position_only_client_ = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");
    goal_tool_control_client_ = nh.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
    goal_task_space_path_from_present_client_ = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present");
    goal_task_space_path_client_ = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path");
    
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    std::vector<double> goalPose;  goalPose.resize(3, 0.0);
    std::vector<double> gripper_joint;
    double delta = 0.01;

    //////////////////////////////////////////// START EXAMPLES /////////////////////////////////////////////

    // Kontrol Joint /////////////////////////////////////////////////////////
    double path_time1 = 2.0;
    
    joint_name.clear();
    joint_angle.clear();
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time1);
    ros::Duration(2.0).sleep();
    
    joint_name.clear();
    joint_angle.clear();
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time1);
    ros::Duration(2.0).sleep();

    ///////////////////////////////////////////////////////////////////////////
    
    
    
    // Kontrol Pose from present (position only) /////////////////////////////
    double path_time2 = .5;
    
    goalPose.clear();  goalPose.resize(3, 0.0);
    goalPose.at(0) = delta * 2;  // x
    goalPose.at(1) = delta * 2;  // y
    goalPose.at(2) = delta * 2;  // z
    setTaskSpacePathFromPresentPositionOnly(nh, goalPose, path_time2);
    ros::Duration(2.0).sleep();
    
    goalPose.clear();  goalPose.resize(3, 0.0);
    goalPose.at(0) = -delta * 2;  // x
    goalPose.at(1) = -delta * 2;  // y
    goalPose.at(2) = -delta * 2;  // z
    setTaskSpacePathFromPresentPositionOnly(nh, goalPose, path_time2);
    ros::Duration(2.0).sleep();
    
    ///////////////////////////////////////////////////////////////////////////
    
    
    
    // Kontrol Gripper ///////////////////////////////////////////////////////
    gripper_joint.clear();
    gripper_joint.push_back(0.01);  // open
    setToolControl(nh, gripper_joint);
    ros::Duration(2.0).sleep();
    
    gripper_joint.clear();
    gripper_joint.push_back(-0.01);  // tutup
    setToolControl(nh, gripper_joint);
    ros::Duration(2.0).sleep();
    
    // ///////////////////////////////////////////////////////////////////////////
    


    
    // Kontrol Pose from present //////////////////////////////////////////////
    double path_time3 = 3.0;
    
    goalPose.clear();  goalPose.resize(6, 0.0);
    goalPose.at(0) = delta * 5;  // x
    goalPose.at(1) = 0;  // y
    goalPose.at(2) = -delta * 1;  // z
    goalPose.at(3) = 0;  // roll
    goalPose.at(4) = -0.3;  // pitch
    goalPose.at(5) = 0;  // yaw
    setTaskSpacePathFromPresent(nh, goalPose, path_time3);
    ros::Duration(2.0).sleep();
    
    ///////////////////////////////////////////////////////////////////////////
    
    


    
    // // Kontrol Pose ////////////////////////////////////////////// (belum bisa)
    // double path_time4 = 3.0;
    
    // goalPose.clear();  goalPose.resize(6, 0.0);
    // goalPose.at(0) = 0.135685;  // x
    // goalPose.at(1) = 0.000190;  // y
    // goalPose.at(2) = 0.237399;  // z
    // goalPose.at(3) = -0.000000;  // roll
    // goalPose.at(4) = 0.019942;  // pitch
    // goalPose.at(5) = 0.001534;  // yaw
    // setTaskSpacePath(nh, goalPose, path_time4);
    // ros::Duration(2.0).sleep();

    
    ///////////////////////////////////////////////////////////////////////////


    //////////////////////////////////////////// END EXAMPLES /////////////////////////////////////////////
    
    
    return 0;
}
