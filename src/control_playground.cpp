#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define NUM_OF_JOINT 4

ros::ServiceClient goal_joint_space_path_client_ ;
ros::ServiceClient goal_tool_control_client_;
ros::ServiceClient goal_task_space_path_from_present_client_;
ros::ServiceClient goal_task_space_path_from_present_position_only_client_;
std::vector<double> present_joint_angle_;
open_manipulator_msgs::KinematicsPose current_pose;
bool pose_received = false;
bool joint_received = false;
double current_roll, current_pitch, current_yaw;
tf2::Quaternion cur_q;

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

void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
    current_pose = *msg;
    pose_received = true;

    // Get Current RPY
    cur_q.setX(current_pose.pose.orientation.x);
    cur_q.setY(current_pose.pose.orientation.y);
    cur_q.setZ(current_pose.pose.orientation.z);
    cur_q.setW(current_pose.pose.orientation.w);
    tf2::Matrix3x3(cur_q).getRPY(current_roll, current_pitch, current_yaw);
}

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
  joint_received = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_control_node");
    ros::NodeHandle nh;

    ros::Rate rate(10);
    present_joint_angle_.resize(NUM_OF_JOINT);

    goal_joint_space_path_client_ = nh.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
    goal_tool_control_client_ = nh.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
    goal_task_space_path_from_present_client_ = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present");
    goal_task_space_path_from_present_position_only_client_ = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");

    ros::Subscriber kinematics_pose_sub_ = nh.subscribe("/gripper/kinematics_pose", 10, kinematicsPoseCallback);;
    ros::Subscriber joint_states_sub_ = nh.subscribe("joint_states", 10, jointStatesCallback);
    
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    std::vector<double> goalPose;  goalPose.resize(3, 0.0);
    std::vector<double> gripper_joint;
    double delta = 0.01;
    double path_time;
    geometry_msgs::Pose target_pose;

    // Waiting for Joint and Pose Data
    while(ros::ok()){
        ros::spinOnce();
        if (!joint_received || !pose_received){
            ROS_WARN_THROTTLE(1.0, "Waiting for joint_states and kinematics_pose...");
            rate.sleep();
            continue;
        }
        break;
    }

    // Perform Ambil Pose State /////////////////////////////////////////////////////////
    path_time = 2.0;
    joint_name.clear();
    joint_angle.clear();
    joint_name.push_back("joint1"); joint_angle.push_back(0.00);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.949534);
    joint_name.push_back("joint3"); joint_angle.push_back(0.018408);
    joint_name.push_back("joint4"); joint_angle.push_back(1.780952);
    setJointSpacePath(joint_name, joint_angle, path_time);
    ros::Duration(2.0).sleep();

    ///////////////////////////////////////////////////////////////////////////
    // ros::spinOnce();
    // ROS_INFO("Pose :\nx: %f, y: %f, z: %f\nroll: %f, pitch: %f, yaw: %f\n", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, current_roll, current_pitch, current_yaw);
    // x: 0.079150, y: -0.000206, z: 0.170789
    // roll: 0.000000, pitch: 0.880505, yaw: -0.003068

    // Kontrol Gripper Open ///////////////////////////////////////////////////////
    gripper_joint.clear();
    gripper_joint.push_back(0.01);  // open
    setToolControl(nh, gripper_joint);
    ros::Duration(2.0).sleep();
    // ///////////////////////////////////////////////////////////////////////////

    // target pose objek di kiri (Transformed)
    // target_pose.position.x = 0.219665;
    // target_pose.position.y = 0.101296;
    // target_pose.position.z = 0.0918007;
    // target_pose.orientation.x = -0.125005;
    // target_pose.orientation.y = 0.548743;
    // target_pose.orientation.z = 0.0369283;
    // target_pose.orientation.w = 0.825767;
    // roll: -0.425102, pitch: 1.156749, yaw: -0.190560

    // target pose objek di kanan (Transformed)
    // target_pose.position.x = 0.225751;
    // target_pose.position.y = -0.0454252;
    // target_pose.position.z = 0.0945027;

    // target pose objek di atas (Transformed)
    // target_pose.position.x = 0.266441;
    // target_pose.position.y = 0.00850331;
    // target_pose.position.z = 0.0551839;
    
    // target pose objek di bawah (Transformed)
    // target_pose.position.x = 0.188347;
    // target_pose.position.y = -0.0415902;
    // target_pose.position.z = 0.0858944;

    // target pose objek Glue Stick (Transformed)
    // target_pose.position.x = 0.279738;
    target_pose.position.x = 0.266441;  // coba ini dah, yg asli fail to solve IK
    target_pose.position.y = -0.0143216;
    target_pose.position.z = 0.0438075;
    // target_pose.position.z = 0.055;  // biar ga jedug

    ros::spinOnce();
    // Get the Error between current and target
    double x_error = target_pose.position.x - current_pose.pose.position.x;
    double y_error = target_pose.position.y - current_pose.pose.position.y;
    double z_error = target_pose.position.z - current_pose.pose.position.z;
    
    // Kontrol Pose from present (position only) /////////////////////////////
    path_time = 4.0;
    goalPose.clear();  goalPose.resize(3, 0.0);
    goalPose.at(0) = x_error;  // x
    goalPose.at(1) = y_error;  // y
    goalPose.at(2) = z_error;  // z
    setTaskSpacePathFromPresentPositionOnly(nh, goalPose, path_time);
    ros::Duration(6.0).sleep();
    // ros::spinOnce();
    // ROS_INFO("Pose :\nx: %f, y: %f, z: %f\nroll: %f, pitch: %f, yaw: %f\n", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, current_roll, current_pitch, current_yaw);

    // Kontrol Gripper Tutup ///////////////////////////////////////////////////////
    gripper_joint.clear();
    gripper_joint.push_back(-0.01);  // tutup
    setToolControl(nh, gripper_joint);
    ros::Duration(2.0).sleep();
    // ///////////////////////////////////////////////////////////////////////////

    // // Kontrol Pose from present (position only) /////////////////////////////
    // path_time = 4.0;
    // goalPose.clear();  goalPose.resize(3, 0.0);
    // goalPose.at(0) = -x_error;  // x
    // goalPose.at(1) = -y_error;  // y
    // goalPose.at(2) = -z_error;  // z
    // setTaskSpacePathFromPresentPositionOnly(nh, goalPose, path_time);
    // ros::Duration(2.0).sleep();
    // // ros::spinOnce();
    // // ROS_INFO("Pose :\nx: %f, y: %f, z: %f\nroll: %f, pitch: %f, yaw: %f\n", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, current_roll, current_pitch, current_yaw);

    // Perform Ambil Pose State /////////////////////////////////////////////////////////
    path_time = 2.0;
    joint_name.clear();
    joint_angle.clear();
    joint_name.push_back("joint1"); joint_angle.push_back(0.00);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.949534);
    joint_name.push_back("joint3"); joint_angle.push_back(0.018408);
    joint_name.push_back("joint4"); joint_angle.push_back(1.780952);
    setJointSpacePath(joint_name, joint_angle, path_time);
    ros::Duration(2.0).sleep();
    
    return 0;
}
