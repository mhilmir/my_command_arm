#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
bool grasp_pose_received = false;
double grasp_score, grasp_width, grasp_height, grasp_depth;
geometry_msgs::Pose grasp_pose;

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

void graspScoreCallback(const std_msgs::Float32::ConstPtr &msg)
{
    grasp_score = msg->data;
    grasp_pose_received = true;
}

void graspWidthCallback(const std_msgs::Float32::ConstPtr &msg)
{
    grasp_width = msg->data;
}

void graspHeightCallback(const std_msgs::Float32::ConstPtr &msg)
{
    grasp_height = msg->data;
}

void graspDepthCallback(const std_msgs::Float32::ConstPtr &msg)
{
    grasp_depth = msg->data;
}

void graspPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    grasp_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_manipulation_node");
    ros::NodeHandle nh;

    ros::Rate rate(10);
    present_joint_angle_.resize(NUM_OF_JOINT);

    goal_joint_space_path_client_ = nh.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
    goal_tool_control_client_ = nh.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
    goal_task_space_path_from_present_client_ = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present");
    goal_task_space_path_from_present_position_only_client_ = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");

    ros::Subscriber kinematics_pose_sub_ = nh.subscribe("/gripper/kinematics_pose", 10, kinematicsPoseCallback);;
    ros::Subscriber joint_states_sub_ = nh.subscribe("joint_states", 10, jointStatesCallback);
    ros::Subscriber grasp_score_sub_ = nh.subscribe("/grasp_result/score", 10, graspScoreCallback);
    ros::Subscriber grasp_width_sub_ = nh.subscribe("/grasp_result/width", 10, graspWidthCallback);
    ros::Subscriber grasp_height_sub_ = nh.subscribe("/grasp_result/height", 10, graspHeightCallback);
    ros::Subscriber grasp_depth_sub_ = nh.subscribe("/grasp_result/depth", 10, graspDepthCallback);
    ros::Subscriber grasp_pose_sub_ = nh.subscribe("/grasp_result/pose", 10, graspPoseCallback);

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    std::vector<double> goalPose;  goalPose.resize(3, 0.0);
    std::vector<double> gripper_joint;
    double delta = 0.01;
    double path_time;
    geometry_msgs::Pose target_pose;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

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

    // Find Object Pose //////////////////////////////////////////////////////////////////
    // Joint States:
    //     joint1: 1.497165, joint2: -0.948000, joint3: 0.039884, joint4: 1.784020
    // Position :
    //     x: 0.016929, y: 0.066817, z: 0.171629
    // Orientation Quaternion :
    //     x: -0.288633, y: 0.310708, z: 0.616367, w: 0.663508
    // Orientation Euler :
    //     roll: 0.000000, pitch: 0.875903, yaw: 1.497165
    // Orientation Matrix Rotation :
    //     [0.0471036, -0.99729, 0.0565065]
    //     [0.638569, 0.0735645, 0.766041]
    //     [-0.768122, 0, 0.640303]
    geometry_msgs::Point find_object_position;
    find_object_position.x = 0.016929;
    find_object_position.y = 0.066817;
    find_object_position.z = 0.171629;
    double find_object_roll = 0.000000;
    double find_object_pitch = 0.875903;
    double find_object_yaw = 1.497165;
    // Perform Find Object Pose /////////////////////////////////////////////////////////
    path_time = 3.0;
    goalPose.clear();  goalPose.resize(6, 0.0);
    goalPose.at(0) = find_object_position.x - current_pose.pose.position.x;  // x
    goalPose.at(1) = find_object_position.y - current_pose.pose.position.y;  // y
    goalPose.at(2) = find_object_position.z - current_pose.pose.position.z;  // z
    goalPose.at(3) = find_object_roll - current_roll;  // roll
    goalPose.at(4) = find_object_pitch - current_pitch;  // pitch
    goalPose.at(5) = find_object_yaw - current_yaw;  // yaw
    if(setTaskSpacePathFromPresent(nh, goalPose, path_time)){
        ROS_INFO("Succeed to plan to Find Object Pose");
    } else{
        ROS_ERROR("Failed when planning to Find Object Pose");
        exit(1);  // Plan Failed
    }
    ros::Duration(2.0).sleep();
    ros::spinOnce();
    ROS_INFO("Pose :\nx: %f, y: %f, z: %f\nroll: %f, pitch: %f, yaw: %f\n", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, current_roll, current_pitch, current_yaw);
    ///////////////////////////////////////////////////////////////////////////////

    // Kontrol Gripper Open ///////////////////////////////////////////////////////
    gripper_joint.clear();
    gripper_joint.push_back(0.01);  // open
    setToolControl(nh, gripper_joint);
    ros::Duration(2.0).sleep();
    // ///////////////////////////////////////////////////////////////////////////

    ROS_INFO("Waiting for grasp pose");
    while(ros::ok()){
        ros::spinOnce();
        if(!grasp_pose_received){
            rate.sleep();
            continue;
        }
        ROS_INFO("grasp pose received");
        break;
    }
    ROS_INFO("grasp_score: %f\n", grasp_score);
    ROS_INFO("grasp_width: %f\n", grasp_width);
    ROS_INFO("grasp_height: %f\n", grasp_height);
    ROS_INFO("grasp_depth: %f\n", grasp_depth);
    ROS_INFO("grasp_position: x: %f  y: %f  z: %f\n", grasp_pose.position.x,grasp_pose.position.y, grasp_pose.position.z);
    ROS_INFO("grasp_orientation: x: %f  y: %f  z: %f  w: %f\n", grasp_pose.orientation.x,grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w);

    // Create PoseStamped in camera frame
    geometry_msgs::PoseStamped grasp_pose_camera;
    grasp_pose_camera.header.stamp = ros::Time(0);;
    grasp_pose_camera.header.frame_id = "camera_link_visual";
    grasp_pose_camera.pose.position.x = grasp_pose.position.x;
    grasp_pose_camera.pose.position.y = grasp_pose.position.y;
    grasp_pose_camera.pose.position.z = grasp_pose.position.z;
    grasp_pose_camera.pose.orientation.x = grasp_pose.orientation.x;
    grasp_pose_camera.pose.orientation.y = grasp_pose.orientation.y;
    grasp_pose_camera.pose.orientation.z = grasp_pose.orientation.z;
    grasp_pose_camera.pose.orientation.w = grasp_pose.orientation.w;
    // Transform to world frame
    geometry_msgs::PoseStamped grasp_pose_base;
    try {
        tfBuffer.canTransform("world", grasp_pose_camera.header.frame_id, ros::Time(0), ros::Duration(2.0));
        ros::Duration(1.0).sleep();  // Wait for 1 second to ensure TF is ready
        tfBuffer.transform(grasp_pose_camera, grasp_pose_base, "world", ros::Duration(2.0));
        ROS_INFO_STREAM("Transformed pose in world frame:\n" << grasp_pose_base);
        tf2::Quaternion q(
            grasp_pose_base.pose.orientation.x,
            grasp_pose_base.pose.orientation.y,
            grasp_pose_base.pose.orientation.z,
            grasp_pose_base.pose.orientation.w
        );
        // Convert Quaternion to RPY
        double grasp_roll_base, grasp_pitch_base, grasp_yaw_base;
        tf2::Matrix3x3(q).getRPY(grasp_roll_base, grasp_pitch_base, grasp_yaw_base);
        ROS_INFO("grasp_roll_base: %f, grasp_pitch_base: %f, grasp_yaw_base: %f", grasp_roll_base, grasp_pitch_base, grasp_yaw_base);
    }
    catch (tf2::TransformException &ex) {
        ROS_ERROR("Transform failed: %s", ex.what());
        exit(2);  // transform failed
    }
    
    // // Get the Error between current and target
    double x_error = grasp_pose_base.pose.position.x - current_pose.pose.position.x;
    double y_error = grasp_pose_base.pose.position.y - current_pose.pose.position.y;
    double z_error = grasp_pose_base.pose.position.z - current_pose.pose.position.z;
    
    // Kontrol Pose from present (position only) /////////////////////////////
    path_time = 4.0;
    goalPose.clear();  goalPose.resize(3, 0.0);
    goalPose.at(0) = x_error;  // x
    goalPose.at(1) = y_error;  // y
    goalPose.at(2) = z_error;  // z
    setTaskSpacePathFromPresentPositionOnly(nh, goalPose, path_time);
    ros::Duration(6.0).sleep();
    ros::spinOnce();
    ROS_INFO("Pose :\nx: %f, y: %f, z: %f\nroll: %f, pitch: %f, yaw: %f\n", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, current_roll, current_pitch, current_yaw);

    // Kontrol Gripper Tutup ///////////////////////////////////////////////////////
    gripper_joint.clear();
    gripper_joint.push_back(-0.01);  // tutup
    setToolControl(nh, gripper_joint);
    ros::Duration(2.0).sleep();
    // ///////////////////////////////////////////////////////////////////////////

    // Back to Init Pose
    path_time = 3.0;
    goalPose.clear();  goalPose.resize(6, 0.0);
    goalPose.at(0) = find_object_position.x - current_pose.pose.position.x;  // x
    goalPose.at(1) = find_object_position.y - current_pose.pose.position.y;  // y
    goalPose.at(2) = find_object_position.z - current_pose.pose.position.z;  // z
    goalPose.at(3) = find_object_roll - current_roll;  // roll
    goalPose.at(4) = find_object_pitch - current_pitch;  // pitch
    goalPose.at(5) = find_object_yaw - current_yaw;  // yaw
    if(setTaskSpacePathFromPresent(nh, goalPose, path_time)){
        ROS_INFO("Succeed to plan to Find Object Pose");
    } else{
        ROS_ERROR("Failed when planning to Find Object Pose");
        exit(1);  // Plan Failed
    }
    ros::Duration(2.0).sleep();
    ros::spinOnce();
    ROS_INFO("Pose :\nx: %f, y: %f, z: %f\nroll: %f, pitch: %f, yaw: %f\n", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, current_roll, current_pitch, current_yaw);

    return 0;
}
