#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_test_node");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // // GraspNet output (objek kiri)
    // double translation[3] = {-0.06961557, -0.00540004, 0.207};
    // double rotation_matrix[3][3] = {
    //     { 0.07559296, -0.96571505, -0.24835478},
    //     { 0.28120205,  0.2596043,  -0.9238674 },
    //     { 0.95666665,  0.0,         0.29118532}
    // };
    // GraspNet output (objek kanan)
    double translation[3] = {0.07676842, -0.01203392,  0.209};
    double rotation_matrix[3][3] = {
        {-8.1581585e-02, 9.9666667e-01, -4.3565684e-08},
        {0.0000000e+00 , 4.3711388e-08,  1.0000000e+00},
        { 9.9666667e-01,  8.1581585e-02 ,-3.5660443e-09}
    };
    // // GraspNet output (objek atas)
    // double translation[3] = {0.02310542 ,-0.01957639 , 0.264};
    // double rotation_matrix[3][3] = {
    //     { 2.4000344e-01 ,-1.8454622e-02, -9.7059661e-01},
    //     { -9.6773994e-01,  7.4412398e-02 ,-2.4071190e-01 },
    //     { 7.6666668e-02,  9.9705678e-01, -4.3582737e-08}
    // };
    // // GraspNet output (objek bawah)
    // double translation[3] = {0.07331894 ,0.02160936, 0.191  };
    // double rotation_matrix[3][3] = {
    //     { -9.5967394e-01 , 2.6196003e-01 , 1.0199468e-01},
    //     {9.8394781e-02 ,-2.6858559e-02,  9.9478495e-01},
    //     {2.6333332e-01 , 9.6470493e-01, -4.2168590e-08}
    // };

    // Convert rotation matrix to quaternion
    tf2::Matrix3x3 tf_rotation(
        rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2],
        rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2],
        rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2]
    );
    tf2::Quaternion q;
    tf_rotation.getRotation(q);

    // Create PoseStamped in camera frame
    geometry_msgs::PoseStamped grasp_pose_camera;
    // grasp_pose_camera.header.stamp = ros::Time::now();
    grasp_pose_camera.header.stamp = ros::Time(0);;
    grasp_pose_camera.header.frame_id = "camera_link_visual";

    grasp_pose_camera.pose.position.x = translation[0];
    grasp_pose_camera.pose.position.y = translation[1];
    grasp_pose_camera.pose.position.z = translation[2];

    grasp_pose_camera.pose.orientation.x = q.x();
    grasp_pose_camera.pose.orientation.y = q.y();
    grasp_pose_camera.pose.orientation.z = q.z();
    grasp_pose_camera.pose.orientation.w = q.w();

    // Transform to world frame
    geometry_msgs::PoseStamped grasp_pose_base;

    try {
        tfBuffer.canTransform("world", grasp_pose_camera.header.frame_id,
                              ros::Time(0), ros::Duration(2.0));
        ros::Duration(1.0).sleep();  // Wait for 1 second to ensure TF is ready
        tfBuffer.transform(grasp_pose_camera, grasp_pose_base, "world", ros::Duration(2.0));

        ROS_INFO_STREAM("Transformed pose in world frame:\n" << grasp_pose_base);

        tf2::Quaternion q(
            grasp_pose_base.pose.orientation.x,
            grasp_pose_base.pose.orientation.y,
            grasp_pose_base.pose.orientation.z,
            grasp_pose_base.pose.orientation.w
        );

        // Convert Quaternion to RPY (roll, pitch, yaw)
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        ROS_INFO("roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Transform failed: %s", ex.what());
    }

    return 0;
}
