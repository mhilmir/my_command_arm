#include <ros/ros.h>
#include <std_msgs/Bool.h>

// bool active = false;
// bool sent = false;
// ros::Time start_time;
bool toggle_data;

void callback(const std_msgs::Bool::ConstPtr& msg) {
    // if (msg->data) {
    //     ROS_INFO("Node B: Starting operation.");
    //     active = true;
    //     sent = false;
    //     start_time = ros::Time::now();
    // }
    toggle_data = msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_comm_arm_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/toggle_node", 1);
    ros::Subscriber sub = nh.subscribe("/toggle_node", 1, callback);

    // ROS_INFO("Node B started and waiting for activation.");

    std_msgs::Bool msg; 

    ros::Rate rate(10);
    while (ros::ok()) {
        // if (active) {
        //     ROS_INFO_THROTTLE(1, "Node B is running...");
        //     if (!sent && (ros::Time::now() - start_time).toSec() > 5.0) {
        //         std_msgs::Bool msg;
        //         msg.data = false;
        //         pub.publish(msg);
        //         ROS_INFO("Node B: Handoff back to Node A.");
        //         active = false;
        //         sent = true;
        //     }
        // }
        // ros::spinOnce();
        // rate.sleep();

        ROS_INFO("wait for the quadruped finish the navigation_1 job");
        while(ros::ok() && !(toggle_data==true)){
            ROS_INFO("...");
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("quadruped has been finished the navigation_1 job\n");

        ROS_INFO("arm grasp the object");
        ros::Duration(3.0).sleep();
        msg.data = false; pub.publish(msg);
        ROS_INFO("handoff to quadruped\n");
        
        ROS_INFO("wait for the quadruped finish the navigation_2 job");
        while(ros::ok() && !(toggle_data==true)){
            ROS_INFO("...");
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("quadruped has been finished the navigation_2 job\n");

        ROS_INFO("arm place the object");
        ros::Duration(3.0).sleep();
        msg.data = false; pub.publish(msg);
        ROS_INFO("handoff to quadruped\n");
        
    }

    return 0;
}
