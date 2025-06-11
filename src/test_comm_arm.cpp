#include <ros/ros.h>
#include <std_msgs/Bool.h>

bool toggle_data;
bool active;

void callback(const std_msgs::Bool::ConstPtr& msg) {

    toggle_data = msg->data;
    active = toggle_data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_comm_arm_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/toggle_node", 1);
    ros::Subscriber sub = nh.subscribe("/toggle_node", 1, callback);

    std_msgs::Bool msg;
    active = false;

    ros::Rate rate(10);
    while (ros::ok()) {

        ROS_INFO("wait for the quadruped finish the navigation_1 job");
        while(ros::ok() && (active==false)){
            ROS_INFO("...");
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("quadruped has been finished the navigation_1 job\n");

        ROS_INFO("arm grasp the object");
        ros::Duration(4.0).sleep();
        active = false;
        msg.data = false; pub.publish(msg);
        ROS_INFO("handoff to quadruped\n");
        
        ROS_INFO("wait for the quadruped finish the navigation_2 job");
        while(ros::ok() && (active==false)){
            ROS_INFO("...");
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("quadruped has been finished the navigation_2 job\n");

        ROS_INFO("arm place the object");
        ros::Duration(4.0).sleep();
        active = false;
        msg.data = false; pub.publish(msg);
        ROS_INFO("handoff to quadruped\n");
        
    }

    return 0;
}
