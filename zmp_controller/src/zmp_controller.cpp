#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

void jointsStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO("I heard 2: [%s]", msg->name[0].c_str());
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "autonomy_node");
    ros::NodeHandle nh;


    ros::Subscriber sub = nh.subscribe("/joint_states", 1000, jointsStateCallback);
    ros::spin();
    ROS_INFO("Done with status OK!");
    
    return 0;
}