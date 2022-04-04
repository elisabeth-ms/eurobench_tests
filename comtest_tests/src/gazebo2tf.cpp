#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <gazebo_msgs/LinkStates.h>
#include "std_msgs/String.h"
#include <string.h>
#include <geometry_msgs/PoseArray.h>

#define BODY_LENGTH_M 0.5

void linkStatesCallback(const gazebo_msgs::LinkStatesConstPtr &msg){
    static tf::TransformBroadcaster br;


    for (int i=0;i<msg->name.size();i++)
    {
  		if (msg->name[i].find("reemc_full_ft_hey5::base_link")!=std::string::npos)
        	{
		        tf::Transform transform;

                    	transform.setOrigin( tf::Vector3(msg->pose[i].position.x, msg->pose[i].position.y,msg->pose[i].position.z));
            		tf::Quaternion q(msg->pose[i].orientation.x, msg->pose[i].orientation.y, msg->pose[i].orientation.z,msg->pose[i].orientation.w);
  	    		transform.setRotation(q);
                    	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
		    	break;
		   
		}
    }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "gazebo2tf");


  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("gazebo/link_states", 1, &linkStatesCallback);

  ros::spin();
  return 0;
};