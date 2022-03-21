#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>

#define HEIGHT_FT_ANKLE_SENSOR 0.04

class ZMPController{
  private:
    ros::NodeHandle m_nh;

    ros::Publisher m_zmp_ft_sensors_pub;

    void ftCallback(const geometry_msgs::WrenchStampedConstPtr& left_ft_msg, const geometry_msgs::WrenchStampedConstPtr& right_ft_msg);
  public:
    ZMPController(ros::NodeHandle *nh);

};

ZMPController::ZMPController(ros::NodeHandle *nh){
  m_nh = *nh;
  message_filters::Subscriber<geometry_msgs::WrenchStamped> left_ankle_ft_sub(m_nh, "/left_ankle_ft", 1);
  message_filters::Subscriber<geometry_msgs::WrenchStamped> right_ankle_ft_sub(m_nh, "/right_ankle_ft", 1);

  message_filters::TimeSynchronizer<geometry_msgs::WrenchStamped,geometry_msgs::WrenchStamped> sync_ft(left_ankle_ft_sub, right_ankle_ft_sub, 10);
  sync_ft.registerCallback(boost::bind(&ZMPController::ftCallback, this, _1, _2));
  // ros::spin();
  m_zmp_ft_sensors_pub = m_nh.advertise<geometry_msgs::PointStamped>("/zmp_computed", 10);
  ROS_INFO("Done with status OK!");
      ros::spin();    

    
}


void ZMPController::ftCallback(const geometry_msgs::WrenchStampedConstPtr& left_ft_msg, const geometry_msgs::WrenchStampedConstPtr& right_ft_msg)
{
  double x_zmp_left = ( -left_ft_msg->wrench.torque.y + HEIGHT_FT_ANKLE_SENSOR*left_ft_msg->wrench.force.x)/left_ft_msg->wrench.force.z;
  double x_zmp_right = ( -right_ft_msg->wrench.torque.y + HEIGHT_FT_ANKLE_SENSOR*right_ft_msg->wrench.force.x)/right_ft_msg->wrench.force.z;
  ROS_INFO("X ZMP right: %f , left: %f", x_zmp_right,x_zmp_left);
  double x_zmp = (x_zmp_left*left_ft_msg->wrench.force.z + x_zmp_right*right_ft_msg->wrench.force.z)/(left_ft_msg->wrench.force.z + right_ft_msg->wrench.force.z);
  ROS_INFO("X ZMP: %f", x_zmp);

  double y_zmp_left = (left_ft_msg->wrench.torque.x - HEIGHT_FT_ANKLE_SENSOR*left_ft_msg->wrench.force.y +  0.077*left_ft_msg->wrench.force.z)/left_ft_msg->wrench.force.z;
  double y_zmp_right = (right_ft_msg->wrench.torque.x - HEIGHT_FT_ANKLE_SENSOR*right_ft_msg->wrench.force.y - 0.075*right_ft_msg->wrench.force.z)/right_ft_msg->wrench.force.z;
  ROS_INFO("Y ZMP right: %f , left: %f", y_zmp_right,y_zmp_left);
  double y_zmp = (y_zmp_left*left_ft_msg->wrench.force.z + y_zmp_right*right_ft_msg->wrench.force.z)/(left_ft_msg->wrench.force.z + right_ft_msg->wrench.force.z);
  ROS_INFO("Y ZMP: %f", y_zmp);

  geometry_msgs::PointStamped zmp;
  zmp.header.frame_id = "base_footprint";
  zmp.header.stamp = ros::Time::now();

  zmp.point.x = -x_zmp;
  zmp.point.y = -y_zmp;

  m_zmp_ft_sensors_pub.publish(zmp);

  


}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "zmp_controller_node");
    ros::NodeHandle nh;

    ZMPController zmpController = ZMPController(&nh);


    return 0;
}