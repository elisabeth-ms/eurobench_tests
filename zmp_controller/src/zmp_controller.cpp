#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/Point.h"
#include <stdio.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

#define HEIGHT_FT_ANKLE_SENSOR 0.041
#define G 9.80665
#define RATE 50.0



class ZMPController{
  private:
    ros::NodeHandle m_nh;
  
    ros::Publisher m_zmp_ft_sensors_pub;
    ros::Publisher m_zmp_ref_pub;
    
    ros::Publisher m_zmp_left_ft_pub;
    ros::Publisher m_zmp_right_ft_pub;

    TrajClient* m_traj_client_left_leg;
    TrajClient* m_traj_client_right_leg;
    control_msgs::FollowJointTrajectoryGoal m_goal_left_leg;
    control_msgs::FollowJointTrajectoryGoal m_goal_right_leg;



    std::string m_pathCsvFile;
    FILE * m_pFile;
    ros::Time  m_time   = ros::Time::now();
    ros::Time m_prev_time = ros::Time::now();
    ros::Time m_init_time;
    geometry_msgs::WrenchStamped m_left_ankle_ft;
    geometry_msgs::WrenchStamped m_right_ankle_ft;
    geometry_msgs::Point m_zmp_left;
    geometry_msgs::Point m_zmp_right;
    geometry_msgs::Point m_zmp;
    double m_prev_ankle_angle;
    bool m_first;






    tf2_ros::Buffer * m_tfBuffer = new tf2_ros::Buffer();
    tf2_ros::TransformListener * m_tfListener= new tf2_ros::TransformListener(*m_tfBuffer);
  
    double m_zmp_ref_x;
    uint m_step;
    double m_ang_ref;
    double m_pendulum_longitude;
    double m_sum_x_ft;
    double m_offs_x_ft;

    bool m_command_ankles = true;

    void addToCsvFile();
    void configCsvFile(std::string pathCsvFile);
    void ftCallback(const geometry_msgs::WrenchStampedConstPtr& left_ft_msg, const geometry_msgs::WrenchStampedConstPtr& right_ft_msg);
    void getPendulumLongitude();
    void computeAngRef();
    double computeZmpRef(uint start_slope_step, uint end_slope_step, double ref_zmp_end);
    void commandAnkleAngle();
    void FillJointNames(control_msgs::FollowJointTrajectoryGoal &goal, bool right);
    void FillGoalMessage(control_msgs::FollowJointTrajectoryGoal & goal, double ankleAngle, bool right);

  

  public:
    ZMPController(ros::NodeHandle *nh, std::string pathCsvFile);
    ZMPController(const ZMPController& zmpController){};
};

ZMPController::ZMPController(ros::NodeHandle *nh, std::string pathCsvFile){
  m_nh = *nh;
  m_pathCsvFile = pathCsvFile;
  m_prev_ankle_angle = 0.0;
  configCsvFile(m_pathCsvFile);
  m_first = true;
  message_filters::Subscriber<geometry_msgs::WrenchStamped> left_ankle_ft_sub(m_nh, "/left_ankle_ft", 1);
  message_filters::Subscriber<geometry_msgs::WrenchStamped> right_ankle_ft_sub(m_nh, "/right_ankle_ft", 1);
  // m_left_leg_pub = n.advertise<trajectory_msgs::JointTrajectory>("/left_leg_controller/command", 1);
  // m_right_leg_pub = n.advertise<trajectory_msgs::JointTrajectory>("/right_leg_controller/command", 1);
  m_traj_client_left_leg = new TrajClient("/left_leg_controller/follow_joint_trajectory", true);
  m_traj_client_right_leg = new TrajClient("/right_leg_controller/follow_joint_trajectory", true);


  while(!m_traj_client_left_leg->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the /left_leg_controller/follow_joint_trajectory action server");
  }

  while(!m_traj_client_right_leg->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the /right_leg_controller/follow_joint_trajectory action server");
  }

  FillJointNames(m_goal_left_leg, false);
  FillJointNames(m_goal_right_leg, true);


  message_filters::TimeSynchronizer<geometry_msgs::WrenchStamped,geometry_msgs::WrenchStamped> sync_ft(left_ankle_ft_sub, right_ankle_ft_sub, 10);
  sync_ft.registerCallback(boost::bind(&ZMPController::ftCallback, this, _1, _2));
  // ros::spin();
  m_zmp_ft_sensors_pub = m_nh.advertise<geometry_msgs::PointStamped>("/zmp_ft_sensors", 10);
  m_zmp_ref_pub = m_nh.advertise<geometry_msgs::PointStamped>("/zmp_ref", 10);
  m_zmp_left_ft_pub =  m_nh.advertise<geometry_msgs::PointStamped>("/zmp_left_ft_sensors", 10);
  m_zmp_right_ft_pub =  m_nh.advertise<geometry_msgs::PointStamped>("/zmp_right_ft_sensors", 10);

  if(m_nh.hasParam("/zmp_controller_node/command_ankles"))
  {
    ros::param::get("/zmp_controller_node/command_ankles", m_command_ankles);
    ROS_INFO("/command_ankles set to %d", m_command_ankles);

  }
  else{
    ros::shutdown();
  }
  ROS_INFO("Done with status OK!");
  m_time   = ros::Time::now();
  getPendulumLongitude();
  m_step = 0.0;
  m_prev_time = ros::Time::now();
  ros::spin();    


  // update();
    
}




void ZMPController::configCsvFile(std::string pathCsvFile){
  ROS_INFO("configuring csv file");
  m_pFile = fopen(pathCsvFile.c_str(),"w+");
  fprintf(m_pFile,"Time,Fx_ft0,Fz_ft0,My_ft0,Fx_ft1,Fz_ft1,My_ft1,Xzmp_ft,zmp_RF,zmp_LF,zmp_ref,iter");
}

void ZMPController::addToCsvFile(){
    fprintf(m_pFile,"\n%.2f", (m_time-m_init_time).toSec());

    fprintf(m_pFile,",%.10f", m_left_ankle_ft.wrench.force.x); // f_x - sensor ft left_ankle
    fprintf(m_pFile,",%.10f", m_left_ankle_ft.wrench.force.z); // f_z - sensor ft left ankle
    fprintf(m_pFile,",%.10f",  m_left_ankle_ft.wrench.torque.y); // m_y - sensor ft left ankle

    fprintf(m_pFile,",%.10f", m_right_ankle_ft.wrench.force.x); // f_x - sensor ft right ankle
    fprintf(m_pFile,",%.10f", m_right_ankle_ft.wrench.force.z); // f_z - sensor ft right ankle
    fprintf(m_pFile,",%.10f", m_right_ankle_ft.wrench.torque.y); // m_y - sensor ft right ankle

    fprintf(m_pFile,",%.8f", m_zmp.x); // ZMP body (double support) (frontal plane)
    fprintf(m_pFile,",%.8f", m_zmp_left.x); // zmp (left foot)
    fprintf(m_pFile,",%.8f", m_zmp_right.x); // zmp (right foot)
    fprintf(m_pFile,",%.8f", m_zmp_ref_x);
    fprintf(m_pFile,",%i", m_step);
    

}



void ZMPController::ftCallback(const geometry_msgs::WrenchStampedConstPtr& left_ft_msg, const geometry_msgs::WrenchStampedConstPtr& right_ft_msg)
{
  m_time   = ros::Time::now();
  if(m_time-m_prev_time>=ros::Duration(1/RATE)){
    if(m_first){
      m_init_time = m_time;
      m_first = false;
    }
    // if(m_time-m_prev_time >= )
    m_left_ankle_ft = (*left_ft_msg);
    m_right_ankle_ft = (*right_ft_msg);
    m_zmp_left.x = ( -left_ft_msg->wrench.torque.y - HEIGHT_FT_ANKLE_SENSOR*(left_ft_msg->wrench.force.x))/(left_ft_msg->wrench.force.z);
    m_zmp_right.x = ( -right_ft_msg->wrench.torque.y - HEIGHT_FT_ANKLE_SENSOR*(right_ft_msg->wrench.force.x))/(right_ft_msg->wrench.force.z);
    ROS_INFO("X ZMP right: %f , left: %f", m_zmp_right.x,m_zmp_left.x);
    m_zmp.x = (m_zmp_left.x*left_ft_msg->wrench.force.z + m_zmp_right.x*right_ft_msg->wrench.force.z)/(left_ft_msg->wrench.force.z + right_ft_msg->wrench.force.z);
    ROS_INFO("X ZMP: %f", m_zmp.x);

    m_zmp_left.y  = (-left_ft_msg->wrench.torque.x - HEIGHT_FT_ANKLE_SENSOR*left_ft_msg->wrench.force.y /*+  0.077*left_ft_msg->wrench.force.z*/)/left_ft_msg->wrench.force.z;
    m_zmp_right.y = (-right_ft_msg->wrench.torque.x - HEIGHT_FT_ANKLE_SENSOR*right_ft_msg->wrench.force.y/* - 0.075*right_ft_msg->wrench.force.z*/)/right_ft_msg->wrench.force.z;
    ROS_INFO("Y ZMP right: %f , left: %f",  m_zmp_right.y , m_zmp_left.y);
    m_zmp.y = ( m_zmp_left.y*left_ft_msg->wrench.force.z +  m_zmp_right.y *right_ft_msg->wrench.force.z)/(left_ft_msg->wrench.force.z + right_ft_msg->wrench.force.z);
    ROS_INFO("Y ZMP: %f", m_zmp.y);

      // OFFSET FT - eliminando el offset de (ZMP de ambos sensores en X)
    if (m_step >=1 && m_step < 100){
        m_sum_x_ft = m_zmp.x + m_sum_x_ft;
        m_offs_x_ft = m_sum_x_ft / m_step;
        printf("offs = %f\n", m_offs_x_ft);
    }
    m_zmp.x = m_zmp.x-m_offs_x_ft;


    geometry_msgs::PointStamped zmp, zmp_ref, zmp_left_ft, zmp_right_ft;
    zmp.header.frame_id = "base_link";
    zmp.header.stamp = ros::Time::now();

    zmp_ref = zmp;
    zmp_left_ft = zmp;
    zmp_right_ft = zmp;

    // zmp.point.x = x_zmp;
    // zmp.point.y = -y_zmp;

    
    ROS_INFO("Time between steps: %f", (m_time-m_prev_time).toSec());
    
    if(m_command_ankles){
      m_zmp_ref_x = computeZmpRef(300, 305, 0.05);


      computeAngRef();

      commandAnkleAngle();
      zmp_ref.point.x = m_zmp_ref_x;
      zmp_ref.point.z = -m_pendulum_longitude;
      m_zmp_ref_pub.publish(zmp_ref);
    }

    zmp.point = m_zmp;
    zmp.point.z = -m_pendulum_longitude;

    zmp_left_ft.point = m_zmp_left;
    zmp_left_ft.point.z = -m_pendulum_longitude;
    zmp_right_ft.point = m_zmp_right;
    zmp_right_ft.point.z = -m_pendulum_longitude;
    

    m_zmp_ft_sensors_pub.publish(zmp);
    m_zmp_left_ft_pub.publish(zmp_left_ft);
    m_zmp_right_ft_pub.publish(zmp_right_ft);
    m_prev_time = m_time;
    m_step++;
    addToCsvFile();

    if(m_command_ankles){
      if(m_step >1500){
        fclose(m_pFile);
        fclose(m_pFile);
        ros::shutdown();
      }
    }
    if(!m_nh.ok()){
      fclose(m_pFile);
    }


  }

}
void ZMPController::getPendulumLongitude(){
  
 geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = m_tfBuffer->lookupTransform("right_sole_link", "base_link", ros::Time(0), ros::Duration(10.0));
  }
  catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
  }

  m_pendulum_longitude = transformStamped.transform.translation.z;
  ROS_INFO("Pendulum longitude: %f", m_pendulum_longitude);

}

void ZMPController::FillJointNames(control_msgs::FollowJointTrajectoryGoal &goal, bool right){
  goal.trajectory.joint_names.resize(6);
    for(uint i=1; i<=6;i++){
      if(right){
        goal.trajectory.joint_names[i-1] ="leg_right_"+std::to_string(i)+"_joint";
      }
      else{
        goal.trajectory.joint_names[i-1] ="leg_left_"+std::to_string(i)+"_joint";
      }
  }
}

void ZMPController::FillGoalMessage(control_msgs::FollowJointTrajectoryGoal &goal, double ankleAngle, bool right){
  goal.trajectory.header.stamp = ros::Time::now();

  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(6);
  goal.trajectory.points[0].positions[0] = 0.0;
  goal.trajectory.points[0].positions[1] = 0.0;
  goal.trajectory.points[0].positions[2] = 0.0;
  goal.trajectory.points[0].positions[3] = 0.0;
  goal.trajectory.points[0].positions[4] = ankleAngle;
  goal.trajectory.points[0].positions[5] = 0.0;
  // Velocities
  goal.trajectory.points[0].velocities.resize(6);
  for (size_t j = 0; j < 6; ++j)
  {
      goal.trajectory.points[0].velocities[j] = (ankleAngle-m_prev_ankle_angle)/(1/RATE);
  }
  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration(1/RATE);
  
}

void ZMPController::commandAnkleAngle(){


  FillGoalMessage(m_goal_left_leg, m_ang_ref, false);
  FillGoalMessage(m_goal_right_leg, m_ang_ref, true);

  m_traj_client_left_leg->sendGoal(m_goal_left_leg);
  m_traj_client_right_leg->sendGoal(m_goal_right_leg);
  


}

double ZMPController::computeZmpRef(uint start_slope_step, uint end_slope_step, double ref_zmp_end){
  if(m_step<start_slope_step){
    return 0.0;
  }
  else if(m_step>=start_slope_step && m_step<=end_slope_step){
    ROS_INFO("steps in slope: %d", m_step-start_slope_step);
    return ref_zmp_end/(end_slope_step-start_slope_step)*(m_step-start_slope_step);
  }
  else{
    return ref_zmp_end;
  }

}


void ZMPController::computeAngRef(){
  // double ka = 0.25 * m_zmp_ref_x + 9.95; 
  // ka = 0.0;
  // m_ang_ref = (m_zmp_ref_x*(-G))/ (m_pendulum_longitude*(ka-G));
  m_prev_ankle_angle = m_ang_ref;
  m_ang_ref = -asin(m_zmp_ref_x/m_pendulum_longitude);
  ROS_INFO("step: %d zmp_ref: %f angle_ref: %f",m_step, m_zmp_ref_x, m_ang_ref);

  
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "zmp_controller_node");
    ros::NodeHandle nh;
    std::string nameCsvFile = "/home/user/catkin_ws/test1.csv";
    // -----------------------------------------------------------------------------------------------------------------------------------
    // TODO: ADD A VARIABLE TO HAVE TWO MODES ONE THAT GATHERS THE ZMP DIRECTLY AND ONE THAT CONTROLS THE ANKLE JOINTS AND COMPUTES THE ZMP

    //  geometry_msgs::TransformStamped transformStamped;
    // try{
    //   transformStamped = tfBuffer.lookupTransform("right_sole_link", "", ros::Time(0),  ros::Duration(1.0));
    // }
    // catch (tf2::TransformException &ex) {
    //     ROS_WARN("%s",ex.what());
    //     ros::Duration(1.0).sleep();
    // }

    // double l = transformStamped.transform.translation.z;
    // ROS_INFO("Pendulum longitude: %f", l);


    ZMPController zmpController = ZMPController(&nh, nameCsvFile);


    return 0;
}