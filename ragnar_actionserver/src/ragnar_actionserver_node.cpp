#include <ros/ros.h>
#include <sensor_msgs/JointState.h> // for publishing robot current position
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "ragnar_kinematics/ragnar_kinematics.h"
#include "ragnar_kinematics/ragnar_kinematic_defs.h"

#include "ragnar_actionserver/ragnar_actionserver.h"


bool extractJoints(const sensor_msgs::JointState& msg, double* actuators) 
{  
  // Otherwise copy values, and continue on
  for (int i = 0; i < 4; i++)
  {
    actuators[i] = msg.position[i];
  }
  return true;
}

void publishCurrentCommand(const ros::TimerEvent& timer,
                         ros::Publisher& pub,
                         ragnar_action::RagnarAction& action_ragnar,
                         ros::Publisher& mobile_pub)
{
  action_ragnar.sendFeedback(); 
  std::vector<double> mobile_command;
  action_ragnar.computeTrajectoryPosition(timer.current_real, mobile_command);
  geometry_msgs::Pose posecommand; 
  posecommand.position.x = mobile_command[0];
  posecommand.position.y = mobile_command[1];
  posecommand.position.z = mobile_command[2]; 
  // This will see if the action is success or failure 
  action_ragnar.pollAction();
  action_ragnar.sendFeedback();
  char str[100];
  sprintf(str,"Position: X%f Y%f Z%f", mobile_command[0], mobile_command[1], mobile_command[2]);
  //ROS_INFO(str);
  pub.publish(posecommand);
  //mobile_pub.publish(mobile_pose);
}


void updatPose(const geometry_msgs::PoseStamped::ConstPtr& pose,
                          ragnar_action::RagnarAction& action_ragnar)
{
  ROS_INFO("updating pose");
  action_ragnar.updatePose(*pose);
}

int main(int argc, char** argv)
{
  //const static double default_position[] = {-0.07979196, 0.07044869, 
  //                                          -0.07044869, 0.07979196};
  const static double default_position[] = {-0.018251, 0.005695774, 
                                            -0.005695774, 0.0182518};


  ros::init(argc, argv, "ragnar_action_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh ("~");

  // nh loads joint names if possible
  std::vector<std::string> joint_names;
  if (!nh.getParam("controller_joint_names", joint_names))
  {
    // otherwise, it loads defaults
    joint_names.push_back("joint_1");
    joint_names.push_back("joint_2");
    joint_names.push_back("joint_3");
    joint_names.push_back("joint_4");
  }
  // pnh loads configuration parameters
  std::vector<double> seed_position;
  if (!pnh.getParam("initial_position", seed_position))
  {
    seed_position.assign(default_position, default_position + 4);
  }

  double publish_rate;
  pnh.param<double>("rate", publish_rate, 30.0);
  
  // instantiate simulation
  ragnar_action::RagnarAction action_ragnar (seed_position, joint_names, nh);

  // create pub/subscribers and wire them up
  ros::Publisher current_command_pub = nh.advertise<geometry_msgs::Pose>("mobile_platform_command", 1);
  ros::Publisher mobile_base_pub = nh.advertise<geometry_msgs::PoseStamped>("mobile_platform", 1);
  ros::Subscriber pose_state_sub = 
      nh.subscribe<geometry_msgs::PoseStamped>("ragnar_pose", 
                                                     1, 
                                                     boost::bind(updatPose, 
                                                                 _1, 
                                                                 boost::ref(action_ragnar)));
  ros::Timer state_publish_timer =
      nh.createTimer(ros::Duration(1.0/publish_rate), boost::bind(publishCurrentCommand,
                                                     _1,
                                                     boost::ref(current_command_pub),
                                                     boost::ref(action_ragnar),
                                                     boost::ref(mobile_base_pub)));

  ROS_INFO("Ragnar Action service spinning");
  ros::spin();
  return 0;
}
