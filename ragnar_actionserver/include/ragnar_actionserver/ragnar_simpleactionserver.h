#ifndef RAGNAR_SIMPLEACTIONSERVER_H
#define RAGNAR_SIMPLEACTIONSERVER_H

#include <vector>
#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ragnar_msgs/FollowCartesianTrajectoryAction.h>
#include <ragnar_msgs/CartesianTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

namespace ragnar_action
{

class RagnarAction
{
public:
  RagnarAction(const std::vector<double>& seed_pose, 
                  const std::vector<std::string>& joint_names,
                  ros::NodeHandle &nh);

  const std::vector<std::string>& getJointNames() const { return joint_names_; }

  // Initializes trajectory, start time, and position fields
  bool setTrajectory(const ragnar_msgs::CartesianTrajectory& new_trajectory);
  bool updatePose(const geometry_msgs::PoseStamped& pose);
  // Computes the robot position at a given time based on the currently active
  // trajectory
  bool computeTrajectoryPosition(const ros::Time& tm, std::vector<double>& output) const;

  void pollAction();
  void poseStateCB();
  void sendFeedback();
  std::vector<double> mob_command; 

private:
  // Configuration
  std::vector<std::string> joint_names_;
  
  // Action server
  typedef actionlib::SimpleActionServer<ragnar_msgs::FollowCartesianTrajectoryAction> CartesianTractoryActionServer;

  void executeCB(const ragnar_msgs::FollowCartesianTrajectoryGoalConstPtr & gh);
  void cancelCB(CartesianTractoryActionServer::GoalHandle & gh);
  ragnar_msgs::FollowCartesianTrajectoryFeedback action_feedback_;
  ragnar_msgs::FollowCartesianTrajectoryResult action_result_;

  CartesianTractoryActionServer action_server_;
  ragnar_msgs::FollowCartesianTrajectoryGoalConstPtr active_goal_;
  bool has_active_goal_;

  // State 
  ragnar_msgs::CartesianTrajectory traj_;
  ragnar_msgs::CartesianTrajectoryPoint target_pt_;
  std::vector<double> traj_start_position_; 
  std::vector<double> traj_start_position_cartesian_;
  ros::Time traj_start_time_;
  double _pose[4];
  geometry_msgs::Pose _geopose; 
};

}

#endif
