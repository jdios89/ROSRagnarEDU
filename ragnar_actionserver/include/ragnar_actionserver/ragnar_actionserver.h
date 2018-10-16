#ifndef RAGNAR_ACTIONSERVER_H
#define RAGNAR_ACTIONSERVER_H

#include <vector>
#include <string>

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <ragnar_msgs/FollowCartesianTrajectoryAction.h>
#include <ragnar_msgs/CartesianTrajectory.h>

namespace ragnar_simulator
{

class RagnarSimulator
{
public:
  RagnarSimulator(const std::vector<double>& seed_pose, 
                  const std::vector<std::string>& joint_names,
                  ros::NodeHandle &nh);

  const std::vector<std::string>& getJointNames() const { return joint_names_; }

  // Initializes trajectory, start time, and position fields
  bool setTrajectory(const trajectory_msgs::JointTrajectory& new_trajectory);
  // Computes the robot position at a given time based on the currently active
  // trajectory
  bool computeTrajectoryPosition(const ros::Time& tm, std::vector<double>& output) const;

  void pollAction();

private:
  // Configuration
  std::vector<std::string> joint_names_;

  // Action server
  typedef actionlib::ActionServer<ragnar_msgs::FollowCartesianTrajectoryAction> CartesianTractoryActionServer;

  void goalCB(CartesianTractoryActionServer::GoalHandle & gh);
  void cancelCB(CartesianTractoryActionServer::GoalHandle & gh);

  CartesianTractoryActionServer action_server_;
  CartesianTractoryActionServer::GoalHandle active_goal_;
  bool has_active_goal_;

  // State 
  ragnar_msgs::CartesianTrajectory traj_;
  std::vector<double> traj_start_position_; 
  ros::Time traj_start_time_;
  double _pose[4];
};

}

#endif
