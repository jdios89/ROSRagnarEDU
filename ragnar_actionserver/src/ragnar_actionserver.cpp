#include "ragnar_actionserver/ragnar_actionserver.h"
#include <ros/console.h>
#include <ros/assert.h>
#include "ragnar_kinematics/ragnar_kinematics.h"
#include "ragnar_kinematics/ragnar_kinematic_defs.h"


ragnar_action::RagnarAction::RagnarAction(const std::vector<double>& seed_pose, 
                                                   const std::vector<std::string>& joint_names,
                                                   ros::NodeHandle& nh)
  : joint_names_(joint_names)
  , traj_start_position_(seed_pose)
  , traj_start_time_(ros::Time::now())
  , action_server_(nh, "cartesian_trajectory_action", boost::bind(&RagnarAction::goalCB, this, _1),
                   boost::bind(&RagnarAction::cancelCB, this, _1), false)
  , has_active_goal_(false)
{
  ROS_ASSERT(seed_pose.size() == 4);
  ROS_ASSERT(joint_names.size() == 4);
  double actuators[4];
  for (int i=0; i<4; i++) actuators[i] = seed_pose[i];
  // using joint states, calculate forward kinematics of ragnar
  if (!ragnar_kinematics::forward_kinematics(actuators, _pose))
  {
    ROS_WARN("Could not calculate FK for given pose");
    return;
  }
  action_server_.start();
}

bool ragnar_action::RagnarAction::setTrajectory(const ragnar_msgs::CartesianTrajectory& new_trajectory)
{
  ROS_INFO("Setting new active trajectory");
  // Compute current state
  ros::Time now = ros::Time::now();
  std::vector<double> position;
  computeTrajectoryPosition(now, position);

  // Rollover to the new trajectory
  traj_start_position_ = position;
  traj_ = new_trajectory;
  traj_start_time_ = now;

  return true;
}

static double linearInterpolate(double start, double stop, double ratio)
{
  return start + (stop - start) * ratio;
}

  // Computes the robot position at a given time based on the currently active
  // trajectory
bool ragnar_action::RagnarAction::computeTrajectoryPosition(const ros::Time& tm, 
                                                                  std::vector<double>& output) const
{
  // Check to see if time is in past of traj
  if (tm < traj_start_time_ || traj_.points.empty())
  {
    output = traj_start_position_;
    return true;
  }
  // check to see if time is past end of traj
  else if (tm > traj_start_time_ + traj_.points.back().time_from_start)
  {
    output = traj_.points.back().positions;
    return true;
  }
  
  // Otherwise the traj must be within the trajectory
  ros::Duration dt = tm - traj_start_time_;

  size_t idx = 0;
  for (size_t i = 0; i < traj_.points.size(); ++i)
  {
    if (dt < traj_.points[i].time_from_start)
    {
      idx = i;
      break;
    }
  }

  // Grab the two points and interpolate
  const trajectory_msgs::JointTrajectoryPoint& end_pt = traj_.points[idx];

  // output container
  std::vector<double> point;
  point.reserve(traj_start_position_.size());

  if (idx == 0)
  {
    // interpolate from start position
    double ratio = dt.toSec() / end_pt.time_from_start.toSec();

    for (int i = 0; i < 4; ++i)
    {
      point.push_back(linearInterpolate(traj_start_position_[i], end_pt.positions[i], ratio));
    }
  }
  else
  {
    const trajectory_msgs::JointTrajectoryPoint& start_pt = traj_.points[idx-1];
    // interpolate between two points
    double ratio = (dt - start_pt.time_from_start).toSec() / (end_pt.time_from_start - start_pt.time_from_start).toSec();   

    for (int i = 0; i < 4; ++i)
    {
      point.push_back(linearInterpolate(start_pt.positions[i], end_pt.positions[i], ratio));
    }
  }

  output = point;
  return true;
}

void ragnar_action::RagnarAction::pollAction()
{ // This function is if it has goal 
  if (has_active_goal_ && ros::Time::now() > (traj_start_time_ + traj_.points.back().time_from_start))
  {
    active_goal_.setSucceeded();
    has_active_goal_ = false;
  }
}

void ragnar_action::RagnarAction::goalCB(CartesianTractoryActionServer::GoalHandle& gh)
{
  ROS_INFO("Recieved new goal request");
  bool acceptance = false; 
  if (has_active_goal_)
  {
    ROS_WARN("Received new goal, canceling current one");
    // stop the robot before abort 
    active_goal_.setAborted();
    has_active_goal_ = false;
  }
  // check for valid trajectory
  const ragnar_msgs::CartesianTrajectory& traj = gh.getGoal()->trajectory;   

  int traj_size = traj.points.size();
  for (int i=0; i<traj_size; i++) {
    ragnar_msgs::CartesianTrajectoryPoint point = traj.points[i];
    double tpose[4];  
    double joints[4];
    tpose[0] = point.pose.position.x;
    tpose[1] = point.pose.position.y;
    tpose[2] = point.pose.position.z;
    if (!ragnar_kinematics::inverse_kinematics(pose, joints))
    {
      ROS_WARN_STREAM("Could not solve for: " << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3]);
      ROS_INFO("Cancelling goal");
      active_goal_.setAborted();
      acceptance = false; 
      break; 
    }
    else 
      acceptance = true; 
  }
  if (acceptance) {
    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;
 
    const ragnar_msgs::CartesianTrajectory& traj = active_goal_.getGoal()->trajectory;
    setTrajectory(traj);
  }
}

void ragnar_action::RagnarAction::cancelCB(CartesianTractoryActionServer::GoalHandle &gh)
{
  ROS_INFO("Cancelling goal");
  if (active_goal_ == gh)
  {
    // stop the controller
    traj_start_time_ = ros::Time(0);
    // mark the goal as canceled
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }
}
