#include "ragnar_actionserver/ragnar_simpleactionserver.h"
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
  , action_server_(nh, "cartesian_trajectory_action", boost::bind(&RagnarAction::executeCB, this, _1),
                   /*boost::bind(&RagnarAction::cancelCB, this, _1),*/ false)
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
  _geopose.position.x = _pose[0];
  _geopose.position.y = _pose[1];
  _geopose.position.z = _pose[2];
  char str[100];
  traj_start_position_cartesian_.assign(_pose, _pose + 3);
  //sprintf(str,"Position: X%f Y%f Z%f", traj_start_position_cartesian_[0], traj_start_position_cartesian_[1], traj_start_position_cartesian_[2]);
  //ROS_INFO(str);
  action_feedback_.joint_names.push_back("joint_1");
  action_feedback_.joint_names.push_back("joint_2");
  action_feedback_.joint_names.push_back("joint_3");
  action_feedback_.joint_names.push_back("joint_4");
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
  for (int i=0; i<3; i++) 
    traj_start_position_cartesian_[i] = position[i];
  traj_ = new_trajectory;
  traj_start_time_ = now;
  return true;
}
bool ragnar_action::RagnarAction::updatePose(const geometry_msgs::PoseStamped& pose)
{
  _geopose = pose.pose; 
  return true;
}
void ragnar_action::RagnarAction::sendFeedback()
{
  if (has_active_goal_) {
    ragnar_msgs::CartesianTrajectoryPoint actual_; 
    actual_.pose = _geopose; 
    action_feedback_.actual = actual_; 
    action_server_.publishFeedback(action_feedback_);
  }
  return; 
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
  // output will be appended with velocity and acceleration too 

  // create auxiliary variables 
  geometry_msgs::Pose trPose; 
  double trpose[3]; 
  char str[100];
  // sprintf(str,"time %d, start time %d", tm, traj_start_time_);
  //ROS_INFO(str);
  // Check to see if time is in past of traj
  if (tm < traj_start_time_ || traj_.points.empty())
  { 
    //ROS_INFO("in past of trajectory");  
    output = traj_start_position_cartesian_;
    // fill in zero velocity and zero acc 
    for ( int i =0; i<6; i++) output.push_back(0.0);
    return true;
  }
  // check to see if time is past end of traj
  else if (tm > traj_start_time_ + traj_.points.back().time_from_start)
  {
    //ROS_INFO("in past end of trajectory");  
    
    // extract the pose to a double vector 
    trPose = traj_.points.back().pose;
    trpose[0] = trPose.position.x;
    trpose[1] = trPose.position.y;
    trpose[2] = trPose.position.z;
    output.assign(trpose, trpose + 3);
    // fill in zero velocity and zero acc 
    for ( int i =0; i<6; i++) output.push_back(0.0);
    return true;
  }
  
  // Otherwise the traj must be within the trajectory
  ros::Duration dt = tm - traj_start_time_;
  //sprintf(str,"dt %d", dt);
  
  //ROS_INFO(str);  
    
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
  const ragnar_msgs::CartesianTrajectoryPoint& end_pt = traj_.points[idx];

  // output container
  std::vector<double> point;
  point.reserve(traj_start_position_cartesian_.size());

  if (idx == 0)
  {
    // interpolate from start position
    double ratio = dt.toSec() / end_pt.time_from_start.toSec();

    //for (int i = 0; i < 3; ++i)
    //{
    point.push_back(linearInterpolate(traj_start_position_cartesian_[0], end_pt.pose.position.x, ratio));
    point.push_back(linearInterpolate(traj_start_position_cartesian_[1], end_pt.pose.position.y, ratio));
    point.push_back(linearInterpolate(traj_start_position_cartesian_[2], end_pt.pose.position.z, ratio));
    // fill in interpolation of velocity and acceleration  
    point.push_back(linearInterpolate(0.0, end_pt.twist.linear.x, ratio));
    point.push_back(linearInterpolate(0.0, end_pt.twist.linear.y, ratio));
    point.push_back(linearInterpolate(0.0, end_pt.twist.linear.z, ratio));
    point.push_back(linearInterpolate(0.0, end_pt.accel.linear.x, ratio));
    point.push_back(linearInterpolate(0.0, end_pt.accel.linear.y, ratio));
    point.push_back(linearInterpolate(0.0, end_pt.accel.linear.z, ratio));
    //}
  }
  else
  {
    const ragnar_msgs::CartesianTrajectoryPoint& start_pt = traj_.points[idx-1];
    // interpolate between two points
    double ratio = (dt - start_pt.time_from_start).toSec() / (end_pt.time_from_start - start_pt.time_from_start).toSec();   

    //for (int i = 0; i < 3; ++i)
    //{
    //  point.push_back(linearInterpolate(start_pt.positions[i], end_pt.positions[i], ratio));
    point.push_back(linearInterpolate(start_pt.pose.position.x, end_pt.pose.position.x, ratio));
    point.push_back(linearInterpolate(start_pt.pose.position.y, end_pt.pose.position.y, ratio));
    point.push_back(linearInterpolate(start_pt.pose.position.z, end_pt.pose.position.z, ratio));
    point.push_back(linearInterpolate(start_pt.twist.linear.x, end_pt.twist.linear.x, ratio));
    point.push_back(linearInterpolate(start_pt.twist.linear.y, end_pt.twist.linear.y, ratio));
    point.push_back(linearInterpolate(start_pt.twist.linear.z, end_pt.twist.linear.z, ratio));
    point.push_back(linearInterpolate(start_pt.accel.linear.x, end_pt.accel.linear.x, ratio));
    point.push_back(linearInterpolate(start_pt.accel.linear.y, end_pt.accel.linear.y, ratio));
    point.push_back(linearInterpolate(start_pt.accel.linear.z, end_pt.accel.linear.z, ratio));
    //}
  }

  output = point;
  return true;
}

void ragnar_action::RagnarAction::pollAction()
{  
  if (has_active_goal_ && ros::Time::now() > (traj_start_time_ + traj_.points.back().time_from_start))
  {
    //active_goal_.setSucceeded();
    action_server_.setSucceeded();
    has_active_goal_ = false;
  }
}

void ragnar_action::RagnarAction::poseStateCB() //const geometry_msgs::Pose::ConstPtr& pose)
{
  if(has_active_goal_)
  {
    if(ros::Time::now() - traj_start_time_ > target_pt_.time_from_start)
    {
      // evaluate action 
      has_active_goal_ = false; 
    }

  }
}

void stopTrajectory()
{
  float f = 2;
  return;
}

void ragnar_action::RagnarAction::executeCB(const ragnar_msgs::FollowCartesianTrajectoryGoalConstPtr & gh)
{
  ROS_INFO("Recieved new goal request");
  bool acceptance = false; 
  if (has_active_goal_)
  {
    ROS_WARN("Received new goal, canceling current one");
    // stop the robot before abort 
    stopTrajectory();
    //active_goal_.setAborted();
    has_active_goal_ = false;
  }
  // check for valid trajectory
  //const ragnar_msgs::CartesianTrajectory& traj = gh.getGoal()->trajectory;
  const ragnar_msgs::CartesianTrajectory& traj = gh->trajectory;
  
  int traj_size = traj.points.size();
  for (int i=0; i<traj_size; i++) {
    ragnar_msgs::CartesianTrajectoryPoint point = traj.points[i];
    double tpose[4];  
    double joints[4];
    tpose[0] = point.pose.position.x;
    tpose[1] = point.pose.position.y;
    tpose[2] = point.pose.position.z;
    if (!ragnar_kinematics::inverse_kinematics(tpose, joints))
    {
      ROS_WARN_STREAM("Could not solve for: " << tpose[0] << " " << tpose[1] << " " << tpose[2] << " " << tpose[3]);
      ROS_INFO("Cancelling goal");
      action_server_.setAborted();
      acceptance = false; 
      break; 
    }
    else 
      acceptance = true; 
  }
  if (acceptance) {
    //gh.acceptNewGoal();
    active_goal_ = gh;
    has_active_goal_ = true;
 
    const ragnar_msgs::CartesianTrajectory& traj = active_goal_->trajectory;
    setTrajectory(traj);
  }

  // new code to execute action inside here 
  ros::Time now = ros::Time::now();
  while(now < traj_start_time_ + traj_.points.back().time_from_start)
  {
    now = ros::Time::now();
    computeTrajectoryPosition(now, mob_command);
    sendFeedback();
  }


  action_server_.setSucceeded();
}

static bool pose_in_range(const double(&vec)[4])
{
  const static double MIN_X = -0.4;
  const static double MAX_X = 0.4;
  const static double MIN_Y = -0.4;
  const static double MAX_Y = 0.4;
  const static double MIN_Z = -0.55;
  const static double MAX_Z = 0.0;

  if (vec[0] > MAX_X || vec[0] < MIN_X)
    return false;
  if (vec[1] > MAX_Y || vec[1] < MIN_Y)
    return false;
  if (vec[2] > MAX_Z || vec[2] < MIN_Z)
    return false;
  return true;
}

void ragnar_action::RagnarAction::cancelCB(CartesianTractoryActionServer::GoalHandle &gh)
{
  ROS_INFO("Cancelling goal");
  /*
  if (active_goal_ == gh)
  {
    // stop the controller
    stopTrajectory();
    traj_start_time_ = ros::Time(0);
    // mark the goal as canceled
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }
  */
}


// void ragnar_action::RagnarAction::jointStateCB(const sensor_msgs::JointStateConstPtr &msg)
// {
//   this->cur_joint_pos_ = *msg;
//   if (has_active_goal_)
//   {
//    if (state_ == TransferStates::IDLE && inRange(cur_joint_pos_.position, target_pt_.positions, JOINT_TOL_EPS))
//    {
//      ROS_INFO("Action succeeded");
//      active_goal_.setSucceeded();
//      has_active_goal_ = false;
//    }
//  }
// }

