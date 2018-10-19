#include <ros/ros.h>
#include <ragnar_msgs/FollowCartesianTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <ragnar_kinematics/ragnar_kinematics.h>

static ragnar_msgs::CartesianTrajectory makeCircleTrajectory()
{
  ragnar_msgs::CartesianTrajectory traj;

  // Create circle points
  const double r = 0.15;
  const double dt = 0.05;

  double pose[4];
  double joints[4];

  double total_t = dt;

  for (int i = 0; i < 360; ++i)
  {
    pose[0] = r * std::cos(i * M_PI / 180.0);
    pose[1] = r * std::sin(i * M_PI / 180.0);
    pose[2] = -0.4;
    pose[3] = 0.0;

    ragnar_msgs::CartesianTrajectoryPoint pt; 
    pt.pose.position.x = pose[0]; 
    pt.pose.position.y = pose[1]; 
    pt.pose.position.z = pose[2]; 

    pt.time_from_start = ros::Duration(total_t);
    total_t += dt;
    traj.points.push_back(pt);
    
    traj.points.size();

  }
  ROS_INFO("Getting size %i", traj.points.size());
  return traj;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_test_node");
  actionlib::SimpleActionClient<ragnar_msgs::FollowCartesianTrajectoryAction> ac("cartesian_trajectory_action", true);
  ROS_INFO("Waiting");
  ac.waitForServer();
  ROS_INFO("Connected");

  ragnar_msgs::FollowCartesianTrajectoryGoal goal;
  goal.trajectory = makeCircleTrajectory();
  
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ROS_INFO("Waiting on goal");
  ac.waitForResult();

  actionlib::SimpleClientGoalState state = ac.getState();
  ROS_INFO("Action finished: %s",state.toString().c_str());

  return 0;
}
