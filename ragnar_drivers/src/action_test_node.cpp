#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <ragnar_kinematics/ragnar_kinematics.h>

static trajectory_msgs::JointTrajectory makeCircleTrajectory()
{
  using namespace trajectory_msgs;
  // Header
  JointTrajectory traj;

  // Create circle points
  const double r = 0.2;
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

    JointTrajectoryPoint pt;
    if (!ragnar_kinematics::inverse_kinematics(pose, joints))
    {
      ROS_WARN_STREAM("Could not solve for: " << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3]);
    }
    else
    {
      pt.positions.assign(joints, joints+4);
      pt.time_from_start = ros::Duration(total_t);
      total_t += dt;
      traj.points.push_back(pt);
    }
  }
  return traj;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_test_node");
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
  ROS_INFO("Waiting");
  ac.waitForServer();
  ROS_INFO("Connected");

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = makeCircleTrajectory();
  goal.trajectory.joint_names.push_back("joint_1");
  goal.trajectory.joint_names.push_back("joint_2");
  goal.trajectory.joint_names.push_back("joint_3");
  goal.trajectory.joint_names.push_back("joint_4");

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ROS_INFO("Waiting on goal");
  ac.waitForResult();

  actionlib::SimpleClientGoalState state = ac.getState();
  ROS_INFO("Action finished: %s",state.toString().c_str());

  return 0;
}
