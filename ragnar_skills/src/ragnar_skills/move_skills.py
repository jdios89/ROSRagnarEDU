from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential
from scalable_skills_common.action_client_primitive import PrimitiveActionClient
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
import actionlib
import rospy
import ast
import math
from actionlib_msgs.msg import GoalStatus
from ragnar_msgs.msg import FollowCartesianTrajectoryAction
# from ragnar_msgs.msg import CartesianTrajectory

from geometry_msgs.msg import PoseStamped

class ExecuteCartsianTrajectory(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Time_execution", 10.0, ParamTypes.Required)
        self.addParam("Goal_Position", str, ParamTypes.Required)


class execute_cartsian_trajectory(PrimitiveActionClient):
    """
    This primitive has 3 states
    """
    def createDescription(self):
        """Set the primitive type"""
        self.setDescription(ExecuteCartsianTrajectory(), self.__class__.__name__)

    def onReset(self):
        """Re-initialize the primitive"""
        self._poselists = []
        self._completion = 0.0

    def buildClient(self):
        return actionlib.SimpleActionClient("/cartesian_trajectory_action", FollowCartesianTrajectoryAction)

    def buildGoal(self):
        timing = self.params["Time_execution"].value
        # convert from string
        pose_target = ast.litteral.eval(self.params["Goal_Position"].value)
        # listen to current position 
        msg = rospy.wait_for_message('/ragnar_pose', PoseStamped)
        pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # convert it to FollowCartesianTrajectory
        trajectory_ = getTrajectory(pose, pose_target, timing)
        goal = FollowCartesianTrajectoryGoal()
        goal.trajectory = trajectory_
        return goal

    def getTrajectory(pose, pose_target):
        trajectory_ = CartesianTrajectory()
        trajectory_.header.stamp = rospy.Time.now()
        diff = [x - y for x,y in zip(pose_target, pose)]
        lenght = pow(sum([pow(x, 2) for x in diff]),0.5)
        number_step = math.ceil(lenght/0.005)
        step_lenght = lenght/number_step
        for step in range(number_step):
            final_pose = [pose[i] + (step/number_step)*diff[i] for i in range(pose)]
            pt_ = CartesianTrajectoryPoint()
            pt_.pose.position.x = final_pose[0]
            pt_.pose.position.y = final_pose[1]
            pt_.pose.position.z = final_pose[2]
            pt_.duration = (step/number_step)*timing
            trajectory_.points.append(pt_)
        print(trajectory_)
        return trajectory_


    def onFeedback(self, msg):
        """
        @brief To override. Called every time a new feedback msg is received.
        @return Can return self.success, self.fail or self.step
        
        if rospy.Time.now() > self._to:
            rospy.logwarn("'%s' execution timed out." %(self.__class__.__name__))
            self.onPreempt()
        position = [round(elem,2) for elem in msg.desired.positions]
        if position in self._poselists:
            self._completion = float(self._poselists.index(position))/float(len(self._poselists))*100.0
            return self.step("Progress: {}%".format(round(self._completion)))
        else:
            return self.step("")
        """
        return self.step("")
        
    #def init(one, two, three):
    #    pass 

    def onDone(self, status, msg):
        if ((status == GoalStatus.PREEMPTED) or (status == GoalStatus.ABORTED)):
            return self.fail("Failed", -1)
        elif status == GoalStatus.SUCCEEDED:
            return self.success("Succeeded")
        elif status == GoalStatus.REJECTED:
            rospy.loginfo(status)
            rospy.loginfo(msg)
            return self.fail("Goal was rejected by action server.", -2)
        else:
            rospy.loginfo(status)
            rospy.loginfo(msg)
            return self.fail("Unknown return code.", -100)
