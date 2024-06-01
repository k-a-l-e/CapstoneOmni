#!/usr/bin/env python3

from math import floor, pi
from typing import List

import actionlib
import rospy
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GetJointStates():
    def __init__(self):
        rospy.init_node('get_joint_limits_node')

        self.pr2_joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.pr2_joint_states_callback, queue_size=1)
        self.pr2_joint_trajectory_client = actionlib.SimpleActionClient('/l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
        self.pr2_jointstates = JointState()
        self.pr2_joint_trajectory_client.wait_for_server()
        self.pr2_joint_trajectory_client.cancel_all_goals()

        self.phantom_joint_states_sub = rospy.Subscriber('/phantom/joint_states', JointState, self.phantom_joint_states_callback, queue_size=1)
        self.phantom_jointstates = JointState()

        self.setPr2Position([[0] * 7], 5)
        self.pr2_joint_trajectory_client.wait_for_result()
        input("press enter to start")
        self.pr2_joint_trajectory_client.cancel_all_goals()
        rospy.sleep(2)


        self.setPr2Position([[-pi] * 7], 10)
        input("press enter to continue...")
        rospy.loginfo(self.getPr2JointStatesWithNames(["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]))
        self.pr2_joint_trajectory_client.cancel_all_goals()
        rospy.sleep(2)

        self.setPr2Position([[pi] * 7], 10)
        input("press enter to continue...")
        rospy.loginfo(self.getPr2JointStatesWithNames(["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]))
        self.pr2_joint_trajectory_client.cancel_all_goals()
        rospy.sleep(2)

        self.setPr2Position([[0] * 7], 10)
        self.pr2_joint_trajectory_client.wait_for_result()



    
    def setPr2Position(self, traj: List[List[float]], durations: List[float] or float = []):
        jointTrajectory = JointTrajectoryGoal()
        jointTrajectory.trajectory = JointTrajectory()
        jointTrajectory.trajectory.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]

        jointTrajectory.trajectory.points = []
        timeAccum = 0
        for i, t in enumerate(traj):
            try:
                timeAccum += durations[i]
            except:
                if isinstance(durations, list):
                    timeAccum += 0.1
                else:
                    timeAccum += durations

            jointTrajectory.trajectory.points.append(JointTrajectoryPoint())
            jointTrajectory.trajectory.points[i].positions = t
            jointTrajectory.trajectory.points[i].velocities = [0, 0, 0, 0, 0, 0, 0]
            jointTrajectory.trajectory.points[i].accelerations = [0, 0, 0, 0, 0, 0, 0]
            jointTrajectory.trajectory.points[i].time_from_start = rospy.Duration(floor(timeAccum), int((timeAccum - floor(timeAccum)) * 1000000000))
        
        self.pr2_joint_trajectory_client.send_goal(jointTrajectory)

    
    def pr2_joint_states_callback(self, msg: JointState):
        self.pr2_jointstates = msg
    
    def phantom_joint_states_callback(self, msg: JointState):
        self.phantom_jointstates = msg
        # rospy.loginfo(msg)

    def getPr2JointStatesWithNames(self, names: List[str]) -> List[float]:
        return [self.pr2_jointstates.position[self.pr2_jointstates.name.index(name)] for name in names]

def main():
    node = GetJointStates()
    rospy.spin()

if __name__ == '__main__':
    main()