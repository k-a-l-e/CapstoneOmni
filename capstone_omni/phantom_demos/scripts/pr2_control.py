#!/usr/bin/env python3

from math import floor, pi
from typing import List

import actionlib
import rospy
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# pr2_min_limits = [-0.5641557539405562, -0.3522532598071635, -0.6465902122483422, -2.118837338648079, -5.78470869496044e-05, -1.9974696914131314, pi*2]
# pr2_max_limits = [2.1351178939701665, 1.2970939559613923, 3.7490562882038754, -0.15183768307904333, 0.00017354126085258122, -0.1013509534737066, -pi*2]

pr2_min_limits = [-pi] * 7
pr2_max_limits = [pi] * 7

pr2_min_limits[3] = -pi
pr2_max_limits[3] = 0

pr2_min_limits[5] = 0.0
pr2_max_limits[5] = -2.0

phantom_min_limits = [-0.98, 0, -0.81, 3.92, -0.5, -2.58]
phantom_max_limits = [0.98, 1.75, 1.25, 8.83, 1.75, 2.58]



def map(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class PR2Controller():
    def __init__(self):
        rospy.init_node('pr2_phantom_control_node')

        self.pr2_joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.pr2_joint_states_callback, queue_size=1)
        self.pr2_joint_trajectory_client = actionlib.SimpleActionClient('/l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
        self.pr2_jointstates = JointState()
        self.pr2_joint_trajectory_client.wait_for_server()

        self.phantom_joint_states_sub = rospy.Subscriber('/phantom/joint_states', JointState, self.phantom_joint_states_callback, queue_size=1)
        self.phantom_jointstates = JointState()

        # self.setPr2Position([[0, 0, 0, 0, 0, 0, 0]])
        # self.pr2_joint_trajectory_client.wait_for_result()

        rospy.sleep(1)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.setPr2Position([
                [map(self.phantom_jointstates.position[0], phantom_min_limits[0], phantom_max_limits[0], pr2_min_limits[0], pr2_max_limits[0]),
                map(self.phantom_jointstates.position[1], phantom_min_limits[1], phantom_max_limits[1], pr2_max_limits[1], pr2_min_limits[1]),
                0,
                map(self.phantom_jointstates.position[2], phantom_min_limits[2], phantom_max_limits[2], pr2_max_limits[3], pr2_min_limits[3]),
                map(self.phantom_jointstates.position[3], phantom_min_limits[3], phantom_max_limits[3], pr2_min_limits[4], pr2_max_limits[4]),
                map(self.phantom_jointstates.position[4], phantom_min_limits[4], phantom_max_limits[4], pr2_min_limits[5], pr2_max_limits[5]),
                map(self.phantom_jointstates.position[5], phantom_min_limits[5], phantom_max_limits[5], pr2_min_limits[6], pr2_max_limits[6])]
            ], 1.0)
            rate.sleep()
            # rospy.loginfo(self.getPr2JointStatesWithNames(["l_wrist_flex_joint"]))
            # self.pr2_joint_trajectory_client.wait_for_result()



    
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
                if isinstance(durations, float):
                    timeAccum += durations
                else:
                    timeAccum += 0.1
            
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
    pr2_controller = PR2Controller()
    rospy.spin()

if __name__ == '__main__':
    main()