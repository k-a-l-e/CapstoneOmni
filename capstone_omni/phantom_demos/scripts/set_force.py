#!/usr/bin/env python3

import math
from math import pi

import collib
import numpy as np
import rospy
import spatialmath.base.quaternions as quat
import tf
from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import OmniButtonEvent, OmniFeedback, OmniState
from sensor_msgs.msg import JointState
from simple_pid import PID
from visualization_msgs.msg import Marker, MarkerArray

p_crit = 500
p = 0.6 * p_crit
i = 0.5 * p_crit * 0
d = 0.125 * p_crit * 0
pid = PID(p, i, d, setpoint=0)


phantom_min_limits = [-0.98, 0, -0.81, 3.92, -0.5, -2.58]
phantom_max_limits = [0.98, 1.75, 1.25, 8.83, 1.75, 2.58]


# def point_sphere_collision(point: List[float], marker: Marker, skin: float = None) -> Tuple[bool, List[float], float]:

#     sphere = [marker.pose.position.x,
#               marker.pose.position.y, marker.pose.position.z]
#     radius = marker.scale.x / 2

#     if skin is not None:
#         radius = radius + skin

#     is_collide = math.sqrt((point[0] - sphere[0]) ** 2 + (
#         point[1] - sphere[1]) ** 2 + (point[2] - sphere[2]) ** 2) <= radius
#     normal = [point[0] - sphere[0], point[1] - sphere[1], point[2] - sphere[2]]
#     distance_to_surface = math.sqrt((point[0] - sphere[0]) ** 2 + (
#         point[1] - sphere[1]) ** 2 + (point[2] - sphere[2]) ** 2) - radius
#     normal_normalized = np.array(normal) / np.linalg.norm(normal)
#     return is_collide, normal_normalized, distance_to_surface


# def point_box_collision(point: List[float], marker: Marker, skin: float = None) -> Tuple[bool, List[float], float]:

#     box_pos = np.array(
#         [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
#     size = np.array([marker.scale.x, marker.scale.y, marker.scale.z])

#     if skin is not None:
#         for i in range(3):
#             size[i] = size[i] + skin

#     box_rot = quat.q2r([marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z])
#     box_tform = np.eye(4)
#     box_tform[:3, :3] = box_rot
#     box_tform[:3, 3] = box_pos

#     point_global = np.array(point)
#     point_local = np.linalg.inv(box_tform) @ np.append(point_global, 1)

#     is_collide = True
#     for i in range(3):
#         if point_local[i] < -size[i] / 2 or point_local[i] > size[i] / 2:
#             is_collide = False

#     # calculate distance from center inside cube
#     distances = np.zeros(3)
#     for i in range(3):
#         if point_local[i] < 0:
#             distances[i] = point_local[i] + size[i] / 2
#         else:
#             distances[i] = point_local[i] - size[i] / 2

#     # find closest distance
#     closest_distance_index = np.argmin(np.abs(distances))
#     # - size[closest_distance_index] / 2
#     distance_to_surface = distances[closest_distance_index]

#     # calculate normal
#     normal = np.zeros(3)
#     normal[closest_distance_index] = np.sign(
#         distances[closest_distance_index]) * np.sign(distance_to_surface)

#     # transform normal to global space
#     normal_global = box_tform[:3, :3] @ normal

#     normal_local = normal * np.sign(distance_to_surface)

#     return is_collide, normal_global, distance_to_surface, normal_local


class SetForce():
    def __init__(self):
        rospy.init_node('phantom_set_force_node')

        self.phantom_joint_states_sub = rospy.Subscriber(
            '/phantom/joint_states', JointState, self.phantom_joint_states_callback, queue_size=1)
        self.phantom_states_sub = rospy.Subscriber(
            '/phantom/state', OmniState, self.phantom_states_callback, queue_size=1)
        self.phantom_pose_sub = rospy.Subscriber(
            '/phantom/pose', PoseStamped, self.phantom_pose_callback, queue_size=1)
        self.phantom_button_sub = rospy.Subscriber(
            '/phantom/button', OmniButtonEvent, self.phantom_button_callback, queue_size=1)
        self.phantom_force_pub = rospy.Publisher(
            '/phantom/force_feedback', OmniFeedback, queue_size=1)

        self.phantom_jointstates = JointState()
        self.phantom_state = OmniState()
        self.phantom_button_state = [False, False]
        self.phantom_prev_button_state = [False, False]
        self.phantom_pose = PoseStamped()

        self.marker_pub = rospy.Publisher(
            '/phantom/visualization_marker_array', MarkerArray, queue_size=1)

        self.marker_array = MarkerArray()
        # place markers in a circle Transformer().around origin

        self.first_col_cube_normal = [0, 0, 0]
        self.first_col_cube_local_normal = [0, 0, 0]
        self.first_col_cube_dist = [0, 0, 0]
        self.col_state = False
        self.prev_col_state = False

        self.prev_col: collib.Collision = None

        count = 3
        types = [1, 2, 1]  # 1 = cube, 2 = sphere
        for i in range(count):
            marker = Marker()
            marker.id = len(self.marker_array.markers)
            marker.header.frame_id = 'base_link'
            marker.type = types[i]  # np.random.choice(types)
            marker.action = marker.ADD

            diameter = 0.08 if marker.type == 2 else 0.05
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = diameter

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.8

            marker.pose.position.x = 0.05 * math.cos(2 * pi * i / count)
            marker.pose.position.y = 0.05 * math.sin(2 * pi * i / count) + 0.15
            marker.pose.position.z = 0.15

            # rotate to face origin
            q = tf.transformations.random_quaternion()
            marker.pose.orientation.w = q[0]
            marker.pose.orientation.x = q[1]
            marker.pose.orientation.y = q[2]
            marker.pose.orientation.z = q[3]

            self.marker_array.markers.append(marker)

        # ground marker
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.type = marker.CUBE
        marker.action = marker.ADD

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.8

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = -(marker.scale.z / 2) + 0.01

        # rotate to face origin
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        marker.pose.orientation.w = q[0]
        marker.pose.orientation.x = q[1]
        marker.pose.orientation.y = q[2]
        marker.pose.orientation.z = q[3]

        marker.id = len(self.marker_array.markers)
        self.marker_array.markers.append(marker)

        rospy.sleep(1)

        rate = rospy.Rate(1000)
        msg = OmniFeedback()
        msg.force.x = 0.0
        msg.force.y = 0.0
        msg.force.z = 0.0

        toggle_force = False

        force_to_apply = [0.0, 0.0, 0.0]
        while not rospy.is_shutdown():
            msg.position = self.phantom_pose.pose.position

            # if self.phantom_button_state[0] and not self.phantom_prev_button_state[0]:
            #     toggle_force = not toggle_force

            # if self.phantom_button_state[1] and not self.phantom_prev_button_state[1]:
            #     toggle_force = not toggle_force

            # if toggle_force:
            #     msg.force.x = 0.0
            #     msg.force.y = 0.0
            #     msg.force.z = -100.0
            # else:
            #     msg.force.x = 0.0
            #     msg.force.y = 0.0
            #     msg.force.z = 0.0

            pid_force = np.array([0, 0, 0])

            pt: Marker = None

            point = np.array([self.phantom_pose.pose.position.x,
                              self.phantom_pose.pose.position.y, self.phantom_pose.pose.position.z])
            snap_point = np.copy(point)

            col: collib.Collision = None
            for i, m in enumerate(self.marker_array.markers):
                m: Marker = m
                normal = [0, 0, 0]
                dist = 0
                skin = 0.002

                if m.type == Marker().SPHERE:
                    col = collib.point_sphere_collision(point, m, skin, self.prev_col)
                elif m.type == Marker().CUBE:
                    col = collib.point_box_collision(point, m, skin, self.prev_col)

                if col.hit or col.hit_skin:
                    m.color.r = 1.0
                    m.color.g = 0.0
                    m.color.b = 0.0
                    
                    reverse = 1
                    if not col.hit:
                        m.color.r = 0.0
                        m.color.g = 1.0
                        m.color.b = 0.0
                        reverse = -1

                    snap_point[0] = col.surface_hit_point[0]
                    snap_point[1] = col.surface_hit_point[1]
                    snap_point[2] = col.surface_hit_point[2]

                    pid_force = col.normal * pid(np.linalg.norm(snap_point - point)) * reverse

                    break
                else:
                    m.color.r = 1.0
                    m.color.g = 1.0
                    m.color.b = 1.0

            vel = np.array([self.phantom_state.velocity.x,
                           self.phantom_state.velocity.y, self.phantom_state.velocity.z])

            # damping_factor = 0.01
            force_to_apply = -pid_force # - damping_factor * vel

            # force_to_apply = force_to_apply * 0 # disable PID

            msg.force.x = force_to_apply[0]
            msg.force.y = force_to_apply[1]
            msg.force.z = force_to_apply[2]
            # rospy.loginfo(msg.force)

            e = tf.transformations.euler_from_quaternion([self.marker_array.markers[0].pose.orientation.x, self.marker_array.markers[0].pose.orientation.y,
                                                         self.marker_array.markers[0].pose.orientation.z, self.marker_array.markers[0].pose.orientation.w])
            q = tf.transformations.quaternion_from_euler(
                e[0], e[1], e[2] + 0.01)
            self.marker_array.markers[0].pose.orientation.x = q[0]
            self.marker_array.markers[0].pose.orientation.y = q[1]
            self.marker_array.markers[0].pose.orientation.z = q[2]
            self.marker_array.markers[0].pose.orientation.w = q[3]

            pt = Marker()
            pt.id = len(self.marker_array.markers)
            pt.header.frame_id = 'base_link'
            pt.type = pt.ARROW
            pt.action = pt.ADD
            pt.scale.x = 0.05
            pt.scale.y = 0.0025
            pt.scale.z = 0.0025
            pt.color.r = 0.0
            pt.color.g = 1.0
            pt.color.b = 0.0
            pt.color.a = 1.0

            # q = tf.transformations.quaternion_from_euler(0, 0, 0)
            # q = tf.transformations.quaternion_about_axis(1, (0, 0, 0))

            q = quat.qeye()
            if col.hit or col.hit_skin:
                q = collib.lookAt(col.normal)
            else:
                pt.scale.x = 0
                pt.scale.y = 0
                pt.scale.z = 0

            pt.pose.orientation.w = q[0]
            pt.pose.orientation.x = q[1]
            pt.pose.orientation.y = q[2]
            pt.pose.orientation.z = q[3]
            pt.pose.position.x = snap_point[0]
            pt.pose.position.y = snap_point[1]
            pt.pose.position.z = snap_point[2]

            self.marker_array.markers.append(pt)

            self.marker_pub.publish(self.marker_array)
            self.phantom_force_pub.publish(msg)

            self.phantom_prev_button_state = self.phantom_button_state
            self.prev_col_state = self.col_state
            self.prev_col = col
            rate.sleep()

            self.marker_array.markers.pop()

    def phantom_pose_callback(self, msg: PoseStamped):
        self.phantom_pose = msg
        # rospy.loginfo(msg)

    def phantom_button_callback(self, msg: OmniButtonEvent):
        self.phantom_button_state = [
            msg.grey_button == 1, msg.white_button == 1]
        # rospy.loginfo(msg)

    def phantom_states_callback(self, msg: OmniState):
        self.phantom_state = msg
        # rospy.loginfo(msg)

    def phantom_joint_states_callback(self, msg: JointState):
        self.phantom_jointstates = msg
        # rospy.loginfo(msg)


def main():
    node = SetForce()
    rospy.spin()


if __name__ == '__main__':
    main()
