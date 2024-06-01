
import math
from dataclasses import dataclass
from math import pi
from typing import List, Tuple

import numpy as np
import spatialmath as sm
import spatialmath.base.quaternions as quat
from visualization_msgs.msg import Marker


@dataclass
class Collision:
    hit: bool
    hit_skin: bool
    normal: Tuple[float, float, float]
    local_normal: Tuple[float, float, float]
    surface_hit_point: Tuple[float, float, float]
    distance: float
    signed_distance: float
    marker: Marker

    def get_visual_marker(self):
        marker = Marker()
        marker.header.frame_id = self.marker.header.frame_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        q = lookAt(self.normal)
        marker.pose.orientation.w = q[0]
        marker.pose.orientation.x = q[1]
        marker.pose.orientation.y = q[2]
        marker.pose.orientation.z = q[3]

        marker.pose.position.x = self.surface_hit_point[0]
        marker.pose.position.y = self.surface_hit_point[1]
        marker.pose.position.z = self.surface_hit_point[2]
        return marker


def lookAt(destPoint: List[float], up=np.array([0, 0, 1]), forward=np.array([1, 0, 0])):
    forwardVector = np.array(destPoint) / np.linalg.norm(destPoint)

    dot = np.dot(forward, forwardVector)

    if (np.abs(dot + 1) < 0.000001):
        return quat.qunit([up[0], up[1], up[2], pi])

    if (np.abs(dot - 1) < 0.000001):
        return quat.qeye()

    rotAngle = math.acos(dot)
    rotAxis = np.cross(forward, forwardVector)
    rotAxis = rotAxis / np.linalg.norm(rotAxis)
    mat = sm.SO3.AngleAxis(rotAngle, rotAxis).R
    q = quat.r2q(mat)
    return q


def point_sphere_collision(point: List[float], sphere_marker: Marker, skin: float = None, previous_collision: Collision = None) -> Collision:

    sphere = [sphere_marker.pose.position.x,
              sphere_marker.pose.position.y, sphere_marker.pose.position.z]
    radius = sphere_marker.scale.x / 2

    distance = np.linalg.norm(np.array(point) - np.array(sphere))
    signed_distance = distance - radius
    distance = abs(signed_distance)

    hit = False
    hit = signed_distance < 0

    hit_skin = False
    if skin is not None:
        hit_skin = signed_distance < skin

    normal = np.array(point) - np.array(sphere)
    normal = normal / np.linalg.norm(normal)

    surface_hit_point = np.array(sphere) + radius * normal

    return Collision(hit=hit, hit_skin=hit_skin, normal=normal, local_normal=normal, surface_hit_point=surface_hit_point, distance=distance, signed_distance=signed_distance, marker=sphere_marker)


def point_box_collision(point: List[float], box_marker: Marker, skin: float = None, previous_collision: Collision = None) -> Collision:

    point = np.array(point)

    if skin is None:
        skin = 0

    box_pos = np.array([box_marker.pose.position.x,
                        box_marker.pose.position.y, box_marker.pose.position.z])
    box_size = np.array(
        [box_marker.scale.x, box_marker.scale.y, box_marker.scale.z])

    # Transform point to box frame
    box_rot = quat.q2r([box_marker.pose.orientation.w, box_marker.pose.orientation.x,
                       box_marker.pose.orientation.y, box_marker.pose.orientation.z])
    box_tform = np.eye(4)
    box_tform[:3, :3] = box_rot
    box_tform[:3, 3] = box_pos
    box_tform_inv = np.linalg.inv(box_tform)
    local_point = np.dot(box_tform_inv, np.append(point, 1))[:3]

    # Compute distance to box
    signed_distances = local_point
    shortest_distance_index = np.argmax(np.abs(signed_distances))
    shortest_distance = signed_distances[shortest_distance_index]


    hit = True
    for i in range(3):
        if np.abs(signed_distances[i]) > (box_size[i] / 2.0):
            hit = False
            break
    hit_skin = True
    for i in range(3):
        if np.abs(signed_distances[i]) > (box_size[i] / 2.0 + skin):
            hit_skin = False
            break

    # Compute normal
    local_normal = np.zeros(3)
    local_normal[shortest_distance_index] = np.sign(shortest_distance)

    if previous_collision is not None and previous_collision.hit:
        local_normal = previous_collision.local_normal

    box_tform_rot = np.eye(4)
    box_tform_rot[:3, :3] = box_rot
    # inv_box_tform_rot = np.linalg.inv(box_tform_rot)

    # Transform normal back to world frame
    normal = np.dot(box_tform_rot, np.append(local_normal, 1))[:3]

    # Compute surface hit point
    surface_hit_point = (local_normal * box_size / 2.0)
    # surface_hit_point = np.array(np.dot(box_tform_rot, np.append(surface_hit_point, 1))[:3])
    surface_hit_point = np.array(
        np.dot(box_tform, np.append(surface_hit_point, 1))[:3])

    projected_point = np.array(
        point) - np.dot(point - surface_hit_point, normal) * normal

    return Collision(hit=hit, hit_skin=hit_skin, normal=normal, local_normal=local_normal, surface_hit_point=projected_point, distance=np.abs(shortest_distance), signed_distance=shortest_distance, marker=box_marker)
