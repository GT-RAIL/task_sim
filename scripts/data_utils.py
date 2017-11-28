#!/usr/bin/env python

# Python
import copy
from math import sin, cos, floor, sqrt

# ROS
from geometry_msgs.msg import Point


class DataUtils:

    @staticmethod
    def change_frame(state, frame):
        """Convert a state to a different frame."""
        state_copy = copy.deepcopy(state)
        # Special object cases
        translation = Point()
        if frame.lower() == 'drawer':
            DataUtils.rotate_state(state_copy, -state_copy.drawer_position.theta)
            translation.x = -state_copy.drawer_position.x
            translation.y = -state_copy.drawer_position.y
            #translation.z = -state.drawer_position.z
            translation.z = 0
        elif frame.lower() == 'handle':  # Note: assumes hardcoded drawer height of 2
            DataUtils.rotate_state(state_copy, -state_copy.drawer_position.theta)
            # Note: translation offset has a hardcoded drawer size
            translation.x = -(state_copy.drawer_position.x + state_copy.drawer_opening + 4)
            translation.y = -state_copy.drawer_position.y
            #translation.z = -state.drawer_position.z
            translation.z = -2
        elif frame.lower() == 'box':
            translation.x = -state_copy.box_position.x
            translation.y = -state_copy.box_position.y
            translation.z = -state_copy.box_position.z
        elif frame.lower() == 'lid':
            translation.x = -state_copy.lid_position.x
            translation.y = -state_copy.lid_position.y
            translation.z = -state_copy.lid_position.z
        elif frame.lower() == 'gripper':
            translation.x = -state_copy.gripper_position.x
            translation.y = -state_copy.gripper_position.y
            translation.z = -state_copy.gripper_position.z
        else:  # General object case
            for object in state_copy.objects:
                if frame.lower() == object.name.lower():
                    translation.x = -object.position.x
                    translation.y = -object.position.y
                    translation.z = -object.position.z
                    break

        # Perform translation
        for object in state_copy.objects:
            DataUtils.translate_pose(object.position, translation)
        DataUtils.translate_pose2D(state_copy.drawer_position, translation)
        DataUtils.translate_pose(state_copy.box_position, translation)
        DataUtils.translate_pose(state_copy.lid_position, translation)
        DataUtils.translate_pose(state_copy.gripper_position, translation)

        return state_copy

    @staticmethod
    def change_frame_of_point(point, state, frame):
        state_copy = copy.deepcopy(state)
        point_copy = copy.copy(point)
        if frame.lower() == 'drawer':
            DataUtils.rotate_pose(point_copy, -state_copy.drawer_position.theta)
            DataUtils.rotate_state(state_copy, -state_copy.drawer_position.theta)
            point_copy.x -= state_copy.drawer_position.x
            point_copy.y -= state_copy.drawer_position.y
        elif frame.lower() == 'handle':  # Note: assumes hardcoded drawer height of 2
            DataUtils.rotate_pose(point_copy, -state_copy.drawer_position.theta)
            DataUtils.rotate_state(state_copy, -state_copy.drawer_position.theta)
            # Note: translation offset has a hardcoded drawer size
            point_copy.x -= state_copy.drawer_position.x + state_copy.drawer_opening + 4
            point_copy.y -= state_copy.drawer_position.y
            point_copy.z -= 2
            pass
        elif frame.lower() == 'box':
            point_copy.x -= state_copy.box_position.x
            point_copy.y -= state_copy.box_position.y
            point_copy.z -= state_copy.box_position.z
        elif frame.lower() == 'lid':
            point_copy.x -= state_copy.lid_position.x
            point_copy.y -= state_copy.lid_position.y
            point_copy.z -= state_copy.lid_position.z
        elif frame.lower() == 'gripper':
            point_copy.x -= state_copy.gripper_position.x
            point_copy.y -= state_copy.gripper_position.y
            point_copy.z -= state_copy.gripper_position.z
        else:  # General object case
            for object in state_copy.objects:
                if frame.lower() == object.name.lower():
                    point_copy.x -= object.position.x
                    point_copy.y -= object.position.y
                    point_copy.z -= object.position.z
                    break
        return point_copy

    @staticmethod
    def get_closest_frame(state, position):
        """Get the closest frame to a given position (within a threshold)"""
        min_dst = 10  # Distance threshold for frame change to happen
        frame = ''

        for object in state.objects:
            dst = DataUtils.euclidean_3D(position, object.position)
            if dst < min_dst:
                min_dst = dst
                frame = object.name
        # drawer
        dst = DataUtils.euclidean_3D(position, Point(state.drawer_position.x, state.drawer_position.y, 0))
        if dst < min_dst:
            min_dst = dst
            frame = 'Drawer'

        # handle
        dst = DataUtils.euclidean_3D(position, Point(state.drawer_position.x + state.drawer_opening + 4,
                                                     state.drawer_position.y, 0))
        if dst < min_dst:
            min_dst = dst
            frame = 'Handle'

        # box
        dst = DataUtils.euclidean_3D(position, state.box_position)
        if dst < min_dst:
            min_dst = dst
            frame = 'Box'

        # lid
        dst = DataUtils.euclidean_3D(position, state.lid_position)
        if dst < min_dst:
            min_dst = dst
            frame = 'Lid'

        # gripper
        dst = DataUtils.euclidean_3D(position, state.gripper_position)
        if dst < min_dst:
            min_dst = dst
            frame = 'Gripper'

        return frame

    @staticmethod
    def euclidean_3D(p1, p2):
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2))

    @staticmethod
    def translate_pose(pose, translation):
        pose.x += translation.x
        pose.y += translation.y
        pose.z += translation.z

    @staticmethod
    def translate_pose2D(pose, translation):
        pose.x += translation.x
        pose.y += translation.y

    @staticmethod
    def rotate_pose(pose, rotation):
        rotation *= 3.14159/180.0
        rotated_pose = Point()
        rotated_pose.x = floor(cos(rotation)*pose.x - sin(rotation)*pose.y + 0.5)
        rotated_pose.y = floor(sin(rotation)*pose.x + cos(rotation)*pose.y + 0.5)
        pose.x = int(rotated_pose.x)
        pose.y = int(rotated_pose.y)

    @staticmethod
    def rotate_state(state, rotation):
        for object in state.objects:
            DataUtils.rotate_pose(object.position, rotation)
        DataUtils.rotate_pose(state.drawer_position, rotation)
        state.drawer_position.theta += rotation
        state.drawer_position.theta %= 360
        DataUtils.rotate_pose(state.box_position, rotation)
        DataUtils.rotate_pose(state.lid_position, rotation)
        DataUtils.rotate_pose(state.gripper_position, rotation)

    @staticmethod
    def name_to_int(name):
        if name.lower() == '':
            return 0
        elif name.lower() == 'gripper':
            return 1
        elif name.lower() == 'drawer':
            return 2
        elif name.lower() == 'handle':
            return 3
        elif name.lower == 'box':
            return 4
        elif name.lower() == 'lid':
            return 5
        elif name.lower() == 'apple':
            return 6
        elif name.lower() == 'batteries':
            return 7
        elif name.lower() == 'flashlight':
            return 8
        elif name.lower() == 'granola':
            return 9
        elif name.lower() == 'knife':
            return 10
        else:
            return -1
