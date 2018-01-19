#!/usr/bin/env python

# Python
import copy
from math import sin, cos, atan2, floor, sqrt, pi

# numpy
from numpy import mean, std

# ROS
from geometry_msgs.msg import Point

from task_sim.msg import Action


class DataUtils:

    @staticmethod
    def change_frame(state, frame):
        """Convert a state to a different frame."""
        state_copy = copy.deepcopy(state)
        if frame == '' or frame.lower() == 'table':
            return state_copy
        # Special object cases
        translation = Point()
        if frame.lower() == 'stack':
            DataUtils.rotate_state(state_copy, -state_copy.drawer_position.theta)
            translation.x = -state_copy.drawer_position.x
            translation.y = -state_copy.drawer_position.y
            #translation.z = -state.drawer_position.z
            translation.z = 0
        elif frame.lower() == 'drawer':
            DataUtils.rotate_state(state_copy, -state_copy.drawer_position.theta)
            translation.x = -(state_copy.drawer_position.x + state_copy.drawer_opening)
            translation.y = -state_copy.drawer_position.y
            #translation.z = -state.drawer_position.z
            translation.z = -2
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
        """Change a point from the global coordinate frame to a specific frame."""
        state_copy = copy.deepcopy(state)
        point_copy = copy.copy(point)

        if frame.lower() == 'stack':
            DataUtils.rotate_pose(point_copy, -state_copy.drawer_position.theta)
            DataUtils.rotate_state(state_copy, -state_copy.drawer_position.theta)
            point_copy.x -= state_copy.drawer_position.x
            point_copy.y -= state_copy.drawer_position.y
        elif frame.lower() == 'drawer':
            DataUtils.rotate_pose(point_copy, -state_copy.drawer_position.theta)
            DataUtils.rotate_state(state_copy, -state_copy.drawer_position.theta)
            point_copy.x -= state_copy.drawer_position.x + state_copy.drawer_opening
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
    def get_point_in_global_frame(state, point, frame):
        """Move a point from a local frame to the table coordinate system"""
        state_copy = copy.deepcopy(state)
        point_copy = copy.copy(point)

        if frame.lower() == 'stack':
            DataUtils.rotate_pose(point_copy, state_copy.drawer_position.theta)
            #DataUtils.rotate_state(state_copy, state_copy.drawer_position.theta)
            point_copy.x += state_copy.drawer_position.x
            point_copy.y += state_copy.drawer_position.y
        elif frame.lower() == 'drawer':
            DataUtils.rotate_pose(point_copy, state_copy.drawer_position.theta)
            #DataUtils.rotate_state(state_copy, state_copy.drawer_position.theta)
            point_copy.x += state_copy.drawer_position.x + cos(state.drawer_position.theta*pi/180)*state_copy.drawer_opening
            point_copy.y += state_copy.drawer_position.y + sin(state.drawer_position.theta*pi/180)*state_copy.drawer_opening
        elif frame.lower() == 'handle':  # Note: assumes hardcoded drawer height of 2
            DataUtils.rotate_pose(point_copy, state_copy.drawer_position.theta)
            #DataUtils.rotate_state(state_copy, state_copy.drawer_position.theta)
            # Note: translation offset has a hardcoded drawer size
            point_copy.x += state_copy.drawer_position.x + cos(state.drawer_position.theta*pi/180)*(state_copy.drawer_opening + 4)
            point_copy.y += state_copy.drawer_position.y + sin(state.drawer_position.theta*pi/180)*(state_copy.drawer_opening + 4)
            point_copy.z += 2
            pass
        elif frame.lower() == 'box':
            point_copy.x += state_copy.box_position.x
            point_copy.y += state_copy.box_position.y
            point_copy.z += state_copy.box_position.z
        elif frame.lower() == 'lid':
            point_copy.x += state_copy.lid_position.x
            point_copy.y += state_copy.lid_position.y
            point_copy.z += state_copy.lid_position.z
        elif frame.lower() == 'gripper':
            point_copy.x += state_copy.gripper_position.x
            point_copy.y += state_copy.gripper_position.y
            point_copy.z += state_copy.gripper_position.z
        else:  # General object case
            for object in state_copy.objects:
                if frame.lower() == object.name.lower():
                    point_copy.x += object.position.x
                    point_copy.y += object.position.y
                    point_copy.z += object.position.z
                    break

        return point_copy

    @staticmethod
    def get_closest_frame(state, position):
        """Get the closest frame to a given position (within a threshold)"""
        min_dst = 5  # Distance threshold for frame change to happen
        frame = ''

        for object in state.objects:
            dst = DataUtils.euclidean_3D(position, object.position)
            if dst < min_dst:
                min_dst = dst
                frame = object.name
        # drawer stack
        dst = DataUtils.euclidean_3D(position, Point(state.drawer_position.x, state.drawer_position.y, 0))
        if dst < min_dst:
            min_dst = dst
            frame = 'Stack'

        # drawer
        if state.drawer_position.theta == 0:
            dst = DataUtils.euclidean_3D(position, Point(state.drawer_position.x + state.drawer_opening,
                                                         state.drawer_position.y, 0))
        elif state.drawer_position.theta == 90:
            dst = DataUtils.euclidean_3D(position, Point(state.drawer_position.x,
                                                         state.drawer_position.y + state.drawer_opening, 0))
        elif state.drawer_position.theta == 180:
            dst = DataUtils.euclidean_3D(position, Point(state.drawer_position.x - state.drawer_opening,
                                                         state.drawer_position.y, 0))
        elif state.drawer_position.theta == 270:
            dst = DataUtils.euclidean_3D(position, Point(state.drawer_position.x,
                                                         state.drawer_position.y - state.drawer_opening, 0))
        if dst < min_dst:
            min_dst = dst
            frame = 'Drawer'

        # handle
        if state.drawer_position.theta == 0:
            dst = DataUtils.euclidean_3D(position, Point(state.drawer_position.x + state.drawer_opening + 4,
                                                         state.drawer_position.y, 0))
        elif state.drawer_position.theta == 90:
            dst = DataUtils.euclidean_3D(position, Point(state.drawer_position.x,
                                                         state.drawer_position.y + state.drawer_opening + 4, 0))
        elif state.drawer_position.theta == 180:
            dst = DataUtils.euclidean_3D(position, Point(state.drawer_position.x - state.drawer_opening - 4,
                                                         state.drawer_position.y, 0))
        elif state.drawer_position.theta == 270:
            dst = DataUtils.euclidean_3D(position, Point(state.drawer_position.x,
                                                         state.drawer_position.y - state.drawer_opening - 4, 0))
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
    def get_handle_pos(state):
        """Get the position of the drawer handle"""
        point = Point(state.drawer_position.x, state.drawer_position.y, 2)
        if state.drawer_position.theta == 0:
            point.x += 4 + state.drawer_opening
        elif state.drawer_position.theta == 90:
            point.y += 4 + state.drawer_opening
        elif state.drawer_position.theta == 180:
            point.x -= 4 + state.drawer_opening
        else:
            point.y -= 4 + state.drawer_opening
        return point

    @staticmethod
    def get_task_frame(state, position):
        """Get a task-related frame (e.g. drawer, lid, table, etc.) for actions such as place"""

        # drawer
        if state.drawer_position.theta == 0:
            if (position.x >= state.drawer_position.x - 3 and position.x <= state.drawer_position.x + 3
                and position.y >= state.drawer_position.y - 2 and position.y <= state.drawer_position.y + 2):
                return 'Stack'
            elif (position.x >= state.drawer_position.x + state.drawer_opening - 3
                  and position.x <= state.drawer_position.x + state.drawer_opening + 3
                  and position.y >= state.drawer_position.y - 2 and position.y <= state.drawer_position.y + 2):
                return 'Drawer'
        elif state.drawer_position.theta == 90:
            if (position.x >= state.drawer_position.x - 2 and position.x <= state.drawer_position.x + 2
                and position.y >= state.drawer_position.y - 3 and position.y <= state.drawer_position.y + 3):
                return 'Stack'
            elif (position.x >= state.drawer_position.x - 2 and position.x <= state.drawer_position.x + 2
                  and position.y >= state.drawer_position.y + state.drawer_opening - 3
                  and position.y <= state.drawer_position.y + state.drawer_opening + 3):
                return 'Drawer'
        elif state.drawer_position.theta == 180:
            if (position.x >= state.drawer_position.x - 3 and position.x <= state.drawer_position.x + 3
                and position.y >= state.drawer_position.y - 2 and position.y <= state.drawer_position.y + 2):
                return 'Stack'
            elif (position.x >= state.drawer_position.x - state.drawer_opening - 3
                  and position.x <= state.drawer_position.x - state.drawer_opening + 3
                  and position.y >= state.drawer_position.y - 2 and position.y <= state.drawer_position.y + 2):
                return 'Drawer'
        elif state.drawer_position.theta == 270:
            if (position.x >= state.drawer_position.x - 2 and position.x <= state.drawer_position.x + 2
                and position.y >= state.drawer_position.y - 3 and position.y <= state.drawer_position.y + 3):
                return 'Stack'
            elif (position.x >= state.drawer_position.x - 2 and position.x <= state.drawer_position.x + 2
                  and position.y >= state.drawer_position.y - state.drawer_opening - 3
                  and position.y <= state.drawer_position.y - state.drawer_opening + 3):
                return 'Drawer'

        # box and lid
        if (position.x >= state.lid_position.x - 2 and position.x <= state.lid_position.x + 2
            and position.y >= state.lid_position.y - 2 and position.y <= state.lid_position.y + 2):
            return 'Lid'
        elif (position.x >= state.box_position.x - 2 and position.x <= state.box_position.x + 2
            and position.y >= state.box_position.y - 2 and position.y <= state.box_position.y + 2):
            return 'Box'

        # default
        return 'Table'


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
    def normalize_vector(vector):
        vec = copy.copy(vector)
        means = mean(vec, axis=0)
        stds = std(vec, axis=0)
        for row in vec:
            for i in range(row.shape[0]):
                if stds[i] == 0:
                    row[i] = 0
                else:
                    row[i] = (row[i] - means[i]) / stds[i]
        return vec

    @staticmethod
    def naive_state_vector(state, state_positions=True, state_semantics=True, history_buffer=0):
        """Convert a state message into a simple feature vector."""
        vector = []
        for obj in state.objects:
            if state_positions:
                vector.extend([obj.position.x, obj.position.y, obj.position.z, int(obj.occluded), int(obj.lost)])
            if state_semantics:
                vector.extend([int(obj.in_drawer), int(obj.in_box), int(obj.on_lid), int(obj.in_gripper)])
        if state_positions:
            vector.extend([state.drawer_position.x, state.drawer_position.y, state.drawer_position.theta,
                           state.drawer_opening, state.box_position.x, state.box_position.y, state.box_position.z,
                           state.lid_position.x, state.lid_position.y, state.lid_position.z, state.gripper_position.x,
                           state.gripper_position.y, state.gripper_position.z, int(state.gripper_open)])
        if state_semantics:
            vector.extend([DataUtils.name_to_int(state.object_in_gripper)])

        for i in range(history_buffer):
            vector.append(state.action_history[len(state.action_history) - i - 1])
            if state.result_history[len(state.result_history) - i - 1]:
                vector.append(1)
            else:
                vector.append(0)

        return vector

    @staticmethod
    def create_combined_action(action_type, object, target, state):
        label = 100*action_type
        if action_type in [Action.GRASP, Action.PLACE]:
            label += DataUtils.name_to_int(object)
        elif action_type in [Action.MOVE_ARM]:
            # Note: assumes position is in the global frame
            x = target.x - state.gripper_position.x
            y = target.y - state.gripper_position.y
            angle = atan2(y, x)
            while angle < 0:
                angle += 2*pi
            min_dst = 2*pi
            dir = 0
            for i in range(9):
                dst = abs(i*pi/4 - angle)
                if dst < min_dst:
                    min_dst = dst
                    dir = i
            dir %= 8
            label += dir
        return label

    @staticmethod
    def get_action_from_label(action_label):
        return action_label//100

    @staticmethod
    def get_action_modifier_from_label(action_label):
        return action_label%100

    @staticmethod
    def name_to_int(name):
        if name.lower() == '' or name.lower() == 'table':
            return 0
        elif name.lower() == 'gripper':
            return 1
        elif name.lower() == 'stack':
            return 2
        elif name.lower() == 'drawer':
            return 3
        elif name.lower() == 'handle':
            return 4
        elif name.lower() == 'box':
            return 5
        elif name.lower() == 'lid':
            return 6
        elif name.lower() == 'apple':
            return 7
        elif name.lower() == 'batteries':
            return 8
        elif name.lower() == 'flashlight':
            return 9
        elif name.lower() == 'granola':
            return 10
        elif name.lower() == 'knife':
            return 11
        else:
            return -1

    @staticmethod
    def int_to_name(n):
        if n == 0:
            return ''
        elif n == 1:
            return 'Gripper'
        elif n == 2:
            return 'Stack'
        elif n == 3:
            return 'Drawer'
        elif n == 4:
            return 'Handle'
        elif n == 5:
            return 'Box'
        elif n == 6:
            return 'Lid'
        elif n == 7:
            return 'Apple'
        elif n == 8:
            return 'Batteries'
        elif n == 9:
            return 'Flashlight'
        elif n == 10:
            return 'Granola'
        elif n == 11:
            return 'Knife'
        else:
            return ''
