#!/usr/bin/env python

# Python
import copy
import bidict
import random
from math import sin, cos, atan2, floor, sqrt, pi

# numpy
from numpy import mean, std

# ROS
from geometry_msgs.msg import Point

from task_sim.msg import Action


# Globals

OBJECT_TO_INT_MAP = bidict.frozenbidict({
    '': 0, # Also corresponds to the table
    'gripper': 1,
    'stack': 2,
    'drawer': 3,
    'handle': 4,
    'box': 5,
    'lid': 6,
    'apple': 7,
    'batteries': 8,
    'flashlight': 9,
    'granola': 10,
    'knife': 11,
})

# Functions to handle the different frames of reference

def change_frame(state, frame):
    """Convert a state to a different frame."""
    state_copy = copy.deepcopy(state)
    if frame == '' or frame.lower() == 'table':
        return state_copy
    # Special object cases
    translation = Point()
    if frame.lower() == 'stack':
        rotate_state(state_copy, -state_copy.drawer_position.theta)
        translation.x = -state_copy.drawer_position.x
        translation.y = -state_copy.drawer_position.y
        #translation.z = -state.drawer_position.z
        translation.z = 0
    elif frame.lower() == 'drawer':
        rotate_state(state_copy, -state_copy.drawer_position.theta)
        translation.x = -(state_copy.drawer_position.x + state_copy.drawer_opening)
        translation.y = -state_copy.drawer_position.y
        #translation.z = -state.drawer_position.z
        translation.z = -2
    elif frame.lower() == 'handle':  # Note: assumes hardcoded drawer height of 2
        rotate_state(state_copy, -state_copy.drawer_position.theta)
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
        translate_pose(object.position, translation)
    translate_pose2D(state_copy.drawer_position, translation)
    translate_pose(state_copy.box_position, translation)
    translate_pose(state_copy.lid_position, translation)
    translate_pose(state_copy.gripper_position, translation)

    return state_copy

def change_frame_of_point(point, state, frame):
    """Change a point from the global coordinate frame to a specific frame."""
    state_copy = copy.deepcopy(state)
    point_copy = copy.copy(point)

    if frame.lower() == 'stack':
        rotate_pose(point_copy, -state_copy.drawer_position.theta)
        rotate_state(state_copy, -state_copy.drawer_position.theta)
        point_copy.x -= state_copy.drawer_position.x
        point_copy.y -= state_copy.drawer_position.y
    elif frame.lower() == 'drawer':
        rotate_pose(point_copy, -state_copy.drawer_position.theta)
        rotate_state(state_copy, -state_copy.drawer_position.theta)
        point_copy.x -= state_copy.drawer_position.x + state_copy.drawer_opening
        point_copy.y -= state_copy.drawer_position.y
    elif frame.lower() == 'handle':  # Note: assumes hardcoded drawer height of 2
        rotate_pose(point_copy, -state_copy.drawer_position.theta)
        rotate_state(state_copy, -state_copy.drawer_position.theta)
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

def get_point_in_global_frame(state, point, frame):
    """Move a point from a local frame to the table coordinate system"""
    state_copy = copy.deepcopy(state)
    point_copy = copy.copy(point)

    if frame.lower() == 'stack':
        rotate_pose(point_copy, state_copy.drawer_position.theta)
        #rotate_state(state_copy, state_copy.drawer_position.theta)
        point_copy.x += state_copy.drawer_position.x
        point_copy.y += state_copy.drawer_position.y
    elif frame.lower() == 'drawer':
        rotate_pose(point_copy, state_copy.drawer_position.theta)
        #rotate_state(state_copy, state_copy.drawer_position.theta)
        point_copy.x += state_copy.drawer_position.x + cos(state.drawer_position.theta*pi/180)*state_copy.drawer_opening
        point_copy.y += state_copy.drawer_position.y + sin(state.drawer_position.theta*pi/180)*state_copy.drawer_opening
    elif frame.lower() == 'handle':  # Note: assumes hardcoded drawer height of 2
        rotate_pose(point_copy, state_copy.drawer_position.theta)
        #rotate_state(state_copy, state_copy.drawer_position.theta)
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

def get_closest_frame(state, position):
    """Get the closest frame to a given position (within a threshold)"""
    min_dst = 5  # Distance threshold for frame change to happen
    frame = ''

    for object in state.objects:
        dst = euclidean_3D(position, object.position)
        if dst < min_dst:
            min_dst = dst
            frame = object.name
    # drawer stack
    dst = euclidean_3D(position, Point(state.drawer_position.x, state.drawer_position.y, 0))
    if dst < min_dst:
        min_dst = dst
        frame = 'Stack'

    # drawer
    if state.drawer_position.theta == 0:
        dst = euclidean_3D(position, Point(state.drawer_position.x + state.drawer_opening, state.drawer_position.y, 0))
    elif state.drawer_position.theta == 90:
        dst = euclidean_3D(position, Point(state.drawer_position.x, state.drawer_position.y + state.drawer_opening, 0))
    elif state.drawer_position.theta == 180:
        dst = euclidean_3D(position, Point(state.drawer_position.x - state.drawer_opening, state.drawer_position.y, 0))
    elif state.drawer_position.theta == 270:
        dst = euclidean_3D(position, Point(state.drawer_position.x, state.drawer_position.y - state.drawer_opening, 0))
    if dst < min_dst:
        min_dst = dst
        frame = 'Drawer'

    # handle
    if state.drawer_position.theta == 0:
        dst = euclidean_3D(position, Point(state.drawer_position.x + state.drawer_opening + 4, state.drawer_position.y, 0))
    elif state.drawer_position.theta == 90:
        dst = euclidean_3D(position, Point(state.drawer_position.x, state.drawer_position.y + state.drawer_opening + 4, 0))
    elif state.drawer_position.theta == 180:
        dst = euclidean_3D(position, Point(state.drawer_position.x - state.drawer_opening - 4, state.drawer_position.y, 0))
    elif state.drawer_position.theta == 270:
        dst = euclidean_3D(position, Point(state.drawer_position.x, state.drawer_position.y - state.drawer_opening - 4, 0))
    if dst < min_dst:
        min_dst = dst
        frame = 'Handle'

    # box
    dst = euclidean_3D(position, state.box_position)
    if dst < min_dst:
        min_dst = dst
        frame = 'Box'

    # lid
    dst = euclidean_3D(position, state.lid_position)
    if dst < min_dst:
        min_dst = dst
        frame = 'Lid'

    # gripper
    dst = euclidean_3D(position, state.gripper_position)
    if dst < min_dst:
        min_dst = dst
        frame = 'Gripper'

    return frame

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

# Get key important task related features

def get_object_by_name(state, name):
    """Find an object given an object name"""
    for object in state.objects:
        if object.name == name or object.name.lower() == name:
            return object
    return None

def get_object_at(state, position):
    """Return the object at a given position"""
    for object in state.objects:
        if object.position == position:
            return object
    return None

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

def get_drawer_midpoint_pos(state):
    """Get the midpoint of an open drawer"""
    point = Point(state.drawer_position.x, state.drawer_position.y, 2)
    if state.drawer_position.theta == 0:
        point.x += 4 + state.drawer_opening//2
    elif state.drawer_position.theta == 90:
        point.y += 4 + state.drawer_opening//2
    elif state.drawer_position.theta == 180:
        point.x -= 4 + state.drawer_opening//2
    else:
        point.y -= 4 + state.drawer_opening//2
    return point

# Calculation helper functions

def euclidean_3D(p1, p2):
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2))

def translate_pose(pose, translation):
    pose.x += translation.x
    pose.y += translation.y
    pose.z += translation.z

def translate_pose2D(pose, translation):
    pose.x += translation.x
    pose.y += translation.y

def rotate_pose(pose, rotation):
    rotation *= 3.14159/180.0
    rotated_pose = Point()
    rotated_pose.x = floor(cos(rotation)*pose.x - sin(rotation)*pose.y + 0.5)
    rotated_pose.y = floor(sin(rotation)*pose.x + cos(rotation)*pose.y + 0.5)
    pose.x = int(rotated_pose.x)
    pose.y = int(rotated_pose.y)

def rotate_state(state, rotation):
    for object in state.objects:
        rotate_pose(object.position, rotation)
    rotate_pose(state.drawer_position, rotation)
    state.drawer_position.theta += rotation
    state.drawer_position.theta %= 360
    rotate_pose(state.box_position, rotation)
    rotate_pose(state.lid_position, rotation)
    rotate_pose(state.gripper_position, rotation)

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

# State representations

def naive_state_vector(state, state_positions=True, state_semantics=True, history_buffer=0, position_semantics=False):
    """Convert a state message into a simple feature vector."""
    vector = []
    for obj in state.objects:
        if state_positions:
            vector.extend([
                obj.position.x, obj.position.y, obj.position.z,
                int(obj.occluded), int(obj.lost)
            ])
        if state_semantics:
            vector.extend([
                int(obj.in_drawer),
                int(obj.in_box),
                int(obj.on_lid),
                int(obj.in_gripper)
            ])
            if not state_positions and position_semantics:
                vector.extend([int(obj.occluded), int(obj.lost)])
    if state_positions:
        vector.extend([
            state.drawer_position.x,
            state.drawer_position.y,
            state.drawer_position.theta,
            state.drawer_opening,
            state.box_position.x,
            state.box_position.y,
            state.box_position.z,
            state.lid_position.x,
            state.lid_position.y,
            state.lid_position.z,
            state.gripper_position.x,
            state.gripper_position.y,
            state.gripper_position.z,
            int(state.gripper_open)
        ])
    if state_semantics:
        vector.extend([name_to_int(state.object_in_gripper)])
        if not state_positions and position_semantics:
            vector.extend([int(state.gripper_open)])

    for i in xrange(history_buffer):
        vector.append(state.action_history[len(state.action_history) - i - 1])
        if state.result_history[len(state.result_history) - i - 1]:
            vector.append(1)
        else:
            vector.append(0)

    return vector

def semantic_state_vector(
    state, history_buffer=0, feature_type=int, return_dict=False
):
    """
    Creates a semantic state vector with the following information (boolean
    unless otherwise specified in {}):

    object[5]:
        in_box, in_drawer, on_lid, on_stack, not_visible, near_edge,
        touching_box, touching_lid, touching_stack
    <obj>_x_<obj>_touching[10]
    container[2]:
        open
    gripper:
        open
        object{8}
        relative_height{3} [10]
        relative_x{3} [10]
        relative_y{3} [10]
        near_edge
        touching_<obj>[5]
        touching_box, touching_drawer, touching_lid, touching_stack
    lid:
        near_edge
    """
    global OBJECT_TO_INT_MAP

    # Helper function
    def set_feature(key, value):
        semantic_state_keys.append(key)
        semantic_state[key] = value

    # State variables
    semantic_state_keys = []
    semantic_state = {}

    # Setup the helper variables
    objects = [obj.name.lower() for obj in state.objects]

    # First, get the properties of the objects one by one
    for idx, obj_name in enumerate(objects):
        obj = get_object_by_name(obj_name)

        # Directly from the state - in_box, in_drawer, on_lid, on_stack, not_visible
        set_feature("{}_in_box".format(obj_name), obj.in_box)
        set_feature("{}_in_drawer".format(obj_name), obj.in_drawer)
        set_feature("{}_on_lid".format(obj_name), obj.on_lid)
        set_feature("{}_on_stack".format(obj_name), obj.on_stack)
        set_feature("{}_not_visible".format(obj_name), obj.occluded or obj.lost)

        # Near edge
        set_feature("{}_near_edge".format(obj_name), None) # TODO

        # Touching
        set_feature("touching_{}_x_box".format(obj_name), None) #TODO
        set_feature("touching_{}_x_lid".format(obj_name), None) # TODO
        set_feature("touching_{}_x_stack".format(obj_name), None) # TODO

        # Object touching objects
        for j in xrange(idx+1, len(objects)):
            other_obj = get_object_by_name(objects[j])
            set_feature("touching_{}_x_{}".format(obj_name, objects[j]), None) # TODO

    # Get container open
    set_feature(
        "box_open",
        abs(state.box_position.x - state.lid_position.x) > 1)
            and (abs(state.lid_position.y - state.lid_position.y) > 1
    )
    set_feature("drawer_open", state.drawer_opening > 1)

    # Gripper Semantics
    set_feature("gripper_open", state.gripper_open)
    set_feature("object_in_gripper", name_to_int(state.object_in_gripper))
    set_feature("gripper_near_edge", None) # TODO

    # Relative positions and touching
    for obj_name in OBJECT_TO_INT_MAP.iterkeys():
        if not obj_name or obj_name == 'gripper':
            continue

        # TODO: include logic to get the different position values
        obj_position = None

        # Relative Height
        set_feature("relative_h_{}".format(obj_name), None) # TODO
        set_feature("relative_x_{}".format(obj_name), None) # TODO
        set_feature("relative_y_{}".format(obj_name), None) # TODO

        # Touching. We don't care about handle
        if obj_name == 'handle':
            continue
        set_feature("touching_gripper_{}".format(obj_name), None) # TODO
        # Might need to include different logic for the containers?

    # Lid is near the edge
    set_feature("lid_near_edge", None) # TODO

    # Add in the history
    for i in xrange(history_buffer):
        set_feature("history_action{}".format(i), state.action_history[-1-i])
        set_feature("history_result{}".format(i), state.result_history[-1-i])

    # Cast the data to the desired type based on feature_type
    if feature_type is not None:
        for k,v in semantic_state.iteritems():
            semantic_state[k] = feature_type(v)

    # Return the state vector depending on the type desired
    return (
        [semantic_state[k] for k in semantic_state_keys]
        if not return_dict
        else (semantic_state, semantic_state_keys)
    )


# Action representation helper functions

def semantic_action_to_position(state, target):
    position = Point()
    if target.lower() == 'stack':
        # Pick a random free point on top of the stack of drawers
        points = []
        if state.drawer_position.theta == 0 or state.drawer_position.theta == 180:
            for x in range(int(state.drawer_position.x - 3), int(state.drawer_position.x + 4)):
                for y in range(int(state.drawer_position.y - 2), int(state.drawer_position.y + 3)):
                    clear = True
                    for obj in state.objects:
                        if obj.position.x == x and obj.position.y == y and obj.position.z == 3:
                            clear = False
                            break
                    if clear:
                        points.append(Point(x, y, 3))
        else:
            for x in range(int(state.drawer_position.x - 2), int(state.drawer_position.x + 3)):
                for y in range(int(state.drawer_position.y - 3), int(state.drawer_position.y + 4)):
                    clear = True
                    for obj in state.objects:
                        if obj.position.x == x and obj.position.y == y and obj.position.z == 3:
                            clear = False
                            break
                    if clear:
                        points.append(Point(x, y, 3))
        if len(points) > 0:
            position = random.choice(points)
        else:  # Set position as table center point
            position.x = state.drawer_position.x
            position.y = state.drawer_position.y
            position.z = 3
    elif target.lower() == 'drawer':
        # Pick a random free point in the drawer that's also not in the drawer stack footprint
        points = []
        if state.drawer_position.theta == 0:
            for x in range(int(state.drawer_position.x + 4), int(state.drawer_position.x + state.drawer_opening + 3)):
                for y in range(int(state.drawer_position.y - 1), int(state.drawer_position.y + 2)):
                    clear = True
                    for obj in state.objects:
                        if obj.position.x == x and obj.position.y == y and obj.position.z > 0:
                            clear = False
                            break
                    if clear:
                        points.append(Point(x, y, 2))
        elif state.drawer_position.theta == 180:
            for x in range(int(state.drawer_position.x - state.drawer_opening - 2), int(state.drawer_position.x - 3)):
                for y in range(int(state.drawer_position.y - 1), int(state.drawer_position.y + 2)):
                    clear = True
                    for obj in state.objects:
                        if obj.position.x == x and obj.position.y == y and obj.position.z > 0:
                            clear = False
                            break
                    if clear:
                        points.append(Point(x, y, 2))
        elif state.drawer_position.theta == 90:
            for x in range(int(state.drawer_position.x - 1), int(state.drawer_position.x + 2)):
                for y in range(int(state.drawer_position.y + 4), int(state.drawer_position.y + state.drawer_opening + 3)):
                    clear = True
                    for obj in state.objects:
                        if obj.position.x == x and obj.position.y == y and obj.position.z > 0:
                            clear = False
                            break
                    if clear:
                        points.append(Point(x, y, 2))
        else:
            for x in range(int(state.drawer_position.x - 1), int(state.drawer_position.x + 2)):
                for y in range(int(state.drawer_position.y - state.drawer_opening - 2), int(state.drawer_position.y - 3)):
                    clear = True
                    for obj in state.objects:
                        if obj.position.x == x and obj.position.y == y and obj.position.z > 0:
                            clear = False
                            break
                    if clear:
                        points.append(Point(x, y, 2))
        if len(points) > 0:
            position = random.choice(points)
        else:  # Set position as drawer center point
            if state.drawer_position.theta == 0:
                position.x = state.drawer_position.x + state.drawer_opening
                position.y = state.drawer_position.y
            elif state.drawer_position.theta == 90:
                position.x = state.drawer_position.x
                position.y = state.drawer_position.y + state.drawer_opening
            elif state.drawer_position.theta == 180:
                position.x = state.drawer_position.x - state.drawer_opening
                position.y = state.drawer_position.y
            else:
                position.x = state.drawer_position.x
                position.y = state.drawer_position.y - state.drawer_opening
            position.z = 2
    elif target.lower() == 'box':
        # Special case: holding lid
        if state.object_in_gripper.lower() == 'lid':
            position = state.box_position
        else:
            # Pick a random free point in the box that's also not in the lid footprint
            points = []
            for x in range(int(state.box_position.x - 1), int(state.box_position.x + 2)):
                for y in range(int(state.box_position.y - 1), int(state.box_position.y + 2)):
                    if (x >= state.lid_position.x - 2 and x <= state.lid_position.x + 2
                        and y >= state.lid_position.y - 2 and y <= state.lid_position.y + 2):
                        continue
                    clear = True
                    for obj in state.objects:
                        if obj.position.x == x and obj.position.y == y and obj.position.z <= 1:
                            clear = False
                            break
                    if clear:
                        points.append(Point(x, y, 2))
            if len(points) > 0:
                position = random.choice(points)
            else:  # Set position as box center
                position = state.box_position
    elif target.lower() == 'lid':
        # Pick a random free point on the lid
        points = []
        for x in range(int(state.lid_position.x - 2), int(state.lid_position.x + 3)):
            for y in range(int(state.lid_position.y - 2), int(state.lid_position.y + 3)):
                clear = True
                for obj in state.objects:
                    if obj.position.x == x and obj.position.y == y and obj.position.z == state.lid_position.z:
                        clear = False
                        break
                if clear:
                    points.append(Point(x, y, 2))
        if len(points) > 0:
            position = random.choice(points)
        else:  # Set position to lid center
            position = state.lid_position
    elif target.lower() == 'handle':
        position = get_handle_pos(state)
    elif target.lower() == 'table' or target.lower() == '':  # Pick a random position on the table
        while True:
            position.x = random.randint(0, 40)
            position.y = random.randint(0, 15)
            position.z = 0
            if position.x >= state.box_position.x - 2 and position.x <= state.box_position.x + 2 \
                and position.y >= state.box_position.y - 2 and position.y >= state.box_position.y + 2:
                continue
            if position.x >= state.lid_position.x - 2 and position.x <= state.lid_position.x + 2 \
                and position.y >= state.lid_position.y - 2 and position.y >= state.lid_position.y + 2:
                continue
            if state.drawer_position.theta == 0:
                if position.x >= state.drawer_position.x - 3 and position.x <= state.drawer_position.x + 3 \
                        + state.drawer_opening and position.y >= state.drawer_position.y - 2 \
                        and position.y <= state.drawer_position.y + 2:
                    continue
            elif state.drawer_position.theta == 90:
                if position.x >= state.drawer_position.x - 2 and position.x <= state.drawer_position.x + 2 \
                        and position.y >= state.drawer_position.y - 2 \
                        and position.y <= state.drawer_position.y + 2 + state.drawer_opening:
                    continue
            elif state.drawer_position.theta == 180:
                if position.x >= state.drawer_position.x - 3 - state.drawer_opening \
                        and position.x <= state.drawer_position.x + 3 and position.y >= state.drawer_position.y - 2\
                        and position.y <= state.drawer_position.y + 2:
                    continue
            else:
                if position.x >= state.drawer_position.x - 2 and position.x <= state.drawer_position.x + 2 \
                        and position.y >= state.drawer_position.y - 2 - state.drawer_opening \
                        and position.y <= state.drawer_position.y + 2:
                    continue
            obj_collision = False
            for o in state.objects:
                if position.x == o.position.x and position.y == o.position.y:
                    obj_collision = True
                    break
            if obj_collision:
                continue
            break
    else:
        for o in state.objects:
            if target.lower() == o.name.lower():
                position = o.position
                break

    return position

def create_combined_action(action_type, object, target, state):
    """Combined Action is an integer with the action params specified by the
    tens and ones digit, and the action type specified by the 100s digit on
    """
    label = 100*action_type
    if action_type in [Action.GRASP, Action.PLACE]:
        label += name_to_int(object)
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

def get_action_from_label(action_label):
    """action_label is the integer combined action representation"""
    return action_label//100

def get_action_modifier_from_label(action_label):
    """action_label is the integer combined action representation"""
    return action_label%100

def get_action_obj_offset_candidates(state):
    """Create action parameterizations that are possible in the format
    Action(object, offset), where object is a string, and offset is a tuple.
    The output parameterized action itself is a tuple.

    Input state is of type task_sim.State
    """
    global OBJECT_TO_INT_MAP

    action_candidates = [(Action.NOOP, None, None)]

    # Open, Close Gripper
    action_candidates.extend([
        (Action.OPEN_GRIPPER, None, None,),
        (Action.CLOSE_GRIPPER, None, None),
    ])

    # Raise, Lower, Reset Arm
    action_candidates.extend([
        (Action.RAISE_ARM, None, None),
        (Action.LOWER_ARM, None, None),
        (Action.RESET_ARM, None, None),
    ])

    # Grasp
    for object_key in OBJECT_TO_INT_MAP.iterkeys():
        if not object_key: # Table/Empty object name
            continue

        # Do not allow a grasp on 'Handle' because it's the same as 'Drawer'
        if object_key == 'handle':
            continue

        # Allow the box as a candidate, this should fail
        action_candidates.append(
            (Action.GRASP, to_object_name(object_key), None)
        )

    # Move, Place
    for object_key in OBJECT_TO_INT_MAP.iterkeys():
        # We perform no collision/edge checking. Instead, all options are
        # available and I'm choosing to arbitrarily end limit the offset to
        # -5 < offset < 5 for an object, and the whole range for the table
        # Tuple format (xlim, ylim, symmetric?)
        offset_limits = (5, 5, True) if object_key else (40, 15, False)

        # Iterate through the offsets and add the move and place options
        for x in range(
            -offset_limits[0] if offset_limits[-1] else 0,
            offset_limits[0]+1 if offset_limits[-1] else offset_limits[0]
        ):
            for y in range(
                -offset_limits[1] if offset_limits[-1] else 0,
                offset_limits[1]+1 if offset_limits[-1] else offset_limits[1]
            ):
                action_candidates.extend([
                    (Action.PLACE, to_object_name(object_key), (x,y)),
                    (Action.MOVE_ARM, to_object_name(object_key), (x,y)),
                ])

    return action_candidates

def get_semantic_action_candidates(state):
    """Given a state, this returns the possible semantic action candidates
    that can be used by `semantic_action_to_position()`. Parameters are
    (action, object, target)"""
    global OBJECT_TO_INT_MAP

    action_candidates = [(Action.NOOP, None, None)]

    # Open, Close Gripper
    action_candidates.extend([
        (Action.OPEN_GRIPPER, None, None,),
        (Action.CLOSE_GRIPPER, None, None),
    ])

    # Raise, Lower, Reset Arm
    action_candidates.extend([
        (Action.RAISE_ARM, None, None),
        (Action.LOWER_ARM, None, None),
        (Action.RESET_ARM, None, None),
    ])

    # Grasp
    for object_key in OBJECT_TO_INT_MAP.iterkeys():
        if not object_key: # Table/Empty object name
            continue

        # Do not allow a grasp on 'Handle' because it's the same as 'Drawer'
        if object_key == 'handle':
            continue

        # Allow the box as a candidate, this should fail
        action_candidates.append(
            (Action.GRASP, to_object_name(object_key), None)
        )

    # Move, Place
    for object_key in OBJECT_TO_INT_MAP.iterkeys():
        action_candidates.extend([
            (Action.PLACE, None, to_object_name(object_key)),
            (Action.MOVE_ARM, None, to_object_name(object_key)),
        ])

    return action_candidates

def msg_from_action_obj_offset(state, action_obj_offset):
    """Takes an action parameterization from
    `get_action_obj_offset_candidates` and creates an Action message"""
    action = Action()
    action.action_type = action_obj_offset[0]

    # First grab the object if it is a grasp
    if action.action_type in [Action.GRASP]:
        action.object = action_obj_offset[1]

    # Then, if we have to move or place, translate the offset
    if action.action_type in [Action.MOVE_ARM, Action.PLACE]:
        # First check to see if this is one of the graspable objects
        found_pos = None
        for state_obj in state.objects:
            if action_obj_offset[1] == state_obj.name:
                found_pos = state_obj.position
                break

        # If the object was not found, check stack, drawer, handle, box,
        # lid, and gripper
        if found_pos is None:
            if action_obj_offset[1] == 'Stack':
                found_pos = state.drawer_position
            elif action_obj_offset[1] == 'Drawer':
                found_pos = get_drawer_midpoint_pos(state)
            elif action_obj_offset[1] == 'Handle':
                found_pos = get_handle_pos(state)
            elif action_obj_offset[1] == 'Box':
                found_pos = state.box_position
            elif action_obj_offset[1] == 'Lid':
                found_pos = state.lid_position
            elif action_obj_offset[1] == 'Gripper':
                found_pos = state.gripper_position

        # Update the position in the action object
        action.position.x = (
            found_pos.x + action_obj_offset[2][0]
            if found_pos
            else action_obj_offset[2][0]
        )
        action.position.y = (
            found_pos.y + action_obj_offset[2][1]
            if found_pos
            else action_obj_offset[2][1]
        )

    # We're done. Return the action
    return action

def msg_from_semantic_action(state, semantic_action):
    """Takes an action parameterization from
    `get_semantic_action_candidates` and creates and Action message"""
    action = Action()
    action.action_type = semantic_action[0]

    # Grab the object if it's a grasp
    if action.action_type in [Action.GRASP]:
        action.object = semantic_action[1]

    # Use the semantic_action_to_position to get a position for MOVE, PLACE
    if action.action_type in [Action.MOVE_ARM, Action.PLACE]:
        position = semantic_action_to_position(
            state, semantic_action[2]
        )
        action.position.x = position.x
        action.position.y = position.y

    return action

# Target object name helper functions

def name_to_int(name):
    """Returns the object as an integer. An unknown object is the same as
    the object of `''` (table)"""
    global OBJECT_TO_INT_MAP

    return OBJECT_TO_INT_MAP.get(
        name.lower(),
        OBJECT_TO_INT_MAP['']
    )

def int_to_name(n):
    """Returns the referent object from an int index. If not found, return
    `''` (table)"""
    global OBJECT_TO_INT_MAP

    return OBJECT_TO_INT_MAP.inv.get(
        n,
        OBJECT_TO_INT_MAP.inv[0]
    )

def object_int_pairs():
    """Returns the (obj (lower), int) pairs of the objects"""
    global OBJECT_TO_INT_MAP
    return OBJECT_TO_INT_MAP.items()

def to_object_name(object_key):
    return (object_key[0].upper() + object_key[1:]) if object_key else object_key
