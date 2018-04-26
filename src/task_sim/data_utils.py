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

class Globals:
    # FIXME: This global map won't work with small containers
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

    DEFAULT_ENVIRONMENT = { # Defaults when random level == 1
        'box_radius': 2,
        'box_height': 1,
        'drawer_width': 5,
        'drawer_depth': 7,
        'drawer_height': 2,
        'table_width': 40,
        'table_height': 15,
    }

    # FIXME: This global map won't work with small containers
    TOUCH_SEARCH_OFFSETS = [
        Point(x,y,0) for x in xrange(-1,2) for y in xrange(-1,2)
    ]

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

def get_translated_pose(pose, translation):
    new_pose = Point(pose.x, pose.y, pose.z)
    translate_pose(new_pose, translation)
    return new_pose

def get_translated_pose2D(pose, translation):
    new_pose = Point(pose.x, pose.y, pose.z)
    translate_pose2D(new_pose, translation)
    return new_pose

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

def on_box_edge(position, xmin, xmax, ymin, ymax, zmin, zmax):
    """Detect whether a point is on the perimeter of a given rectangular prism

    Keyword arguments:
    position -- point to check
    (xmin, xmax, ymin, ymax, zmin, zmax) -- bounds of the prism
    """
    return ((((position.x == xmin or position.x == xmax)
              and position.y >= ymin
              and position.y <= ymax)
             or ((position.y == ymin or position.y == ymax)
                 and position.x >= xmin
                 and position.x <= xmax))
            and position.z >= zmin
            and position.z <= zmax)

def in_volume(position, xmin, xmax, ymin, ymax, zmin, zmax):
    """Detect whether a point is within a rectangular prism

    Keyword arguments:
    position -- point to check
    (xmin, xmax, ymin, ymax, zmin, zmax) -- bounds of the prism
    """
    return (position.x >= xmin and position.x <= xmax
        and position.y >= ymin and position.y <= ymax
        and position.z >= zmin and position.z <= zmax)

def is_position_near_edge(
    position, x_margin=4, y_margin=2,
    xlim=Globals.DEFAULT_ENVIRONMENT['table_width'],
    ylim=Globals.DEFAULT_ENVIRONMENT['table_height']
):
    """Check if the position is near the edge"""
    return not (
        (x_margin < position.x < xlim-x_margin)
        and (y_margin < position.y < ylim-y_margin)
    )

def get_relative_position_ternary(pos1, pos2):
    """Given two position scalars pos1 and pos2, return -1 if pos1 < pos2, 0 if
    pos1 == pos2, else +1"""
    return -1 if pos1 < pos2 else (0 if pos1 == pos2 else 1)

def is_adjacent(position1, position2):
    """Given 2 positions, return if they are adjacent"""
    return any([
        position2 == get_translated_pose2D(position1, translation)
        for translation in Globals.TOUCH_SEARCH_OFFSETS
        if translation.x != 0 and translation.y != 0
    ])

# Object Getters

def get_object_by_name(state, name):
    """Find an object given an object name"""
    for object in state.objects:
        if object.unique_name == name or object.unique_name.lower() == name:
            return object
    return None

def get_container_by_name(state, name):
    """Find a container given a container name"""
    for container in state.containers:
        if container.unique_name == name or container.unique_name.lower() == name:
            return container
    return None

def get_object_at(state, position):
    """Return the object at a given position"""
    for object in state.objects:
        if object.position == position:
            return object
    return None

def name_to_int(name):
    """Returns the object as an integer. An unknown object is the same as
    the object of `''` (table)"""
    return Globals.OBJECT_TO_INT_MAP.get(name.lower(), Globals.OBJECT_TO_INT_MAP[''])

def int_to_name(n):
    """Returns the referent object from an int index. If not found, return
    `''` (table)"""
    return Globals.OBJECT_TO_INT_MAP.inv.get(n, Globals.OBJECT_TO_INT_MAP.inv[0])

def object_int_pairs():
    """Returns the (obj (lower), int) pairs of the objects"""
    return Globals.OBJECT_TO_INT_MAP.items()

def to_object_name(object_key):
    return (object_key[0].upper() + object_key[1:]) if object_key else object_key

# Drawer getters

def get_drawer_bounds(
    state,
    drawer_width=Globals.DEFAULT_ENVIRONMENT['drawer_width'],
    drawer_depth=Globals.DEFAULT_ENVIRONMENT['drawer_height']
):
    """Determine the bounds of the drawer stack and drawer itself

    Returns:
    xmin, xmax, ymin, ymax, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer
        (xmin, xmax, ymin, ymax) -- bounding box of the drawer stack
        (xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer) -- bounding box of the drawer
    """
    widthAdjustment = (drawer_width - 1)/2
    depthAdjustment = (drawer_depth - 1)/2
    if state.drawer_position.theta == 0:
        xmin = state.drawer_position.x - depthAdjustment
        xmax = state.drawer_position.x + depthAdjustment
        ymin = state.drawer_position.y - widthAdjustment
        ymax = state.drawer_position.y + widthAdjustment
        xminDrawer = xmin + state.drawer_opening
        xmaxDrawer = xmax + state.drawer_opening
        yminDrawer = ymin
        ymaxDrawer = ymax

    elif state.drawer_position.theta == 90:
        xmin = state.drawer_position.x - widthAdjustment
        xmax = state.drawer_position.x + widthAdjustment
        ymin = state.drawer_position.y - depthAdjustment
        ymax = state.drawer_position.y + depthAdjustment
        xminDrawer = xmin
        xmaxDrawer = xmax
        yminDrawer = ymin + state.drawer_opening
        ymaxDrawer = ymax + state.drawer_opening

    elif state.drawer_position.theta == 180:
        xmin = state.drawer_position.x - depthAdjustment
        xmax = state.drawer_position.x + depthAdjustment
        ymin = state.drawer_position.y - widthAdjustment
        ymax = state.drawer_position.y + widthAdjustment
        xminDrawer = xmin - state.drawer_opening
        xmaxDrawer = xmax - state.drawer_opening
        yminDrawer = ymin
        ymaxDrawer = ymax

    else:  # 270
        xmin = state.drawer_position.x - widthAdjustment
        xmax = state.drawer_position.x + widthAdjustment
        ymin = state.drawer_position.y - depthAdjustment
        ymax = state.drawer_position.y + depthAdjustment
        xminDrawer = xmin
        xmaxDrawer = xmax
        yminDrawer = ymin - state.drawer_opening
        ymaxDrawer = ymax - state.drawer_opening

    return int(xmin), int(xmax), int(ymin), int(ymax), int(xminDrawer), int(xmaxDrawer), int(yminDrawer), int(ymaxDrawer)

def get_handle_pos(
    state,
    drawer_depth=Globals.DEFAULT_ENVIRONMENT['drawer_depth'],
    drawer_height=Globals.DEFAULT_ENVIRONMENT['drawer_height']
):
    """Calculate the point corresponding to the handle on the drawer"""
    depthAdjustment = ((drawer_depth - 1)/2)
    handle_point = Point(state.drawer_position.x, state.drawer_position.y, drawer_height)
    offset = depthAdjustment + state.drawer_opening + 1
    if state.drawer_position.theta == 0:
        handle_point.x += offset
    elif state.drawer_position.theta == 90:
        handle_point.y += offset
    elif state.drawer_position.theta == 180:
        handle_point.x -= offset
    else:
        handle_point.y -= offset
    return handle_point

def get_drawer_midpoint_pos(
    state,
    drawer_depth=Globals.DEFAULT_ENVIRONMENT['drawer_depth'],
    drawer_height=Globals.DEFAULT_ENVIRONMENT['drawer_height'],
):
    """Get the midpoint of an open drawer"""
    point = Point(state.drawer_position.x, state.drawer_position.y, drawer_height)
    depthAdjustment = ((drawer_depth - 1)/2)
    offset = depthAdjustment + state.drawer_opening//2 + 1
    if state.drawer_position.theta == 0:
        point.x += offset
    elif state.drawer_position.theta == 90:
        point.y += offset
    elif state.drawer_position.theta == 180:
        point.x -= offset
    else:
        point.y -= offset
    return point

# Collision Helper functions

def gripper_collision(state, position):
    """Detect collision with only the gripper"""
    return on_box_edge(
        position,
        state.gripper_position.x - 1, state.gripper_position.x + 1,
        state.gripper_position.y - 1, state.gripper_position.y + 1,
        state.gripper_position.z, state.gripper_position.z
    )

def object_collision(state, position, ignore=None):
    """Detect collision with any object"""
    for object in state.objects:
        if ignore is not None and object.unique_name == ignore:
            continue
        if object.position == position:
            return object
    return None

def box_collision(
    state, position,
    box_radius=Globals.DEFAULT_ENVIRONMENT['box_radius'],
    box_height=Globals.DEFAULT_ENVIRONMENT['box_height']
):
    """Detect collision with only the box"""
    return on_box_edge(
        position,
        state.box_position.x - box_radius, state.box_position.x + box_radius,
        state.box_position.y - box_radius, state.box_position.y + box_radius,
        state.box_position.z, state.box_position.z + box_height - 1
    )

def lid_collision(
    state, position,
    box_radius=Globals.DEFAULT_ENVIRONMENT['box_radius']
):
    """Detect collision with only the lid"""
    return in_volume(
        position,
        state.lid_position.x - box_radius, state.lid_position.x + box_radius,
        state.lid_position.y - box_radius, state.lid_position.y + box_radius,
        state.lid_position.z, state.lid_position.z
    )

def drawer_collision(
    state, position,
    drawer_width=Globals.DEFAULT_ENVIRONMENT['drawer_width'],
    drawer_depth=Globals.DEFAULT_ENVIRONMENT['drawer_depth'],
    drawer_height=Globals.DEFAULT_ENVIRONMENT['drawer_height']
):
    """Detect collision with only the drawer

    Returns:
    List of collisions as follows:
    [drawer stack collision, drawer bottom collision, drawer edge collision]
    """
    xmin, xmax, ymin, ymax, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer = get_drawer_bounds(state, drawer_width, drawer_depth)

    return [
        in_volume(position, xmin, xmax, ymin, ymax, 0, drawer_height),
        in_volume(position, xminDrawer + 1, xmaxDrawer - 1, yminDrawer + 1, ymaxDrawer - 1, drawer_height - 1, drawer_height - 1),
        on_box_edge(position, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer,
                           drawer_height - 1, drawer_height)
    ]

def environment_collision(
    state, position,
    box_radius=Globals.DEFAULT_ENVIRONMENT['box_radius'],
    box_height=Globals.DEFAULT_ENVIRONMENT['box_height'],
    drawer_width=Globals.DEFAULT_ENVIRONMENT['drawer_width'],
    drawer_depth=Globals.DEFAULT_ENVIRONMENT['drawer_depth'],
    drawer_height=Globals.DEFAULT_ENVIRONMENT['drawer_height']
):
    """Detect collision with either the box, lid, or drawer"""
    return (
        box_collision(state, position, box_radius, box_height)
        or lid_collision(state, position, box_radius)
        or any(drawer_collision(state, position, drawer_width, drawer_depth, drawer_height))
    )

def in_collision(
    state, position,
    box_radius=Globals.DEFAULT_ENVIRONMENT['box_radius'],
    box_height=Globals.DEFAULT_ENVIRONMENT['box_height'],
    drawer_width=Globals.DEFAULT_ENVIRONMENT['drawer_width'],
    drawer_depth=Globals.DEFAULT_ENVIRONMENT['drawer_depth'],
    drawer_height=Globals.DEFAULT_ENVIRONMENT['drawer_height'],
    ignore=None
):
    """Detect collision with anything in the environment (gripper not included, ignores objects in _ignore_)"""
    return (
        object_collision(state, position, ignore)
        or environment_collision(
            state, position, box_radius, box_height,
            drawer_width, drawer_depth, drawer_height
        )
    )

# Touch helper functions

def get_neighbor_count(
    state, position,
    box_radius=Globals.DEFAULT_ENVIRONMENT['box_radius'],
    box_height=Globals.DEFAULT_ENVIRONMENT['box_height'],
    drawer_width=Globals.DEFAULT_ENVIRONMENT['drawer_width'],
    drawer_depth=Globals.DEFAULT_ENVIRONMENT['drawer_depth'],
    drawer_height=Globals.DEFAULT_ENVIRONMENT['drawer_height']
):
    """Get the count of the number of neighbours at a position"""
    count = 0
    in_cont = in_container(state, position)
    for x in range(-1, 2):
        for y in range(-1, 2):
            if x == 0 and y == 0:
                continue
            if in_collision(
                state, Point(position.x + x, position.y + y, position.z),
                box_radius, box_height,
                drawer_width, drawer_depth, drawer_height
            ):
                count += 1
            if not in_cont and in_container(state, Point(position.x + x, position.y + y, position.z)):
                    count += 1
    return count

def in_container(state, pos, ignore=None):
        for c in state.containers:
            if ignore is not None and c.unique_name in ignore:
                continue
            if in_volume(pos, c.position.x, c.position.x + c.width - 1, c.position.y, c.position.y + c.height - 1,
                             c.position.z, c.position.z):
                return True
        return False

def is_touching_box(
    state, position,
    box_radius=Globals.DEFAULT_ENVIRONMENT['box_radius'],
    box_height=Globals.DEFAULT_ENVIRONMENT['box_height']
):
    return any([
        box_collision(state, get_translated_pose2D(position, translation), box_radius, box_height)
        for translation in Globals.TOUCH_SEARCH_OFFSETS
        if translation.x != 0 or translation.y != 0
    ])

def is_touching_lid(
    state, position,
    box_radius=Globals.DEFAULT_ENVIRONMENT['box_radius'],
    box_height=Globals.DEFAULT_ENVIRONMENT['box_height']
):
    return any([
        lid_collision(state, get_translated_pose2D(position, translation), box_radius)
        for translation in Globals.TOUCH_SEARCH_OFFSETS
        if translation.x != 0 or translation.y != 0
    ])

def is_touching_stack(
    state, position,
    drawer_width=Globals.DEFAULT_ENVIRONMENT['drawer_width'],
    drawer_depth=Globals.DEFAULT_ENVIRONMENT['drawer_depth'],
    drawer_height=Globals.DEFAULT_ENVIRONMENT['drawer_height']
):
    return any([
        drawer_collision(
            state, get_translated_pose2D(position, translation),
            drawer_width, drawer_depth, drawer_height
        )[0]
        for translation in Globals.TOUCH_SEARCH_OFFSETS
        if translation.x != 0 and translation.y != 0
    ])

def is_touching_drawer(
    state, position,
    drawer_width=Globals.DEFAULT_ENVIRONMENT['drawer_width'],
    drawer_depth=Globals.DEFAULT_ENVIRONMENT['drawer_depth'],
    drawer_height=Globals.DEFAULT_ENVIRONMENT['drawer_height']
):
    return any([
        any(drawer_collision(
            state, get_translated_pose2D(position, translation),
            drawer_width, drawer_depth, drawer_height
        ))
        for translation in Globals.TOUCH_SEARCH_OFFSETS
        if translation.x != 0 and translation.y != 0
    ])

def is_gripper_touching(
    state, target,
    box_radius=Globals.DEFAULT_ENVIRONMENT['box_radius'],
    box_height=Globals.DEFAULT_ENVIRONMENT['box_height'],
    drawer_width=Globals.DEFAULT_ENVIRONMENT['drawer_width'],
    drawer_depth=Globals.DEFAULT_ENVIRONMENT['drawer_depth'],
    drawer_height=Globals.DEFAULT_ENVIRONMENT['drawer_height']
):
    """Check if the gripper is touching the target object"""
    gripper_extremes = [
        get_translated_pose2D(state.gripper_position, translation)
        for translation in Globals.TOUCH_SEARCH_OFFSETS
        if abs(translation.x) == abs(translation.y)
    ]
    if target == 'stack':
        return any([
            is_touching_stack(state, pos, drawer_width, drawer_depth, drawer_height)
            for pos in gripper_extremes
        ])
    elif target == 'drawer':
        return any([
            is_touching_drawer(state, pos, drawer_width, drawer_depth, drawer_height)
            for pos in gripper_extremes
        ])
    elif target == 'box':
        return any([
            is_touching_box(state, pos, box_radius, box_height)
            for pos in gripper_extremes
        ])
    elif target == 'lid':
        return any([
            is_touching_lid(state, pos, box_radius, box_height)
            for pos in gripper_extremes
        ])
    else: # Some object
        obj_position = get_object_by_name(state, target).position
        return any([
            gripper_collision(state, get_translated_pose2D(obj_position, translation))
            for translation in Globals.TOUCH_SEARCH_OFFSETS
            if translation.x != 0 or translation.y != 0
        ])

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
            if frame.lower() == object.unique_name.lower():
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
            if frame.lower() == object.unique_name.lower():
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
            if frame.lower() == object.unique_name.lower():
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
            frame = object.unique_name
    # drawer stack
    dst = euclidean_3D(position, Point(state.drawer_position.x, state.drawer_position.y, 0))
    if dst < min_dst:
        min_dst = dst
        frame = 'stack'

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
        frame = 'drawer'

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
        frame = 'handle'

    # box
    dst = euclidean_3D(position, state.box_position)
    if dst < min_dst:
        min_dst = dst
        frame = 'box'

    # lid
    dst = euclidean_3D(position, state.lid_position)
    if dst < min_dst:
        min_dst = dst
        frame = 'lid'

    # gripper
    dst = euclidean_3D(position, state.gripper_position)
    if dst < min_dst:
        min_dst = dst
        frame = 'gripper'

    return frame

def get_task_frame(state, position):
    """Get a task-related frame (e.g. drawer, lid, table, etc.) for actions such as place"""

    # drawer
    if state.drawer_position.theta == 0:
        if (position.x >= state.drawer_position.x - 3 and position.x <= state.drawer_position.x + 3
            and position.y >= state.drawer_position.y - 2 and position.y <= state.drawer_position.y + 2):
            return 'stack'
        elif (position.x >= state.drawer_position.x + state.drawer_opening - 3
              and position.x <= state.drawer_position.x + state.drawer_opening + 3
              and position.y >= state.drawer_position.y - 2 and position.y <= state.drawer_position.y + 2):
            return 'drawer'
    elif state.drawer_position.theta == 90:
        if (position.x >= state.drawer_position.x - 2 and position.x <= state.drawer_position.x + 2
            and position.y >= state.drawer_position.y - 3 and position.y <= state.drawer_position.y + 3):
            return 'stack'
        elif (position.x >= state.drawer_position.x - 2 and position.x <= state.drawer_position.x + 2
              and position.y >= state.drawer_position.y + state.drawer_opening - 3
              and position.y <= state.drawer_position.y + state.drawer_opening + 3):
            return 'drawer'
    elif state.drawer_position.theta == 180:
        if (position.x >= state.drawer_position.x - 3 and position.x <= state.drawer_position.x + 3
            and position.y >= state.drawer_position.y - 2 and position.y <= state.drawer_position.y + 2):
            return 'stack'
        elif (position.x >= state.drawer_position.x - state.drawer_opening - 3
              and position.x <= state.drawer_position.x - state.drawer_opening + 3
              and position.y >= state.drawer_position.y - 2 and position.y <= state.drawer_position.y + 2):
            return 'drawer'
    elif state.drawer_position.theta == 270:
        if (position.x >= state.drawer_position.x - 2 and position.x <= state.drawer_position.x + 2
            and position.y >= state.drawer_position.y - 3 and position.y <= state.drawer_position.y + 3):
            return 'stack'
        elif (position.x >= state.drawer_position.x - 2 and position.x <= state.drawer_position.x + 2
              and position.y >= state.drawer_position.y - state.drawer_opening - 3
              and position.y <= state.drawer_position.y - state.drawer_opening + 3):
            return 'drawer'

    # box and lid
    if (position.x >= state.lid_position.x - 2 and position.x <= state.lid_position.x + 2
        and position.y >= state.lid_position.y - 2 and position.y <= state.lid_position.y + 2):
        return 'lid'
    elif (position.x >= state.box_position.x - 2 and position.x <= state.box_position.x + 2
        and position.y >= state.box_position.y - 2 and position.y <= state.box_position.y + 2):
        return 'box'

    # default
    return 'table'


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
    FIXME: This does not work with containers

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

    :history_buffer: The length of history to include in the feature vector
    :feature_type: Cast the data into this type. Ignored if retrurn_scaled
    :return_dict: Return the state vector as a dictionary of k,v pairs. Also
                    returns the list of all the keys
    """
    # Helper function
    def set_feature(key, value):
        semantic_state_keys.append(key)
        semantic_state[key] = value

    # State variables
    semantic_state_keys = []
    semantic_state = {}

    # Setup the helper variables
    objects = [obj.unique_name.lower() for obj in state.objects]

    # First, get the properties of the objects one by one
    for idx, obj_name in enumerate(objects):
        obj = get_object_by_name(state, obj_name)

        # Directly from the state - in_box, in_drawer, on_lid, on_stack, not_visible
        set_feature("{}_in_box".format(obj_name), obj.in_box)
        set_feature("{}_in_drawer".format(obj_name), obj.in_drawer)
        set_feature("{}_on_lid".format(obj_name), obj.on_lid)
        set_feature("{}_on_stack".format(obj_name), obj.on_stack)
        set_feature("{}_not_visible".format(obj_name), obj.occluded or obj.lost)

        # Near edge
        set_feature(
            "{}_near_edge".format(obj_name), is_position_near_edge(obj.position)
        )

        # Touching
        set_feature("touching_{}_x_box".format(obj_name), is_touching_box(state, obj.position))
        set_feature("touching_{}_x_lid".format(obj_name), is_touching_lid(state, obj.position))
        set_feature("touching_{}_x_stack".format(obj_name), is_touching_stack(state, obj.position))

        # Object touching objects
        for j in xrange(idx+1, len(objects)):
            other_obj = get_object_by_name(state, objects[j])
            set_feature(
                "touching_{}_x_{}".format(obj_name, objects[j]),
                is_adjacent(obj.position, other_obj.position)
            )

    # Get container open
    set_feature(
        "box_open",
        (
            abs(state.box_position.x - state.lid_position.x) > 1)
            and (abs(state.lid_position.y - state.lid_position.y) > 1
        )
    )
    set_feature("drawer_open", state.drawer_opening > 1)

    # Gripper Semantics
    set_feature("gripper_open", state.gripper_open)
    set_feature("object_in_gripper", name_to_int(state.object_in_gripper))
    set_feature("gripper_near_edge", is_position_near_edge(state.gripper_position))

    # Relative positions and touching
    for obj_name in Globals.OBJECT_TO_INT_MAP.iterkeys():
        if not obj_name or obj_name == 'gripper':
            continue

        # Get the different position values
        obj_position = None
        if obj_name in objects:
            obj_position = get_object_by_name(state, obj_name).position
        elif obj_name == 'stack':
            obj_position = Point(state.drawer_position.x, state.drawer_position.y, 0)
        elif obj_name == 'drawer':
            obj_position = get_drawer_midpoint_pos(state)
        elif obj_name == 'handle':
            obj_position = get_handle_pos(state)
        elif obj_name == 'box':
            obj_position = state.box_position
        elif obj_name == 'lid':
            obj_position = state.lid_position

        # Relative Height
        set_feature(
            "relative_h_{}".format(obj_name),
            get_relative_position_ternary(state.gripper_position.z, obj_position.z)
        )
        set_feature(
            "relative_x_{}".format(obj_name),
            get_relative_position_ternary(state.gripper_position.x, obj_position.x)
        )
        set_feature(
            "relative_y_{}".format(obj_name),
            get_relative_position_ternary(state.gripper_position.y, obj_position.y)
        )

        # Touching. We don't care about handle
        if obj_name == 'handle':
            continue
        set_feature("touching_gripper_{}".format(obj_name), is_gripper_touching(state, obj_name))

    # Lid is near the edge
    lid_near_edge = False
    for dx in [-Globals.DEFAULT_ENVIRONMENT['box_radius'],Globals.DEFAULT_ENVIRONMENT['box_radius']]:
        for dy in [-Globals.DEFAULT_ENVIRONMENT['box_radius'],Globals.DEFAULT_ENVIRONMENT['box_radius']]:
            lid_near_edge = (
                lid_near_edge
                or is_position_near_edge(
                    Point(state.lid_position.x+dx, state.lid_position.y, 0)
                )
            )
    set_feature("lid_near_edge", lid_near_edge)

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

def readable_state(semantic_state):
    s = ''
    for k in semantic_state:
        s += k + ': ' + str(semantic_state[k]) + '\n'
    return s

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
            # Finally, do a safety check for the limits if there is an object in
            # the gripper
            if state.object_in_gripper and is_position_near_edge(position):
                continue

            # All checks have passed. Move/Place at the location
            break
    else:
        for o in state.objects:
            if target.lower() == o.unique_name.lower():
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
    for object_key in Globals.OBJECT_TO_INT_MAP.iterkeys():
        if not object_key: # Table/Empty object name
            continue

        # Do not allow a grasp on 'handle' because it's the same as 'drawer'
        if object_key == 'handle':
            continue

        # Allow the box as a candidate, this should fail
        action_candidates.append(
            (Action.GRASP, to_object_name(object_key), None)
        )

    # Move, Place
    for object_key in Globals.OBJECT_TO_INT_MAP.iterkeys():
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
    """
    Given a state, this returns the possible semantic action candidates
    that can be used by `semantic_action_to_position()`. Parameters are
    (action, object, target)
    """
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
    for object_key in Globals.OBJECT_TO_INT_MAP.iterkeys():
        if not object_key: # Table/Empty object name
            continue

        # Do not allow a grasp on 'handle' because it's the same as 'drawer'
        if object_key == 'handle':
            continue

        # Allow the box as a candidate, this should fail
        action_candidates.append(
            (Action.GRASP, to_object_name(object_key), None)
        )

    # Move, Place
    for object_key in Globals.OBJECT_TO_INT_MAP.iterkeys():
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
        action.object = to_object_name(
            action_obj_offset[1]
            if type(action_obj_offset[1]) != int
            else int_to_name(action_obj_offset[1])
        )

    # Then, if we have to move or place, translate the offset
    if action.action_type in [Action.MOVE_ARM, Action.PLACE]:
        obj_name = to_object_name(
            action_obj_offset[1]
            if type(action_obj_offset[1]) != int
            else int_to_name(action_obj_offset[1])
        )

        # First check to see if this is one of the graspable objects
        found_pos = None
        for state_obj in state.objects:
            if obj_name == state_obj.unique_name:
                found_pos = state_obj.position
                break

        # If the object was not found, check stack, drawer, handle, box,
        # lid, and gripper
        if found_pos is None:
            if obj_name == 'stack':
                found_pos = state.drawer_position
            elif obj_name == 'drawer':
                found_pos = get_drawer_midpoint_pos(state)
            elif obj_name == 'handle':
                found_pos = get_handle_pos(state)
            elif obj_name == 'box':
                found_pos = state.box_position
            elif obj_name == 'lid':
                found_pos = state.lid_position
            elif obj_name == 'gripper':
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
        action.object = to_object_name(
            semantic_action[1]
            if type(semantic_action[1]) != int
            else int_to_name(semantic_action[1])
        )

    # Use the semantic_action_to_position to get a position for MOVE, PLACE
    if action.action_type in [Action.MOVE_ARM, Action.PLACE]:
        position = semantic_action_to_position(
            state,
            to_object_name(
                semantic_action[2]
                if type(semantic_action[2]) != int
                else int_to_name(semantic_action[1])
            )
        )
        action.position.x = position.x
        action.position.y = position.y

    return action
