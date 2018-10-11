#!/usr/bin/env python

from copy import deepcopy


def get_object(state, unique_name):

    if unique_name in state.boxes:
        return state.containers[unique_name]

    if unique_name in state.containers:
        return state.containers[unique_name]

    if unique_name in state.drawers:
        return state.drawers[unique_name]

    if unique_name in state.grippers:
        return state.grippers[unique_name]

    if unique_name in state.items:
        return state.items[unique_name]

    if unique_name in state.lids:
        return state.lids[unique_name]

    if unique_name in state.stacks:
        return state.stacks[unique_name]

    return None

def ray_trace(state, x, y):
    z = 0
    for s in state.stacks:
        w = (s.width - 1)/2
        h = (s.depth - 1)/2
        if s.x - w <= x <= s.x + w and s.y - h <= y <= s.y + h:
            z = 3
            break
    for d in state.drawers:
        w = (d.width - 1)/2
        h = (d.depth - 1)/2
        if d.x - w <= x <= d.x + w and d.y - h <= y <= d.y + h and z < 2:
            z = 2
            break
    for l in state.lids:
        if l.x - l.radius <= x <= l.x + l.radius and l.y - l.radius <= y <= l.y + l.radius and z < l.z:
            z = l.z
    for c in state.containers:
        if c.x <= x < c.x + c.width and c.y <= y < c.y + c.depth and z < c.z:
            z = c.z
    return z

def gripper_object(state):
    gripper = state.grippers[state.grippers[0]]
    for o in state.items:
        if o.name == gripper.holding:
            if o.x == gripper.x and o.y == gripper.y and o.z == gripper.z:
                return o
    for l in state.lids:
        if l.name == gripper.holding:
            if l.x == gripper.x and l.y == gripper.y and l.z == gripper.z:
                return l
    for d in state.drawer:
        if d.name == gripper.holding:
            if d.x + (d.width - 1)/2 + 1 == gripper.x and d.y == gripper.y:
                return d
    for c in state.containers:
        if c.name == gripper.holding:
            if c.z == gripper.z and c.x <= gripper.x < c.x + c.width and c.y <= gripper.y < c.y + c.depth:
                return c
    return None

def copy_state_move_object(state, obj_name, dx, dy, dz):
    result = deepcopy(state)
    obj = get_object(result, obj_name)
    obj.x += dx
    obj.y += dy
    obj.z += dz
    return result
