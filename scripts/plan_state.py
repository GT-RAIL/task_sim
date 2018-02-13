#!/usr/bin/env python

from copy import deepcopy
from math import sqrt

from plan_action import PlanAction

class PlanState:
    def __init__(self, state):
        # Objects
        self.objects = {}
        for o in state.objects:
            obj = PlanObject(o)
            self.objects[obj.name] = obj

        # Containers
        dst = sqrt(pow(state.lid_position.x - state.box_position.x, 2)
                   + pow(state.lid_position.y - state.box_position.y, 2))
        if dst < 2:
            box_open = False
        else:
            box_open = True
        if state.drawer_opening < 2:
            drawer_open = False
        else:
            drawer_open = True
        self.containers = {}
        container_list = [PlanContainer('handle', drawer_open), PlanContainer('drawer', drawer_open),
                          PlanContainer('stack', drawer_open), PlanContainer('box', box_open),
                          PlanContainer('lid', box_open)]
        for c in container_list:
            self.containers[c.name] = c

        # Gripper
        self.gripper_open = state.gripper_open
        self.object_in_gripper = state.object_in_gripper.lower()

    def __str__(self):
        s = ''
        s += 'objects:\n' + str(self.objects)
        s += '\ncontainers:\n' + str(self.containers)
        s += '\ngripper_open: ' + str(self.gripper_open) + '\nobject_in_gripper: ' + str(self.object_in_gripper)
        return s

    def __repr__(self):
        return str(self)

    def check_action(self, action):
        """Check preconditions of an action, where action is a PlanAction"""
        if action.object in self.objects:
            if action.object_in_drawer != self.objects[action.object].in_drawer \
                or action.object_in_box != self.objects[action.object].in_box \
                or action.object_on_lid != self.objects[action.object].on_lid:
                return False
        if action.object in self.containers:
            if action.object_open != self.containers[action.object].open:
                return False
        if action.target in self.containers:
            if action.target_open != self.containers[action.target].open:
                return False
        if action.gripper_open != self.gripper_open:
            return False
        if action.object_in_gripper != self.object_in_gripper:
            return False
        return True

    def apply_action(self, action):
        # return a new state based on effects of taking the action
        result = deepcopy(self)

        drawers = ['drawer', 'handle', 'stack']
        boxes = ['box', 'lid']

        for key, effect in action.effects.iteritems():
            if key == 'in_drawer':
                for o in effect:
                    if o in result.objects:
                        result.objects[o].in_drawer = True
            elif key == 'not_in_drawer':
                for o in effect:
                    if o in result.objects:
                        result.objects[o].in_drawer = False
            elif key == 'in_box':
                for o in effect:
                    if o in result.objects:
                        result.objects[o].in_box = True
            elif key == 'not_in_box':
                for o in effect:
                    if o in result.objects:
                        result.objects[o].in_box = False
            elif key == 'on_lid':
                for o in effect:
                    if o in result.objects:
                        result.objects[o].on_lid = True
            elif key == 'not_on_lid':
                for o in effect:
                    if o in result.objects:
                        result.objects[o].on_lid = False
            elif key == 'open':
                for o in effect:
                    if o in result.containers:
                        result.containers[o].open = True
                        # special cases for object groups
                        if o in drawers:
                            for c in drawers:
                                result.containers[c].open = True
                        if o in boxes:
                            for c in boxes:
                                result.containers[c].open = True
            elif key == 'closed':
                for o in effect:
                    if o in result.containers:
                        result.containers[o].open = False
                        # special cases for object groups
                        if o in drawers:
                            for c in drawers:
                                result.containers[c].open = False
                        if o in boxes:
                            for c in boxes:
                                result.containers[c].open = False
            elif key == 'object_in_gripper':
                result.object_in_gripper = ''
                for o in effect:
                    result.object_in_gripper = o
            elif key == 'change_gripper':
                for e in effect:
                    if e == 'open':
                        result.gripper_open = True
                    elif e == 'closed':
                        result.gripper_open = False

        return result


class PlanObject:
    def __init__(self, o):
        self.name = o.name.lower()
        self.in_drawer = o.in_drawer
        self.in_box = o.in_box
        self.on_lid = o.on_lid

    def __str__(self):
        s = ''
        s += 'name: ' + str(self.name) + '\n' \
             + 'in_drawer: ' + str(self.in_drawer) + '\n' \
             + 'in_box: ' + str(self.in_box) + '\n' \
             + 'on_lid: ' + str(self.on_lid) + '\n'
        return s

    def __repr__(self):
        return str(self)

class PlanContainer:
    def __init__(self, name, open):
        self.name = name
        self.open = open

    def __str__(self):
        s = ''
        s += 'name: ' + str(self.name) + '\n' \
             + 'open: ' + str(self.open) + '\n'
        return s

    def __repr__(self):
        return str(self)