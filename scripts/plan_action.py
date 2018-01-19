#!/usr/bin/env python

# Python
from math import sqrt

# ROS
import copy
from geometry_msgs.msg import Point
from task_sim.msg import Action

from data_utils import DataUtils

class PlanAction():

    def __init__(self, s0, a, s1):
        self.action = a.action_type

        # extract object and target from action message
        if a.action_type == Action.GRASP:
            self.object = a.object
            self.target = None
        elif a.action_type == Action.PLACE:
            if s0.object_in_gripper is not None and s0.object_in_gripper != '':
                self.object = s0.object_in_gripper
            self.target = DataUtils.get_task_frame(s0, a.position)
            if self.target.lower() == 'table':
                handle_pos = DataUtils.get_handle_pos(s0)
                if a.position.x == handle_pos.x and a.position.y == handle_pos.y:
                    self.target = 'Handle'
                else:
                    for o in s0.objects:
                        if a.position.x == o.position.x and a.position.y == o.position.y:
                            self.target = o.name
                            break
        elif a.action_type == Action.MOVE_ARM:
            if s0.object_in_gripper is not None and s0.object_in_gripper != '':
                self.object = s0.object_in_gripper
            else:
                self.object = None
            self.target = DataUtils.get_task_frame(s0, a.position)
            # check for objects as targets
            if self.target.lower() == 'table':
                handle_pos = DataUtils.get_handle_pos(s0)
                if a.position.x == handle_pos.x and a.position.y == handle_pos.y:
                    self.target = 'Handle'
                else:
                    for o in s0.objects:
                        if a.position.x == o.position.x and a.position.y == o.position.y:
                            self.target = o.name
                            break
            # check if target was to touch something
            if self.target.lower() == 'table':
                pos = copy.copy(a.position)
                if pos.x > 0:
                    pos.x += 1
                elif pos.x < 0:
                    pos.x -= 1
                if pos.y > 0:
                    pos.y += 1
                elif pos.y < 0:
                    pos.y -= 1
                extended_target = DataUtils.get_task_frame(s0, pos)
                if extended_target.lower() != 'table':
                    self.target = extended_target
        elif a.action_type in [Action.OPEN_GRIPPER, Action.RAISE_ARM, Action.LOWER_ARM, Action.RESET_ARM]:
            if s0.object_in_gripper is not None and s0.object_in_gripper != '':
                self.object = s0.object_in_gripper
            else:
                self.object = None
            self.target = None
        elif a.action_type == Action.CLOSE_GRIPPER:
            if s1.object_in_gripper is not None and s1.object_in_gripper != '':
                self.object = s1.object_in_gripper
            else:
                self.object = None
            self.target = None
        else:
            self.object = None
            self.target = None


        # some things we'll need to calculate preconditions and effects...
        obj0 = None
        if self.object is not None:
            for o in s0.objects:
                if o.name == self.object:
                    obj0 = o
                    break

        dst = sqrt(pow(s0.lid_position.x - s0.box_position.x, 2) + pow(s0.lid_position.y - s0.box_position.y, 2))
        if dst < 2:
            box_open_pre = False
        else:
            box_open_pre = True
        dst = sqrt(pow(s1.lid_position.x - s1.box_position.x, 2) + pow(s1.lid_position.y - s1.box_position.y, 2))
        if dst < 2:
            box_open_post = False
        else:
            box_open_post = True

        if s0.drawer_opening < 2:
            drawer_open_pre = False
        else:
            drawer_open_pre = True
        if s1.drawer_opening < 2:
            drawer_open_post = False
        else:
            drawer_open_post = True


        # preconditions
        if obj0 is not None:
            self.object_in_drawer = obj0.in_drawer
            self.object_in_box = obj0.in_box
            self.object_on_lid = obj0.on_lid
        else:
            self.object_in_drawer = False
            self.object_in_box = False
            self.object_on_lid = False

        self.object_open = False
        if self.object is not None:
            if self.object.lower() == 'drawer':
                self.object_open = drawer_open_pre
            elif self.object.lower() == 'lid':
                self.object_open = box_open_pre

        self.gripper_open = s0.gripper_open
        self.object_in_gripper = s0.object_in_gripper

        if self.target is not None:
            if self.target.lower() == 'box':
                self.target_open = box_open_pre
            elif self.target.lower() == 'drawer' or self.target.lower() == 'handle':
                self.target_open = drawer_open_pre
            else:
                self.target_open = True

        # effects
        self.effects = dict()
        # small object effects
        for o0 in s0.objects:
            for o1 in s1.objects:
                if o0.name == o1.name:
                    state_change = False
                    if o0.in_drawer != o1.in_drawer:
                        if o1.in_drawer:
                            self.add_effect('in_drawer', o1.name)
                        else:
                            self.add_effect('not_in_drawer', o1.name)
                        state_change = True
                    if o0.in_box != o1.in_box:
                        if o1.in_box:
                            self.add_effect('in_box', o1.name)
                        else:
                            self.add_effect('not_in_box', o1.name)
                        state_change = True
                    if o0.on_lid != o1.on_lid:
                        if o1.on_lid:
                            self.add_effect('on_lid', o1.name)
                        else:
                            self.add_effect('not_on_lid', o1.name)
                        state_change = True
                    if not state_change:
                        if o0.position != o1.position:
                            self.add_effect('moved', o1.name)

        # large object effects
        box_state_change = False
        if box_open_pre != box_open_post:
            box_state_change = True
            if box_open_post:
                self.add_effect('open', 'Box')
            else:
                self.add_effect('closed', 'Box')
        if not box_state_change:
            if s0.lid_position != s1.lid_position:
                self.add_effect('moved', 'Lid')

        drawer_state_change = False
        if drawer_open_pre != drawer_open_post:
            drawer_state_change = True
            if drawer_open_post:
                self.add_effect('open', 'Drawer')
            else:
                self.add_effect('closed', 'Drawer')
        if not drawer_state_change:
            if s0.drawer_opening != s1.drawer_opening:
                self.add_effect('moved', 'Drawer')
                self.add_effect('moved', 'Handle')

        # gripper effects
        if s0.object_in_gripper != s1.object_in_gripper:
            if s1.object_in_gripper is None or s1.object_in_gripper == '':
                self.add_effect('object_in_gripper', '')
            else:
                self.add_effect('object_in_gripper', s1.object_in_gripper)
        if s0.gripper_open != s1.gripper_open:
            if s1.gripper_open:
                self.add_effect('change_gripper', 'Open')
            else:
                self.add_effect('change_gripper', 'Closed')


    def __str__(self):
        s = ''
        s += 'action: ' + str(self.action) + '\n' \
             + 'object: ' + str(self.object) + '\n' \
             + 'target: ' + str(self.target) + '\n' \
             + 'preconditions:' + '\n' \
             + '\tobject_in_drawer: ' + str(self.object_in_drawer) + '\n' \
             + '\tobject_in_box: ' + str(self.object_in_box) + '\n' \
             + '\tobject_on_lid: ' + str(self.object_on_lid) + '\n' \
             + '\tobject_open: ' + str(self.object_open) + '\n' \
             + '\tgripper_open: ' + str(self.gripper_open) + '\n' \
             + '\tobject_in_gripper: ' + str(self.object_in_gripper) + '\n' \
             + 'effects:' + '\n'
        if len(self.effects.keys()) == 0:
            s += '\tNone'
        else:
            for key in self.effects.keys():
                s += '\t' + key + ': ' + str(self.effects[key]) + '\n'
        return s


    def add_effect(self, key, value):
        if self.effects.has_key(key):
            self.effects[key].append(value)
        else:
            self.effects[key] = [value]