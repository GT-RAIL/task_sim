#!/usr/bin/env python

# Python
from math import sqrt

# ROS
import copy
from task_sim.msg import Action

from data_utils import DataUtils

class PlanAction:

    def __init__(self, s0, a, s1=None):
        self.action = a.action_type

        # extract object and target from action message
        if a.action_type == Action.GRASP:
            self.object = a.object.lower()
            self.target = None
        elif a.action_type == Action.PLACE:
            if s0.object_in_gripper is not None and s0.object_in_gripper != '':
                self.object = s0.object_in_gripper.lower()
            self.target = DataUtils.get_task_frame(s0, a.position).lower()
            if self.target == 'table':
                handle_pos = DataUtils.get_handle_pos(s0)
                if a.position.x == handle_pos.x and a.position.y == handle_pos.y:
                    self.target = 'handle'
                else:
                    for o in s0.objects:
                        if a.position.x == o.position.x and a.position.y == o.position.y:
                            self.target = o.name.lower()
                            break
        elif a.action_type == Action.MOVE_ARM:
            if s0.object_in_gripper is not None and s0.object_in_gripper != '':
                self.object = s0.object_in_gripper.lower()
            else:
                self.object = None
            self.target = DataUtils.get_task_frame(s0, a.position).lower()
            # check for objects as targets
            if self.target == 'table':
                handle_pos = DataUtils.get_handle_pos(s0)
                if a.position.x == handle_pos.x and a.position.y == handle_pos.y:
                    self.target = 'handle'
                else:
                    for o in s0.objects:
                        if a.position.x == o.position.x and a.position.y == o.position.y:
                            self.target = o.name.lower()
                            break
            # check if target was to touch something
            if self.target == 'table':
                pos = copy.copy(a.position)
                if pos.x > 0:
                    pos.x += 1
                elif pos.x < 0:
                    pos.x -= 1
                if pos.y > 0:
                    pos.y += 1
                elif pos.y < 0:
                    pos.y -= 1
                extended_target = DataUtils.get_task_frame(s0, pos).lower()
                if extended_target != 'table':
                    self.target = extended_target
        elif a.action_type in [Action.OPEN_GRIPPER, Action.RAISE_ARM, Action.LOWER_ARM, Action.RESET_ARM]:
            if s0.object_in_gripper is not None and s0.object_in_gripper != '':
                self.object = s0.object_in_gripper.lower()
            else:
                self.object = None
            self.target = None
        elif a.action_type == Action.CLOSE_GRIPPER:
            self.object = None
            if s1 is not None:
                if s1.object_in_gripper is not None and s1.object_in_gripper != '':
                    self.object = s1.object_in_gripper.lower()
            else:
                if DataUtils.get_handle_pos(s0) == s0.gripper_position:
                    self.object = 'drawer'
                else:
                    for obj in s0.objects:
                        if obj.position == s0.gripper_position:
                            self.object = s0.obj.name.lower()
                            break
            self.target = None
        else:
            self.object = None
            self.target = None


        # some things we'll need to calculate preconditions and effects...
        obj0 = None
        if self.object is not None:
            for o in s0.objects:
                if o.name.lower() == self.object:
                    obj0 = o
                    break

        dst = sqrt(pow(s0.lid_position.x - s0.box_position.x, 2) + pow(s0.lid_position.y - s0.box_position.y, 2))
        if dst < 2:
            box_open_pre = False
        else:
            box_open_pre = True
        if s1 is not None:
            dst = sqrt(pow(s1.lid_position.x - s1.box_position.x, 2) + pow(s1.lid_position.y - s1.box_position.y, 2))
            if dst < 2:
                box_open_post = False
            else:
                box_open_post = True

        if s0.drawer_opening < 2:
            drawer_open_pre = False
        else:
            drawer_open_pre = True
        if s1 is not None:
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
            if self.object.lower() in ['drawer', 'stack', 'handle']:
                self.object_open = drawer_open_pre
            elif self.object.lower() in ['lid', 'box']:
                self.object_open = box_open_pre

        self.gripper_open = s0.gripper_open
        self.object_in_gripper = s0.object_in_gripper.lower()

        if self.target is not None:
            if self.target.lower() == 'box' or self.target.lower() == 'lid':
                self.target_open = box_open_pre
            elif self.target.lower() == 'drawer' or self.target.lower() == 'handle' or self.target.lower() == 'stack':
                self.target_open = drawer_open_pre
            else:
                self.target_open = True
        else:
            self.target_open = True

        # effects
        self.effects = dict()
        if s1 is not None:
            # small object effects
            for o0 in s0.objects:
                for o1 in s1.objects:
                    if o0.name.lower() == o1.name.lower():
                        state_change = False
                        if o0.in_drawer != o1.in_drawer:
                            if o1.in_drawer:
                                self.add_effect('in_drawer', o1.name.lower())
                            else:
                                self.add_effect('not_in_drawer', o1.name.lower())
                            state_change = True
                        if o0.in_box != o1.in_box:
                            if o1.in_box:
                                self.add_effect('in_box', o1.name.lower())
                            else:
                                self.add_effect('not_in_box', o1.name.lower())
                            state_change = True
                        if o0.on_lid != o1.on_lid:
                            if o1.on_lid:
                                self.add_effect('on_lid', o1.name.lower())
                            else:
                                self.add_effect('not_on_lid', o1.name.lower())
                            state_change = True
                        if not state_change:
                            if o0.position != o1.position:
                                self.add_effect('moved', o1.name.lower())

            # large object effects
            box_state_change = False
            if box_open_pre != box_open_post:
                box_state_change = True
                if box_open_post:
                    self.add_effect('open', 'box')
                else:
                    self.add_effect('closed', 'box')
            if not box_state_change:
                if s0.lid_position != s1.lid_position:
                    self.add_effect('moved', 'lid')

            drawer_state_change = False
            if drawer_open_pre != drawer_open_post:
                drawer_state_change = True
                if drawer_open_post:
                    self.add_effect('open', 'drawer')
                else:
                    self.add_effect('closed', 'drawer')
            if not drawer_state_change:
                if s0.drawer_opening != s1.drawer_opening:
                    self.add_effect('moved', 'drawer')
                    self.add_effect('moved', 'handle')

            # gripper effects
            if s0.object_in_gripper != s1.object_in_gripper:
                if s1.object_in_gripper is None or s1.object_in_gripper == '':
                    self.add_effect('object_in_gripper', '')
                else:
                    self.add_effect('object_in_gripper', s1.object_in_gripper.lower())
            if s0.gripper_open != s1.gripper_open:
                if s1.gripper_open:
                    self.add_effect('change_gripper', 'open')
                else:
                    self.add_effect('change_gripper', 'closed')


    def check_preconditions(self, state, obj, target, object_to_cluster = None):
        if obj not in ['drawer', 'stack', 'handle', 'box', 'lid']:
            for o in state.objects:
                if o.name.lower() == obj:
                    if (self.object_in_drawer == o.in_drawer and self.object_in_box == o.in_box
                        and self.object_on_lid == o.on_lid):
                        break
                    else:
                        return False

        box_open = sqrt(pow(state.lid_position.x - state.box_position.x, 2)
                       + pow(state.lid_position.y - state.box_position.y, 2)) >= 2
        drawer_open = state.drawer_opening >= 2
        if obj in ['drawer', 'stack', 'handle']:
            if drawer_open != self.object_open:
                return False
        elif obj in ['box', 'lid']:
            if box_open != self.object_open:
                return False
        if target in ['drawer', 'stack', 'handle']:
            if drawer_open != self.target_open:
                return False
        elif target in ['box', 'lid']:
            if box_open != self.target_open:
                return False

        if self.gripper_open != state.gripper_open:
            return False

        if object_to_cluster is not None:
            obj = state.object_in_gripper.lower()
            if object_to_cluster.has_key(obj):
                obj = object_to_cluster[obj]
            if self.object_in_gripper != obj:
                return False

        return True

    def check_effects(self, state, object_to_cluster = None):
        box_open = sqrt(pow(state.lid_position.x - state.box_position.x, 2)
                        + pow(state.lid_position.y - state.box_position.y, 2)) >= 2
        drawer_open = state.drawer_opening >= 2

        drawer_name = self.to_cluster('drawer', object_to_cluster)
        box_name = self.to_cluster('box', object_to_cluster)

        for key in self.effects:
            if key == 'open':
                for obj in self.effects[key]:
                    if obj == drawer_name and obj == box_name:
                        if not (drawer_open or box_open):
                            return False
                    elif obj == drawer_name:
                        if not drawer_open:
                            return False
                    elif obj == box_name:
                        if not box_open:
                            return False
            elif key == 'closed':
                for obj in self.effects[key]:
                    if obj == drawer_name and obj == box_name:
                        if drawer_open or box_open:
                            return False
                    elif obj == 'drawer':
                        if drawer_open:
                            return False
                    elif obj == 'box':
                        if box_open:
                            return False

            elif key == 'object_in_gripper':
                obj = self.to_cluster(state.object_in_gripper.lower(), object_to_cluster)
                if obj != self.effects[key][0]:
                    return False

            elif key == 'change_gripper':
                if self.effects[key][0] == 'open':
                    if not state.gripper_open:
                        return False
                else:
                    if state.gripper_open:
                        return False

            elif key == 'in_drawer':
                in_drawer = False
                for obj in state.objects:
                    if self.to_cluster(obj.name.lower(), object_to_cluster) in self.effects[key]:
                        in_drawer = in_drawer or obj.in_drawer
                if not in_drawer:
                    return False
            elif key == 'not_in_drawer':
                not_in_drawer = False
                for obj in state.objects:
                    if self.to_cluster(obj.name.lower(), object_to_cluster) in self.effects[key]:
                        not_in_drawer = not_in_drawer or (not obj.in_drawer)
                if not not_in_drawer:
                    return False

            elif key == 'in_box':
                in_box = False
                for obj in state.objects:
                    if self.to_cluster(obj.name.lower(), object_to_cluster) in self.effects[key]:
                        in_box = in_box or obj.in_box
                if not in_box:
                    return False
            elif key == 'not_in_box':
                not_in_box = False
                for obj in state.objects:
                    if self.to_cluster(obj.name.lower(), object_to_cluster) in self.effects[key]:
                        not_in_box = not_in_box or (not obj.in_box)
                if not not_in_box:
                    return False

            elif key == 'on_lid':
                on_lid = False
                for obj in state.objects:
                    if self.to_cluster(obj.name.lower(), object_to_cluster) in self.effects[key]:
                        on_lid = on_lid or obj.on_lid
                if not on_lid:
                    return False
            elif key == 'not_on_lid':
                not_on_lid = False
                for obj in state.objects:
                    if self.to_cluster(obj.name.lower(), object_to_cluster) in self.effects[key]:
                        not_on_lid = not_on_lid or (not obj.on_lid)
                if not not_on_lid:
                    return False

        return True

    def to_cluster(self, str, object_to_cluster):
        if object_to_cluster is not None:
            if object_to_cluster.has_key(str):
                return object_to_cluster[str]
        return str

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
             + '\ttarget_open: ' + str(self.target_open) + '\n' \
             + '\tgripper_open: ' + str(self.gripper_open) + '\n' \
             + '\tobject_in_gripper: ' + str(self.object_in_gripper) + '\n' \
             + 'effects:' + '\n'
        if len(self.effects.keys()) == 0:
            s += '\tNone'
        else:
            for key in self.effects.keys():
                s += '\t' + key + ': ' + str(self.effects[key]) + '\n'
        return s

    def __repr__(self):
        return str(self)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.action == other.action and self.object == other.object and self.target == other.target \
                   and self.object_in_drawer == other.object_in_drawer and self.object_in_box == other.object_in_box \
                   and self.object_on_lid == other.object_on_lid and self.object_open == other.object_open \
                   and self.gripper_open == other.gripper_open and self.object_in_gripper == other.object_in_gripper \
                   and self.target_open == other.target_open and self.effects == other.effects
        return False

    def __ne__(self, other):
        return not self == other

    def __hash__(self):
        return hash((self.action, self.object, self.target, self.object_in_drawer, self.object_in_box,
                     self.object_on_lid, self.object_open, self.gripper_open, self.object_in_gripper, self.target_open,
                     tuple(self.effects.keys())))

    def add_effect(self, key, value):
        if self.effects.has_key(key):
            self.effects[key].append(value)
        else:
            self.effects[key] = [value]