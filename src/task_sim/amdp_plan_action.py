#!/usr/bin/env python

# Python
from math import sqrt

# ROS
import copy
from geometry_msgs.msg import Point
from task_sim.msg import Action
from task_sim import data_utils as DataUtils

class AMDPPlanAction:

    def __init__(self, s0, a, s1=None):
        self.action = a.action_type

        # convert action into something that fits into the new action list
        if a.action_type == Action.PLACE:
            self.action.object = DataUtils.get_task_frame(s0, a.position)
            a.position = Point()
        elif a.action_type == Action.MOVE_ARM:
            a.object = DataUtils.get_task_frame(s0, a.position)
            if a.object != 'stack' and a.object != 'drawer' and a.object != 'box' and a.object != 'lid':
                for o in s0.objects:
                    if o.name != 'apple':
                        continue
                    if a.position == o.position:
                        a.object = 'apple'
                        break
                if a.object != 'apple':
                    x = s0.gripper_position.x
                    y = s0.gripper_position.y
                    px = a.position.x
                    py = a.position.y
                    if px == x and py > y:
                        a.object = 'b'
                    elif px < x and py > y:
                        a.object = 'bl'
                    elif px < x and py == y:
                        a.object = 'l'
                    elif px < x and py < y:
                        a.object = 'fl'
                    elif px == x and py < y:
                        a.object = 'f'
                    elif px > x and py < y:
                        a.object = 'fr'
                    elif px > x and py == y:
                        a.object = 'r'
                    else:
                        a.object = 'br'
            a.position = Point()
        elif a.action_type == Action.GRASP:
            a.position = Point()
        else:
            a.position = Point()
            a.object = ''

        # extract object and target from action message
        if a.action_type == Action.GRASP:
            # object: what's being grasped
            # target: none
            self.object = a.object.lower()
            self.target = None
        elif a.action_type == Action.PLACE:
            # object: what's in the gripper
            # target: place location
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
            # object: what's in the gripper
            # target: move location
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
        elif a.action_type == Action.OPEN_GRIPPER:
            # object: what's in gripper
            # target: what's below the gripper
            if s0.object_in_gripper is not None and s0.object_in_gripper != '':
                self.object = s0.object_in_gripper.lower()
            else:
                self.object = None
            self.target = DataUtils.get_task_frame(s0, s0.gripper_position).lower()
        elif a.action_type in [Action.RAISE_ARM, Action.LOWER_ARM, Action.RESET_ARM]:
            # object: what's in the gripper
            # target: none
            if s0.object_in_gripper is not None and s0.object_in_gripper != '':
                self.object = s0.object_in_gripper.lower()
            else:
                self.object = None
            self.target = None
        elif a.action_type == Action.CLOSE_GRIPPER:
            # object: what will be in the gripper
            # target: none
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

        # compute semantic state representation
        s0_prime = DataUtils.semantic_state_vector(s0, return_dict=True)[0]
        if s1 is not None:
            s1_prime = DataUtils.semantic_state_vector(s1, return_dict=True)[0]

        # ********************************  Preconditions  ********************************
        # defaults
        self.object_in = []
        self.object_not_visible = 0
        self.object_open = 1
        self.object_near_edge = 0
        self.object_touching = []
        self.gripper_object_height = 0
        self.gripper_object_x = 0
        self.gripper_object_y = 0
        self.gripper_open = 0
        self.gripper_near_edge = 0
        self.in_gripper = ''
        self.gripper_touching = []

        if a.action_type in [Action.GRASP, Action.CLOSE_GRIPPER, Action.RESET_ARM, Action.RAISE_ARM, Action.LOWER_ARM]:
            # Preconditions refer to self.object
            if self.object is not None:
                for i in range(len(PlanAction.containers)):
                    key = self.object + '_' + PlanAction.prepositions[i] + '_' + PlanAction.containers[i]
                    if key in s0_prime and s0_prime[key]:
                        self.object_in.append(PlanAction.containers[i])

                key = self.object + '_not_visible'
                if key in s0_prime:
                    self.object_not_visible = s0_prime[key]

                if self.object in PlanAction.drawer_parts:
                    if 'drawer_open' in s0_prime:
                        self.object_open = s0_prime['drawer_open']
                elif self.object in PlanAction.box_parts:
                    if 'box_open' in s0_prime:
                        self.object_open = s0_prime['box_open']

                key = self.object + '_near_edge'
                if key in s0_prime:
                    self.object_near_edge = s0_prime[key]

                for i in range(len(PlanAction.everything)):
                    key1 = 'touching_' + self.object + '_x_' + PlanAction.everything[i]
                    key2 = 'touching_' + PlanAction.everything[i] + '_x_' + self.object
                    if (key1 in s0_prime and s0_prime[key1]) or (key2 in s0_prime and s0_prime[key2]):
                        self.object_touching.append(PlanAction.everything[i])

                key = 'relative_h_' + self.object
                if key in s0_prime:
                    self.gripper_object_height = s0_prime[key]

                key = 'relative_x_' + self.object
                if key in s0_prime:
                    self.gripper_object_x = s0_prime[key]

                key = 'relative_y_' + self.object
                if key in s0_prime:
                    self.gripper_object_y = s0_prime[key]
        else:
            # Preconditions refer to self.target
            if self.target is not None:
                for i in range(len(PlanAction.containers)):
                    if PlanAction.containers[i] == self.target:
                        suffix = '_' + PlanAction.prepositions[i] + '_' + PlanAction.containers[i]
                        for o in PlanAction.objects:
                            key = o + suffix
                            if key in s0_prime and s0_prime[key]:
                                self.object_in.append(o)

                if self.target in PlanAction.drawer_parts:
                    if 'drawer_open' in s0_prime:
                        self.object_open = s0_prime['drawer_open']
                elif self.target in PlanAction.box_parts:
                    if 'box_open' in s0_prime:
                        self.object_open = s0_prime['box_open']

                for i in range(len(PlanAction.everything)):
                    key1 = 'touching_' + self.target + '_x_' + PlanAction.everything[i]
                    key2 = 'touching_' + PlanAction.everything[i] + '_x_' + self.target
                    if (key1 in s0_prime and s0_prime[key1]) or (key2 in s0_prime and s0_prime[key2]):
                        self.object_touching.append(PlanAction.everything[i])

                key = 'relative_h_' + self.target
                if key in s0_prime:
                    self.gripper_object_height = s0_prime[key]

                key = 'relative_x_' + self.target
                if key in s0_prime:
                    self.gripper_object_x = s0_prime[key]

                key = 'relative_y_' + self.target
                if key in s0_prime:
                    self.gripper_object_y = s0_prime[key]

        # Gripper preconditions
        key = 'gripper_open'
        if key in s0_prime:
            self.gripper_open = s0_prime[key]

        key = 'gripper_near_edge'
        if key in s0_prime:
            self.gripper_near_edge = s0_prime[key]

        key = 'object_in_gripper'
        if key in s0_prime:
            self.in_gripper = DataUtils.int_to_name(s0_prime[key])

        for i in range(len(PlanAction.everything)):
            key1 = 'touching_gripper_' + PlanAction.everything[i]
            key2 = 'touching_' + PlanAction.everything[i] + '_gripper'
            if (key1 in s0_prime and s0_prime[key1]) or (key2 in s0_prime and s0_prime[key2]):
                self.gripper_touching.append(PlanAction.everything[i])

        # ********************************  Effects  ********************************
        def add_effect(key, s0, s1):
            if key in s0 and key in s1 and s0[key] != s1[key]:
                self.effects[key] = s1[key]

        self.effects = dict()
        if s1 is not None:
            # object effects
            if self.object is not None:
                for i in range(len(PlanAction.containers)):
                    add_effect(self.object + '_' + PlanAction.prepositions[i] + '_' + PlanAction.containers[i], s0_prime, s1_prime)
                add_effect(self.object + '_not_visible', s0_prime, s1_prime)
                if self.object in PlanAction.drawer_parts:
                    add_effect('drawer_open', s0_prime, s1_prime)
                elif self.object in PlanAction.box_parts:
                    add_effect('box_open', s0_prime, s1_prime)
                add_effect(self.object + '_near_edge', s0_prime, s1_prime)
                for i in range(len(PlanAction.everything)):
                    add_effect('touching_' + self.object + '_x_' + PlanAction.everything[i], s0_prime, s1_prime)
                    add_effect('touching_' + PlanAction.everything[i] + '_x_' + self.object, s0_prime, s1_prime)
                add_effect('relative_h_' + self.object, s0_prime, s1_prime)
                add_effect('relative_x_' + self.object, s0_prime, s1_prime)
                add_effect('relative_y_' + self.object, s0_prime, s1_prime)

            # target effects
            if self.target is not None:
                for i in range(len(PlanAction.containers)):
                    add_effect(self.target + '_' + PlanAction.prepositions[i] + '_' + PlanAction.containers[i], s0_prime, s1_prime)
                add_effect(self.target + '_not_visible', s0_prime, s1_prime)
                if self.target in PlanAction.drawer_parts:
                    add_effect('drawer_open', s0_prime, s1_prime)
                elif self.target in PlanAction.box_parts:
                    add_effect('box_open', s0_prime, s1_prime)
                add_effect(self.target + '_near_edge', s0_prime, s1_prime)
                for i in range(len(PlanAction.everything)):
                    add_effect('touching_' + self.target + '_x_' + PlanAction.everything[i], s0_prime, s1_prime)
                    add_effect('touching_' + PlanAction.everything[i] + '_x_' + self.target, s0_prime, s1_prime)
                add_effect('relative_h_' + self.target, s0_prime, s1_prime)
                add_effect('relative_x_' + self.target, s0_prime, s1_prime)
                add_effect('relative_y_' + self.target, s0_prime, s1_prime)

            # gripper effects
            add_effect('gripper_open', s0_prime, s1_prime)
            add_effect('gripper_near_edge', s0_prime, s1_prime)
            add_effect('object_in_gripper', s0_prime, s1_prime)
            for i in range(len(PlanAction.everything)):
                add_effect('touching_gripper_' + PlanAction.everything[i], s0_prime, s1_prime)
                add_effect('touching_' + PlanAction.everything[i] + '_gripper', s0_prime, s1_prime)


    def check_preconditions(self, state, obj, target, object_to_cluster = None):
        s_prime = DataUtils.semantic_state_vector(state, return_dict=True)[0]

        if self.action in [Action.GRASP, Action.CLOSE_GRIPPER, Action.RESET_ARM, Action.RAISE_ARM, Action.LOWER_ARM]:
            pass  # preconditions refer to object
        else:
            pass  # preconditions refer to target
        # gripper preconditions

        # if obj not in ['drawer', 'stack', 'handle', 'box', 'lid']:
        #     for o in state.objects:
        #         if o.name.lower() == obj:
        #             if (self.object_in_drawer == o.in_drawer and self.object_in_box == o.in_box
        #                 and self.object_on_lid == o.on_lid):
        #                 break
        #             else:
        #                 return False
        #
        # box_open = sqrt(pow(state.lid_position.x - state.box_position.x, 2)
        #                + pow(state.lid_position.y - state.box_position.y, 2)) >= 2
        # drawer_open = state.drawer_opening >= 2
        # if obj in ['drawer', 'stack', 'handle']:
        #     if drawer_open != self.object_open:
        #         return False
        # elif obj in ['box', 'lid']:
        #     if box_open != self.object_open:
        #         return False
        # if target in ['drawer', 'stack', 'handle']:
        #     if drawer_open != self.target_open:
        #         return False
        # elif target in ['box', 'lid']:
        #     if box_open != self.target_open:
        #         return False
        #
        # if self.gripper_open != state.gripper_open:
        #     return False
        #
        # if object_to_cluster is not None:
        #     obj = state.object_in_gripper.lower()
        #     if object_to_cluster.has_key(obj):
        #         obj = object_to_cluster[obj]
        #     if self.object_in_gripper != obj:
        #         return False

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
        '''
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
        '''
        s += 'action: ' + str(self.action) + '\n' \
             + 'object: ' + str(self.object) + '\n' \
             + 'target ' + str(self.target) + '\n'

        s += 'preconditions:\n'
        s += '\tobject in:\n'
        for o in self.object_in:
            s += '\t\t' + str(o) + '\n'
        s += '\tobject_not_visible: ' + str(self.object_not_visible) + '\n'
        s += '\tobject_open: ' + str(self.object_open) + '\n'
        s += '\tobject_near_edge: ' + str(self.object_near_edge) + '\n'
        s += '\tobject touching:\n'
        for o in self.object_touching:
            s += '\t\t' + str(o) + '\n'
        s += '\tgripper_object_height: ' + str(self.gripper_object_height) + '\n'
        s += '\tgripper_object_x: ' + str(self.gripper_object_x) + '\n'
        s += '\tgripper_object_y: ' + str(self.gripper_object_y) + '\n'
        s += '\tgripper_open: ' + str(self.gripper_open) + '\n'
        s += '\tgripper_near_edge: ' + str(self.gripper_near_edge) + '\n'
        s += '\tin_gripper: ' + str(self.in_gripper) + '\n'
        s += '\tobject gripper_touching:\n'
        for o in self.gripper_touching:
            s += '\t\t' + str(o) + '\n'

        s += 'effects:\n'
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
