#!/usr/bin/env python

from copy import copy

class AMDPState:
    gripper_drawer_relation_list = [
        'gripper_left_of_drawer',
        'gripper_right_of_drawer',
        'gripper_in_front_of_drawer',
        'gripper_behind_drawer',
        'gripper_above_drawer',
        'gripper_below_drawer',
        'gripper_open',
        'gripper_touching_drawer',
        'gripper_touching_stack',
        'drawer_closing_stack',
        'gripper_holding_drawer'
    ]

    object_gripper_drawer_relation_list = [
        'apple_left_of_drawer',
        'apple_right_of_drawer',
        'apple_in_front_of_drawer',
        'apple_behind_drawer',
        'apple_above_drawer',
        'apple_below_drawer',
        'apple_left_of_gripper',
        'apple_right_of_gripper',
        'apple_in_front_of_gripper',
        'apple_behind_gripper',
        'apple_above_gripper',
        'apple_below_gripper',
        'gripper_left_of_drawer',
        'gripper_right_of_drawer',
        'gripper_in_front_of_drawer',
        'gripper_behind_drawer',
        'gripper_above_drawer',
        'gripper_below_drawer',
        'gripper_open',
        'apple_touching_drawer',
        'apple_touching_stack',
        'gripper_touching_drawer',
        'gripper_touching_stack',
        'drawer_closing_stack',
        'gripper_holding_drawer',
        'gripper_holding_apple'
    ]

    high_level_relation_list = [
        'apple_inside_drawer',
        'drawer_closing_stack'
    ]

    def __init__(self, amdp_id=0, state=None):
        '''States for amdps at different levels, indicated by type

        AMDP Types:
            0 : open drawer
            1 : close drawer
            2 : put apple in drawer
            3 : high level object-drawer task
        '''
        self.relation_names = []
        self.relations = {}
        self.amdp_id = amdp_id

        if self.amdp_id == 0 or self.amdp_id == 1:
            self.relation_names = copy(self.gripper_drawer_relation_list)
        elif self.amdp_id == 2 or self.amdp_id == -1:
            self.relation_names = copy(self.object_gripper_drawer_relation_list)
        elif self.amdp_id == 3:
            self.relation_names = copy(self.high_level_relation_list)

        self.relation_names.sort()

        for relation_name in self.relation_names:
                self.relations[relation_name] = False

        if state is not None:
            self.project_state(state)

    def project_state(self, state):
        for relation_name in self.relation_names:
            if relation_name in state.relations:
                self.relations[relation_name] = True
        if 'gripper_holding_drawer' in self.relation_names and state.grippers['gripper'].holding == 'drawer':
            self.relations['gripper_holding_drawer'] = True
        if 'gripper_holding_apple' in self.relation_names and state.grippers['gripper'].holding == 'apple':
            self.relations['gripper_holding_apple'] = True
        if 'gripper_open' in self.relation_names:
            self.relations['gripper_open'] = not state.grippers['gripper'].closed

    def to_vector(self):
        v = []
        for r in self.relation_names:
            v.append(int(self.relations[r]))
        return v

    def __str__(self):
        s = ''
        for key in self.relations:
            s += str(key)
            s += str(self.relations[key])
            s += '\n'
        return s

    def __repr__(self):
        return str(self)

    def __hash__(self):
        return hash(frozenset(self.relations.items()))

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.relations == other.relations
        return False
