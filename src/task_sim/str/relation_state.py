#!/usr/bin/env python

class RelationState:
    relation_list = [
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
        'drawer_closing_stack'
    ]

    def __init__(self, state=None):
        self.relations = {}
        for relation_name in self.relation_list:
            self.relations[relation_name] = False
        self.gripper_holding = ''

        if state is not None:
            self.init_from_state(state)

    def init_from_state(self, state):
        for relation_name in self.relation_list:
            if relation_name in state.relations:
                self.relations[relation_name] = True
        self.relations['gripper_open'] = not state.grippers['gripper'].closed
        if state.grippers['gripper'].holding == 'drawer':
            self.gripper_holding = 'drawer'
        if state.grippers['gripper'].holding == 'apple':
            self.gripper_holding = 'apple'

    def __hash__(self):
        return hash((frozenset(self.relations.items()), self.gripper_holding))

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.relations == other.relations and self.gripper_holding == other.gripper_holding
        return False
