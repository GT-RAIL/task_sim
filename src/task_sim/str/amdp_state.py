#!/usr/bin/env python

from copy import copy

item_map = {0:'apple', 1:'banana', 2:'carrot'}

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

gripper_box_relation_list = [
    'gripper_left_of_box',
    'gripper_right_of_box',
    'gripper_in_front_of_box',
    'gripper_behind_box',
    'gripper_above_box',
    'gripper_below_box',
    'gripper_left_of_lid',
    'gripper_right_of_lid',
    'gripper_in_front_of_lid',
    'gripper_behind_lid',
    'gripper_above_lid',
    'gripper_below_lid',
    'lid_closing_box',
    'gripper_open',
    'gripper_touching_box',
    'gripper_holding_lid'
]

object_gripper_box_relation_list = [
    'apple_left_of_box',
    'apple_right_of_box',
    'apple_in_front_of_box',
    'apple_behind_box',
    'apple_above_box',
    'apple_below_box',
    'apple_left_of_gripper',
    'apple_right_of_gripper',
    'apple_in_front_of_gripper',
    'apple_behind_gripper',
    'apple_above_gripper',
    'apple_below_gripper',
    'gripper_left_of_box',
    'gripper_right_of_box',
    'gripper_in_front_of_box',
    'gripper_behind_box',
    'gripper_above_box',
    'gripper_below_box',
    'gripper_left_of_lid',
    'gripper_right_of_lid',
    'gripper_in_front_of_lid',
    'gripper_behind_lid',
    'gripper_above_lid',
    'gripper_below_lid',
    'lid_closing_box',
    'gripper_open',
    'gripper_touching_box',
    'gripper_holding_lid',
    'gripper_holding_apple',
    'apple_touching_box',
    'gripper_holding_apple'
]

object2_gripper_drawer_relation_list = [
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
    'banana_left_of_drawer',
    'banana_right_of_drawer',
    'banana_in_front_of_drawer',
    'banana_behind_drawer',
    'banana_above_drawer',
    'banana_below_drawer',
    'banana_left_of_gripper',
    'banana_right_of_gripper',
    'banana_in_front_of_gripper',
    'banana_behind_gripper',
    'banana_above_gripper',
    'banana_below_gripper',
    'apple_left_of_banana',
    'apple_right_of_banana',
    'apple_in_front_of_banana',
    'apple_behind_banana',
    'apple_above_banana',
    'apple_below_banana',
    'gripper_left_of_drawer',
    'gripper_right_of_drawer',
    'gripper_in_front_of_drawer',
    'gripper_behind_drawer',
    'gripper_above_drawer',
    'gripper_below_drawer',
    'gripper_open',
    'apple_touching_drawer',
    'apple_touching_stack',
    'banana_touching_drawer',
    'banana_touching_stack',
    'apple_touching_banana',
    'gripper_touching_drawer',
    'gripper_touching_stack',
    'drawer_closing_stack',
    'gripper_holding_drawer',
    'gripper_holding_apple',
    'gripper_holding_banana'
]

object3_gripper_drawer_relation_list = [
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
    'banana_left_of_drawer',
    'banana_right_of_drawer',
    'banana_in_front_of_drawer',
    'banana_behind_drawer',
    'banana_above_drawer',
    'banana_below_drawer',
    'banana_left_of_gripper',
    'banana_right_of_gripper',
    'banana_in_front_of_gripper',
    'banana_behind_gripper',
    'banana_above_gripper',
    'banana_below_gripper',
    'carrot_left_of_drawer',
    'carrot_right_of_drawer',
    'carrot_in_front_of_drawer',
    'carrot_behind_drawer',
    'carrot_above_drawer',
    'carrot_below_drawer',
    'carrot_left_of_gripper',
    'carrot_right_of_gripper',
    'carrot_in_front_of_gripper',
    'carrot_behind_gripper',
    'carrot_above_gripper',
    'carrot_below_gripper',
    'apple_left_of_banana',
    'apple_right_of_banana',
    'apple_in_front_of_banana',
    'apple_behind_banana',
    'apple_above_banana',
    'apple_below_banana',
    'apple_left_of_carrot',
    'apple_right_of_carrot',
    'apple_in_front_of_carrot',
    'apple_behind_carrot',
    'apple_above_carrot',
    'apple_below_carrot',
    'banana_left_of_carrot',
    'banana_right_of_carrot',
    'banana_in_front_of_carrot',
    'banana_behind_carrot',
    'banana_above_carrot',
    'banana_below_carrot',
    'gripper_left_of_drawer',
    'gripper_right_of_drawer',
    'gripper_in_front_of_drawer',
    'gripper_behind_drawer',
    'gripper_above_drawer',
    'gripper_below_drawer',
    'gripper_open',
    'apple_touching_drawer',
    'apple_touching_stack',
    'banana_touching_drawer',
    'banana_touching_stack',
    'carrot_touching_drawer',
    'carrot_touching_stack',
    'apple_touching_banana',
    'apple_touching_carrot',
    'banana_touching_carrot',
    'gripper_touching_drawer',
    'gripper_touching_stack',
    'drawer_closing_stack',
    'gripper_holding_drawer',
    'gripper_holding_apple',
    'gripper_holding_banana',
    'gripper_holding_carrot'
]

high_level_relation_list = [
    'apple_inside_drawer',
    'drawer_closing_stack'
]

high_level_2_object_relation_list = [
    'apple_inside_drawer',
    'banana_inside_drawer',
    'drawer_closing_stack'
]

high_level_3_object_relation_list = [
    'apple_inside_drawer',
    'banana_inside_drawer',
    'carrot_inside_drawer',
    'drawer_closing_stack'
]

high_level_box_relation_list = [
    'carrot_inside_box',
    'lid_closing_box'
]

high_level_2_box_relation_list = [
    'carrot_inside_box',
    'daikon_inside_box',
    'lid_closing_box'
]

high_level_sort_relation_list = [
    'apple_inside_drawer',
    'banana_inside_drawer',
    'carrot_inside_box',
    'drawer_closing_stack',
    'lid_closing_box'
]

high_level_4_sort_relation_list = [
    'apple_inside_drawer',
    'banana_inside_drawer',
    'carrot_inside_box',
    'daikon_inside_box',
    'drawer_closing_stack',
    'lid_closing_box'
]

class AMDPState:

    def __init__(self, amdp_id=0, state=None, ground_items=None):
        '''States for amdps at different levels, indicated by type

        AMDP Types:
            0 : open drawer
            1 : close drawer
            2 : put apple in drawer
            3 : high level object-drawer task
            4 : high level 2 object-drawer task
            5 : high level 3 object-drawer task

            6 : open box
            7 : close box
            8 : put apple in box
            9 : high level object-box task
            10 : high level 3 object-drawer-box task
            11 : high level 2 object-box task

            -1 : flat 1 object 1 drawer
            -2 : flat 2 object 1 drawer
            -3 : flat 3 object 1 drawer
        '''
        self.relation_names = []
        self.relations = {}
        self.amdp_id = amdp_id
        self.ground_items = ground_items

        if self.amdp_id == 0 or self.amdp_id == 1:
            self.relation_names = copy(gripper_drawer_relation_list)
        elif self.amdp_id == 2 or self.amdp_id == -1:
            self.relation_names = copy(object_gripper_drawer_relation_list)
        elif self.amdp_id == 3:
            self.relation_names = copy(high_level_relation_list)
        elif self.amdp_id == 4:
            self.relation_names = copy(high_level_2_object_relation_list)
        elif self.amdp_id == 5:
            self.relation_names = copy(high_level_3_object_relation_list)
        elif self.amdp_id == -2:
            self.relation_names = copy(object2_gripper_drawer_relation_list)
        elif self.amdp_id == -3:
            self.relation_names = copy(object3_gripper_drawer_relation_list)
        elif self.amdp_id == 6 or self.amdp_id == 7:
            self.relation_names = copy(gripper_box_relation_list)
        elif self.amdp_id == 8:
            self.relation_names = copy(object_gripper_box_relation_list)
        elif self.amdp_id == 9:
            self.relation_names = copy(high_level_box_relation_list)
        elif self.amdp_id == 10:
            self.relation_names = copy(high_level_sort_relation_list)
        elif self.amdp_id == 11:
            self.relation_names = copy(high_level_2_box_relation_list)

        self.relation_names.sort()

        for relation_name in self.relation_names:
                self.relations[relation_name] = False

        if state is not None:
            self.project_state(state)

    def project_state(self, state):
        for relation_name in self.relation_names:
            if self.ground_items is not None:
                for i in range(len(self.ground_items)):
                    grounded_relation = relation_name.replace(item_map[i], self.ground_items[i])
            else:
                grounded_relation = relation_name
            if grounded_relation in state.relations:
                self.relations[relation_name] = True

        if 'gripper_holding_drawer' in self.relation_names and state.grippers['gripper'].holding == 'drawer':
            self.relations['gripper_holding_drawer'] = True

        if 'gripper_holding_lid' in self.relation_names and state.grippers['gripper'].holding == 'lid':
            self.relations['gripper_holding_lid'] = True

        if self.ground_items is not None:
            item = self.ground_items[0]
        else:
            item = 'apple'
        if 'gripper_holding_apple' in self.relation_names and state.grippers['gripper'].holding == item:
            self.relations['gripper_holding_apple'] = True

        if self.amdp_id <= -2:
            if 'gripper_holding_banana' in self.relation_names and state.grippers['gripper'].holding == 'banana':
                self.relations['gripper_holding_banana'] = True
            if 'gripper_holding_carrot' in self.relation_names and state.grippers['gripper'].holding == 'carrot':
                self.relations['gripper_holding_carrot'] = True

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
