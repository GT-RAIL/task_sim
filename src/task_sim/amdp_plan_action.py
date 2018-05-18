#!/usr/bin/env python

# Python
from math import sqrt

# ROS
import copy
from geometry_msgs.msg import Point
from task_sim.msg import Action
from task_sim import data_utils as DataUtils
from task_sim.oomdp.oo_state import OOState
from task_sim.str.amdp_state import AMDPState

class AMDPPlanAction:

    precondition_domain = {}
    precondition_domain[0] = ['gripper_holding_drawer', 'drawer_closing_stack']
    precondition_domain[2] = ['apple_in_drawer', 'gripper_holding_drawer', 'gripper_holding_apple',
                              'drawer_closing_stack']
    precondition_domain[6] = ['gripper_holding_lid', 'lid_closing_box']
    precondition_domain[8] = ['apple_in_box', 'gripper_holding_lid', 'gripper_holding_apple', 'lid_closing_box']

    def __init__(self, s0, a, s1=None, amdp_id=2):
        self.action_type = a.action_type
        self.action_object = a.object
        self.amdp_id = amdp_id

        # convert action into something that fits into the new action list
        if a.action_type == Action.PLACE:
            self.action_object = DataUtils.get_task_frame(s0, a.position)
        elif a.action_type == Action.MOVE_ARM:
            self.action_object = DataUtils.get_task_frame(s0, a.position)
            if (amdp_id <= 2 and self.action_object != 'stack' and self.action_object != 'drawer') or \
                                (amdp_id >= 6 and self.action_object != 'box' and self.action_object != 'lid'):
                for o in s0.objects:
                    if o.name != 'apple':
                        continue
                    if a.position == o.position:
                        self.action_object = 'apple'
                        break
                if self.action_object != 'apple':
                    x = s0.gripper_position.x
                    y = s0.gripper_position.y
                    px = a.position.x
                    py = a.position.y
                    if px == x and py > y:
                        self.action_object = 'b'
                    elif px < x and py > y:
                        self.action_object = 'bl'
                    elif px < x and py == y:
                        self.action_object = 'l'
                    elif px < x and py < y:
                        self.action_object = 'fl'
                    elif px == x and py < y:
                        self.action_object = 'f'
                    elif px > x and py < y:
                        self.action_object = 'fr'
                    elif px > x and py == y:
                        self.action_object = 'r'
                    else:
                        self.action_object = 'br'
        elif a.action_type == Action.GRASP:
            pass
        else:
            self.action_object = ''

        # compute amdp state representation
        s0_prime = AMDPState(amdp_id=self.amdp_id, state=OOState(state=s0))
        if s1 is not None:
            s1_prime = AMDPState(amdp_id=self.amdp_id, state=OOState(state=s1))

        # ********************************  Preconditions  ********************************
        self.preconditions = self.state_to_preconditions(s0_prime)

        # ********************************  Effects  ********************************
        self.effects = {}
        if s1 is not None:
            result = self.state_to_preconditions(s1_prime)
            for key, value in result.iteritems():
                if key in self.preconditions and self.preconditions[key] != value:
                    self.effects[key] = value

    def state_to_preconditions(self, s):
        preconditions = {}
        for p in self.precondition_domain[self.amdp_id]:
            if p == 'apple_in_drawer':
                if not (s.relations['apple_left_of_drawer'] or s.relations['apple_right_of_drawer'] or
                        s.relations['apple_in_front_of_drawer'] or s.relations['apple_behind_drawer'] or
                        s.relations['apple_above_drawer'] or s.relations['apple_below_drawer']):
                    preconditions[p] = True
                else:
                    preconditions[p] = False
            elif p == 'apple_in_box':
                if not (s.relations['apple_left_of_box'] or s.relations['apple_right_of_box'] or
                        s.relations['apple_in_front_of_box'] or s.relations['apple_behind_box'] or
                        s.relations['apple_above_box'] or s.relations['apple_below_box']):
                    preconditions[p] = True
                else:
                    preconditions[p] = False
            else:
                if p in s.relations.keys():
                    preconditions[p] = s.relations[p]
        return preconditions

    def check_preconditions(self, state):
        s = AMDPState(amdp_id=self.amdp_id, state=OOState(state=state))
        ps = self.state_to_preconditions(s)
        for key, value in self.preconditions.iteritems():
            if not (key in ps and ps[key] == value):
                return False
        return True

    def check_effects(self, state):
        s = AMDPState(amdp_id=self.amdp_id, state=OOState(state=state))
        ps = self.state_to_preconditions(s)
        for key, value in self.effects.iteritems():
            if not (key in ps and ps[key] == value):
                return False

        return True

    def __str__(self):
        return 'amdp_id:' + str(self.amdp_id) + 'action:' + str(self.action_type) + ':' \
               + str(self.action_object) + '\npreconditions:' + str(self.preconditions) \
               + '\neffects:' + str(self.effects)

    def __repr__(self):
        return str(self)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.action_type == other.action_type and self.action_object == other.action_object \
                   and self.preconditions == other.preconditions and self.effects == other.effects \
                   and self.amdp_id == other.amdp_id
        return False

    def __ne__(self, other):
        return not self == other

    def __hash__(self):
        return hash((self.amdp_id, self.action_type, self.action_object, tuple(self.preconditions.items()),
                     tuple(self.effects.items())))
