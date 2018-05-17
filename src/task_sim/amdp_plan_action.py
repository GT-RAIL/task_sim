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

    precondition_domain = ['apple_in_drawer', 'apple_in_box', 'gripper_holding_drawer', 'gripper_holding_lid',
                           'gripper_holding_apple', 'drawer_closing_stack', 'lid_closing_box']

    def __init__(self, s0, a, s1=None, amdp_id=2):
        self.action = copy.deepcopy(a)

        # convert action into something that fits into the new action list
        if a.action_type == Action.PLACE:
            self.action.object = DataUtils.get_task_frame(s0, a.position)
            self.action.position = Point()
        elif a.action_type == Action.MOVE_ARM:
            self.action.object = DataUtils.get_task_frame(s0, a.position)
            if (amdp_id <= 2 and self.action.object != 'stack' and self.action.object != 'drawer') or \
                                (amdp_id >= 6 and self.action.object != 'box' and self.action.object != 'lid'):
                for o in s0.objects:
                    if o.name != 'apple':
                        continue
                    if a.position == o.position:
                        self.action.object = 'apple'
                        break
                if a.object != 'apple':
                    x = s0.gripper_position.x
                    y = s0.gripper_position.y
                    px = a.position.x
                    py = a.position.y
                    if px == x and py > y:
                        self.action.object = 'b'
                    elif px < x and py > y:
                        self.action.object = 'bl'
                    elif px < x and py == y:
                        self.action.object = 'l'
                    elif px < x and py < y:
                        self.action.object = 'fl'
                    elif px == x and py < y:
                        self.action.object = 'f'
                    elif px > x and py < y:
                        self.action.object = 'fr'
                    elif px > x and py == y:
                        self.action.object = 'r'
                    else:
                        self.action.object = 'br'
            self.action.position = Point()
        elif a.action_type == Action.GRASP:
            self.action.position = Point()
        else:
            self.action.position = Point()
            self.action.object = ''

        # compute amdp state representation
        s0_prime = AMDPState(amdp_id=amdp_id, state=OOState(state=s0))
        if s1 is not None:
            s1_prime = AMDPState(amdp_id=amdp_id, state=OOState(state=s1))

        # ********************************  Preconditions  ********************************
        self.preconditions = self.state_to_preconditions(s0_prime, amdp_id)

        # ********************************  Effects  ********************************
        self.effects = {}
        if s1 is not None:
            result = self.state_to_preconditions(s1_prime, amdp_id)
            for key, value in result.iteritems():
                if key in self.preconditions and self.preconditions[key] != value:
                    self.effects[key] = value

    def state_to_preconditions(self, s, amdp_id):
        preconditions = {}
        for p in self.precondition_domain:
            if (amdp_id <= 2 and 'lid' in p) or (amdp_id >= 6 and 'drawer' in p):
                continue
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

    def check_preconditions(self, state, amdp_id):
        s = AMDPState(amdp_id=amdp_id, state=OOState(state=state))
        ps = self.state_to_preconditions(s, amdp_id)
        for key, value in self.preconditions:
            if not (key in ps and ps[key] == value):
                return False
        return True

    def check_effects(self, state, amdp_id):
        s = AMDPState(amdp_id=amdp_id, state=OOState(state=state))
        ps = self.state_to_preconditions(s, amdp_id)
        for key, value in self.effects:
            if not (key in ps and ps[key] == value):
                return False

        return True

    def __str__(self):
        return 'action:' + str(self.action.action_type) + ':' + str(self.action.object) + '\npreconditions:' \
               + str(self.preconditions) + '\neffects:' + str(self.effects)

    def __repr__(self):
        return str(self)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.action == other.action and self.preconditions == other.preconditions \
                   and self.effects == other.effects
        return False

    def __ne__(self, other):
        return not self == other

    def __hash__(self):
        return hash((self.action, tuple(self.preconditions), tuple(self.effects)))
