#!/usr/bin/env python


def reward(s, amdp_id=0):
    if amdp_id == 0:
        if not s.relations['drawer_closing_stack']:
            return 100
    elif amdp_id == 1:
        if s.relations['drawer_closing_stack'] and not s.relations['gripper_holding_drawer']:
            return 100
    elif amdp_id == 2:
        if not (s.relations['apple_left_of_drawer'] or s.relations['apple_right_of_drawer']
                 or s.relations['apple_in_front_of_drawer'] or s.relations['apple_behind_drawer']
                 or s.relations['apple_above_drawer'] or s.relations['apple_below_drawer']):
            return 100
    elif amdp_id == 3:
        if s.relations['drawer_closing_stack'] and s.relations['apple_inside_drawer']:
            return 100
    elif amdp_id == 4:
        if s.relations['drawer_closing_stack'] and s.relations['apple_inside_drawer'] \
                and s.relations['banana_inside_drawer']:
            return 100
    elif amdp_id == 5:
        if s.relations['drawer_closing_stack'] and s.relations['apple_inside_drawer'] \
                and s.relations['banana_inside_drawer'] and s.relations['carrot_inside_drawer']:
            return 100
    elif amdp_id == -1:
        if not (s.relations['apple_left_of_drawer'] or s.relations['apple_right_of_drawer']
                or s.relations['apple_in_front_of_drawer'] or s.relations['apple_behind_drawer']
                or s.relations['apple_above_drawer'] or s.relations['apple_below_drawer']) \
                and s.relations['drawer_closing_stack'] and not s.relations['gripper_holding_drawer']:
            return 100
    elif amdp_id == -2:
        if not (s.relations['apple_left_of_drawer'] or s.relations['apple_right_of_drawer']
                or s.relations['apple_in_front_of_drawer'] or s.relations['apple_behind_drawer']
                or s.relations['apple_above_drawer'] or s.relations['apple_below_drawer']) \
                and not (s.relations['banana_left_of_drawer'] or s.relations['banana_right_of_drawer']
                or s.relations['banana_in_front_of_drawer'] or s.relations['banana_behind_drawer']
                or s.relations['banana_above_drawer'] or s.relations['banana_below_drawer']) \
                and s.relations['drawer_closing_stack'] and not s.relations['gripper_holding_drawer']:
            return 100
    elif amdp_id == -3:
        if not (s.relations['apple_left_of_drawer'] or s.relations['apple_right_of_drawer']
                or s.relations['apple_in_front_of_drawer'] or s.relations['apple_behind_drawer']
                or s.relations['apple_above_drawer'] or s.relations['apple_below_drawer']) \
                and not (s.relations['banana_left_of_drawer'] or s.relations['banana_right_of_drawer']
                or s.relations['banana_in_front_of_drawer'] or s.relations['banana_behind_drawer']
                or s.relations['banana_above_drawer'] or s.relations['banana_below_drawer']) \
                and not (s.relations['carrot_left_of_drawer'] or s.relations['carrot_right_of_drawer']
                or s.relations['carrot_in_front_of_drawer'] or s.relations['carrot_behind_drawer']
                or s.relations['carrot_above_drawer'] or s.relations['carrot_below_drawer']) \
                and s.relations['drawer_closing_stack'] and not s.relations['gripper_holding_drawer']:
            return 100

    return -1


def is_terminal(s, amdp_id=0):
    if amdp_id == 0:
        return not s.relations['drawer_closing_stack'] and s.relations['gripper_holding_drawer']
    elif amdp_id == 1:
        return s.relations['drawer_closing_stack'] and s.relations['gripper_holding_drawer']
    elif amdp_id == 2:
        return not (s.relations['apple_left_of_drawer'] or s.relations['apple_right_of_drawer']
                 or s.relations['apple_in_front_of_drawer'] or s.relations['apple_behind_drawer']
                 or s.relations['apple_above_drawer'] or s.relations['apple_below_drawer'])
    elif amdp_id == 3:
        return s.relations['drawer_closing_stack'] and s.relations['apple_inside_drawer']
    elif amdp_id == 4:
        return s.relations['drawer_closing_stack'] and s.relations['apple_inside_drawer'] \
                and s.relations['banana_inside_drawer']
    elif amdp_id == 5:
        return s.relations['drawer_closing_stack'] and s.relations['apple_inside_drawer'] \
                and s.relations['banana_inside_drawer'] and s.relations['carrot_inside_drawer']
    elif amdp_id == -1:
        return not (s.relations['apple_left_of_drawer'] or s.relations['apple_right_of_drawer']
                    or s.relations['apple_in_front_of_drawer'] or s.relations['apple_behind_drawer']
                    or s.relations['apple_above_drawer'] or s.relations['apple_below_drawer']) \
                    and s.relations['drawer_closing_stack'] and not s.relations['gripper_holding_drawer']
    elif amdp_id == -2:
        return not (s.relations['apple_left_of_drawer'] or s.relations['apple_right_of_drawer']
                    or s.relations['apple_in_front_of_drawer'] or s.relations['apple_behind_drawer']
                    or s.relations['apple_above_drawer'] or s.relations['apple_below_drawer']) \
               and not (s.relations['banana_left_of_drawer'] or s.relations['banana_right_of_drawer']
                        or s.relations['banana_in_front_of_drawer'] or s.relations['banana_behind_drawer']
                        or s.relations['banana_above_drawer'] or s.relations['banana_below_drawer']) \
               and s.relations['drawer_closing_stack'] and not s.relations['gripper_holding_drawer']
    elif amdp_id == -3:
        return not (s.relations['apple_left_of_drawer'] or s.relations['apple_right_of_drawer']
                    or s.relations['apple_in_front_of_drawer'] or s.relations['apple_behind_drawer']
                    or s.relations['apple_above_drawer'] or s.relations['apple_below_drawer']) \
               and not (s.relations['banana_left_of_drawer'] or s.relations['banana_right_of_drawer']
                    or s.relations['banana_in_front_of_drawer'] or s.relations['banana_behind_drawer']
                    or s.relations['banana_above_drawer'] or s.relations['banana_below_drawer']) \
               and not (s.relations['carrot_left_of_drawer'] or s.relations['carrot_right_of_drawer']
                    or s.relations['carrot_in_front_of_drawer'] or s.relations['carrot_behind_drawer']
                    or s.relations['carrot_above_drawer'] or s.relations['carrot_below_drawer']) \
               and s.relations['drawer_closing_stack'] and not s.relations['gripper_holding_drawer']
