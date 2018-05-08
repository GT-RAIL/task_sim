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
    elif amdp_id == 6:
        if not s.relations['lid_closing_box']:
            return 100
    elif amdp_id == 7:
        if s.relations['lid_closing_box'] and not s.relations['gripper_holding_lid']:
            return 100
    elif amdp_id == 8:
        if not (s.relations['apple_left_of_box'] or s.relations['apple_right_of_box']
                 or s.relations['apple_in_front_of_box'] or s.relations['apple_behind_box']
                 or s.relations['apple_above_box'] or s.relations['apple_below_box']):
            return 100
    elif amdp_id == 9:
        if s.relations['carrot_inside_box'] and s.relations['lid_closing_box']:
            return 100
    elif amdp_id == 10:
        reward = 0
        if s.relations['apple_inside_drawer'] and s.relations['banana_inside_drawer'] \
                and s.relations['drawer_closing_stack']:
            reward += 100
        if s.relations['carrot_inside_box'] and s.relations['lid_closing_box']:
            reward += 50
    elif amdp_id == 11:
        if s.relations['carrot_inside_box'] and s.relations['daikon_inside_box'] \
                and s.relations['lid_closing_box']:
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

    elif amdp_id == 6:
        return not s.relations['lid_closing_box']
    elif amdp_id == 7:
        return s.relations['lid_closing_box'] and not s.relations['gripper_holding_lid']
    elif amdp_id == 8:
        return not (s.relations['apple_left_of_box'] or s.relations['apple_right_of_box']
                    or s.relations['apple_in_front_of_box'] or s.relations['apple_behind_box']
                    or s.relations['apple_above_box'] or s.relations['apple_below_box'])
    elif amdp_id == 9:
        return s.relations['carrot_inside_box'] and s.relations['lid_closing_box']
    elif amdp_id == 10:
        return s.relations['apple_inside_drawer'] and s.relations['banana_inside_drawer'] \
                and s.relations['drawer_closing_stack'] and s.relations['carrot_inside_box'] \
               and s.relations['lid_closing_box']
    elif amdp_id == 11:
        return s.relations['carrot_inside_box'] and s.relations['daikon_inside_box'] \
               and s.relations['lid_closing_box']
