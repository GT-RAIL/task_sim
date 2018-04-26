#!/usr/bin/env python

def reward(s):
    if s.relations['drawer_closing_stack'] and \
            not (s.relations['apple_left_of_drawer'] or s.relations['apple_right_of_drawer']
                 or s.relations['apple_in_front_of_drawer'] or s.relations['apple_behind_drawer']
                 or s.relations['apple_above_drawer'] or s.relations['apple_below_drawer']):
        return 100

    # elif not (s.relations['apple_left_of_drawer'] or s.relations['apple_right_of_drawer']
    #           or s.relations['apple_in_front_of_drawer'] or s.relations['apple_behind_drawer']
    #           or s.relations['apple_above_drawer'] or s.relations['apple_below_drawer']):
    #     return -.01

    else:
        return -.1

def is_terminal(s):
    return s.relations['drawer_closing_stack'] and \
        not (s.relations['apple_left_of_drawer'] or s.relations['apple_right_of_drawer']
             or s.relations['apple_in_front_of_drawer'] or s.relations['apple_behind_drawer']
             or s.relations['apple_above_drawer'] or s.relations['apple_below_drawer'])
