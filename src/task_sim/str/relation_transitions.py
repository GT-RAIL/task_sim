#!/usr/bin/env python

from copy import deepcopy
from task_sim.msg import Action


def transition_function(s, a):
    """Calculate likely new states from executing an action in a given state (space: relations -> relations)

    Returns:
        List of tuples in the form (probability, relation state)
    """
    results = []

    if a.action_type == Action.GRASP:
        s_prime = deepcopy(s)

        if a.object == 'apple':
            alpha = 0.2  # chance of grasp failing
            if s.relations['apple_touching_drawer']:
                alpha += 0.1
            if s.relations['apple_touching_stack']:
                alpha += 0.1
            if s.relations['gripper_touching_drawer']:
                alpha += 0.2
            if s.relations['gripper_touching_stack']:
                alpha += 0.2

            s_prime.relations['apple_left_of_gripper'] = False
            s_prime.relations['apple_right_of_gripper'] = False
            s_prime.relations['apple_in_front_of_gripper'] = False
            s_prime.relations['apple_behind_gripper'] = False
            s_prime.relations['apple_above_gripper'] = False
            s_prime.relations['apple_below_gripper'] = False

            s_prime.relations['gripper_left_of_drawer'] = s_prime.relations['apple_left_of_drawer']
            s_prime.relations['gripper_right_of_drawer'] = s_prime.relations['apple_right_of_drawer']
            s_prime.relations['gripper_in_front_of_drawer'] = s_prime.relations['apple_in_front_of_drawer']
            s_prime.relations['gripper_behind_drawer'] = s_prime.relations['apple_behind_drawer']
            s_prime.relations['gripper_above_drawer'] = s_prime.relations['apple_above_drawer']
            s_prime.relations['gripper_below_drawer'] = s_prime.relations['apple_below_drawer']

            s_prime.relations['gripper_touching_drawer'] = s_prime.relations['apple_touching_drawer']
            s_prime.relations['gripper_touching_stack'] = s_prime.relations['apple_touching_stack']

            s_prime.relations['gripper_open'] = False
            s_prime.gripper_holding = 'apple'

            results.append((alpha, deepcopy(s)))
            results.append((1 - alpha, s_prime))
        elif a.object == 'drawer':
            alpha = 0.2  # chance of grasp failing
            if s.relations['gripper_touching_drawer']:
                alpha += 0.1
            if s.relations['gripper_touching_stack']:
                alpha += 0.1

            s_prime.relations['gripper_left_of_drawer'] = False
            s_prime.relations['gripper_right_of_drawer'] = False
            s_prime.relations['gripper_in_front_of_drawer'] = False
            s_prime.relations['gripper_behind_drawer'] = False
            s_prime.relations['gripper_above_drawer'] = False
            s_prime.relations['gripper_below_drawer'] = False

            if s_prime.relations['apple_right_of_drawer']:
                s_prime.relations['apple_right_of_gripper'] = True
                s_prime.relations['apple_left_of_gripper'] = False
            else:
                s_prime.relations['apple_right_of_gripper'] = False
                s_prime.relations['apple_left_of_gripper'] = True
            s_prime.relations['apple_above_gripper'] = s_prime.relations['apple_above_drawer']
            s_prime.relations['apple_below_gripper'] = s_prime.relations['apple_below_drawer']

            s_prime.relations['gripper_touching_drawer'] = True
            if s.relations['drawer_closing_stack']:
                s_prime.relations['gripper_touching_stack'] = True
            else:
                s_prime.relations['gripper_touching_stack'] = False

            s_prime.relations['gripper_open'] = False
            s_prime.gripper_holding = 'drawer'

            if s_prime.relations['apple_in_front_of_drawer']:
                s_prime.relations['apple_in_front_of_gripper'] = True
                s_prime.relations['apple_behind_gripper'] = False
                results.append((alpha, deepcopy(s)))
                results.append((1 - alpha, s_prime))
            elif s_prime.relations['apple_in_front_of_drawer']:
                s_prime.relations['apple_in_front_of_gripper'] = True
                s_prime.relations['apple_behind_gripper'] = False
                results.append((alpha, deepcopy(s)))
                results.append((1 - alpha, s_prime))
            else:
                results.append((alpha, deepcopy(s)))
                s_prime.relations['apple_in_front_of_gripper'] = False
                s_prime.relations['apple_behind_gripper'] = False
                results.append((1.0*(1 - alpha)/5.0, s_prime))
                s_prime.relations['apple_in_front_of_gripper'] = True
                s_prime.relations['apple_behind_gripper'] = False
                results.append((2.0*(1 - alpha)/5.0, s_prime))
                s_prime.relations['apple_in_front_of_gripper'] = False
                s_prime.relations['apple_behind_gripper'] = True
                results.append((2.0*(1 - alpha)/5.0, s_prime))

    elif a.action_type == Action.PLACE:
        if s.gripper_holding == 'drawer':
            results.append((1.0, s))
        else:
            states = []
            probs = []
            s_prime = deepcopy(s)
            if a.object == 'drawer':
                if s.relations['drawer_closing_stack']:
                    s_prime.relations['gripper_above_drawer'] = True
                    if s.gripper_holding == 'apple':
                        s_prime.relations['apple_above_drawer'] = True
                else:
                    s_prime.relations['gripper_above_drawer'] = False
                    if s.gripper_holding == 'apple':
                        s_prime.relations['apple_above_drawer'] = False
                s_prime.relations['gripper_left_of_drawer'] = False
                s_prime.relations['gripper_right_of_drawer'] = False
                s_prime.relations['gripper_in_front_of_drawer'] = False
                s_prime.relations['gripper_behind_drawer'] = False
                s_prime.relations['gripper_below_drawer'] = False
                if s.gripper_holding == 'apple':
                    s_prime.relations['apple_left_of_drawer'] = False
                    s_prime.relations['apple_right_of_drawer'] = False
                    s_prime.relations['apple_in_front_of_drawer'] = False
                    s_prime.relations['apple_behind_drawer'] = False
                    s_prime.relations['apple_below_drawer'] = False
                s_prime.relations['gripper_open'] = True
                s_prime.gripper_holding = ''
                states.append(s_prime)
                probs.append(1.0)
            if a.object == 'stack':
                if s.relations['drawer_closing_stack']:
                    s_prime.relations['gripper_left_of_drawer'] = False
                    if s.gripper_holding == 'apple':
                        s_prime.relations['apple_left_of_drawer'] = False
                else:
                    s_prime.relations['gripper_left_of_drawer'] = True
                    if s.gripper_holding == 'apple':
                        s_prime.relations['apple_left_of_drawer'] = True
                s_prime.relations['gripper_above_drawer'] = True
                s_prime.relations['gripper_right_of_drawer'] = False
                s_prime.relations['gripper_in_front_of_drawer'] = False
                s_prime.relations['gripper_behind_drawer'] = False
                s_prime.relations['gripper_below_drawer'] = False
                if s.gripper_holding == 'apple':
                    s_prime.relations['apple_above_drawer'] = True
                    s_prime.relations['apple_right_of_drawer'] = False
                    s_prime.relations['apple_in_front_of_drawer'] = False
                    s_prime.relations['apple_behind_drawer'] = False
                    s_prime.relations['apple_below_drawer'] = False
                s_prime.relations['gripper_open'] = True
                s_prime.gripper_holding = ''
                states.append(s_prime)
                states.append(1.0)
            elif a.object == '':
                s_prime.relations['gripper_below_drawer'] = True
                s_prime.relations['gripper_above_drawer'] = False
                if s.gripper_holding == 'apple':
                    s_prime.relations['apple_below_drawer'] = True
                    s_prime.relations['apple_above_drawer'] = False
                prob = 1.0
                for i in range(3):
                    s_prime.relations['gripper_left_of_drawer'] = False
                    s_prime.relations['gripper_right_of_drawer'] = False
                    if s.gripper_holding == 'apple':
                        s_prime.relations['apple_left_of_drawer'] = False
                        s_prime.relations['apple_right_of_drawer'] = False
                    if i == 0:
                        prob2 = prob * 0.3
                    elif i == 1:
                        prob2 = prob * 0.35
                        s_prime.relations['gripper_left_of_drawer'] = True
                        if s.gripper_holding == 'apple':
                            s_prime.relations['apple_left_of_drawer'] = True
                    else:
                        prob2 = prob * 0.35
                        s_prime.relations['gripper_right_of_drawer'] = True
                        if s.gripper_holding == 'apple':
                            s_prime.relations['apple_right_of_drawer'] = True
                    for j in range(3):
                        s_prime.relations['gripper_in_front_of_drawer'] = False
                        s_prime.relations['gripper_behind_drawer'] = False
                        if s.gripper_holding == 'apple':
                            s_prime.relations['apple_in_front_of_drawer'] = False
                            s_prime.relations['apple_behind_drawer'] = False
                        if j == 0:
                            prob3 = prob2 * 0.3
                        elif j == 1:
                            prob3 = prob2 * 0.35
                            s_prime.relations['gripper_in_front_of_drawer'] = True
                            if s.gripper_holding == 'apple':
                                s_prime.relations['apple_in_front_of_drawer'] = True
                        else:
                            prob3 = prob2 * 0.35
                            s_prime.relations['gripper_behind_drawer'] = True
                            if s.gripper_holding == 'apple':
                                s_prime.relations['apple_behind_drawer'] = True
                        if s_prime.relations['gripper_in_front_of_drawer'] \
                                or s_prime.relations['gripper_behind_drawer'] \
                                or s_prime.relations['gripper_left_of_drawer'] \
                                or s_prime.relations['gripper_right_of_drawer']:
                            states.append(deepcopy(s_prime))
                            probs.append(prob3)

            p_sum = 0.0
            for i in range(len(probs)):
                p_sum += probs[i]
            for i in range(len(probs)):
                probs[i] /= p_sum
            results = zip(probs, states)

    elif a.action_type == Action.MOVE_ARM:
        # either: move to goal successfully, or
        # make it part way, now touching either the stack or the drawer
        # also: drawer physics
        s_prime = deepcopy(s)
        if s.gripper_holding == 'drawer':
            if s.relations['drawer_closing_stack']:
                if a.object == 'r':
                    s_prime.relations['drawer_closing_stack'] = False
                    results.append((1.0, s_prime))
                elif a.object == 'br' or a.object == 'fr':
                    results.append((0.5, deepcopy(s_prime)))
                    s_prime.relations['drawer_closing_stack'] = False
                    results.append((0.5, s_prime))
                else:
                    results.append((1.0, s_prime))
            else:
                if a.object == 'stack' or a.object == 'l':
                    s_prime.relations['drawer_closing_stack'] = True
                    results.append((1.0, s_prime))
                elif a.object == 'bl' or a.object == 'fl' or a.object == 'drawer':
                    results.append((0.75, deepcopy(s_prime)))
                    s_prime.relations['drawer_closing_stack'] = True
                    results.append((0.25, s_prime))
                else:
                    results.append((1.0, s_prime))
        elif s.gripper_holding == 'apple':
            states = []
            probs = []
            if a.object == 'l' or a.object == 'bl' or a.object == 'fl':
                for i in range(3):
                    if i == 0:
                        if not s.relations['gripper_right_of_drawer']:
                            continue
                        prob = 0.333
                    elif i == 1:
                        if s.relations['gripper_left_of_drawer'] or (not
                                (s.relations['gripper_below_drawer'] or
                                 s.relations['gripper_above_drawer']) and not
                                (s_prime.relations['gripper_in_front_of_drawer'] or
                                 s_prime.relations['gripper_behind_drawer'])):
                            continue
                        s_prime.relations['gripper_right_of_drawer'] = False
                        s_prime.relations['apple_right_of_drawer'] = False
                        prob = 0.333
                    else:
                        s_prime.relations['gripper_right_of_drawer'] = False
                        s_prime.relations['apple_right_of_drawer'] = False
                        s_prime.relations['gripper_left_of_drawer'] = True
                        s_prime.relations['apple_left_of_drawer'] = True
                        prob = 0.333

                    if a.object == 'bl':
                        for j in range(3):
                            if j == 0:
                                if not s.relations['gripper_in_front_of_drawer']:
                                    continue
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            elif j == 1:
                                if s.relations['gripper_behind_drawer'] or (not
                                        (s.relations['gripper_below_drawer'] or
                                         s.relations['gripper_above_drawer']) and not
                                        (s_prime.relations['gripper_left_of_drawer'] or
                                         s_prime.relations['gripper_right_of_drawer'])):
                                    continue
                                s_prime.relations['gripper_in_front_of_drawer'] = False
                                s_prime.relations['apple_in_front_of_drawer'] = False
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            else:
                                s_prime.relations['gripper_in_front_of_drawer'] = False
                                s_prime.relations['apple_in_front_of_drawer'] = False
                                s_prime.relations['gripper_behind_drawer'] = True
                                s_prime.relations['apple_behind_drawer'] = True
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                    elif a.object == 'fl':
                        for j in range(3):
                            if j == 0:
                                if not s.relations['gripper_behind_drawer']:
                                    continue
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            elif j == 1:
                                if s.relations['gripper_in_front_of_drawer'] or (not
                                        (s.relations['gripper_below_drawer'] or
                                         s.relations['gripper_above_drawer']) and not
                                        (s_prime.relations['gripper_left_of_drawer'] or
                                         s_prime.relations['gripper_right_of_drawer'])):
                                    continue
                                s_prime.relations['gripper_behind_drawer'] = False
                                s_prime.relations['apple_behind_drawer'] = False
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            else:
                                s_prime.relations['gripper_in_front_of_drawer'] = True
                                s_prime.relations['apple_in_front_of_drawer'] = True
                                s_prime.relations['gripper_behind_drawer'] = False
                                s_prime.relations['apple_behind_drawer'] = False
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                    else:
                        probs.append(prob)
                        states.append(deepcopy(s_prime))

                # drawer close chance
                if not s.relations['drawer_closing_stack'] and \
                        not (s.relations['gripper_above_drawer'] or s.relations['gripper_below_drawer']):
                    new_probs = []
                    new_states = []
                    for i in range(len(states)):
                        s_prime = states[i]
                        if not s_prime.relations['gripper_left_of_drawer'] and \
                                not (s.relations['gripper_behind_drawer'] or
                                     s.relations['gripper_in_front_of_drawer']):
                            new_probs.append(0.5*probs[i])
                            new_states.append(deepcopy(s_prime))
                            s_prime.relations['gripper_right_of_drawer'] = True
                            s_prime.relations['drawer_closing_stack'] = True
                            s_prime.relations['gripper_in_front_of_drawer'] = False
                            s_prime.relations['gripper_behind_drawer'] = False
                            s_prime.relations['apple_right_of_drawer'] = True
                            s_prime.relations['apple_in_front_of_drawer'] = False
                            s_prime.relations['apple_behind_drawer'] = False
                            new_probs.append(0.5*probs[i])
                            new_states.append(s_prime)
                        else:
                            new_probs.append(probs[i])
                            new_states.append(s_prime)
                    probs = new_probs
                    states = new_states

            elif a.object == 'r' or a.object == 'br' or a.object == 'fr':
                for i in range(3):
                    if i == 0:
                        if not s.relations['gripper_left_of_drawer']:
                            continue
                        prob = 0.333
                    elif i == 1:
                        if s.relations['gripper_right_of_drawer'] or (not
                                (s.relations['gripper_below_drawer'] or
                                 s.relations['gripper_above_drawer']) and not
                                (s_prime.relations['gripper_in_front_of_drawer'] or
                                 s_prime.relations['gripper_behind_drawer'])):
                            continue
                        s_prime.relations['gripper_left_of_drawer'] = False
                        s_prime.relations['apple_left_of_drawer'] = False
                        prob = 0.333
                    else:
                        s_prime.relations['gripper_left_of_drawer'] = False
                        s_prime.relations['apple_left_of_drawer'] = False
                        s_prime.relations['gripper_right_of_drawer'] = True
                        s_prime.relations['apple_right_of_drawer'] = True
                        prob = 0.333
                    if a.object == 'br':
                        for j in range(3):
                            if j == 0:
                                if not s.relations['gripper_in_front_of_drawer']:
                                    continue
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            elif j == 1:
                                if s.relations['gripper_behind_drawer'] or (not
                                        (s.relations['gripper_below_drawer'] or
                                         s.relations['gripper_above_drawer']) and not
                                        (s_prime.relations['gripper_left_of_drawer'] or
                                         s_prime.relations['gripper_right_of_drawer'])):
                                    continue
                                s_prime.relations['gripper_in_front_of_drawer'] = False
                                s_prime.relations['apple_in_front_of_drawer'] = False
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            else:
                                s_prime.relations['gripper_in_front_of_drawer'] = False
                                s_prime.relations['apple_in_front_of_drawer'] = False
                                s_prime.relations['gripper_behind_drawer'] = True
                                s_prime.relations['apple_behind_drawer'] = True
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                    elif a.object == 'fr':
                        for j in range(3):
                            if j == 0:
                                if not s.relations['gripper_behind_drawer']:
                                    continue
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            elif j == 1:
                                if s.relations['gripper_in_front_of_drawer'] or (not
                                        (s.relations['gripper_below_drawer'] or
                                         s.relations['gripper_above_drawer']) and not
                                        (s_prime.relations['gripper_left_of_drawer'] or
                                         s_prime.relations['gripper_right_of_drawer'])):
                                    continue
                                s_prime.relations['gripper_behind_drawer'] = False
                                s_prime.relations['apple_behind_drawer'] = False
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            else:
                                s_prime.relations['gripper_behind_drawer'] = False
                                s_prime.relations['apple_behind_drawer'] = False
                                s_prime.relations['gripper_in_front_of_drawer'] = True
                                s_prime.relations['apple_in_front_of_drawer'] = True
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                    else:
                        probs.append(prob)
                        states.append(deepcopy(s_prime))
            elif a.object == 'b':
                for i in range(3):
                    if i == 0:
                        if not s.relations['gripper_in_front_of_drawer']:
                            continue
                        prob = 0.333
                        states.append(deepcopy(s_prime))
                        probs.append(prob)
                    elif i == 1:
                        if s.relations['gripper_behind_drawer'] or (not
                                (s.relations['gripper_below_drawer'] or
                                 s.relations['gripper_above_drawer']) and not
                                (s_prime.relations['gripper_left_of_drawer'] or
                                 s_prime.relations['gripper_right_of_drawer'])):
                            continue
                        s_prime.relations['gripper_in_front_of_drawer'] = False
                        s_prime.relations['apple_in_front_of_drawer'] = False
                        prob = 0.333
                        states.append(deepcopy(s_prime))
                        probs.append(prob)
                    else:
                        s_prime.relations['gripper_in_front_of_drawer'] = False
                        s_prime.relations['apple_in_front_of_drawer'] = False
                        s_prime.relations['gripper_behind_drawer'] = True
                        s_prime.relations['apple_behind_drawer'] = True
                        prob = 0.333
                        states.append(deepcopy(s_prime))
                        probs.append(prob)
            elif a.object == 'u':
                for i in range(3):
                    if i == 0:
                        if not s.relations['gripper_behind_drawer']:
                            continue
                        prob = 0.333
                        states.append(deepcopy(s_prime))
                        probs.append(prob)
                    elif i == 1:
                        if s.relations['gripper_in_front_of_drawer'] or (not
                                (s.relations['gripper_below_drawer'] or
                                 s.relations['gripper_above_drawer']) and not
                                (s_prime.relations['gripper_left_of_drawer'] or
                                 s_prime.relations['gripper_right_of_drawer'])):
                            continue
                        s_prime.relations['gripper_behind_drawer'] = False
                        s_prime.relations['apple_behind_drawer'] = False
                        prob = 0.333
                        states.append(deepcopy(s_prime))
                        probs.append(prob)
                    else:
                        s_prime.relations['gripper_behind_drawer'] = False
                        s_prime.relations['apple_behind_drawer'] = False
                        s_prime.relations['gripper_in_front_of_drawer'] = True
                        s_prime.relations['apple_in_front_of_drawer'] = True
                        prob = 0.333
                        states.append(deepcopy(s_prime))
                        probs.append(prob)
            elif a.object == 'stack':
                if not s.relations['drawer_closing_stack'] and \
                        not (s.relations['gripper_above_drawer'] or s.relations['gripper_below_drawer']):
                    if not s.relations['gripper_left_of_drawer'] and \
                            not (s.relations['gripper_behind_drawer'] or s.relations['gripper_in_front_of_drawer']):
                        probs.append(0.75)
                        states.append(deepcopy(s_prime))
                        s_prime.relations['gripper_right_of_drawer'] = True
                        s_prime.relations['drawer_closing_stack'] = True
                        s_prime.relations['gripper_in_front_of_drawer'] = False
                        s_prime.relations['gripper_behind_drawer'] = False
                        s_prime.relations['apple_right_of_drawer'] = True
                        s_prime.relations['apple_in_front_of_drawer'] = False
                        s_prime.relations['apple_behind_drawer'] = False
                        probs.append(0.25)
                        states.append(s_prime)
                    else:
                        probs.append(1.0)
                        states.append(s_prime)
                else:
                    probs.append(1.0)
                    states.append(s_prime)
            elif a.object == 'drawer':
                if not s.relations['drawer_closing_stack'] and \
                        not (s.relations['gripper_above_drawer'] or s.relations['gripper_below_drawer']):
                    if not s.relations['gripper_left_of_drawer'] and \
                            not (s.relations['gripper_behind_drawer'] or s.relations['gripper_in_front_of_drawer']):
                        probs.append(0.9)
                        states.append(deepcopy(s_prime))
                        s_prime.relations['gripper_right_of_drawer'] = True
                        s_prime.relations['drawer_closing_stack'] = True
                        s_prime.relations['gripper_in_front_of_drawer'] = False
                        s_prime.relations['gripper_behind_drawer'] = False
                        s_prime.relations['apple_right_of_drawer'] = True
                        s_prime.relations['apple_in_front_of_drawer'] = False
                        s_prime.relations['apple_behind_drawer'] = False
                        probs.append(0.1)
                        states.append(s_prime)
                    else:
                        probs.append(1.0)
                        states.append(s_prime)
                else:
                    probs.append(1.0)
                    states.append(s_prime)

            new_probs = []
            new_states = []
            for i in range(len(states)):
                if not s.relations['gripper_above_drawer']:
                    s_prime = states[i]
                    s_prime.relations['gripper_touching_stack'] = False
                    s_prime.relations['apple_touching_stack'] = False
                    new_probs.append(0.8*probs[i])
                    new_states.append(deepcopy(s_prime))
                    s_prime.relations['gripper_touching_stack'] = True
                    s_prime.relations['apple_touching_stack'] = True
                    new_probs.append(0.2*probs[i])
                    new_states.append(deepcopy(s_prime))
            probs = new_probs
            states = new_states
            new_probs = []
            new_states = []
            for i in range(len(states)):
                if not s.relations['gripper_above_drawer'] and not s.relations['gripper_below_drawer']:
                    s_prime = states[i]
                    s_prime.relations['gripper_touching_drawer'] = False
                    s_prime.relations['apple_touching_drawer'] = False
                    new_probs.append(0.8*probs[i])
                    new_states.append(deepcopy(s_prime))
                    s_prime.relations['gripper_touching_drawer'] = True
                    s_prime.relations['apple_touching_drawer'] = True
                    new_probs.append(0.2*probs[i])
                    new_states.append(deepcopy(s_prime))
            probs = new_probs
            states = new_states

            probs = norm_prob(probs)
            results = zip(probs, states)

        else:
            states = []
            probs = []
            if a.object == 'l' or a.object == 'bl' or a.object == 'fl':
                for i in range(3):
                    if i == 0:
                        if not s.relations['gripper_right_of_drawer']:
                            continue
                        prob = 0.333
                    elif i == 1:
                        if s.relations['gripper_left_of_drawer'] or (not
                                (s.relations['gripper_below_drawer'] or
                                 s.relations['gripper_above_drawer']) and not
                                (s_prime.relations['gripper_in_front_of_drawer'] or
                                 s_prime.relations['gripper_behind_drawer'])):
                            continue
                        s_prime.relations['gripper_right_of_drawer'] = False
                        prob = 0.333
                    else:
                        s_prime.relations['gripper_right_of_drawer'] = False
                        s_prime.relations['gripper_left_of_drawer'] = True
                        prob = 0.333

                    if a.object == 'bl':
                        for j in range(3):
                            if j == 0:
                                if not s.relations['gripper_in_front_of_drawer']:
                                    continue
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            elif j == 1:
                                if s.relations['gripper_behind_drawer'] or (not
                                        (s.relations['gripper_below_drawer'] or
                                         s.relations['gripper_above_drawer']) and not
                                        (s_prime.relations['gripper_left_of_drawer'] or
                                         s_prime.relations['gripper_right_of_drawer'])):
                                    continue
                                s_prime.relations['gripper_in_front_of_drawer'] = False
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            else:
                                s_prime.relations['gripper_in_front_of_drawer'] = False
                                s_prime.relations['gripper_behind_drawer'] = True
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                    elif a.object == 'fl':
                        for j in range(3):
                            if j == 0:
                                if not s.relations['gripper_behind_drawer']:
                                    continue
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            elif j == 1:
                                if s.relations['gripper_in_front_of_drawer'] or (not
                                        (s.relations['gripper_below_drawer'] or
                                         s.relations['gripper_above_drawer']) and not
                                        (s_prime.relations['gripper_left_of_drawer'] or
                                         s_prime.relations['gripper_right_of_drawer'])):
                                    continue
                                s_prime.relations['gripper_behind_drawer'] = False
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            else:
                                s_prime.relations['gripper_in_front_of_drawer'] = True
                                s_prime.relations['gripper_behind_drawer'] = False
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                    else:
                        probs.append(prob)
                        states.append(deepcopy(s_prime))

                # drawer close chance
                if not s.relations['drawer_closing_stack'] and \
                        not (s.relations['gripper_above_drawer'] or s.relations['gripper_below_drawer']):
                    new_probs = []
                    new_states = []
                    for i in range(len(states)):
                        s_prime = states[i]
                        if not s_prime.relations['gripper_left_of_drawer'] and \
                                not (s.relations['gripper_behind_drawer'] or
                                     s.relations['gripper_in_front_of_drawer']):
                            new_probs.append(0.5*probs[i])
                            new_states.append(deepcopy(s_prime))
                            s_prime.relations['gripper_right_of_drawer'] = True
                            s_prime.relations['drawer_closing_stack'] = True
                            s_prime.relations['gripper_in_front_of_drawer'] = False
                            s_prime.relations['gripper_behind_drawer'] = False
                            new_probs.append(0.5*probs[i])
                            new_states.append(s_prime)
                        else:
                            new_probs.append(probs[i])
                            new_states.append(s_prime)
                    probs = new_probs
                    states = new_states

            elif a.object == 'r' or a.object == 'br' or a.object == 'fr':
                for i in range(3):
                    if i == 0:
                        if not s.relations['gripper_left_of_drawer']:
                            continue
                        prob = 0.333
                    elif i == 1:
                        if s.relations['gripper_right_of_drawer'] or (not
                                (s.relations['gripper_below_drawer'] or
                                 s.relations['gripper_above_drawer']) and not
                                (s_prime.relations['gripper_in_front_of_drawer'] or
                                 s_prime.relations['gripper_behind_drawer'])):
                            continue
                        s_prime.relations['gripper_left_of_drawer'] = False
                        prob = 0.333
                    else:
                        s_prime.relations['gripper_left_of_drawer'] = False
                        s_prime.relations['gripper_right_of_drawer'] = True
                        prob = 0.333
                    if a.object == 'br':
                        for j in range(3):
                            if j == 0:
                                if not s.relations['gripper_in_front_of_drawer']:
                                    continue
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            elif j == 1:
                                if s.relations['gripper_behind_drawer'] or (not
                                        (s.relations['gripper_below_drawer'] or
                                         s.relations['gripper_above_drawer']) and not
                                        (s_prime.relations['gripper_left_of_drawer'] or
                                         s_prime.relations['gripper_right_of_drawer'])):
                                    continue
                                s_prime.relations['gripper_in_front_of_drawer'] = False
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            else:
                                s_prime.relations['gripper_in_front_of_drawer'] = False
                                s_prime.relations['gripper_behind_drawer'] = True
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                    elif a.object == 'fr':
                        for j in range(3):
                            if j == 0:
                                if not s.relations['gripper_behind_drawer']:
                                    continue
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            elif j == 1:
                                if s.relations['gripper_in_front_of_drawer'] or (not
                                        (s.relations['gripper_below_drawer'] or
                                         s.relations['gripper_above_drawer']) and not
                                        (s_prime.relations['gripper_left_of_drawer'] or
                                         s_prime.relations['gripper_right_of_drawer'])):
                                    continue
                                s_prime.relations['gripper_behind_drawer'] = False
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                            else:
                                s_prime.relations['gripper_behind_drawer'] = False
                                s_prime.relations['gripper_in_front_of_drawer'] = True
                                prob2 = prob*0.333
                                states.append(deepcopy(s_prime))
                                probs.append(prob2)
                    else:
                        probs.append(prob)
                        states.append(deepcopy(s_prime))
            elif a.object == 'b':
                for i in range(3):
                    if i == 0:
                        if not s.relations['gripper_in_front_of_drawer']:
                            continue
                        prob = 0.333
                        states.append(deepcopy(s_prime))
                        probs.append(prob)
                    elif i == 1:
                        if s.relations['gripper_behind_drawer'] or (not
                                (s.relations['gripper_below_drawer'] or
                                 s.relations['gripper_above_drawer']) and not
                                (s_prime.relations['gripper_left_of_drawer'] or
                                 s_prime.relations['gripper_right_of_drawer'])):
                            continue
                        s_prime.relations['gripper_in_front_of_drawer'] = False
                        prob = 0.333
                        states.append(deepcopy(s_prime))
                        probs.append(prob)
                    else:
                        s_prime.relations['gripper_in_front_of_drawer'] = False
                        s_prime.relations['gripper_behind_drawer'] = True
                        prob = 0.333
                        states.append(deepcopy(s_prime))
                        probs.append(prob)
            elif a.object == 'u':
                for i in range(3):
                    if i == 0:
                        if not s.relations['gripper_behind_drawer']:
                            continue
                        prob = 0.333
                        states.append(deepcopy(s_prime))
                        probs.append(prob)
                    elif i == 1:
                        if s.relations['gripper_in_front_of_drawer'] or (not
                                (s.relations['gripper_below_drawer'] or
                                 s.relations['gripper_above_drawer']) and not
                                (s_prime.relations['gripper_left_of_drawer'] or
                                 s_prime.relations['gripper_right_of_drawer'])):
                            continue
                        s_prime.relations['gripper_behind_drawer'] = False
                        prob = 0.333
                        states.append(deepcopy(s_prime))
                        probs.append(prob)
                    else:
                        s_prime.relations['gripper_behind_drawer'] = False
                        s_prime.relations['gripper_in_front_of_drawer'] = True
                        prob = 0.333
                        states.append(deepcopy(s_prime))
                        probs.append(prob)
            elif a.object == 'stack':
                if not s.relations['drawer_closing_stack'] and \
                        not (s.relations['gripper_above_drawer'] or s.relations['gripper_below_drawer']):
                    if not s.relations['gripper_left_of_drawer'] and \
                            not (s.relations['gripper_behind_drawer'] or s.relations['gripper_in_front_of_drawer']):
                        probs.append(0.75)
                        states.append(deepcopy(s_prime))
                        s_prime.relations['gripper_right_of_drawer'] = True
                        s_prime.relations['drawer_closing_stack'] = True
                        s_prime.relations['gripper_in_front_of_drawer'] = False
                        s_prime.relations['gripper_behind_drawer'] = False
                        probs.append(0.25)
                        states.append(s_prime)
                    else:
                        probs.append(1.0)
                        states.append(s_prime)
                else:
                    probs.append(1.0)
                    states.append(s_prime)
            elif a.object == 'drawer':
                if not s.relations['drawer_closing_stack'] and \
                        not (s.relations['gripper_above_drawer'] or s.relations['gripper_below_drawer']):
                    if not s.relations['gripper_left_of_drawer'] and \
                            not (s.relations['gripper_behind_drawer'] or s.relations['gripper_in_front_of_drawer']):
                        probs.append(0.9)
                        states.append(deepcopy(s_prime))
                        s_prime.relations['gripper_right_of_drawer'] = True
                        s_prime.relations['drawer_closing_stack'] = True
                        s_prime.relations['gripper_in_front_of_drawer'] = False
                        s_prime.relations['gripper_behind_drawer'] = False
                        probs.append(0.1)
                        states.append(s_prime)
                    else:
                        probs.append(1.0)
                        states.append(s_prime)
                else:
                    probs.append(1.0)
                    states.append(s_prime)

            new_probs = []
            new_states = []
            for i in range(len(states)):
                if not s.relations['gripper_above_drawer']:
                    s_prime = states[i]
                    s_prime.relations['gripper_touching_stack'] = False
                    new_probs.append(0.8*probs[i])
                    new_states.append(deepcopy(s_prime))
                    s_prime.relations['gripper_touching_stack'] = True
                    new_probs.append(0.2*probs[i])
                    new_states.append(deepcopy(s_prime))
            probs = new_probs
            states = new_states
            new_probs = []
            new_states = []
            for i in range(len(states)):
                if not s.relations['gripper_above_drawer'] and not s.relations['gripper_below_drawer']:
                    s_prime = states[i]
                    s_prime.relations['gripper_touching_drawer'] = False
                    new_probs.append(0.8*probs[i])
                    new_states.append(deepcopy(s_prime))
                    s_prime.relations['gripper_touching_drawer'] = True
                    new_probs.append(0.2*probs[i])
                    new_states.append(deepcopy(s_prime))
            probs = new_probs
            states = new_states

            # apple pushing cases
            if not (s.relations['apple_below_gripper'] or s.relations['apple_above_gripper']):
                if a.object == 'apple' \
                        or (a.object == 'l' and s.relations['apple_left_of_gripper'] and
                            not s.relations['apple_behind_gripper'] and not s.relations['apple_in_front_of_gripper']) \
                        or (a.object == 'b' and s.relations['apple_behind_gripper'] and
                            not s.relations['apple_left_of_gripper'] and not s.relations['apple_right_of_gripper']) \
                        or (a.object == 'r' and s.relations['apple_right_of_gripper'] and
                            not s.relations['apple_in_front_of_gripper'] and not s.relations['apple_behind_gripper']) \
                        or (a.object == 'f' and s.relations['apple_in_front_of_gripper'] and
                            not s.relations['apple_left_of_gripper'] and not s.relations['apple_right_of_gripper']) \
                        or (a.object == 'bl' and s.relations['apple_behind_gripper']
                            and s.relations['apple_left_of_gripper']) \
                        or (a.object == 'br' and s.relations['apple_behind_gripper']
                            and s.relations['apple_right_of_gripper']) \
                        or (a.object == 'fr' and s.relations['apple_in_front_of_gripper']
                            and s.relations['apple_right_of_gripper']) \
                        or (a.object == 'fl' and s.relations['apple_in_front_of_gripper']
                            and s.relations['apple_left_of_gripper']):
                    new_states = []
                    new_probs = []
                    # special case: pushing apple in drawer
                    if (a.object == 'r' or a.object == 'br' or a.object == 'fr' or a.object == 'drawer') \
                            and (s.relations['apple_right_of_gripper'] and
                                 not (s.relations['gripper_in_front_of_drawer'] or
                                      s.relations['gripper_behind_drawer'])) \
                            and s.relations['apple_above_drawer'] and not s.relations['drawer_closing_stack']:
                        for i in range(len(states)):
                            s_prime = states[i]
                            alpha = 0.75
                            if a.object == 'r':
                                alpha = 0.5
                            elif a.object == 'br' or a.object == 'fr':
                                alpha = 0.3
                            new_probs.append((1.0 - alpha) * probs[i])
                            new_states.append(deepcopy(s_prime))
                            s_prime.relations['apple_above_drawer'] = False
                            s_prime.relations['apple_left_of_drawer'] = False
                            s_prime.relations['apple_right_of_drawer'] = False
                            s_prime.relations['apple_in_front_of_drawer'] = False
                            s_prime.relations['apple_behind_drawer'] = False
                            s_prime.relations['apple_below_gripper'] = True
                            new_probs.append(alpha * probs[i])
                            new_states.append(deepcopy(s_prime))
                        probs = new_probs
                        states = new_states
                    else:
                        for i in range(len(states)):
                            s_prime = states[i]
                            new_probs.append(0.5 * probs[i])
                            new_states.append(deepcopy(s_prime))
                            if s_prime.relations['gripper_left_of_drawer'] or \
                                    s_prime.relations['gripper_right_of_drawer'] or \
                                    s_prime.relations['gripper_in_front_of_drawer'] or \
                                    s_prime.relations['gripper_behind_drawer'] or \
                                    s_prime.relations['gripper_above_drawer'] or \
                                    s_prime.relations['gripper_below_drawer']:
                                s_prime.relations['apple_left_of_drawer'] = s_prime.relations['gripper_left_of_drawer']
                                s_prime.relations['apple_right_of_drawer'] = s_prime.relations['gripper_right_of_drawer']
                                s_prime.relations['apple_in_front_of_drawer'] = \
                                    s_prime.relations['gripper_in_front_of_drawer']
                                s_prime.relations['apple_behind_drawer'] = s_prime.relations['gripper_behind_drawer']
                            prob = 0.5 * probs[i]
                            for j in range(3):
                                s_prime.relations['apple_left_of_gripper'] = False
                                s_prime.relations['apple_right_of_gripper'] = False
                                if j == 0:
                                    prob2 = .2*prob
                                elif j == 1:
                                    s_prime.relations['apple_left_of_gripper'] = True
                                    prob2 = .4*prob
                                else:
                                    s_prime.relations['apple_right_of_gripper'] = True
                                    prob2 = .4*prob
                                for k in range(3):
                                    s_prime.relations['apple_in_front_of_gripper'] = False
                                    s_prime.relations['apple_behind_gripper'] = False
                                    if k == 0:
                                        prob3 = .2 * prob2
                                    elif k == 1:
                                        s_prime.relations['apple_in_front_of_gripper'] = True
                                        prob3 = .4 * prob2
                                    else:
                                        s_prime.relations['apple_behind_gripper'] = True
                                        prob3 = .4 * prob2
                                    s_prime.relations['apple_touching_stack'] = False
                                    s_prime.relations['apple_touching_gripper'] = True
                                    new_probs.append(0.4*prob3)
                                    new_states.append(deepcopy(s_prime))
                                    s_prime.relations['apple_touching_stack'] = True
                                    new_probs.append(0.1*prob3)
                                    new_states.append(deepcopy(s_prime))
                                    s_prime.relations['apple_touching_stack'] = False
                                    s_prime.relations['apple_touching_gripper'] = False
                                    new_probs.append(0.4*prob3)
                                    new_states.append(deepcopy(s_prime))
                                    s_prime.relations['apple_touching_stack'] = True
                                    new_probs.append(0.1*prob3)
                                    new_states.append(deepcopy(s_prime))
                        probs = new_probs
                        states = new_states

            probs = norm_prob(probs)
            results = zip(probs, states)

    elif a.action_type == Action.OPEN_GRIPPER:
        # object fall case
        # open gripper, remove object
        s_prime = deepcopy(s)
        s_prime.gripper_holding = ''
        s_prime.relations['gripper_open'] = True
        if s.gripper_holding == 'apple':
            if s.relations['gripper_below_drawer']:
                results.append((1.0, deepcopy(s_prime)))
            else:
                p_sum = 0.0
                states = []
                probs = []
                states.append(deepcopy(s_prime))
                probs.append(0.3)
                p_sum += 0.3

                base_prob = .7
                s_prime.relations['apple_below_gripper'] = True
                for i in range(3):
                    prob = base_prob
                    if i == 0:
                        if not s.relations['gripper_above_drawer']:
                            continue
                        s_prime.relations['apple_above_drawer'] = True
                        prob2 = prob * 0.333
                    elif i == 1:
                        prob2 = prob * 0.333
                        if s_prime.relations['drawer_closing_stack']:
                            continue
                        s_prime.relations['apple_above_drawer'] = False
                        s_prime.relations['apple_below_drawer'] = False
                    else:
                        prob2 = prob * 0.333
                        s_prime.relations['apple_above_drawer'] = False
                        s_prime.relations['apple_below_drawer'] = True
                    for j in range(3):
                        s_prime.relations['apple_left_of_gripper'] = False
                        s_prime.relations['apple_right_of_gripper'] = False
                        if j == 0:
                            prob3 = prob2 * .5
                        elif j == 1:
                            prob3 = prob2 * .25
                            s_prime.relations['apple_left_of_gripper'] = True
                        else:
                            prob3 = prob2 * .25
                            s_prime.relations['apple_right_of_gripper'] = True
                        for k in range(3):
                            s_prime.relations['apple_in_front_of_gripper'] = False
                            s_prime.relations['apple_behind_gripper'] = False
                            if k == 0:
                                prob4 = prob3 * .5
                            elif j == 2:
                                prob4 = prob3 * .25
                                s_prime.relations['apple_in_front_of_gripper'] = True
                            else:
                                prob4 = prob3 * .25
                                s_prime.relations['apple_behind_gripper'] = True
                            states.append(deepcopy(s_prime))
                            probs.append(prob4)
                            p_sum += prob4
                for i in range(len(probs)):
                    probs[i] /= p_sum
                results = zip(probs, states)
        else:
            results.append((1.0, s_prime))

    elif a.action_type == Action.CLOSE_GRIPPER:
        s_prime = deepcopy(s)
        s_prime.relations['gripper_open'] = False
        if not (s.relations['apple_left_of_gripper'] or s.relations['apple_right_of_gripper'] or
                s.relations['apple_above_gripper'] or s.relations['apple_below_gripper'] or
                s.relations['apple_in_front_of_gripper'] or s.relations['apple_behind_gripper']):
            s_prime.gripper_holding = 'apple'
            results.append((1.0, s_prime))
        elif s.relations['gripper_touching_drawer'] and s.relations['gripper_right_of_drawer'] and not \
                (s.relations['gripper_above_drawer'] or s.relations['gripper_below_drawer'] or
                 s.relations['gripper_in_front_of_drawer'] or s.relations['gripper_behind_drawer']):
            results.append((0.8, deepcopy(s_prime)))
            s_prime.gripper_holding = 'drawer'
            results.append((0.2, s_prime))
        else:
            results.append((1.0, s_prime))

    elif a.action_type == Action.RAISE_ARM:
        s_prime = deepcopy(s)
        probs = []
        states = []
        if s.gripper_holding == 'drawer' \
                or (s.relations['gripper_below_drawer'] and not
                    (s.relations['gripper_left_of_drawer'] or s.relations['gripper_right_of_drawer'] or
                     s.relations['gripper_in_front_of_drawer'] or s.relations['gripper_behind_drawer'])):
            results.append((1.0, s_prime))
        else:
            if s.gripper_holding == 'apple':
                if s.relations['gripper_below_drawer']:
                    s_prime.relations['gripper_below_drawer'] = False
                    s_prime.relations['apple_below_drawer'] = False
                    states.append(s_prime)
                    probs.append(1.0)
                elif not s.relations['gripper_above_drawer']:
                    states.append(deepcopy(s_prime))
                    probs.append(0.5)
                    s_prime.relations['gripper_above_drawer'] = True
                    s_prime.relations['apple_above_drawer'] = True
                    s_prime.relations['gripper_touching_drawer'] = False
                    s_prime.relations['gripper_touching_stack'] = False
                    s_prime.relations['apple_touching_drawer'] = False
                    s_prime.relations['apple_touching_stack'] = False
                    states.append(s_prime)
                    probs.append(0.5)
                else:
                    states.append(s_prime)
                    probs.append(1.0)
            else:
                if s.relations['gripper_below_drawer']:
                    s_prime.relations['gripper_below_drawer'] = False
                    states.append(deepcopy(s_prime))
                    probs.append(1.0)
                elif not s.relations['gripper_above_drawer']:
                    states.append(deepcopy(s_prime))
                    probs.append(0.5)
                    s_prime.relations['gripper_above_drawer'] = True
                    s_prime.relations['gripper_touching_drawer'] = False
                    s_prime.relations['gripper_touching_stack'] = False
                    states.append(deepcopy(s_prime))
                    probs.append(0.5)
                else:
                    states.append(deepcopy(s_prime))
                    probs.append(1.0)

                new_states = []
                new_probs = []
                for i in range(len(states)):
                    if s.relations['apple_above_gripper']:
                        s_prime = states[i]
                        new_states.append(deepcopy(s_prime))
                        new_probs.append(0.5*probs[i])
                        s_prime.relations['apple_above_gripper'] = False
                        new_states.append(deepcopy(s_prime))
                        new_probs.append(0.5*probs[i])
                    elif not s.relations['apple_below_gripper']:
                        s_prime = states[i]
                        s_prime.relations['apple_below_gripper'] = True
                        new_states.append(deepcopy(s_prime))
                        new_probs.append(probs[i])
                    else:
                        new_states.append(states[i])
                        new_probs.append(probs[i])
                results = zip(new_probs, new_states)

    elif a.action_type == Action.LOWER_ARM:
        if s.relations['gripper_below_drawer'] or s.gripper_holding == 'drawer':
            results.append((1.0, deepcopy(s)))
        else:
            states = []
            probs = []
            if s.gripper_holding == 'apple':
                if s.relations['gripper_above_drawer']:
                    s_prime = deepcopy(s)
                    states.append(deepcopy(s_prime))
                    probs.append(0.5)
                    if (s.relations['drawer_closing_stack'] and
                        (s.relations['gripper_left_of_drawer'] or s.relations['gripper_right_of_drawer'] or
                         s.relations['gripper_in_front_of_drawer'] or s.relations['gripper_behind_drawer'])) or \
                            not s.relations['drawer_closing_stack']:
                        s_prime.relations['gripper_above_drawer'] = False
                        s_prime.relations['apple_above_drawer'] = False
                        states.append(deepcopy(s_prime))
                        probs.append(0.5)
                else:
                    s_prime = deepcopy(s)
                    states.append(deepcopy(s_prime))
                    probs.append(0.5)
                    s_prime.relations['gripper_below_drawer'] = True
                    s_prime.relations['apple_below_drawer'] = True
                    states.append(deepcopy(s_prime))
                    probs.append(0.5)
            else:
                if s.relations['gripper_above_drawer']:
                    s_prime = deepcopy(s)
                    states.append(deepcopy(s_prime))
                    probs.append(0.5)
                    if s.relations['gripper_left_of_drawer'] or s.relations['gripper_right_of_drawer'] \
                            or s.relations['gripper_in_front_of_drawer'] or s.relations['gripper_behind_drawer']:
                        s_prime.relations['gripper_above_drawer'] = False
                        states.append(deepcopy(s_prime))
                        probs.append(0.5)
                else:
                    s_prime = deepcopy(s)
                    states.append(deepcopy(s_prime))
                    probs.append(0.5)
                    s_prime.relations['gripper_below_drawer'] = True
                    states.append(deepcopy(s_prime))
                    probs.append(0.5)
                new_states = []
                new_probs = []
                for i in range(len(states)):
                    if s.relations['apple_below_gripper']:
                        s_prime = states[i]
                        new_states.append(deepcopy(s_prime))
                        new_probs.append(0.5*probs[i])
                        s_prime.relations['apple_below_gripper'] = False
                        new_states.append(deepcopy(s_prime))
                        new_probs.append(0.5*probs[i])
                    elif not s.relations['apple_above_gripper']:
                        s_prime = states[i]
                        s_prime.relations['apple_above_gripper'] = True
                        new_states.append(deepcopy(s_prime))
                        new_probs.append(probs[i])
                    else:
                        new_states.append(states[i])
                        new_probs.append(probs[i])
                states = new_states
                probs = new_probs
            probs = norm_prob(probs)
            results = zip(probs, states)

    elif a.action_type == Action.RESET_ARM:
        if s.gripper_holding == 'drawer':
            results.append((1.0, s))
        else:
            s_prime = deepcopy(s)
            s_prime.relations['gripper_above_drawer'] = False
            s_prime.relations['gripper_below_drawer'] = False
            s_prime.relations['gripper_in_front_of_drawer'] = True
            s_prime.relations['gripper_behind_drawer'] = False
            s_prime.relations['gripper_left_of_drawer'] = True
            s_prime.relations['gripper_right_of_drawer'] = False
            s_prime.relations['gripper_touching_drawer'] = False
            s_prime.relations['gripper_touching_stack'] = False

            if s.gripper_holding == 'apple':
                s_prime.relations['apple_above_drawer'] = False
                s_prime.relations['apple_below_drawer'] = False
                s_prime.relations['apple_in_front_of_drawer'] = True
                s_prime.relations['apple_behind_drawer'] = False
                s_prime.relations['apple_left_of_drawer'] = True
                s_prime.relations['apple_right_of_drawer'] = False
                s_prime.relations['apple_touching_drawer'] = False
                s_prime.relations['apple_touching_stack'] = False
                results.append((1.0, s_prime))
            else:
                states = []
                probs = []

                s_prime.relations['apple_above_gripper'] = False
                s_prime.relations['apple_below_gripper'] = False
                for i in range(3):
                    prob = 0.2
                    if i == 1:
                        s_prime.relations['apple_above_gripper'] = True
                    else:
                        s_prime.relations['apple_below_gripper'] = True
                        s_prime.relations['apple_above_gripper'] = False
                        prob = 0.6
                    for j in range(3):
                        s_prime.relations['apple_left_of_gripper'] = False
                        s_prime.relations['apple_right_of_gripper'] = False

                        if j == 0:
                            prob2 = prob * 1.0/41.0
                        elif j == 1:
                            prob2 = prob * 8.0/41.0
                            s_prime.relations['apple_left_of_gripper'] = True
                        else:
                            prob2 = prob * 32.0/41.0
                            s_prime.relations['apple_right_of_gripper'] = True
                        for k in range(3):
                            s_prime.relations['apple_in_front_of_gripper'] = False
                            s_prime.relations['apple_behind_gripper'] = False
                            if k == 0:
                                prob3 = prob2 * 1.0/41.0
                            elif j == 2:
                                prob3 = prob2 * 1.0/41.0
                                s_prime.relations['apple_in_front_of_gripper'] = True
                            else:
                                prob3 = prob2 * 39.0/41.0
                                s_prime.relations['apple_behind_gripper'] = True
                            states.append(deepcopy(s_prime))
                            probs.append(prob3)
                results = zip(probs, states)

    if len(results) == 0:
        results.append((1.0, s))
    return results


def norm_prob(probs):
    p_sum = 0.0
    for i in range(len(probs)):
        p_sum += probs[i]
    for i in range(len(probs)):
        probs[i] /= p_sum
    return probs
