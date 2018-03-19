#!/usr/bin/env python

from copy import deepcopy

from task_sim.msg import Action
from task_sim.oomdp.oomdp_classes import Box, Container, Drawer, Gripper, Item, Lid, Stack
from task_sim.oomdp.oo_state import OOState
import task_sim.oomdp.oomdp_utils as utils


on = -1
left = 0
back_left = 1
back = 2
back_right = 3
right = 4
front_right = 5
front = 6
front_left = 7

def get_direction(x0, y0, x1, y1):
    """Calculate a rought direction from (x0, y0) to (x1, y1)"""

    # same point special case
    if x0 == x1 and y0 == y1:
        return on

    # vertical special cases
    if x0 == x1:
        if y1 > y0:
            return back
        else:
            return front

    slope = float(y1 - y0)/(x1 - x0)
    if x1 > x0:
        if slope < -2:
            return front
        elif slope < -.5:
            return front_right
        elif slope < .5:
            return right
        elif slope < 2:
            return back_right
        else:
            return back
    else:
        if slope < -2:
            return back
        elif slope < -.5:
            return back_left
        elif slope < .5:
            return left
        elif slope < 2:
            return front_left
        else:
            return front

def transition_function(state, action):
    """Calculate likely new states from executing an action in a given state

    Note: currently ignoring T_C until we find a reasonable way to represent it for our problem domain

    Returns:
        List of tuples in the form (probability, OO-MDP state)
    """
    results = []

    if action.action_type == Action.NOOP:
        results.append((state, 1.0))

    elif action.action_type == Action.GRASP:
        # point distribution for success mixed with point distribution for failure
        alpha = 0.8

        # success - gripper moves to object position and holds object
        success_state = deepcopy(state)
        obj = utils.get_object(success_state, action.name)
        if obj is None:
            alpha = 0
        else:
            gripper = utils.get_object(success_state, 'gripper')
            if obj.__class__ == Drawer:
                gripper.x = obj.x + (obj.width - 1)/2 + 1
                gripper.y = obj.y
                gripper.z = 2
            else:
                gripper.x = obj.x
                gripper.y = obj.y
                gripper.z = obj.z
            gripper.holding = obj.name
            gripper.closed = True

            results.append((success_state, alpha*1.0))

        # failure - no change
        results.append((state, (1 - alpha)*1.0))

    elif action.action_type == Action.PLACE:
        gripper = utils.get_object(state, 'gripper')
        new_z = utils.ray_trace(action.position.x, action.position.y)

        # point distribution for success mixed with point distribution for failure
        alpha = 0.8

        # success - gripper moves to place position and releases object
        success_state = deepcopy(state)
        obj = utils.gripper_object(success_state)
        gripper_move = utils.get_object(success_state, 'gripper')
        if obj is not None and obj.__class__ == Drawer:
            alpha = 0
        else:
            if obj is not None:
                if obj.__class__ == Container:
                    obj.x = action.position.x + obj.x - gripper_move.x
                    obj.y = action.position.y + obj.y - gripper_move.y
                else:
                    obj.x = action.position.x
                    obj.y = action.position.y
                obj.z = new_z
            gripper_move.x = action.position.x
            gripper_move.y = action.position.y
            gripper_move.z = new_z
            gripper_move.closed = False
            gripper_move.holding = ''
            results.append((success_state, alpha*1.0))

        # failure - no change
        results.append((state, (1 - alpha)*1.0))

    elif action.action_type == Action.OPEN_GRIPPER:
        gripper = utils.get_object(state, 'gripper')
        if not gripper.closed:
            results.append((state, 1.0))
        else:
            success_state = deepcopy(state)
            gripper = utils.get_object(state, 'gripper')
            gripper.closed = False
            obj = utils.gripper_object(success_state)
            if obj is None:
                results.append((success_state, 1.0))
            else:
                states = [success_state]
                probs = [1.0]
                prob_sum = 0
                decay = 1.0
                for z in range(obj.z - 1, -1, -1):
                    decay *= 0.8
                    if obj.__class__ == Item:
                        for i in range(obj.z - z, obj.z + z + 1):
                            for j in range(obj.z - z, obj.z + z + 1):
                                states.append(utils.copy_state_move_object(success_state, obj.unique_name, i, j, z - obj.z))
                                p = 1.0/(pow(2*(obj.z - z) + 1, 2))
                                p *= decay
                                probs.append(p)
                                prob_sum += p
                    elif obj.__class__ == Container:
                        for i in range(int((obj.z - z)/2), int((obj.z + z)/2) + 1):
                            for j in range(int((obj.z - z)/2), int((obj.z + z)/2) + 1):
                                states.append(utils.copy_state_move_object(success_state, obj.unique_name, i, j, z - obj.z))
                                p = 1.0/(pow(2*(int((obj.z - z)/2)) + 1, 2))
                                p *= decay
                                probs.append(p)
                                prob_sum += p
                    elif obj.__class__ == Lid:
                        states.append(utils.copy_state_move_object(success_state, obj, 0, 0, z - obj.z))
                        probs.append(decay)
                for i in range(len(probs)):
                    probs[i] /= prob_sum
                results.extend(zip(states, probs))

    elif action.action_type == Action.CLOSE_GRIPPER:
        gripper = utils.get_object(state, 'gripper')
        if gripper.closed:
            results.append((state, 1.0))
        else:
            success_state = deepcopy(state)
            gripper = utils.get_object(state, 'gripper')
            gripper.closed = True
            if 'gripper_on_apple' and 'gripper_level_with_apple':
                gripper.holding = 'apple'
                results.append((success_state, 1.0))
            elif 'gripper_on_batteries' and 'gripper_level_with_batteries':
                gripper.holding = 'batteries'
                results.append((success_state, 1.0))
            elif 'gripper_on_flashlight' and 'gripper_level_with_flashlight':
                gripper.holding = 'flashlight'
                results.append((success_state, 1.0))
            elif 'gripper_on_granola' and 'gripper_level_with_granola':
                gripper.holding = 'granola'
                results.append((success_state, 1.0))
            elif 'gripper_on_knife' and 'gripper_level_with_knife':
                gripper.holding = 'knife'
                results.append((success_state, 1.0))
            elif 'gripper_on_small' and 'gripper_level_with_small':
                gripper.holding = 'small'
                results.append((success_state, 1.0))
            elif 'gripper_on_lid' and 'gripper_level_with_lid':
                failure_state = deepcopy(success_state)
                gripper.holding = 'lid'
                results.append((success_state, 0.1))
                results.append((failure_state, 0.9))
            elif 'gripper_touching_drawer' and 'gripper_right_of_drawer' and 'gripper_level_with_drawer':
                failure_state = deepcopy(success_state)
                gripper.holding = 'drawer'
                results.append((success_state, 0.2))
                results.append((failure_state, 0.8))
            elif 'gripper_on_large' and 'gripper_level_with_large':
                failure_state = deepcopy(success_state)
                gripper.holding = 'large'
                results.append((success_state, 0.875))
                results.append((failure_state, 0.125))

    elif action.action_type == Action.MOVE_ARM:
        pass

    elif action.action_type == Action.RAISE_ARM:
        alpha = 1.0
        gripper = utils.get_object(state, 'gripper')
        if 'gripper_on_lid' in state.relations and 'gripper_below_lid' in state.relations:
            alpha *= 0.8
        if 'gripper_on_drawer' in state.relations and 'gripper_below_drawer' in state.relations:
            alpha *= 0.8
        if 'gripper_on_stack' in state.relations and 'gripper_below_stack' in state.relations:
            alpha *= 0.8
        if 'gripper_on_small' in state.relations and 'gripper_below_small' in state.relations:
            alpha *= 0.8
        if 'gripper_on_large' in state.relations and 'gripper_below_large' in state.relations:
            alpha *= 0.8
        if gripper.holding in ['lid', 'small', 'large']:
            alpha *= 0.8
        success_state = deepcopy(state)
        gripper = utils.get_object(success_state, 'gripper')
        gripper.z += 1
        if gripper.z > 4:
            gripper.z = 4
        obj = utils.gripper_object(success_state)
        obj.z += 1
        if obj.z > 4:
            obj.z = 4
        results.append((success_state, alpha*1.0))

        # failure - no change
        results.append((state, (1 - alpha)*1.0))

    elif action.action_type == Action.LOWER_ARM:
        alpha = 1.0
        if 'gripper_on_lid' in state.relations and 'gripper_level_with_lid' in state.relations \
                or 'gripper_on_small' in state.relations and 'gripper_level_with_small' in state.relations \
                or 'gripper_on_large' in state.relations and 'gripper_level_with_large' in state.relations:
            alpha = 0
        else:
            gripper = utils.get_object(state, 'gripper')
            if 'gripper_on_drawer' in state.relations and 'gripper_above_drawer' in state.relations:
                alpha *= 0.8
            if 'gripper_on_stack' in state.relations and 'gripper_above_stack' in state.relations:
                alpha *= 0.8
            if gripper.holding in ['lid', 'small', 'large']:
                alpha *= 0.8
            success_state = deepcopy(state)
            gripper = utils.get_object(success_state, 'gripper')
            gripper.z -= 1
            if gripper.z < 0:
                gripper.z = 0
            obj = utils.gripper_object(success_state)
            obj.z -= 1
            if obj.z < 0:
                obj.z = 0
            results.append((success_state, alpha*1.0))

        # failure - no change
        results.append((state, (1 - alpha)*1.0))

    elif action.action_type == Action.RESET_ARM:
        # point distribution for success mixed with point distribution for failure
        alpha = 0.8

        # success - gripper moves to object position and holds object
        success_state = deepcopy(state)
        gripper = utils.get_object(success_state, 'gripper')
        gripper.x = 8
        gripper.y = 1
        gripper.z = 2

        results.append((success_state, alpha*1.0))

        # failure - no change
        results.append((state, (1 - alpha)*1.0))

    return results
