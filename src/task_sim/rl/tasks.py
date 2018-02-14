#!/usr/bin/env python
# Stores the tasks for each of the learners

from __future__ import division, print_function

import sys
import os
import pickle
import numpy as np

import rospy

from task_sim.msg import State, Action, Status
from task_sim.data_utils import DataUtils

# Class definitions

class Task(object):
    """Abstract super class that all task definitions implement. Implements a
    simple timeout check by default"""

    def __init__(self,
        success_reward=1,
        fail_penalty=-1,
        default_reward=0,
        timeout=100,
        viz=None,
        *args, **kwargs
    ):
        self.name = self.__class__.__name__
        self.timeout = timeout
        self.success_reward = success_reward
        self.fail_penalty = fail_penalty
        self.default_reward = default_reward
        self.viz = viz
        self.num_steps = 0
        self.world_state = None
        self.action = None

    def actions(self, agent_state):
        """
        Return the actions available to the agent at the state. The action
        representation is upto the task specification. Returns a list
        """
        raise NotImplementedError("Don't know what actions are available")

    def create_action_msg(self, action):
        """
        Given the desired state and action, create an action message
        """
        raise NotImplementedError("Don't know how to translate the action to a msg")

    def increment_steps(self):
        """Explicit call to increment the number of timesteps that occurred"""
        self.num_steps += 1

    def reset(self):
        """Reset the task state for another iteration"""
        self.num_steps = 0
        self.world_state = self.action = None

    def set_world_state(self, world_state, action):
        """Set the world state"""
        self.world_state = world_state
        self.action = action

    def get_agent_state(self):
        """Given a world state, returns the agent's observation model. By
        default the agent has access to the complete world state"""
        return self.world_state

    def reward(self):
        """
        Reward at state. Optional action makes this into the reward for
        performing the action at state. **MUST `set_world_state` before this**
        return: float
        """
        if self.num_steps >= self.timeout:
            return self.fail_penalty

        return self.default_reward

    def status(self):
        """
        Returns a Status code of whether the task is complete or not
        state:  some state specification
        return: task_sim.Status
        """
        if self.num_steps >= self.timeout:
            return Status.TIMEOUT

        return Status.IN_PROGRESS

    def _save(self):
        """
        Create the data structure to save the task
        """
        raise NotImplementedError("Don't know how to save the task")

    def save(self, filename=None):
        """
        Save the task. If filename is None, then simply return the parameters
        of the task to save. Internally calls _save
        :filename: location to save the task. Default: None
        """
        data = self._save()
        if filename is not None:
            with open(filename, 'wb') as fd:
                pickle.dump(data, fd)
        return data

    def _load(self, data):
        """
        Load the task from the data dictionary
        """
        raise NotImplementedError("Don't know how to load the task")

    def load(self, filename=None, data=None):
        """
        Load the task from a filename or from a data dictionary. One of the two
        MUST be specified. Internally calls _load. Returns the loaded object
        """
        if filename is not None and data is None:
            with open(filename, 'rb') as fd:
                data = pickle.load(fd)
        elif filename is None and data is not None:
            pass
        else:
            raise ValueError("Exactly one of filename and data must be specified")

        return self._load(data)


# Debug task definitions

class DebugTask1(Task):
    """
    Goal is to grab any X different objects that can be picked up and then
    releasing them. Fail occurs if any of the objects is lost.

    Uses the (Action, Object, Offset) parameterization of available actions
    """

    GRABBABLE_OBJECTS = ["Apple", "Batteries", "Flashlight", "Granola", "Knife"]

    def __init__(
        self,
        num_to_grab=2, state_vector_args={},
        grab_reward=1, time_penalty=-0.04,
        fail_penalty=-1, timeout=100,
        *args, **kwargs
    ):
        super(DebugTask1, self).__init__(
            grab_reward,  # Success reward
            fail_penalty,
            time_penalty, # Default reward
            timeout,
            *args, **kwargs
        )

        self.state_vector_args = state_vector_args
        self.grabbed_objects = []
        self.num_to_grab = num_to_grab
        self.object_in_gripper = ''

    def _is_fail(self):
        """Failure condition if any of the objects is lost"""
        failed = False
        for obj in self.world_state.objects:
            failed = failed or obj.lost
        return failed

    def actions(self, agent_state):
        # All actions are available at a given state

        # Action, Object, Offset
        # return DataUtils.get_action_obj_offset_candidates(self.world_state)

        # Action, Object, Target
        return DataUtils.get_semantic_action_candidates(self.world_state)

    def create_action_msg(self, action):
        # This is simply a wrapper to the DataUtils function

        # Action, Object, Offset
        # return DataUtils.msg_from_action_obj_offset(self.world_state, action)

        # Action, Object, Target
        return DataUtils.msg_from_semantic_action(self.world_state, action)

    def get_agent_state(self):
        return tuple(DataUtils.naive_state_vector(
            self.world_state,
            state_positions=self.state_vector_args.get('state_positions', False),
            state_semantics=self.state_vector_args.get('state_semantics', True),
            position_semantics=self.state_vector_args.get('position_semantics', True),
            history_buffer=self.state_vector_args.get('history_buffer', 0),
        ))

    def reset(self):
        super(DebugTask1, self).reset()
        self.grabbed_objects = []
        self.object_in_gripper = ''

    def reward(self):
        # First check to see if there was an object that we were tracking and
        # if that object has been released. Also make sure that the object in
        # the gripper is a grabbable object and not one we have grabbed before.
        if (
            self.object_in_gripper
            and not self.world_state.object_in_gripper
            and self.object_in_gripper in DebugTask1.GRABBABLE_OBJECTS
            and self.object_in_gripper not in self.grabbed_objects
        ):
            # This is a valid grab that should be rewarded. Update the list of
            # objects that have been grabbed
            self.grabbed_objects.append(self.object_in_gripper)
            self.object_in_gripper = self.world_state.object_in_gripper
            # print("Objects so far", self.grabbed_objects)
            return self.success_reward

        # Check for a fail - if an object has fallen off the table
        if self._is_fail():
            return self.fail_penalty

        # Update the object in the gripper
        self.object_in_gripper = self.world_state.object_in_gripper

        # Return the default reward post timeout check
        return super(DebugTask1, self).reward()

    def status(self):
        # Check to see if we've picked up the requisite number of grabbable
        # objects.
        if len(self.grabbed_objects) >= self.num_to_grab:
            return Status.COMPLETED

        # Check for a fail
        if self._is_fail():
            return Status.FAILED

        # Otherwise, do a timeout check
        return super(DebugTask1, self).status()

    def _save(self):
        return {
            "name": self.name,
            "state_vector_args": self.state_vector_args,
            "grab_reward": self.success_reward,
            "fail_penalty": self.fail_penalty,
            "time_penalty": self.default_reward,
            "timeout": self.timeout,
        }

    def _load(self, data):
        self.state_vector_args = data['state_vector_args']
        self.success_reward = data['grab_reward']
        self.fail_penalty = data['fail_penalty']
        self.time_penalty = data['time_penalty']
        self.timeout = data['timeout']
        return self

# Main task definitions

class Task1(Task):
    """
    Goal is to grab put the batteries and the flashlight in the drawer and the
    apple in the box.
    """

    def __init__(
        self, state_vector_args={}, rewards={},
        timeout=100,
        *args, **kwargs
    ):
        super(Task1, self).__init__(
            rewards.get('success_reward', 0.0), # Success Reward
            rewards.get('timeout_penalty', -100.0), # Fail Penalty
            rewards.get('time_penalty', -0.4), # Default Reward,
            timeout,
            *args, **kwargs
        )
        self.state_vector_args = state_vector_args

        # Setup the rewards
        self.reward_tests = {
            "drawer": self._is_drawer_complete,
            "box": self._is_box_complete,
            "batteries": self._is_batteries_complete,
            "flashlight": self._is_flashlight_complete,
            "apple": self._is_apple_complete,
        }
        self.reward_keys = self.reward_tests.keys()
        self.rewards = rewards
        self.rewards_awarded = { x: False for x in self.reward_keys }
        self.reward_counts = { x: 0 for x in self.reward_keys } # Not reset by default

        self._update_viz()

    def _save(self):
        return {
            'name': self.name,
            'rewards': self.rewards,
            'state_vector_args': self.state_vector_args,
            'timeout': self.timeout
        }

    def _load(self, data):
        self.state_vector_args = data['state_vector_args']
        self.rewards = data['rewards']
        self.success_reward = data['rewards'].get('success_reward', 0.0)
        self.fail_penalty = data['rewards']['timeout_penalty']
        self.default_reward = data['rewards']['time_penalty']
        self.timeout = data['timeout']
        return self

    def _update_viz(self):
        if self.viz is not None:
            self.viz.update_bar(
                [self.reward_counts[x] for x in self.reward_keys],
                "reward_counts",
                rownames=self.reward_keys
            )

    def _is_fail(self):
        failed = False
        for obj in self.world_state.objects:
            failed = failed or obj.lost
        return failed

    def _is_drawer_complete(self):
        for obj in self.world_state.objects:
            if obj.name.lower() == 'batteries':
                batteries = obj
            if obj.name.lower() == 'flashlight':
                flashlight = obj

        return (
            batteries.in_drawer
            and flashlight.in_drawer
            and self.world_state.drawer_opening < 1
        )

    def _is_box_complete(self):
        for obj in self.world_state.objects:
            if obj.name.lower() == 'apple':
                apple = obj
                break

        return (
            apple.in_box
            and self.world_state.box_position.x == self.world_state.lid_position.x
            and self.world_state.box_position.y == self.world_state.lid_position.y
        )

    def _is_apple_complete(self):
        for obj in self.world_state.objects:
            if obj.name.lower() == 'apple':
                return obj.in_box

    def _is_flashlight_complete(self):
        for obj in self.world_state.objects:
            if obj.name.lower() == 'flashlight':
                return obj.in_drawer

    def _is_batteries_complete(self):
        for obj in self.world_state.objects:
            if obj.name.lower() == 'batteries':
                return obj.in_drawer

    def actions(self, agent_state):
        # All actions are available at a given state

        # Action, Object, Offset
        # return DataUtils.get_action_obj_offset_candidates(self.world_state)

        # Action, Object, Target
        return DataUtils.get_semantic_action_candidates(self.world_state)

    def create_action_msg(self, action):
        # This is simply a wrapper to the DataUtils function

        # Action, Object, Offset
        # return DataUtils.msg_from_action_obj_offset(self.world_state, action)

        # Action, Object, Target
        return DataUtils.msg_from_semantic_action(self.world_state, action)

    def get_agent_state(self):
        return tuple(DataUtils.naive_state_vector(
            self.world_state,
            state_positions=self.state_vector_args.get('state_positions', False),
            state_semantics=self.state_vector_args.get('state_semantics', True),
            position_semantics=self.state_vector_args.get('position_semantics', True),
            history_buffer=self.state_vector_args.get('history_buffer', 0),
        ))

    def reset(self, reset_counts=False):
        super(Task1, self).reset()
        self.rewards_awarded = { x: False for x in self.reward_keys }
        if reset_counts:
            self.reward_counts = { x: 0 for x in self.reward_keys }

    def reward(self):
        # Check for an unrewarded subgoal success
        for goal in self.reward_keys:
            if self.reward_tests[goal]() and not self.rewards_awarded[goal]:
                rospy.logdebug("Rewarding: " + goal)
                self.reward_counts[goal] += 1
                self.rewards_awarded[goal] = True
                return self.rewards.get(goal, 0.0)

        # Check for a fail
        if self._is_fail():
            return self.rewards.get('fail_penalty', -100.0)

        # Check for a timeout
        return super(Task1, self).reward()


    def status(self):
        # Update the visulization
        self._update_viz()

        # Check if all subgoals are fulfilled at the moment
        if self._is_drawer_complete() and self._is_box_complete():
            return Status.COMPLETED

        # Check if we've failed
        if self._is_fail():
            return Status.FAILED

        # Otherwise do a timeout check
        return super(Task1, self).status()
