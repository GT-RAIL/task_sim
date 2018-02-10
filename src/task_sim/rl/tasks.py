#!/usr/bin/env python
# Stores the tasks for each of the learners

import sys
import os
import numpy as np

import rospy

from task_sim.msg import State, Action, Status
from task_sim.srv import QueryStatus, SelectAction
from task_sim.data_utils import DataUtils

# Class definitions

class Task(object):
    """Abstract super class that all task definitions implement. Implements a
    simple timeout check by default"""

    def __init__(self,
        success_reward=1,
        fail_penalty=-1,
        default_reward=0,
        timeout=100
    ):
        self.timeout = timeout
        self.success_reward = success_reward
        self.fail_penalty = fail_penalty
        self.default_reward = default_reward
        self._num_steps = 0

    def actions(self, state):
        """
        Return the actions available to the agent at the state. The action
        representation is upto the task specification. Returns a list
        """
        raise NotImplementedError("Don't know what actions are available")

    def create_action_msg(self, state, action):
        """
        Given the desired state and action, create an action message
        """
        raise NotImplementedError("Don't know how to translate the action to a msg")

    def R(self, state, action=None):
        """
        Reward at state. Optional action makes this into the reward for
        performing the action at state.
        state:  some state specification
        action: some action specification
        return: float
        """
        if self._num_steps >= self.timeout:
            return self.fail_penalty

        return self.default_reward

    def is_done(self, state):
        """
        Returns a Status code of whether the task is complete or not
        state:  some state specification
        return: task_sim.Status
        """
        self._num_steps += 1
        if self._num_steps >= self.timeout:
            return Status.TIMEOUT

        return Status.IN_PROGRESS


# Debug task definitions

class DebugTask1(Task):
    """
    Goal is to grab any X different objects that can be picked up and then
    releasing them. Fail occurs if any of the objects is lost.

    Uses the (Action, Object, Offset) parameterization of available actions
    """

    GRABBABLE_OBJECTS = ["Apple", "Batteries", "Flashlight", "Granola", "Knife"]

    def __init__(
        self, num_to_grab=2, grab_reward=1, time_penalty=-0.04,
        fail_penalty=-1, timeout=100
    ):
        super(DebugTask1, self).__init__(
            grab_reward,  # Success reward
            fail_penalty,
            time_penalty, # Default reward
            timeout
        )

        self.grabbed_objects = []
        self.num_to_grab = num_to_grab
        self.object_in_gripper = ''

    def _is_fail(self, state):
        """Failure condition if any of the objects is lost"""
        failed = False
        for obj in state.objects:
            failed = failed or obj.lost
        return failed

    def actions(self, state):
        # All actions are available at a given state
        return DataUtils.get_action_obj_offset_candidates(state)

    def create_action_msg(self, state, action):
        # This is simply a wrapper to the DataUtils function
        return DataUtils.msg_from_action_obj_offset(state, action)

    def R(self, state, action=None):
        # First check to see if there was an object that we were tracking and
        # if that object has been released. Also make sure that the object in
        # the gripper is a grabbable object and not one we have grabbed before.
        if (
            self.object_in_gripper
            and not state.object_in_gripper
            and self.object_in_gripper in DebugTask1.GRABBABLE_OBJECTS
            and self.object_in_gripper not in self.grabbed_objects
        ):
            # This is a valid grab that should be rewarded. Update the list of
            # objects that have been grabbed
            self.grabbed_objects.append(self.object_in_gripper)
            self.object_in_gripper = state.object_in_gripper
            return self.success_reward

        # Check for a fail - if an object has fallen off the gripper
        if self._is_fail(state):
            return self.fail_penalty

        # Update the object in the gripper
        self.object_in_gripper = state.object_in_gripper

        # Return the default reward post timeout check
        return super(DebugTask1, self).R(state, action)

    def is_done(self, state):
        # Check to see if we've picked up the requisite number of grabbable
        # objects.
        if len(self.grabbed_objects) >= self.num_to_grab:
            return self.success_reward

        # Check for a fail
        if self._is_fail(state):
            return self.fail_penalty

        # Otherwise, do a timeout check
        return super(DebugTask1, self).is_done(state)
