#!/usr/bin/env python
# Stores the tasks for each of the learners

from __future__ import division, print_function

import sys
import os
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

# Main task definitions

class Task1(Task):
    """
    Goal is to grab put the batteries and the flashlight in the drawer and the
    apple in the box.
    """

    def __init__(
        self, state_vector_args={},
        subgoal_reward=100.0, time_penalty=-0.4,
        fail_penalty=-100.0, timeout_penalty=-100.0,
        timeout=100,
        *args, **kwargs
    ):
        super(Task1, self).__init__(
            0.0, # Success Reward
            timeout_penalty,
            time_penalty, # Default Reward,
            timeout,
            *args, **kwargs
        )
        self.subgoal_reward = subgoal_reward
        self.fail_penalty = fail_penalty
        self.state_vector_args = state_vector_args
        self._drawer_objective_rewarded = False
        self._box_objective_rewarded = False

        # Initialize the counts of rewards. These are not reset by default
        self.reward_counts = { "Box": 0, "Drawer": 0 }
        self.reward_bar_window_name = "reward_counts"
        self._update_viz()

    def _update_viz(self):
        if self.viz is not None:
            self.viz.update_bar(
                self.reward_counts.values(), self.reward_bar_window_name,
                rownames=self.reward_counts.keys()
            )

    def _is_fail(self):
        failed = False
        for obj in self.world_state.objects:
            failed = failed or obj.lost
        return failed

    def _is_drawer_complete(self):
        for obj in self.world_state.objects:
            if obj.name.lower() == 'batteries':
                battery = obj
            if obj.name.lower() == 'flashlight':
                flashlight = obj

        return (
            battery.in_drawer
            and flashlight.in_drawer
            and self.world_state.drawer_opening == 0
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
        self._drawer_objective_rewarded = self._box_objective_rewarded = False
        if reset_counts:
            self.reward_counts = { "Box": 0, "Drawer": 0 }

    def reward(self):
        # Check for an unrewarded subgoal success
        if self._is_drawer_complete() and not self._drawer_objective_rewarded:
            rospy.logdebug("Rewarding DRAWER")
            self.reward_counts["Drawer"] += 1
            self._drawer_objective_rewarded = True
            return self.subgoal_reward

        if self._is_box_complete() and not self._box_objective_rewarded:
            rospy.logdebug("Rewarding BOX")
            self.reward_counts["Box"] += 1
            self._box_objective_rewarded = True
            return self.subgoal_reward

        # Check for a fail
        if self._is_fail():
            return self.fail_penalty

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
