#!/usr/bin/env python
# This implements Q Learning. The learners are all model-free, but vary in their
# exploration policies

from __future__ import division, print_function

import os
import sys
import h5py
import copy
import pickle
import random
import numpy as np

from collections import defaultdict

from task_sim import data_utils as DataUtils
from task_sim.msg import Status, Action
from task_sim.rl import tasks

# Base Q-Learning Agent

class QLearningAgent(object):
    """
    An exploratory Q-learning agent. It avoids having to learn the transition
    model because the Q-value of a state can be related directly to those of
    its neighbors. (AIMA)

    This is just a base class. Sub classes need to implement the exploration
    function.
    """

    def __init__(self, task, gamma, alpha=None, viz=None, *args, **kwargs):
        """
        Arguments:
        :task: Task definition in `tasks.py`
        :gamma: Float for the discount factor of the agent
        :alpha: Lambda expression for a learning rate.
        :viz: Visdom object to visualize data

        Other Effects:
        :Q: Default - None. Q values for states. Either parameterized or a table
        :Nsa: Default - None. Counter (can be approximate)
        :pi: - Default - None. Policy
        """

        self.task = task   # Task definition from tasks.py
        self.gamma = gamma # Discount factor for the agent
        self.Q = None      # Q value representation
        self.pi = None     # Learned policy so far
        self.s = None      # Current state observed by the agent
        self.a = None      # Current action taken by the agent
        self.r = None      # Current reward received by the agent

        if alpha:
            self.alpha = alpha
        else:
            self.alpha = lambda n: 1./(1+n)  # udacity video

        self.viz = viz

    def choose_action(self, episode=None, train=True):
        """
        Return an action based on the desired exploration/exploitation method.
        Keep in mind that it is best not to explore when train is set to False.
        However, also remember to deal with unseen states when train=False.

        Before calling this method, the state will be updated to the new percept
        """
        raise NotImplementedError("Don't know how to pick the best action")

    def actions_in_state(self, state):
        """By default the actions available to the agent are task constrained"""
        return self.task.actions(state)

    def update_Q(self, percept, episode=None):
        """
        Needs to be implemented, but the internal function used to update the
        Q table. Returns the updated Q params
        """
        raise NotImplementedError("Don't know how to update Q")

    def update_pi(self):
        """Update the learned policy"""
        raise NotImplementedError("Don't know how to update the policy")

    def __call__(self, percept, episode=None, train=True):
        """Percept is of the form (state, reward). If train is false, simply use
        the learned policy to return actions"""
        percept = self.update_percept(percept)
        if train:
            self.update_Q(percept, episode)

        self.s, self.r = percept
        self.a = self.choose_action(episode, train)
        return self.a

    def update_percept(self, percept):
        """Translate the true world state to the agent's observable state.
        By default, the task defines the observation model. However, if there
        are agent specific state changes, they can be done here"""
        world_state, reward = percept
        state = self.task.get_agent_state()
        return (state, reward)

    def reset(self):
        """Reset the agent's state"""
        self.s = self.a = self.r = None

    def save(self, filename):
        """Save the agent to the filename"""
        raise NotImplementedError("Don't know how to save the agent")

    def load(self, filename):
        """Load the saved agent from the filename"""
        raise NotImplementedError("Don't know how to resurrect the agent")



# Different Q Learners

class EpsilonGreedyQTableAgent(QLearningAgent):
    """Uses epsilon greedy to choose actions during explore/exploit. Store the
    Q table as """

    def __init__(
        self, task, gamma,
        epsilon=None, alpha=None, default_Q=-10.0,
        *args, **kwargs
    ):
        """Epsilon is also a lambda expression that takes into account the
        training episode"""
        super(EpsilonGreedyQTableAgent, self).__init__(
            task, gamma, alpha,
            *args, **kwargs
        )

        # Use a Q table
        self.Q = defaultdict(lambda: float(default_Q))
        self.default_Q = default_Q
        self.pi = {}

        if epsilon:
            self.epsilon = epsilon
        else:
            self.epsilon = lambda n: 0.1 * (0.9**n)

    def actions_in_state(self, state):
        actions = super(EpsilonGreedyQTableAgent, self).actions_in_state(state)
        return actions

    def choose_action(self, episode=None, train=True):
        """Use epsilon-greedy to explore. Choose a random action in an unknown
        state (we can also request an intervention?)"""

        # Don't choose an action if we are done
        if self.task.status() != Status.IN_PROGRESS:
            return None

        # Otherwise, choose the best action unless we want to explore
        action = best_action = self.pi.get(self.s, None)
        action_candidates = self.actions_in_state(self.s)

        # Fetch the best action if we are training, or if the current state that
        # we see now has been seen before
        if train or best_action is None:
            best_action = max(action_candidates, key=lambda a: self.Q[self.s,a])

        # If we're training, use epsilon to decide if we want to explore.
        # Otherwise, pick the best
        if train:
            if random.uniform(0, 1) < self.epsilon(episode):
                action = random.choice(action_candidates)
            else:
                action = best_action
        else: # Not training. Pick the best action or just be random
            action = best_action or random.choice(action_candidates)

        # Return the action
        return action

    def update_Q(self, percept, episode):
        s1, r1 = percept
        s, a, r, gamma = self.s, self.a, self.r, self.gamma
        Q = self.Q

        # Check to see if this is the end. Else update the Q value
        if self.task.status() != Status.IN_PROGRESS:
            Q[s1, None] = r1
            Q[s,a] += self.alpha(episode) * (r + gamma*Q[s1,None] - Q[s,a])
        elif s is not None:
            Q[s, a] += self.alpha(episode) * (
                r + gamma*max(Q[s1,a1] for a1 in self.actions_in_state(s1)) - Q[s,a]
            )

        return Q

    def update_pi(self):
        # Iterate through the seen states and actions to update the policy
        known_sa = defaultdict(list)
        for (s,a) in self.Q.iterkeys():
            known_sa[s].append(a)

        for s in known_sa.iterkeys():
            action = max(known_sa[s], key=lambda a: self.Q[s,a])
            self.pi[s] = action
        return self.pi

    def save(self, filename):
        # We only save the Q table and the policy. Epsilon, Alpha
        # need to be reloaded separately

        # Reset the task before saving
        # task = copy.deepcopy(self.task)
        # task.reset()

        data = {
            'Q': dict(self.Q),
            'pi': self.pi,
            'gamma': self.gamma,
            'default_Q': self.default_Q,
            'task': self.task.save(),
        }
        with open(filename, 'wb') as fd:
            pickle.dump(data, fd)

    def load(self, filename):
        data = None
        with open(filename, 'rb') as fd:
            data = pickle.load(fd)

        try:
            task_name = data['task']['name']
            task = getattr(tasks, task_name)(**data['task'])
            self.task = task
        except Exception as e:
            rospy.logerror("Error loading task: {}".format(e))
        self.default_Q = data['default_Q']
        self.gamma = data['gamma']
        self.pi = data['pi']
        self.Q = defaultdict(lambda: float(self.default_Q))
        for k,v in data['Q']:
            self.Q[k] = v

        # Reset the agent
        self.reset()

class EpsilonGreedyQTiledAgent(QLearningAgent):
    """Uses epsilon greedy to choose actions during explore/exploit. Store the
    Q values in a tiled has table instead"""

    def __init__(
        self, task, gamma,
        epsilon=None, alpha=lambda eps: 1./(1+eps),
        num_tiles=512, tiles_max_size=1024**2,
        missing_param_value=-5, use_iht=True,
        *args, **kwargs
    ):
        """Epsilon is also a lambda expression that takes into account the
        training episode"""
        super(EpsilonGreedyQTiledAgent, self).__init__(
            task, gamma, lambda eps: alpha(eps)/num_tiles,
            *args, **kwargs
        )

        # Need to reset the random seed for consistent hashing
        random.seed(0)
        from task_sim.rl import tile_coder
        self.tile_coder = tile_coder

        # Use a Q table
        self.IHT = self.tile_coder.IHT(tiles_max_size) if use_iht else tiles_max_size
        self.num_tiles = num_tiles
        self.Q = np.zeros((tiles_max_size,), dtype=np.float)
        self.missing_param_value = missing_param_value

        if epsilon:
            self.epsilon = epsilon
        else:
            self.epsilon = lambda n: 0.1 * (0.9**n)

    def _tile_sa(self, state, action=[], readonly=False):
        """Given a state, action; tile it"""
        return self.tile_coder.tiles(
            self.IHT, # ihtORsize
            self.num_tiles, # numtilings
            state, # floats
            action, # ints
            readonly # readonly
        )

    def actions_in_state(self, state):
        actions = super(EpsilonGreedyQTiledAgent, self).actions_in_state(state)

        # Actions are as tuples. Convert them to a type that can be tiled
        actions = [
            self._convert_action_to_actionlist(act)
            for act in actions
        ]

        # Actions is now a list of [action, object_int|missing_param_value]
        return actions

    def _convert_actionlist_to_action(self, action):
        """Given an action, convert it to a meaningful action for the task"""
        action = (
            action[0],
            DataUtils.int_to_name(action[1]) if action[0] in [Action.GRASP] else None,
            DataUtils.int_to_name(action[1]) if action[0] in [Action.PLACE, Action.MOVE_ARM] else None,
        )
        return action

    def _convert_action_to_actionlist(self, action):
        """Given a meaningful action for the task, convert to a list of ints"""
        action_param = (
            DataUtils.name_to_int(action[1]) if action[1] is not None
            else (
                DataUtils.name_to_int(action[2]) if action[2] is not None
                else self.missing_param_value
            )
        )
        return [action[0], action_param]

    def choose_action(self, episode=None, train=True):
        """Use epsilon-greedy to explore. Choose a random action in an unknown
        state (we can also request an intervention?)"""

        # Don't choose an action if we are done
        if self.task.status() != Status.IN_PROGRESS:
            return None

        # We don't have a policy that we save
        action = best_action = None
        action_candidates = self.actions_in_state(self.s)

        # Fetch the best action if we are training, or if the current state that
        # we see now has been seen before
        best_action = max(action_candidates, key=lambda a: np.sum(self.Q[self._tile_sa(self.s, a)]))

        # If we're training, use epsilon to decide if we want to explore.
        # Otherwise, pick the best
        if train and random.uniform(0, 1) < self.epsilon(episode):
                action = random.choice(action_candidates)
        else:
            action = best_action

        # Return the action
        return self._convert_actionlist_to_action(action)

    def update_Q(self, percept, episode):
        s1, r1 = percept
        s, r, gamma = self.s, self.r, self.gamma
        a = self._convert_action_to_actionlist(self.a) if self.a is not None else None
        Q = self.Q
        sa_tiled = self._tile_sa(s, a) if s is not None else None

        # If this is the end, update the terminal states
        if self.task.status() != Status.IN_PROGRESS:
            s1_tiled = self._tile_sa(s1)
            Q[s1_tiled] = r1
            Q[sa_tiled] += self.alpha(episode) * (
                r + (gamma * np.sum(Q[s1_tiled])) - np.sum(Q[sa_tiled])
            )
        elif sa_tiled is not None:
            Q[sa_tiled] += self.alpha(episode) * (
                r
                + (gamma * max(np.sum(Q[self._tile_sa(s1,a1)]) for a1 in self.actions_in_state(s1)))
                - np.sum(Q[sa_tiled])
            )

        return Q

    def update_pi(self):
        # Cannot update the policy for this method of saving states and actions
        # Pass for now
        return None

    def save(self, filename):
        # We only save the Q table and the policy. Epsilon, Alpha
        # need to be reloaded separately

        data = {
            'IHT': self.IHT,
            'num_tiles': self.num_tiles,
            'missing_param_value': self.missing_param_value,
            'Q_filename': os.path.splitext(filename)[0] + '.hdf5',
            'gamma': self.gamma,
            'task': self.task.save()
        }

        with open(filename, 'wb') as fd:
            pickle.dump(data, fd)

        with h5py.File(data['Q_filename'], 'w') as h:
            h.create_dataset('Q', data=self.Q)

    def load(self, filename):
        data = None
        with open(filename, 'rb') as fd:
            data = pickle.load(fd)

        try:
            task_name = data['task']['name']
            task = getattr(tasks, task_name)(**data['task'])
            self.task = task
        except Exception as e:
            rospy.logerror("Error loading task: {}".format(e))

        self.gamma = data['gamma']
        self.IHT = data['IHT']
        self.num_tiles = data['num_tiles']
        self.missing_param_value = data['missing_param_value']

        with h5py.File('data/task1/models/' + data['Q_filename'], 'r') as h:
            self.Q = h['Q'][()]

        # Reset the agent
        self.reset()
