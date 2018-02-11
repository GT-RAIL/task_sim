#!/usr/bin/env python
# This implements Q Learning. The learners are all model-free, but vary in their
# exploration policies

import copy
import pickle
import random
import numpy as np

from collections import defaultdict

from task_sim.msg import Status
from task_sim.data_utils import DataUtils

# Base Q-Learning Agent

class QLearningAgent(object):
    """
    An exploratory Q-learning agent. It avoids having to learn the transition
    model because the Q-value of a state can be related directly to those of
    its neighbors. (AIMA)

    This is just a base class. Sub classes need to implement the exploration
    function.
    """

    def __init__(self, task, gamma, alpha=None):
        """
        Arguments:
        :task: Task definition in `tasks.py`
        :gamma: Float for the discount factor of the agent
        :alpha: Lambda expression for a learning rate.

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

    def __init__(self, task, gamma, epsilon=None, alpha=None, default_Q=-10.0):
        """Epsilon is also a lambda expression that takes into account the
        training episode"""
        super(EpsilonGreedyQTableAgent, self).__init__(task, gamma, alpha)

        # Use a Q table
        self.Q = defaultdict(lambda: float(default_Q))
        self.default_Q = default_Q
        self.pi = {}

        if epsilon:
            self.epsilon = epsilon
        else:
            self.epsilon = lambda n: 0.1 * (0.9**n)

    def actions_in_state(self, state):
        # Add another twist - shuffle the actions randomly
        actions = super(EpsilonGreedyQTableAgent, self).actions_in_state(state)
        random.shuffle(actions)
        return actions

    def choose_action(self, episode=None, train=True):
        """Use epsilon-greedy to explore. Choose a random action in an unknown
        state (we can also request an intervention?)"""
        action = best_action = self.pi.get(self.s, None)
        action_candidates = self.actions_in_state(self.s)

        # Fetch the best action if we are training, or if the current state that
        # we see now has been seen before
        if train or best_action is None:
            best_action = max(action_candidates, key=lambda a: self.Q[self.s, a])

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
        elif s is not None:
            Q[s, a] += self.alpha(episode) * (
                r + gamma*max(Q[s1, a1] for a1 in self.actions_in_state(s1)) - Q[s,a]
            )

        return Q

    def update_pi(self):
        # Iterate through the seen states and actions to update the policy
        for (s,a) in self.Q.iterkeys():
            if self.pi.get(s) is None:
                action = max(self.actions_in_state(s), key=lambda a: self.Q[s,a])
                self.pi[s] = action
        return self.pi

    def save(self, filename):
        # We only save the Q table and the policy. Epsilon, Alpha
        # need to be reloaded separately

        # Reset the task before saving
        task = copy.deepcopy(self.task)
        task.reset()

        data = {
            'Q': dict(self.Q),
            'pi': self.pi,
            'gamma': self.gamma,
            'default_Q': self.default_Q,
            'task': task,
        }
        with open(filename, 'wb') as fd:
            pickle.dump(data, fd)

    def load(self, filename):
        data = None
        with open(filename, 'rb') as fd:
            data = pickle.load(fd)

        self.task = data['task']
        self.default_Q = data['default_Q']
        self.gamma = data['gamma']
        self.pi = data['pi']
        self.Q = defaultdict(lambda: float(self.default_Q))
        for k,v in data['Q']:
            self.Q[k] = v

        # Reset the agent
        self.reset()
