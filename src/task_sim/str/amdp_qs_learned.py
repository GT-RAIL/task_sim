#!/usr/bin/env python

from copy import deepcopy
import ast
import pickle
import h5py
import numpy as np
from random import randint, random

from task_sim.str.amdp_state import AMDPState
from task_sim.msg import Action
from task_sim.str.stochastic_state_action import StochasticState
from task_sim.str.amdp_reward import reward, is_terminal

class AMDPQsLearned:

    def __init__(self, amdp_id=0, filename=None, reinit=True, mode=0):
        self.amdp_id = amdp_id

        # flag to run as either:
        #   0: standard q-learning mode (exploit from q-values or select actions randomly based on epsilon)
        #   1: guided q-learning mode (exploit from q-values or perform action selection from an outside source)
        self.mode = mode

        # If there is an HDF5 file to save, then populate this function with the
        # learners' code
        if filename is not None:
            self.filename = filename
            if reinit:
                with h5py.File(self.filename, 'w') as fd:
                    fd.attrs["amdp_id"] = self.amdp_id

            self.Q = h5py.File(self.filename, 'a')
            assert self.amdp_id == self.Q.attrs["amdp_id"], \
                "AMDP ID mismatch. filename: {}, expected: {}, observed: {}".format(
                    self.filename, self.amdp_id, self.Q.attrs["amdp_id"]
                )

            self._state_idx = lambda s: str(s.to_vector())
            self._action_idx = lambda a: str([a.action_type, a.object])
            self._state_template = AMDPState(self.amdp_id)

        self.init_q_agent()

    def init_q_agent(self, epsilon=0.5):
        self.s = None
        self.a = None
        self.r = None

        self.epsilon = epsilon


    def learn_q(self, s_prime, alpha=0.1, action_list=None):
        a_prime = None
        r_prime = reward(s_prime, amdp_id=self.amdp_id)

        noop = Action()
        noop.action_type = Action.NOOP

        if is_terminal(s_prime, amdp_id=self.amdp_id):
            sa_group = "{}/{}".format(self._state_idx(s_prime), self._action_idx(noop))
            if sa_group not in self.Q:
                self.Q.create_dataset(sa_group, data=[0.])
            self.Q[sa_group][0] = r_prime

        if self.s is not None:
            # Update the Q table
            sa_group = "{}/{}".format(self._state_idx(self.s), self._action_idx(self.a))
            s_prime_key = self._state_idx(s_prime)

            if sa_group in self.Q:
                Q_sa = self.Q[sa_group]
            else:
                Q_sa = self.Q.create_dataset(sa_group, data=[0.])

            # get best action and max Q value
            Q_sa_prime = -9999999
            actions = []
            action_list_extended = deepcopy(action_list)
            action_list_extended.append(noop)
            for act in action_list_extended:
                sa_prime_group = "{}/{}".format(self._state_idx(s_prime), self._action_idx(act))
                if sa_prime_group in self.Q:
                    q = self.Q[sa_prime_group][0]
                else:
                    q = 0.0
                if act.action_type == Action.NOOP and q == 0:
                    continue
                if q > Q_sa_prime:
                    Q_sa_prime = q
                    actions = [act]
                elif q == Q_sa_prime:
                    actions.append(act)

            if len(actions) > 1:
                a_prime = actions[randint(0, len(actions) - 1)]
            else:
                a_prime = actions[0]

            Q_sa[0] += alpha*(self.r + 0.8*Q_sa_prime - Q_sa[0])

        self.s = deepcopy(s_prime)
        self.r = deepcopy(r_prime)

        if a_prime is None or random() < self.epsilon:
            if self.mode == 0:
                a_prime = action_list[randint(0, len(action_list) - 1)]
            else:
                return None

        self.a = deepcopy(a_prime)

        return a_prime

    def set_action(self, a):
        '''Sets last action, required when actions are chosen outside of q-learning'''
        self.a = deepcopy(a)

    def select_action(self, s_prime, action_list=None):
        a_prime = None
        # get best action and max Q value
        Q_sa_prime = -9999999
        actions = []
        q_values = []
        for act in action_list:
            sa_prime_group = "{}/{}".format(self._state_idx(s_prime), self._action_idx(act))
            if sa_prime_group in self.Q:
                q = self.Q[sa_prime_group][0]
            else:
                q = 0.0
            q_values.append(q)

            if q > Q_sa_prime:
                Q_sa_prime = q
                actions = [act]
            elif q == Q_sa_prime:
                actions.append(act)

        if len(actions) > 1:
            a_prime = actions[randint(0, len(actions) - 1)]
        else:
            a_prime = actions[0]

        return deepcopy(a_prime)

    def get_states(self):
        s = set()
        for state_s, transitions in self.transition.iteritems():
            state_v = ast.literal_eval(state_s)
            s.add(self._state_template.from_vector(state_v))

            for action_s, results in transitions.iteritems():
                for s_prime_s in results.keys():
                    s_prime_v = ast.literal_eval(s_prime_s)
                    s.add(self._state_template.from_vector(s_prime_v))

        return list(s)


    def save(self, suffix=''):
        self.Q.flush()


if __name__ == '__main__':
    pass
