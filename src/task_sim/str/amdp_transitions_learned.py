#!/usr/bin/env python

from copy import deepcopy
import ast
import pickle
import h5py
import numpy as np

from task_sim.str.amdp_state import AMDPState
from task_sim.msg import Action
from task_sim.str.stochastic_state_action import StochasticState

class AMDPTransitionsLearned:

    def __init__(self, amdp_id=0, filename=None, reinit=True):
        self.amdp_id = amdp_id

        # If there is an HDF5 file to save, then populate this function with the
        # learners' code
        if filename is not None:
            self.filename = filename
            if reinit:
                with h5py.File(self.filename, 'w') as fd:
                    fd.attrs["amdp_id"] = self.amdp_id

            self.transition = h5py.File(self.filename, 'a')
            assert self.amdp_id == self.transition.attrs["amdp_id"], \
                "AMDP ID mismatch. filename: {}, expected: {}, observed: {}".format(
                    self.filename, self.amdp_id, self.transition.attrs["amdp_id"]
                )

            self._state_idx = lambda s: str(s.to_vector())
            self._action_idx = lambda a: str([a.action_type, a.object])
            self._state_template = AMDPState(self.amdp_id)


    def update_transition(self, s, a, s_prime):
        # Update the new transition function
        sa_group = "{}/{}".format(self._state_idx(s), self._action_idx(a))
        s_prime_key = self._state_idx(s_prime)

        if sa_group in self.transition:
            sa = self.transition[sa_group]
        else:
            sa = self.transition.create_group(sa_group)
            sa.attrs["total"] = 0.

        if s_prime_key in sa:
            sas = sa[s_prime_key]
        else:
            sas = sa.create_dataset(s_prime_key, data=[0.])

        # Update the dataset and the attr
        sas[0] += 1
        sa.attrs["total"] += 1

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

    def transition_function(self, s, a):
        if self.amdp_id == 3:
            # hand-coded abstract transitions
            s_prime = deepcopy(s)
            if s.relations['apple_inside_drawer'] and s.relations['drawer_closing_stack']:
                if a.action_type == 0:
                    # open drawer
                    s_prime.relations['drawer_closing_stack'] = False
                elif a.action_type == 1:
                    # close drawer
                    pass
                elif a.action_type == 2:
                    # place apple in drawer
                    pass
            if not s.relations['apple_inside_drawer'] and s.relations['drawer_closing_stack']:
                if a.action_type == 0:
                    # open drawer
                    s_prime.relations['drawer_closing_stack'] = False
                elif a.action_type == 1:
                    # close drawer
                    pass
                elif a.action_type == 2:
                    # place apple in drawer
                    pass
            if s.relations['apple_inside_drawer'] and not s.relations['drawer_closing_stack']:
                if a.action_type == 0:
                    # open drawer
                    pass
                elif a.action_type == 1:
                    # close drawer
                    s_prime.relations['drawer_closing_stack'] = True
                elif a.action_type == 2:
                    # place apple in drawer
                    pass
            if not s.relations['apple_inside_drawer'] and not s.relations['drawer_closing_stack']:
                if a.action_type == 0:
                    # open drawer
                    pass
                elif a.action_type == 1:
                    # close drawer
                    s_prime.relations['drawer_closing_stack'] = True
                elif a.action_type == 2:
                    # place apple in drawer
                    s_prime.relations['apple_inside_drawer'] = True
            return [(1.0, s_prime)]
        elif self.amdp_id == 4:
            # hand-coded abstract transitions
            s_prime = deepcopy(s)
            if a.action_type == 0:
                # open drawer
                s_prime.relations['drawer_closing_stack'] = False
            elif a.action_type == 1:
                s_prime.relations['drawer_closing_stack'] = True
            elif a.action_type == 2:
                if not s.relations['drawer_closing_stack']:
                    if a.object == 'apple':
                        s_prime.relations['apple_inside_drawer'] = True
                    elif a.object == 'banana':
                        s_prime.relations['banana_inside_drawer'] = True
            return [(1.0, s_prime)]
        elif self.amdp_id == 5:
            # hand-coded abstract transitions
            s_prime = deepcopy(s)
            if a.action_type == 0:
                # open drawer
                s_prime.relations['drawer_closing_stack'] = False
            elif a.action_type == 1:
                s_prime.relations['drawer_closing_stack'] = True
            elif a.action_type == 2:
                if not s.relations['drawer_closing_stack']:
                    if a.object == 'apple':
                        s_prime.relations['apple_inside_drawer'] = True
                    elif a.object == 'banana':
                        s_prime.relations['banana_inside_drawer'] = True
                    elif a.object == 'carrot':
                        s_prime.relations['carrot_inside_drawer'] = True
            return [(1.0, s_prime)]
        elif self.amdp_id == 9:
            # hand-coded abstract transitions
            s_prime = deepcopy(s)
            if a.action_type == 6:
                # open box
                s_prime.relations['lid_closing_box'] = False
            elif a.action_type == 7:
                # close box
                s_prime.relations['lid_closing_box'] = True
            elif a.action_type == 8:
                if not s.relations['lid_closing_box']:
                    if a.object == 'carrot':
                        s_prime.relations['carrot_inside_box'] = True
            return [(1.0, s_prime)]
        elif self.amdp_id == 10:
            # hand-coded abstract transitions
            s_prime = deepcopy(s)
            if a.action_type == 4:
                s_prime.relations['apple_inside_drawer'] = True
                s_prime.relations['banana_inside_drawer'] = True
                s_prime.relations['drawer_closing_stack'] = True
            elif a.action_type == 9:
                s_prime.relations['carrot_inside_box'] = True
                s_prime.relations['lid_closing_box'] = True
            return [(1.0, s_prime)]
        elif self.amdp_id == 11:
            # hand-coded abstract transitions
            s_prime = deepcopy(s)
            if a.action_type == 6:
                # open box
                s_prime.relations['lid_closing_box'] = False
            elif a.action_type == 7:
                # close box
                s_prime.relations['lid_closing_box'] = True
            elif a.action_type == 8:
                if not s.relations['lid_closing_box']:
                    if a.object == 'carrot':
                        s_prime.relations['carrot_inside_box'] = True
                    if a.object == 'daikon':
                        s_prime.relations['daikon_inside_box'] = True
            return [(1.0, s_prime)]
        elif self.amdp_id == 12:
            # hand-coded abstract transitions
            s_prime = deepcopy(s)
            if a.action_type == 4:
                s_prime.relations['apple_inside_drawer'] = True
                s_prime.relations['banana_inside_drawer'] = True
                s_prime.relations['drawer_closing_stack'] = True
            elif a.action_type == 11:
                s_prime.relations['carrot_inside_box'] = True
                s_prime.relations['daikon_inside_box'] = True
                s_prime.relations['lid_closing_box'] = True
            return [(1.0, s_prime)]

        # Otherwise, use the transition function that we're learning
        else:
            sa_group = "{}/{}".format(self._state_idx(s), self._action_idx(a))
            if sa_group in self.transition:
                next_states = []
                total = self.transition[sa_group].attrs["total"]
                for s_prime_s, freq in self.transition[sa_group].iteritems():
                    s_prime_v = ast.literal_eval(s_prime_s)
                    s_prime = self._state_template.from_vector(s_prime_v)
                    next_states.append((freq[0]/total, s_prime,))

                return next_states
            else:
                return [(1.0, s)]




    def save(self, suffix=''):
        self.transition.flush()


if __name__ == '__main__':
    pass
