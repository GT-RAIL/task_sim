#!/usr/bin/env python

from copy import deepcopy
import pickle

from task_sim.str.amdp_state import AMDPState
from task_sim.msg import Action
from task_sim.str.stochastic_state_action import StochasticState

class AMDPTransitionsLearned:

    def __init__(self, amdp_id=0, load=False):
        self.amdp_id = amdp_id
        if load:
            self.transition = pickle.load(file('T' + str(self.amdp_id) + '.pkl', mode='r'))
            print 'Loaded T' + str(self.amdp_id) + '.pkl'
            print str(len(self.transition.keys()))
        else:
            self.transition = {}

    def update_transition(self, s, a, s_prime):
        q = (s, a.action_type, a.object)
        if q not in self.transition:
            self.transition[q] = StochasticState()
        self.transition[q].add_state(s_prime)

    def get_states(self):
        s = []
        for q in self.transition:
            if q[0] not in s:
                s.append(q[0])
            for s_prime in self.transition[q].states:
                if s_prime not in s:
                    s.append(s_prime)
        return s

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
        else:
            q = (s, a.action_type, a.object)
            # states = []
            # print 'state space size: ' + str(len(self.transition.keys()))
            # for state in states:
            #     print str(state)
            # print str(s.relations)
            # print str(a.action_type)
            # print str(a.object)
            # print str(self.transition.keys()[0][0].relations)
            # print str(self.transition.keys()[0][1])
            # print str(self.transition.keys()[0][2])
            if q in self.transition:
                return self.transition[q].get_distribution()
            else:
                return [(1.0, s)]

    def save(self, suffix=''):
        pickle.dump(self.transition, file('T' + str(self.amdp_id) + str(suffix) + '.pkl', mode='w'))
        print 'Transition function saved.'


if __name__ == '__main__':
    a = AMDPTransitionsLearned(load=True)

    print 'Success, can load pickle file'
