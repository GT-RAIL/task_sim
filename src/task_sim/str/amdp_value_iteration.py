#!/usr/bin/env python

from copy import deepcopy
import datetime
import pickle

from task_sim.msg import Action

from task_sim.str.amdp_state import AMDPState
from task_sim.str.amdp_transitions_learned import AMDPTransitionsLearned
from task_sim.str.amdp_reward import reward, is_terminal


class AMDPValueIteration:
    grasp_objects_drawer = ['drawer', 'apple']
    place_objects_drawer = ['stack', 'drawer', '']
    move_objects_drawer = ['drawer', 'stack', 'apple', 'l', 'f', 'r', 'b', 'fl', 'fr', 'br', 'bl']
    gripper_objects_drawer = ['', 'drawer', 'apple']
    grasp_objects_box = ['lid', 'apple']
    place_objects_box = ['lid', 'box', '']
    move_objects_box = ['lid', 'box', 'apple', 'l', 'f', 'r', 'b', 'fl', 'fr', 'br', 'bl']
    gripper_objects_box = ['', 'lid', 'apple']

    abstract_amdps = [3, 4, 5, 9, 10, 11, 12]

    def __init__(self, amdp_id, transition_function):
        self.U = {}
        self.actions = []
        self.amdp_id = amdp_id

        self.initialize(transition_function)

    def initialize(self, transition_function):
        # initialize action list
        a = Action()

        if self.amdp_id == 3:
            # actions are overloaded here to use the same message type
            # if amdp_id is 3 (the highest-level abstract mdp), then the actions correspond to the amdp_id, i.e.:
            #   0 - open drawer
            #   1 - close drawer
            #   2 - put apple in drawer
            a.action_type = 0
            self.actions.append(deepcopy(a))
            a.action_type = 1
            self.actions.append(deepcopy(a))
            a.action_type = 2
            self.actions.append(deepcopy(a))
        elif self.amdp_id == 4:
            # actions are overloaded here to use the same message type
            # if amdp_id is 3 (the highest-level abstract mdp), then the actions correspond to the amdp_id, i.e.:
            #   0 - open drawer
            #   1 - close drawer
            #   2 - put apple in drawer
            # ground items are stored in the Action.object member
            a.action_type = 0
            self.actions.append(deepcopy(a))
            a.action_type = 1
            self.actions.append(deepcopy(a))
            a.action_type = 2
            a.object = 'apple'
            self.actions.append(deepcopy(a))
            a.object = 'banana'
            self.actions.append(deepcopy(a))
        elif self.amdp_id == 5:
            # actions are overloaded here to use the same message type
            # if amdp_id is 3 (the highest-level abstract mdp), then the actions correspond to the amdp_id, i.e.:
            #   0 - open drawer
            #   1 - close drawer
            #   2 - put apple in drawer
            # ground items are stored in the Action.object member
            a.action_type = 0
            self.actions.append(deepcopy(a))
            a.action_type = 1
            self.actions.append(deepcopy(a))
            a.action_type = 2
            a.object = 'apple'
            self.actions.append(deepcopy(a))
            a.object = 'banana'
            self.actions.append(deepcopy(a))
            a.object = 'carrot'
            self.actions.append(deepcopy(a))
        elif self.amdp_id == 9:
            # actions are overloaded here to use the same message type
            # the actions correspond to the amdp_id, i.e.:
            #   6 - open box
            #   7 - close box
            #   8 - put carrot in box
            # ground items are stored in the Action.object member
            a.action_type = 6
            self.actions.append(deepcopy(a))
            a.action_type = 7
            self.actions.append(deepcopy(a))
            a.action_type = 8
            a.object = 'carrot'
            self.actions.append(deepcopy(a))
        elif self.amdp_id == 10:
            # actions are overloaded here to use the same message type
            # the actions correspond to the amdp_id, i.e.:
            #   4 - put fruits in drawer high-level amdp
            #   9 - put vegetable in box high-level amdp
            a.action_type = 4
            self.actions.append(deepcopy(a))
            a.action_type = 9
            self.actions.append(deepcopy(a))
        elif self.amdp_id == 11:
            # actions are overloaded here to use the same message type
            # the actions correspond to the amdp_id, i.e.:
            #   6 - open box
            #   7 - close box
            #   8 - put item in box
            # ground items are stored in the Action.object member
            a.action_type = 6
            self.actions.append(deepcopy(a))
            a.action_type = 7
            self.actions.append(deepcopy(a))
            a.action_type = 8
            a.object = 'carrot'
            self.actions.append(deepcopy(a))
            a.object = 'daikon'
            self.actions.append(deepcopy(a))
        elif self.amdp_id == 12:
            # actions are overloaded here to use the same message type
            # the actions correspond to the amdp_id, i.e.:
            #   4 - put fruits in drawer high-level amdp
            #   11 - put vegetables in box high-level amdp
            a.action_type = 4
            self.actions.append(deepcopy(a))
            a.action_type = 11
            self.actions.append(deepcopy(a))
        else:
            a.action_type = Action.GRASP
            if self.amdp_id >= 0 and self.amdp_id <= 2:
                for o in self.grasp_objects_drawer:
                    a.object = o
                    self.actions.append(deepcopy(a))
            elif self.amdp_id >= 6 and self.amdp_id <= 8:
                for o in self.grasp_objects_box:
                    a.object = o
                    self.actions.append(deepcopy(a))
            if self.amdp_id == -2:
                a.object = 'banana'
                self.actions.append(deepcopy(a))
            elif self.amdp_id == -3:
                a.object = 'banana'
                self.actions.append(deepcopy(a))
                a.object = 'carrot'
                self.actions.append(deepcopy(a))

            a.action_type = Action.PLACE
            if self.amdp_id >= 0 and self.amdp_id <= 2:
                for o in self.place_objects_drawer:
                    a.object = o
                    self.actions.append(deepcopy(a))
            elif self.amdp_id >= 6 and self.amdp_id <= 8:
                for o in self.place_objects_box:
                    a.object = o
                    self.actions.append(deepcopy(a))

            a.action_type = Action.MOVE_ARM
            if self.amdp_id >= 0 and self.amdp_id <= 2:
                for o in self.move_objects_drawer:
                    a.object = o
                    self.actions.append(deepcopy(a))
            elif self.amdp_id >= 6 and self.amdp_id <= 8:
                for o in self.move_objects_box:
                    a.object = o
                    self.actions.append(deepcopy(a))
            if self.amdp_id == -2:
                a.object = 'banana'
                self.actions.append(deepcopy(a))
            elif self.amdp_id == -3:
                a.object = 'banana'
                self.actions.append(deepcopy(a))
                a.object = 'carrot'
                self.actions.append(deepcopy(a))

            a.object = ''
            a.action_type = Action.OPEN_GRIPPER
            self.actions.append(deepcopy(a))

            a.action_type = Action.CLOSE_GRIPPER
            self.actions.append(deepcopy(a))

            a.action_type = Action.RAISE_ARM
            self.actions.append(deepcopy(a))

            a.action_type = Action.LOWER_ARM
            self.actions.append(deepcopy(a))

            a.action_type = Action.RESET_ARM
            self.actions.append(deepcopy(a))

        # initialize transition function
        assert transition_function is not None, "Unknown transition function!"
        self.T = transition_function

        # initialize utilities for states in transition function
        self.init_utilities()

    def init_utilities(self, debug=0):
        self.U = {}
        if self.amdp_id in self.abstract_amdps:
            s = AMDPState(amdp_id=self.amdp_id)
            self.enumerate_relations(s)
        else:
            if debug > 0:
                print 'Initializing utilities over all states in the state list...'
            states = self.T.get_states()
            for s in states:
                self.U[deepcopy(s)] = 0.0
            if debug > 0:
                print 'Utilities initialized.'

    def enumerate_relations(self, s, i=0):
        '''recursively set all attributes in an amdp state to cover all possible state assignments, store them in U'''
        if i == len(s.relations.keys()) - 1:
            s.relations[s.relations.keys()[i]] = True
            self.U[deepcopy(s)] = 0.0
            s.relations[s.relations.keys()[i]] = False
            self.U[deepcopy(s)] = 0.0
            return
        s.relations[s.relations.keys()[i]] = True
        self.enumerate_relations(s, i + 1)
        s.relations[s.relations.keys()[i]] = False
        self.enumerate_relations(s, i + 1)

    def solve(self, debug=0):
        gamma = 0.8
        epsilon = 1
        n = 0
        # termination_check = False
        start_time = datetime.datetime.now()
        num_states = 0

        print 'States in utility function:', len(self.U.keys())

        while True:
            n += 1
            if debug > 0:
                print 'Iteration ' + str(n)
            # if termination_check:
            #     print '\t(now checking for termination)'
            total = len(self.U.keys())
            # print '\tSize of state space: ' + str(total)
            count = 0.0
            U_prime = {}
            delta = 0.0
            for s in self.U:
                count += 1
                if is_terminal(s, amdp_id=self.amdp_id):
                    u = reward(s, amdp_id=self.amdp_id)
                    U_prime[s] = u
                    # termination_check = True
                    d = abs(self.U[s] - u)
                    if d > delta:
                        delta = d
                    continue

                max_u = -999999
                for a in self.actions:
                    successors = self.T.transition_function(s, a)
                    current_u = 0.0
                    for i in range(len(successors)):
                        p = successors[i][0]
                        s_prime = successors[i][1]

                        current_u += p*self.U[s_prime]

                    if current_u > max_u:
                        max_u = current_u

                u = reward(s, amdp_id=self.amdp_id) + gamma*max_u
                U_prime[s] = u
                # if termination_check:
                d = abs(self.U[s] - u)
                if d > delta:
                    delta = d

                if count % 10000 == 0 and debug > 0:
                    print '\tprogress: ' + str(count/total)

            self.U = U_prime
            # if termination_check:
            if delta < epsilon*(1 - gamma)/gamma:
                break

            if debug > 0:
                print 'Delta: ' + str(delta) + ', continuing...'
                print 'Elapsed time: ' + str(datetime.datetime.now() - start_time)

        if debug > 0:
            print 'Total elapsed time: ' + str(datetime.datetime.now() - start_time)
        # print 'Finished. Saving...'
        # self.save()

    def save(self, suffix=''):
        pickle.dump(self.U, file('U' + str(self.amdp_id) + str(suffix) + '.pkl', mode='w'))
        print 'Utilities saved.'


if __name__ == '__main__':
    a = AMDPValueIteration(2, None)
    a.solve()
