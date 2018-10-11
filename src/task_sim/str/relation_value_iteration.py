#!/usr/bin/env python

from copy import deepcopy
import datetime
import pickle

from task_sim.msg import Action

from task_sim.str.relation_state import RelationState
from task_sim.str.relation_transitions import transition_function
from task_sim.str.relation_reward import reward, is_terminal


class RelationValueIteration:
    grasp_objects = ['drawer', 'apple']
    place_objects = ['stack', 'drawer', '']
    move_objects = ['drawer', 'stack', 'apple', 'l', 'f', 'r', 'b', 'fl', 'fr', 'br', 'bl']
    gripper_objects = ['', 'drawer', 'apple']

    def __init__(self, load=False):
        self.U = {}
        self.actions = []

        # if load:
        #     self.U = pickle.load(file('U0.pkl'))
        #     self.actions = pickle.load(file('A.pkl'))
        # else:
        #     self.initialize()
        self.initialize()

    def initialize(self):
        # initialize state list
        # print 'Enumerating states (started at: ' + str(datetime.datetime.now())  + ")"
        s = RelationState()
        s.relations['apple_right_of_drawer'] = True
        s.relations['apple_in_front_of_drawer'] = True
        s.relations['apple_below_drawer'] = True
        s.relations['apple_right_of_gripper'] = True
        s.relations['apple_behind_gripper'] = True
        s.relations['apple_below_gripper'] = True
        s.relations['gripper_in_front_of_drawer'] = True
        s.relations['gripper_open'] = True
        s.relations['drawer_closing_stack'] = True
        s.gripper_holding = ''
        self.U[deepcopy(s)] = 0.0

        # for i in range(3):
        #     s.relations['apple_left_of_drawer'] = False
        #     s.relations['apple_right_of_drawer'] = False
        #     if i == 1:
        #         s.relations['apple_left_of_drawer'] = True
        #     elif i == 2:
        #         s.relations['apple_right_of_drawer'] = True
        #
        #     for i2 in range(3):
        #         s.relations['apple_in_front_of_drawer'] = False
        #         s.relations['apple_behind_drawer'] = False
        #         if i2 == 1:
        #             s.relations['apple_in_front_of_drawer'] = True
        #         elif i2 == 2:
        #             s.relations['apple_behind_drawer'] = True
        #
        #         for i3 in range(3):
        #             s.relations['apple_above_drawer'] = False
        #             s.relations['apple_below_drawer'] = False
        #             if i3 == 1:
        #                 s.relations['apple_above_drawer'] = True
        #             elif i3 == 2:
        #                 s.relations['apple_below_drawer'] = True
        #
        #             for i4 in range(3):
        #                 s.relations['apple_left_of_gripper'] = False
        #                 s.relations['apple_right_of_gripper'] = False
        #                 if i4 == 1:
        #                     s.relations['apple_left_of_gripper'] = True
        #                 elif i4 == 2:
        #                     s.relations['apple_right_of_gripper'] = True
        #
        #                 for i5 in range(3):
        #                     s.relations['apple_in_front_of_gripper'] = False
        #                     s.relations['apple_behind_gripper'] = False
        #                     if i5 == 1:
        #                         s.relations['apple_in_front_of_gripper'] = True
        #                     elif i5 == 2:
        #                         s.relations['apple_behind_gripper'] = True
        #
        #                     for i6 in range(3):
        #                         s.relations['apple_above_gripper'] = False
        #                         s.relations['apple_below_gripper'] = False
        #                         if i6 == 1:
        #                             s.relations['apple_above_gripper'] = True
        #                         elif i6 == 2:
        #                             s.relations['apple_below_gripper'] = True
        #
        #                         for i7 in range(3):
        #                             s.relations['gripper_left_of_drawer'] = False
        #                             s.relations['gripper_right_of_drawer'] = False
        #                             if i7 == 1:
        #                                 s.relations['gripper_left_of_drawer'] = True
        #                             elif i7 == 2:
        #                                 s.relations['gripper_right_of_drawer'] = True
        #
        #                             for i8 in range(3):
        #                                 s.relations['gripper_in_front_of_drawer'] = False
        #                                 s.relations['gripper_behind_drawer'] = False
        #                                 if i8 == 1:
        #                                     s.relations['gripper_in_front_of_drawer'] = True
        #                                 elif i8 == 2:
        #                                     s.relations['gripper_behind_drawer'] = True
        #
        #                                 for i9 in range(3):
        #                                     s.relations['gripper_above_drawer'] = False
        #                                     s.relations['gripper_below_drawer'] = False
        #                                     if i9 == 1:
        #                                         s.relations['gripper_above_drawer'] = True
        #                                     elif i9 == 2:
        #                                         s.relations['gripper_below_drawer'] = True
        #
        #                                     for i10 in range(2):
        #                                         s.relations['apple_touching_drawer'] = False
        #                                         if i10 == 1:
        #                                             s.relations['apple_touching_drawer'] = True
        #
        #                                         for i11 in range(2):
        #                                             s.relations['apple_touching_stack'] = False
        #                                             if i11 == 1:
        #                                                 s.relations['apple_touching_stack'] = True
        #
        #                                             for i12 in range(2):
        #                                                 s.relations['gripper_touching_drawer'] = False
        #                                                 if i12 == 1:
        #                                                     s.relations['gripper_touching_drawer'] = True
        #
        #                                                 for i13 in range(2):
        #                                                     s.relations['gripper_touching_stack'] = False
        #                                                     if i13 == 1:
        #                                                         s.relations['gripper_touching_stack'] = True
        #
        #                                                     for i14 in range(2):
        #                                                         s.relations['drawer_closing_stack'] = False
        #                                                         if i14 == 1:
        #                                                             s.relations['drawer_closing_stack'] = True
        #
        #                                                         for i15 in range(2):
        #                                                             s.relations['gripper_open'] = False
        #                                                             if i15 == 1:
        #                                                                 s.relations['gripper_open'] = True
        #
        #                                                             s.gripper_holding = ''
        #                                                             self.U[deepcopy(s)] = 0.0
        #
        #                                                             if not (s.relations['apple_left_of_gripper'] or
        #                                                                     s.relations['apple_right_of_gripper'] or
        #                                                                     s.relations['apple_in_front_of_gripper'] or
        #                                                                     s.relations['apple_behind_gripper'] or
        #                                                                     s.relations['apple_above_gripper'] or
        #                                                                     s.relations['apple_below_gripper']) \
        #                                                                     and not s.relations['gripper_open']:
        #                                                                 s.gripper_holding = 'apple'
        #                                                                 self.U[deepcopy(s)] = 0.0
        #                                                             elif not (s.relations['gripper_above_drawer'] or
        #                                                                       s.relations['gripper_below_drawer'] or
        #                                                                       s.relations['gripper_in_front_of_drawer']
        #                                                                       or s.relations['gripper_behind_drawer'] or
        #                                                                       s.relations['gripper_left_of_drawer']) \
        #                                                                     and s.relations['gripper_right_of_drawer']:
        #                                                                 s.gripper_holding = 'drawer'
        #                                                                 self.U[deepcopy(s)] = 0.0
        #
        # print 'Finished enumerating states (finished at: ' + str(datetime.datetime.now()) + ")"

        # initialize action list
        a = Action()

        a.action_type = Action.GRASP
        for o in self.grasp_objects:
            a.object = o
            self.actions.append(deepcopy(a))

        a.action_type = Action.PLACE
        for o in self.place_objects:
            a.object = o
            self.actions.append(deepcopy(a))

        a.action_type = Action.MOVE_ARM
        for o in self.move_objects:
            a.object = o
            self.actions.append(deepcopy(a))

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

        #pickle.dump(self.U, file('U0.pkl', mode='w'))
        #pickle.dump(self.actions, file('A.pkl', mode='w'))

    def solve(self):
        gamma = 0.8
        epsilon = 10
        n = 0
        termination_check = False
        start_time = datetime.datetime.now()

        while True:
            n += 1
            print 'Iteration ' + str(n)
            if termination_check:
                print '\t(now checking for termination)'
            total = len(self.U.keys())
            print '\tSize of state space: ' + str(total)
            count = 0.0
            U_prime = {}
            delta = 0.0
            for s in self.U:
                count += 1
                if is_terminal(s):
                    u = reward(s)
                    U_prime[deepcopy(s)] = u
                    termination_check = True
                    if s in self.U:
                        d = abs(self.U[s] - u)
                        if d > delta:
                            delta = d
                    else:
                        if abs(u) > delta:
                            delta = abs(u)
                    continue

                max_u = -999999
                for a in self.actions:
                    successors = transition_function(s, a)
                    current_u = 0.0
                    for i in range(len(successors)):
                        p = successors[i][0]
                        s_prime = successors[i][1]

                        if s_prime not in self.U:
                            # Fix state size at iteration 8
                            if n > 8:
                                continue
                            else:
                                U_prime[deepcopy(s_prime)] = 0.0
                        else:
                            current_u += p*self.U[s_prime]

                    if current_u > max_u:
                        max_u = current_u

                u = reward(s) + gamma*max_u
                U_prime[deepcopy(s)] = u
                if termination_check:
                    if s in self.U:
                        d = abs(self.U[s] - u)
                    else:
                        d = abs(u)
                    if d > delta:
                        delta = d

                if count % 10000 == 0:
                    print '\tprogress: ' + str(count/total)

            self.U = U_prime
            if termination_check:
                if delta < epsilon*(1 - gamma)/gamma:
                    break
                print 'Delta: ' + str(delta) + ', continuing...'
                pickle.dump(self.U, file('U_iter_' + str(n) + '.pkl', mode='w'))
            print 'Elapsed time: ' + str(datetime.datetime.now() - start_time)

        print 'Total elapsed time: ' + str(datetime.datetime.now() - start_time)
        print 'Finished. Saving...'
        pickle.dump(self.U, file('trained_U.pkl', mode='w'))
        print 'Utilities saved.'


if __name__ == '__main__':
    r = RelationValueIteration(load=True)
    r.solve()
