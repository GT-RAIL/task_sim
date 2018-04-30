#!/usr/bin/env python

from copy import deepcopy
import datetime
import pickle

from task_sim.msg import Action

from task_sim.str.amdp_state import AMDPState
from task_sim.str.amdp_transitions_learned import AMDPTransitionsLearned
from task_sim.str.amdp_reward import reward, is_terminal


class AMDPValueIteration:
    grasp_objects = ['drawer', 'apple']
    place_objects = ['stack', 'drawer', '']
    move_objects = ['drawer', 'stack', 'apple', 'l', 'f', 'r', 'b', 'fl', 'fr', 'br', 'bl']
    gripper_objects = ['', 'drawer', 'apple']

    def __init__(self, amdp_id=0):
        self.U = {}
        self.actions = []
        self.amdp_id = amdp_id

        self.initialize()

    def initialize(self):
        # initialize state list
        # print 'Enumerating states (started at: ' + str(datetime.datetime.now())  + ")"
        s = AMDPState(amdp_id=self.amdp_id)
        if self.amdp_id == 0:
            s.relations['gripper_in_front_of_drawer'] = True
            s.relations['gripper_open'] = True
            s.relations['drawer_closing_stack'] = True
            self.U[deepcopy(s)] = 0.0
        elif self.amdp_id == 1:
            self.U[deepcopy(s)] = 0.0
            s.relations['gripper_open'] = True
            self.U[deepcopy(s)] = 0.0
            s.relations['gripper_touching_drawer'] = True
            self.U[deepcopy(s)] = 0.0
            s.relations['gripper_open'] = False
            self.U[deepcopy(s)] = 0.0
            s.relations['gripper_open'] = True
            s.relations['gripper_touching_stack'] = True
            self.U[deepcopy(s)] = 0.0
            s.relations['gripper_open'] = False
            self.U[deepcopy(s)] = 0.0
            s.relations['gripper_touching_stack'] = False
            s.relations['gripper_touching_drawer'] = False
            s.relations['gripper_above_drawer'] = True
            self.U[deepcopy(s)] = 0.0
        elif self.amdp_id == 2:
            s.relations['apple_right_of_drawer'] = True
            s.relations['apple_in_front_of_drawer'] = True
            s.relations['apple_below_drawer'] = True
            s.relations['apple_right_of_gripper'] = True
            s.relations['apple_in_front_of_gripper'] = True
            s.relations['apple_below_gripper'] = True
            s.relations['gripper_right_of_drawer'] = True
            s.relations['gripper_holding_drawer'] = True
            s.relations['gripper_touching_drawer'] = True
            self.U[deepcopy(s)] = 0.0
            s.relations['gripper_right_of_drawer'] = False
            s.relations['gripper_holding_drawer'] = False
            s.relations['gripper_touching_drawer'] = False
            self.U[deepcopy(s)] = 0.0
            s.relations['gripper_touching_drawer'] = True
            self.U[deepcopy(s)] = 0.0
            s.relations['gripper_touching_drawer'] = True
            s.relations['gripper_touching_stack'] = True
            self.U[deepcopy(s)] = 0.0
            s.relations['gripper_touching_drawer'] = False
            s.relations['gripper_touching_stack'] = True
            self.U[deepcopy(s)] = 0.0
            s.relations['gripper_touching_stack'] = False
            s.relations['gripper_above_drawer'] = True
            self.U[deepcopy(s)] = 0.0
        elif self.amdp_id == 3:
            s.relations['apple_inside_drawer'] = False
            s.relations['drawer_closing_stack'] = True
            self.U[deepcopy(s)] = 0.0
        elif self.amdp_id == 4:
            s.relations['apple_inside_drawer'] = False
            s.relations['banana_inside_drawer'] = False
            s.relations['drawer_closing_stack'] = True
            self.U[deepcopy(s)] = 0.0
        elif self.amdp_id == 5:
            s.relations['apple_inside_drawer'] = False
            s.relations['banana_inside_drawer'] = False
            s.relations['carrot_inside_drawer'] = False
            s.relations['drawer_closing_stack'] = True
            self.U[deepcopy(s)] = 0.0
        elif self.amdp_id == -1:
            s.relations['apple_right_of_drawer'] = True
            s.relations['apple_in_front_of_drawer'] = True
            s.relations['apple_below_drawer'] = True
            s.relations['apple_right_of_gripper'] = True
            s.relations['apple_behind_gripper'] = True
            s.relations['apple_below_gripper'] = True
            s.relations['gripper_in_front_of_drawer'] = True
            s.relations['gripper_open'] = True
            s.relations['drawer_closing_stack'] = True
            self.U[deepcopy(s)] = 0.0
        elif self.amdp_id == -2:
            s.relations['apple_behind_gripper'] = True
            s.relations['gripper_in_front_of_drawer'] = True
            s.relations['banana_in_front_of_drawer'] = True
            s.relations['apple_below_gripper'] = True
            s.relations['apple_below_drawer'] = True
            s.relations['banana_below_gripper'] = True
            s.relations['apple_right_of_gripper'] = True
            s.relations['banana_behind_gripper'] = True
            s.relations['apple_in_front_of_drawer'] = True
            s.relations['banana_below_drawer'] = True
            s.relations['drawer_closing_stack'] = True
            s.relations['gripper_open'] = True
            s.relations['banana_right_of_gripper'] = True
            s.relations['apple_right_of_drawer'] = True
            s.relations['apple_right_of_banana'] = True
            s.relations['banana_right_of_drawer'] = True
            s.relations['apple_behind_banana'] = True
            self.U[deepcopy(s)] = 0.0
        elif self.amdp_id == -3:
            s.relations['apple_behind_gripper'] = True
            s.relations['gripper_in_front_of_drawer'] = True
            s.relations['apple_left_of_carrot'] = True
            s.relations['banana_left_of_carrot'] = True
            s.relations['banana_in_front_of_drawer'] = True
            s.relations['apple_below_gripper'] = True
            s.relations['carrot_below_drawer'] = True
            s.relations['apple_below_drawer'] = True
            s.relations['carrot_behind_gripper'] = True
            s.relations['banana_below_gripper'] = True
            s.relations['apple_right_of_gripper'] = True
            s.relations['banana_behind_gripper'] = True
            s.relations['apple_in_front_of_drawer'] = True
            s.relations['banana_below_drawer'] = True
            s.relations['carrot_right_of_gripper'] = True
            s.relations['drawer_closing_stack'] = True
            s.relations['carrot_right_of_drawer'] = True
            s.relations['carrot_below_gripper'] = True
            s.relations['gripper_open'] = True
            s.relations['banana_in_front_of_carrot'] = True
            s.relations['banana_right_of_gripper'] = True
            s.relations['apple_in_front_of_carrot'] = True
            s.relations['apple_right_of_drawer'] = True
            s.relations['carrot_in_front_of_drawer'] = True
            s.relations['apple_right_of_banana'] = True
            s.relations['banana_right_of_drawer'] = True
            s.relations['apple_behind_banana'] = True
            self.U[deepcopy(s)] = 0.0
        else:
            self.U[deepcopy(s)] = 0.0

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
        else:
            a.action_type = Action.GRASP
            for o in self.grasp_objects:
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
            for o in self.place_objects:
                a.object = o
                self.actions.append(deepcopy(a))

            a.action_type = Action.MOVE_ARM
            for o in self.move_objects:
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
        if self.amdp_id >= 3 and self.amdp_id <= 5:
            self.T = AMDPTransitionsLearned(amdp_id=self.amdp_id, load=False)
        else:
            self.T = AMDPTransitionsLearned(amdp_id=self.amdp_id, load=True)

    def solve(self):
        gamma = 0.8
        epsilon = 1
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
                if is_terminal(s, amdp_id=self.amdp_id):
                    u = reward(s, amdp_id=self.amdp_id)
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
                    successors = self.T.transition_function(s, a)
                    current_u = 0.0
                    for i in range(len(successors)):
                        p = successors[i][0]
                        s_prime = successors[i][1]

                        if s_prime not in self.U:
                            # # Fix state size at iteration 8
                            # if n > 8:
                            #     continue
                            # else:
                            #     U_prime[deepcopy(s_prime)] = 0.0
                            U_prime[deepcopy(s_prime)] = 0.0
                        else:
                            current_u += p*self.U[s_prime]

                    if current_u > max_u:
                        max_u = current_u

                u = reward(s, amdp_id=self.amdp_id) + gamma*max_u
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
                pickle.dump(self.U, file('U' + str(self.amdp_id) + '_iter_' + str(n) + '.pkl', mode='w'))
            print 'Elapsed time: ' + str(datetime.datetime.now() - start_time)

        print 'Total elapsed time: ' + str(datetime.datetime.now() - start_time)
        print 'Finished. Saving...'
        pickle.dump(self.U, file('trained_U' + str(self.amdp_id) + '.pkl', mode='w'))
        print 'Utilities saved.'


if __name__ == '__main__':
    a = AMDPValueIteration(amdp_id=-3)
    a.solve()
