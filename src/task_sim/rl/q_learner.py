#!/usr/bin/env python
# This implements Q Learning. The learners are all model-free, but vary in their
# exploration policies

from collections import defaultdict
from utils import argmax
from mdp import MDP
import random
import numpy as np

class QLearningAgent:
    def __init__(self, mdp, N_OAC, OAC, alpha=0.25):
        #Vanilla Q-learning
        self.gamma = mdp.gamma
        self.terminals = mdp.terminals
        self.all_act = mdp.actlist
        self.Q = defaultdict(float)
        self.state_init = mdp.state_init
        self.a = None
        self.r = None
        self.s = None

        if alpha:
            self.alpha = alpha
        else:
            self.alpha = lambda n: 1./(1+n)

        #ACR+Q learning
        self.OAC = OAC
        self.N_OAC = N_OAC
        self.obj_act = mdp.obj_act
        self.obj_loc = mdp.obj_loc
        self.obj_order = mdp.obj_order
        self.non_obj_loc = mdp.non_obj_loc
        self.agent_act = ["up","down","left","right"]

        state1 = list(mdp.state_init)
        self.s = tuple([None]+state1[1:])

    def EpsilonGreedy(self, Q, s, epsilon, episode):
        actions_in_state = self.actions_in_state
        A = actions_in_state(s, episode)
        a = argmax(A, key=lambda a1:Q[s,a1])

        rand_num = random.uniform(0,1)

        if rand_num < epsilon:
            a = random.choice(A)

        return a

    def actions_in_state(self, state, episode):
        #ACR+Q learning
        agent_act = self.agent_act
        state1 = list(state)
        agent_state = state1[0]

        if agent_state in self.terminals:
            return [None]

        if self.OAC and episode < self.N_OAC:
            obj_loc = self.obj_loc
            obj_act = self.obj_act
            if agent_state in obj_loc.keys():
                obj = obj_loc[agent_state]
                idx = self.obj_order.index(obj) + 1
                if state1[idx] != 0:
                    return obj_act[obj]
                else:
                    return agent_act
            else:
                return agent_act
        else:
            return self.all_act

        #Vanilla Q-learning
        '''if state in self.terminals:
            return [None]
        else:
            return self.all_act'''

    def __call__(self, percept, epsilon, episode):
        s1, r1 = self.update_state(percept)
        Q, s, a, r = self.Q, self.s, self.a, self.r
        alpha, gamma, terminals = self.alpha, self.gamma, self.terminals

        state_list = list(s)
        agent_state = state_list[0]
        obj_state = state_list[1:]

        if agent_state in terminals:
            Q[s, None] = r1
        if agent_state is not None:
            Q[s, a] += alpha*(r1 + gamma*max(Q[s1,a1] for a1 in actions_in_state(s1, episode))
                                  - Q[s, a])

        if agent_state in terminals:
            self.s = tuple([None]+obj_state)
            self.a = self.r = None
        else:
            self.s, self.r = s1, r1
            self.a = self.EpsilonGreedy(Q, s1, epsilon, episode)

        return self.a

    def update_state(self, percept):
        return percept

#____________________________Evaluation________________________________________

    def Q_policy(self,pi,episode):  #Learned policy
        actions_in_state = self.actions_in_state
        Q = self.Q
        for keys in Q.keys():
            s = keys[0]
            a = argmax(actions_in_state(s, episode), key=lambda a1:Q[s,a1])
            pi[s] = a
        return pi

    def policy_eval(self, pi, mdp, episode, T = 50): #Evaluate learned policy
        current_state = mdp.state_init

        mdp.reset()
        take_single_action = self.take_single_action
        r_total = 0
        t = 0
        while current_state[0] not in mdp.terminals:
            prev_state = current_state
            if current_state in pi.keys():
                next_action = pi[current_state]
            else:
                next_action = random.choice(self.actions_in_state(current_state, episode))

            current_state = take_single_action(mdp, current_state, next_action)
            current_reward = mdp.R(current_state)
            r_total += current_reward
            t += 1

            if prev_state == current_state or t > T: #No state change
                return r_total

        return r_total

#____________________________Update MDP_____________________________________

    def take_single_action(self, mdp, s, a):
        obj_loc = mdp.obj_loc
        state_list = list(s)
        agent_state = state_list[0]

        if agent_state in obj_loc.keys():
            obj = obj_loc[agent_state]
        else:
            obj = None

        return mdp.T(s, a, obj)

#____________________________Run Single Trial_________________________________

def run_single_trial(agent_program, mdp, epsilon, episode):

    cumu_r = 0
    current_state = mdp.state_init[:]
    #mdp.reset()
    time_trial = 0

    while True:
        current_reward = mdp.R(current_state)
        percept = (current_state, current_reward)
        next_action = agent_program(percept, epsilon, episode)

        #Evaluation metrics
        cumu_r += current_reward
        time_trial += 1

        agent_state = list(current_state)

        if agent_state[0] in mdp.terminals: #Agent reached terminal state
            #return cumu_r, time_trial
            break

        current_state = agent_program.take_single_action(mdp, current_state, next_action)
