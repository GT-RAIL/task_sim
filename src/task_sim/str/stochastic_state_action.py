#!/usr/bin/env python
# Stochastic state and action definitions

from copy import deepcopy
from random import random
import h5py

class StochasticAction:
    """Set a distribution over states and sample based on probabilities"""
    def __init__(self, a):
        self.actions = [deepcopy(a)]
        self.frequency = [1]
        self.p = [1.0]

    def update(self, a):
        if a in self.actions:
            self.frequency[self.actions.index(a)] += 1
        else:
            self.actions.append(deepcopy(a))
            self.frequency.append(1)
            self.p.append(0.0)

        # Update probabilities
        total = float(sum(self.frequency))
        for i in range(len(self.frequency)):
            self.p[i] = self.frequency[i]/total

    def select_action(self):
        r = random()
        n = 0
        for i in range(len(self.actions)):
            n += self.p[i]
            if n > r:
                return self.actions[i]
        return self.actions[len(self.actions) - 1]

class StochasticState:
    """Get a distribution over states and their associated probabilities"""
    def __init__(self):
        self.states = []
        self.frequency = []
        self.p = []

    def add_state(self, s):
        if s in self.states:
            self.frequency[self.states.index(s)] += 1
        else:
            self.states.append(deepcopy(s))
            self.frequency.append(1)
            self.p.append(0.0)

        # Update probabilities
        total = float(sum(self.frequency))
        for i in range(len(self.frequency)):
            self.p[i] = self.frequency[i]/total

    def get_distribution(self):
        return zip(self.p, self.states)
