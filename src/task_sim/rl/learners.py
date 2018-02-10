#!/usr/bin/env python
# This is just a file that contains the different learners that we might want
# to expose

import pickle

from q_learners import EpsilonGreedyQTableAgent

def save_agent(agent, filename):
    """Save the agent to the filename"""
    with open(filename, 'wb') as fd:
        pickle.dump(agent, fd)

def load_agent(filename):
    """Load a saved agent from a filename"""
    agent = None
    with open(filename, 'rb') as fd:
        agent = pickle.load(fd)
    return agent
