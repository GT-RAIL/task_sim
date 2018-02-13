#!/usr/bin/env python

import heapq

from plan_action import PlanAction
from plan_state import PlanState

class ForwardPlanner:
    def __init__(self, actions):
        self.actions = actions
        self.node = None
        self.frontier = []
        self.state_hash_table = {}
        self.explored = []

        print 'Actions: \n' + str(self.actions)

    def initialize(self, state):
        self.node = PlanNode(PlanState(state), [])
        self.frontier = []
        self.state_hash_table = {}
        self.add_to_queue(self.node)
        # self.frontier.put((self.node.cost, self.node))
        # self.frontier_state_to_cost[self.node.state] = self.node.cost
        self.explored = []

    def plan(self, state=None):
        if state is not None:
            self.initialize(state)

        if self.node is None:
            print 'Planner not initialized!'
            return

        print '\n--------------------------------------------------------------------------\n'
        print 'Initial state:\n'
        print str(self.node.state)
        print '\n--------------------------------------------------------------------------\n'

        while True:
            if len(self.frontier) == 0:
                return None
            self.node = self.pop_frontier()
            if self.node is None:
                return None
            if self.goal_test():
                print '\n--------------------------------------------------------------------------\n'
                print 'Goal state:\n'
                print str(self.node.state)

                print '\n\nGoal distance: ' + str(goal_distance(self.node.state))
                print '\n--------------------------------------------------------------------------\n'
                return self.node.path
            self.explored.append(self.node.state)
            for action in self.actions:
                if self.node.state.check_action(action):
                    child_state = self.node.state.apply_action(action)
                    child_path = []
                    child_path.extend(self.node.path)
                    child_path.append(action)
                    child = PlanNode(child_state, child_path)
                    if (child.state not in self.explored and child.state not in self.state_hash_table) or \
                            (child.state in self.state_hash_table and child.cost < self.state_hash_table[child.state].cost):
                        self.add_to_queue(child)

    def goal_test(self):
        """Check if a state satisfies the goal (hardcoded goal)"""
        return goal_distance(self.node.state) == 0

    def add_to_queue(self, node):
        if node.state in self.state_hash_table:
            # Replace
            self.state_hash_table[node.state].replaced = True
        # Add
        self.state_hash_table[node.state] = node
        heapq.heappush(self.frontier, (node.cost, self.state_hash_table[node.state]))

    def pop_frontier(self):
        node = heapq.heappop(self.frontier)[1]
        while node.replaced:
            if len(self.frontier) == 0:
                return None
            node = heapq.heappop(self.frontier)[1]
        del self.state_hash_table[node.state]
        return node


class PlanNode():
    def __init__(self, plan_state, path):
        self.state = plan_state

        # Action history
        self.path = path

        # Path cost (used for planner)
        self.cost = len(path) + goal_distance(self.state)

        # Flag for whether a node has been replaced by the same node with lower cost
        self.replaced = False


def goal_distance(state):
    """Heuristic for distance from goal (hardcoded goal)"""
    dst = 0

    if 'apple' in state.objects:
        if not state.objects['apple'].in_box:
            dst += 1

    if 'batteries' in state.objects:
        if not state.objects['batteries'].in_drawer:
            dst += 1

    if 'flashlight' in state.objects:
        if not state.objects['flashlight'].in_drawer:
            dst += 1

    if 'drawer' in state.containers:
        if state.containers['drawer'].open:
            dst += 1

    if 'box' in state.containers:
        if state.containers['box'].open:
            dst += 1

    dst /= 2.0

    return dst