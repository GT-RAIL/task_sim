#!/usr/bin/env python

# Python
import copy
import datetime
import glob
import pickle
import time
from random import randint, shuffle

# Network
import matplotlib.pyplot as plt
import networkx as nx

# ROS
import rosbag
import rospkg
import rospy

from task_sim.msg import Action
from task_sim.amdp_plan_action import AMDPPlanAction


class AMDPPlanNetwork:

    def __init__(self, construct=False):
        # data structures to store parsed information (used in graph construction)
        self.action_list = []
        self.edges = []
        self.edge_weights = {}
        self.object_to_index = {}  # object -> index mapping
        self.index_to_object = {}  # index -> object mapping
        self.action_context_to_index = {}  # (action, context) -> index mapping
        self.index_to_action_context = {}  # index -> (action, context) mapping
        self.action_object_to_index = {}  # (action, object) -> index mapping
        self.index_to_action_object = {}  # index -> (action, object) mapping
        self.object_by_action_context = []  # probability table for how often each object has an action with a context applied to it
        self.context_by_action_object = []  # probability table for how often each object acts as context under a certain action for another object
        self.object_frequency_count = []  # frequency table for

        # network
        self.plan_network = None
        self.node_labels = {}

        if construct:
            self.construct_network()
            self.test_output()

    def construct_network(self, task='task1', affordance_threshold=0.5, output_suffix=None):
        demo_list = glob.glob(rospkg.RosPack().get_path('task_sim') + '/data/' + task + "/demos/*.bag")
        output_suffix = output_suffix or ('_' + str(datetime.date.today()))
        print 'Loading demonstrations for ' + task + '...'
        print 'Found ' + str(len(demo_list)) + ' demonstrations.'

        self.parse_actions(demo_list)

        print 'Saving action list (for planners)'
        path = rospkg.RosPack().get_path('task_sim') + '/data/' + task + '/models/'
        pickle.dump(self.action_list, open(path + 'amdp_plan_action_list' + output_suffix + '.pkl', 'w'))
        print 'Action list saved to data/' + task + '/models directory'

        print 'Nodes: ' + str(len(self.action_list))
        # print str(self.action_list)
        print 'Edges: ' + str(len(self.edges))
        # print str(self.edges)
        print 'Edge weights: ' + str(len(self.edge_weights))
        # print str(self.edge_weights)

        self.build_network()

        self.save_graph(task, output_suffix)

    def parse_actions(self, demo_list):
        print 'Parsing demonstrations for actions'

        for demo_file in demo_list:
            s0 = None
            prev_act = 'start'
            objects_used = []
            action_context_pairs = {}
            action_object_pairs = {}
            print '\nReading ' + demo_file + '...'
            bag = rosbag.Bag(demo_file)
            for topic, msg, t in bag.read_messages(topics=['/table_sim/task_log']):
                s1 = msg.state
                a = msg.action

                if s0 is None:
                    s0 = copy.deepcopy(s1)
                    continue
                else:
                    if a.action_type == Action.NOOP:
                        continue

                    # parse action
                    act = AMDPPlanAction(s0, a, s1)

                    if act not in self.action_list:
                        self.action_list.append(act)

                    edge = (prev_act, act)
                    if edge not in self.edges:
                        self.edges.append(edge)
                        self.edge_weights[edge] = 1
                    else:
                        self.edge_weights[edge] += 1

                    # update counts for interaction probabilities
                    obj = act.object
                    target = act.target
                    act_type = act.action
                    if obj is not None and object != '':
                        if obj not in objects_used:
                            objects_used.append(obj)
                        pair = (act_type, target)
                        if obj not in action_context_pairs:
                            action_context_pairs[obj] = [pair]
                        elif pair not in action_context_pairs[obj]:
                            action_context_pairs[obj].append(pair)
                    if target is not None and target != '':
                        if target not in objects_used:
                            objects_used.append(target)
                        pair = (act_type, obj)
                        if target not in action_object_pairs:
                            action_object_pairs[target] = [pair]
                        elif pair not in action_object_pairs[target]:
                            action_object_pairs[target].append(pair)

                    prev_act = act

                s0 = copy.deepcopy(s1)

            bag.close()

        print 'Demos parsed.'

    def build_network(self):
        self.plan_network = nx.DiGraph()

        # Add nodes and labels
        self.plan_network.add_node('start')
        self.plan_network.add_nodes_from(self.action_list)
        for node in self.plan_network.nodes:
            if node == 'start':
                self.node_labels[node] = 'start'
            else:
                label = str(node.action) + '-' + str(node.object) + '-' + str(node.target)
                if len(node.effects) == 0:
                    label += '-f'
                self.node_labels[node] = label

        # Add edges
        for edge in self.edges:
            self.plan_network.add_edge(edge[0], edge[1], weight=self.edge_weights[edge])

    def save_graph(self, task, output_suffix):
        print 'Saving plan network, labels, and object clusters...'
        path = rospkg.RosPack().get_path('task_sim') + '/data/' + task + '/models/'
        nx.write_gpickle(self.plan_network, path + 'plan_network' + output_suffix + '.pkl')
        print 'Plan network saved to model directory.'
        pickle.dump(self.node_labels, open(path + 'node_labels' + output_suffix + '.pkl', 'w'))
        print 'Node labels saved.'
        pickle.dump(self.cluster_to_objects, open(path + 'clusters' + output_suffix + '.pkl', 'w'))
        print 'Cluster list saved.'
        pickle.dump(self.object_to_cluster, open(path + 'clusters_reverse' + output_suffix + '.pkl', 'w'))
        print 'Cluster reverse lookup list saved.\n'

    def read_graph(self, task, suffix):
        path = rospkg.RosPack().get_path('task_sim') + '/data/' + task + '/models/'
        self.plan_network = nx.read_gpickle(path + 'plan_network' + suffix + '.pkl')
        self.node_labels = pickle.load(open(path + 'node_labels' + suffix + '.pkl'))
        self.cluster_to_objects = pickle.load(open(path + 'clusters' + suffix + '.pkl'))
        self.object_to_cluster = pickle.load(open(path + 'clusters_reverse' + suffix + '.pkl'))
        print 'Plan network loaded.'

    def has_node(self, node):
        return self.plan_network.has_node(node)

    def get_successor_actions(self, node, state):
        action_list = []  # List of successor actions, each entry in form [node, object, target, probability]
        total_weight = 0
        for candidate in self.plan_network.successors(node):
            weight = self.plan_network.get_edge_data(node, candidate)['weight']
            temp_action_list = []
            pre_met_count = 0
            # de-generalize objects and targets, iterate through all combinations:
            objects = self.cluster_to_objects[candidate.object]

            # object is fixed for certain actions... constrain object list for those cases
            if candidate.action in [Action.PLACE, Action.OPEN_GRIPPER, Action.MOVE_ARM, Action.RAISE_ARM,
                                    Action.LOWER_ARM, Action.RESET_ARM]:
                if state.object_in_gripper.lower() in objects:
                    objects = [state.object_in_gripper.lower()]
                else:
                    continue
            if candidate.action == Action.CLOSE_GRIPPER:
                test_obj = ''
                for obj in state.objects:
                    if obj.position == state.gripper_position:
                        test_obj = obj.name.lower()
                        break
                if test_obj in objects:
                    objects = [test_obj]
                else:
                    continue

            targets = self.cluster_to_objects[candidate.target]
            for obj in objects:
                for target in targets:
                    # print '\n******************'
                    # print 'Testing preconditions for ' + str(candidate.action) + ', ' + str(obj) + ', ' + str(target)
                    # print str(candidate)
                    if candidate.check_preconditions(state, obj, target, self.object_to_cluster):
                        # print 'Success'
                        temp_action_list.append([candidate, obj, target, weight])
                        pre_met_count += 1
                    # else:
                        # print 'Failure'
                    # print '****************\n'
            if pre_met_count > 0:
                for act in temp_action_list:
                    act[3] /= float(pre_met_count)
                action_list.extend(temp_action_list)
                total_weight += weight
        if len(action_list) > 0:
            # normalize by weight
            for act in action_list:
                act[3] /= float(total_weight)
        return action_list

    def find_suitable_node(self, state):
        nodes = list(self.plan_network.nodes)
        # shuffle(nodes)
        indices = range(len(nodes))
        shuffle(indices)

        #for node in nodes:
        for i in indices:
            # not sure why this is happening, but sometimes predecessors() complains a node is not in the graph,
            # so node shuffling was switched to index shuffling, which seems to work for now
            node = nodes[i]
            if node == 'start':
                continue
            objects = self.cluster_to_objects[node.object]
            targets = self.cluster_to_objects[node.target]
            parents_checked = False
            for obj in objects:
                for target in targets:
                    if node.check_preconditions(state, obj, target, self.object_to_cluster):
                        # check that any parent's effects match the state
                        valid_nodes = []
                        for parent in self.plan_network.predecessors(node):
                            if parent == 'start':
                                continue
                            if parent.check_effects(state, self.object_to_cluster):
                                valid_nodes.append(parent)
                        if len(valid_nodes) > 0:
                            return valid_nodes[randint(0, len(valid_nodes) - 1)]
                        parents_checked = True
                    if parents_checked:
                        break
                if parents_checked:
                    break

        # No valid nodes found if this point is reached...
        return None

    def show_graph(self):
        layout = nx.spring_layout(self.plan_network)
        nx.draw_networkx_nodes(self.plan_network, layout)
        nx.draw_networkx_edges(self.plan_network, layout)
        nx.draw_networkx_labels(self.plan_network, layout, self.node_labels, font_size=10)
        plt.axis('off')
        plt.show()

    def test_output(self):
        print 'Objects: '
        print str(self.object_to_index)

        print '\nAction-Contexts:'
        print str(self.action_context_to_index)

        print '\nAction-Objects:'
        print str(self.action_object_to_index)

        print '\nObject frequencies:'
        print str(self.object_frequency_count)

        print '\n Object by Action-Context table:'
        for i in range(len(self.object_by_action_context)):
            out = 'Object ' + str(i)
            for j in range(len(self.object_by_action_context[i])):
                out += '\t' + str(self.object_by_action_context[i][j])
            print out

        print '\n Context by Action-Object table:'
        for i in range(len(self.context_by_action_object)):
            out = 'Context ' + str(i)
            for j in range(len(self.context_by_action_object[i])):
                out += '\t' + str(self.context_by_action_object[i][j])
            print out

        print '\nObject clusters: '
        print str(self.cluster_to_objects)

        print '\n Reverse lookup check: '
        print str(self.object_to_cluster)

        self.show_graph()


class TaskObject:

    def __init__(self, name):
        self.name = name
        self.action_affordances = []
        self.context_affordances = []

    def add_action_affordance(self, pair):
        if pair not in self.action_affordances:
            self.action_affordances.append(pair)

    def add_context_affordance(self, pair):
        if pair not in self.context_affordances:
            self.context_affordances.append(pair)


if __name__ == '__main__':
    rospy.init_node('plan_network')
    plan_network = PlanNetwork(construct=True)
