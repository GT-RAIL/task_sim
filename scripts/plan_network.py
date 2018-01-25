#!/usr/bin/env python

# Python
import copy
import datetime
import glob

# Network
import matplotlib.pyplot as plt
import networkx as nx

# ROS
import rosbag
import rospkg
import rospy
from task_sim.msg import Action

from plan_action import PlanAction


class PlanNetwork:

    def __init__(self):
        self.task = rospy.get_param('~task', 'task1')
        self.affordance_threshold = rospy.get_param('~affordance_threshold', 0.7)
        self.demo_list = glob.glob(rospkg.RosPack().get_path('task_sim') + '/data/' + self.task + "/demos/*.bag")
        self.output_suffix = rospy.get_param('~output_suffix', '_' + str(datetime.date.today()))

        # data structures to store parsed information
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
        self.object_to_cluster = {}  # object -> cluster mapping
        self.cluster_to_objects = {}  # cluster -> [obj_1, obj_2, ..., obj_n] mapping

        # network
        self.plan_network = None
        self.node_labels = {}

        self.construct_network()

        self.test_output()

    def construct_network(self):
        print 'Loading demonstrations for ' + self.task + '...'
        print 'Found ' + str(len(self.demo_list)) + ' demonstrations.'

        self.parse_actions()

        self.cluster_objects()

        print '\n\nBefore generalization:'
        print 'Nodes: ' + str(len(self.action_list))
        # print str(self.action_list)
        print 'Edges: ' + str(len(self.edges))
        # print str(self.edges)
        print 'Edge weights: ' + str(len(self.edge_weights))
        # print str(self.edge_weights)

        self.generalize_nodes_and_edges()

        print '\n\nAfter generalization:'
        print 'Nodes: ' + str(len(self.action_list))
        # print str(self.action_list)
        print 'Edges: ' + str(len(self.edges))
        # print str(self.edges)
        print 'Edge weights: ' + str(len(self.edge_weights))
        # print str(self.edge_weights)
        print '\n\n'

        self.build_network()

        self.show_graph()

    def parse_actions(self):
        print 'Parsing demonstrations for actions'

        for demo_file in self.demo_list:
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
                    act = PlanAction(s0, a, s1)

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

            # update global object, object_action, and object_context counts
            for obj in objects_used:
                if obj not in self.object_to_index:
                    self.object_by_action_context.append([])
                    self.context_by_action_object.append([])
                    if len(self.object_by_action_context) > 1:
                        for n in range(len(self.object_by_action_context[0])):
                            self.object_by_action_context[len(self.object_by_action_context) - 1].append(0)
                    if len(self.context_by_action_object) > 1:
                        for n in range(len(self.context_by_action_object[0])):
                            self.context_by_action_object[len(self.context_by_action_object) - 1].append(0)
                    self.object_frequency_count.append(1)
                    index = len(self.object_frequency_count) - 1
                    self.object_to_index[obj] = index
                    self.index_to_object[index] = obj

                else:
                    self.object_frequency_count[self.object_to_index[obj]] += 1

            for obj in action_context_pairs.keys():
                action_context_pair_list = action_context_pairs[obj]
                for pair in action_context_pair_list:
                    i = self.object_to_index[obj]
                    if pair not in self.action_context_to_index:
                        for n in range(len(self.object_by_action_context)):
                            if n == i:
                                self.object_by_action_context[n].append(1)
                            else:
                                self.object_by_action_context[n].append(0)
                        index = len(self.object_by_action_context[i]) - 1
                        self.action_context_to_index[pair] = index
                        self.index_to_action_context[index] = pair
                    else:
                        self.object_by_action_context[i][self.action_context_to_index[pair]] += 1

            for obj in action_object_pairs.keys():
                action_object_pair_list = action_object_pairs[obj]
                i = self.object_to_index[obj]
                for pair in action_object_pair_list:
                    if pair not in self.action_object_to_index:
                        for n in range(len(self.context_by_action_object)):
                            if n == i:
                                self.context_by_action_object[n].append(1)
                            else:
                                self.context_by_action_object[n].append(0)
                        index = len(self.context_by_action_object[i]) - 1
                        self.action_object_to_index[pair] = index
                        self.index_to_action_object[index] = pair
                    else:
                        self.context_by_action_object[i][self.action_object_to_index[pair]] += 1

        print 'Demos parsed.'

    def cluster_objects(self):
        # create TaskObjects for each identified object
        task_objects = []
        for i in range(len(self.index_to_object)):
            task_objects.append(TaskObject(self.index_to_object[i]))

        # calculate probabilities for objects
        for i in range(len(self.object_by_action_context)):
            n = self.object_frequency_count[i]
            for j in range(len(self.object_by_action_context[i])):
                self.object_by_action_context[i][j] /= float(n)

        # TODO: scale probabilities across actions (note: actions, not action-contexts) (should we do this?)
        action_probs = {}
        for i in range(len(self.object_by_action_context)):
            for j in range(len(self.object_by_action_context[i])):
                action = self.index_to_action_context[j][0]
                if action in action_probs:
                    action_probs[action].append(self.object_by_action_context[i][j])
                else:
                    action_probs[action] = [self.object_by_action_context[i][j]]
        for i in range(len(self.object_by_action_context)):
            for j in range(len(self.object_by_action_context)):
                self.object_by_action_context[i][j] /= float(max(action_probs[self.index_to_action_context[j][0]]))


        # add high probability actions for each object's action affordance list
        for i in range(len(self.object_by_action_context)):
            for j in range(len(self.object_by_action_context[i])):
                if self.object_by_action_context[i][j] >= self.affordance_threshold:
                    task_objects[i].add_action_affordance(self.index_to_action_context[j])

        # calculate probabilities for objects used as context
        for i in range(len(self.context_by_action_object)):
            n = self.object_frequency_count[i]
            for j in range(len(self.context_by_action_object[i])):
                self.context_by_action_object[i][j] /= float(n)

        # TODO: scale probabilities across actions (note: actions, not action-objects) (should we do this?)
        action_probs = {}
        for i in range(len(self.context_by_action_object)):
            for j in range(len(self.context_by_action_object[i])):
                action = self.index_to_action_object[j][0]
                if action in action_probs:
                    action_probs[action].append(self.context_by_action_object[i][j])
                else:
                    action_probs[action] = [self.context_by_action_object[i][j]]
        for i in range(len(self.context_by_action_object)):
            for j in range(len(self.context_by_action_object)):
                self.context_by_action_object[i][j] /= float(max(action_probs[self.index_to_action_object[j][0]]))

        # add high probability actions for each object's context affordance list
        for i in range(len(self.context_by_action_object)):
            for j in range(len(self.context_by_action_object[i])):
                if self.context_by_action_object[i][j] >= self.affordance_threshold:
                    task_objects[i].add_context_affordance(self.index_to_action_object[j])

        # create clusters for anything with identical affordance lists
        num_clusters = 0
        for i in range(len(task_objects)):
            for j in range(i + 1, len(task_objects)):
                if (task_objects[i].action_affordances == task_objects[j].action_affordances
                    and task_objects[i].context_affordances == task_objects[j].context_affordances):
                    if task_objects[i].name.lower() in self.object_to_cluster and task_objects[j].name.lower() in self.object_to_cluster:
                        continue
                    if task_objects[i].name.lower() in self.object_to_cluster:
                        group = self.object_to_cluster[task_objects[i].name.lower()]
                        self.object_to_cluster[task_objects[j].name.lower()] = group
                        self.cluster_to_objects[group].append(task_objects[j].name.lower())
                    elif task_objects[j].name.lower() in self.object_to_cluster:
                        group = self.object_to_cluster[task_objects[j].name.lower()]
                        self.object_to_cluster[task_objects[i].name.lower()] = group
                        self.cluster_to_objects[group].append(task_objects[i].name.lower())
                    else:
                        group = 'cluster' + str(num_clusters)
                        self.object_to_cluster[task_objects[i].name.lower()] = group
                        self.object_to_cluster[task_objects[j].name.lower()] = group
                        self.cluster_to_objects[group] = [task_objects[i].name.lower(), task_objects[j].name.lower()]
                        num_clusters += 1

        # add non-clustered objects to the cluster maps
        for obj in task_objects:
            if obj.name.lower() in self.object_to_cluster:
                continue
            self.object_to_cluster[obj.name.lower()] = obj.name.lower()
            self.cluster_to_objects[obj.name.lower()] = obj.name.lower()

    def generalize_nodes_and_edges(self):
        general_actions = []
        general_edges = []
        general_edge_weights = {}
        for act in self.action_list:
            general_act = self.generalize_action(act)
            if general_act not in general_actions:
                general_actions.append(general_act)
        for edge in self.edges:
            general_edge = (self.generalize_action(edge[0]), self.generalize_action(edge[1]))
            if general_edge in general_edges:
                general_edge_weights[general_edge] += self.edge_weights[edge]
            else:
                general_edges.append(general_edge)
                general_edge_weights[general_edge] = self.edge_weights[edge]
        self.action_list = general_actions
        self.edges = general_edges
        self.edge_weights = general_edge_weights

    def generalize_action(self, act):
        if act == 'start':
            return act
        result = copy.deepcopy(act)
        if act.object is not None and act.object != '':
            if act.object.lower() in self.object_to_cluster:
                result.object = self.object_to_cluster[act.object.lower()]
            else:
                result.object = act.object.lower()
        if act.target is not None and act.target != '':
            if act.target.lower() in self.object_to_cluster:
                result.target = self.object_to_cluster[act.target.lower()]
            else:
                result.target = act.target.lower()
        if act.object_in_gripper is not None and act.object_in_gripper != '':
            if act.object_in_gripper.lower() in self.object_to_cluster:
                result.object_in_gripper = self.object_to_cluster[act.object_in_gripper.lower()]
            else:
                result.object_in_gripper = act.object_in_gripper.lower()
        for effect in act.effects:
            effect_list = []
            for item in act.effects[effect]:
                if item.lower() in self.object_to_cluster:
                    effect_list.append(self.object_to_cluster[item.lower()])
                else:
                    effect_list.append(item.lower())
            result.effects[effect] = effect_list
        return result

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

    def show_graph(self):
        layout = nx.spectral_layout(self.plan_network)
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
    plan_network = PlanNetwork()
