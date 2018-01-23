#!/usr/bin/env python

# Python
import copy
import datetime
import glob

# ROS
import rosbag
import rospkg
import rospy
from task_sim.msg import Action

from plan_action import PlanAction


class PlanNetwork:

    def __init__(self):
        self.task = rospy.get_param('~task', 'task1')
        self.affordance_threshold = rospy.get_param('~affordance_threshold', 0.4)
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

        self.construct_network()

        self.test_output()

    def construct_network(self):
        print 'Loading demonstrations for ' + self.task + '...'
        print 'Found ' + str(len(self.demo_list)) + ' demonstrations.'

        self.parse_actions()

        self.cluster_objects()

    def parse_actions(self):
        print 'Parsing demonstrations for actions'

        s0 = None
        prev_act = 'start'

        for demo_file in self.demo_list:
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
                    if task_objects[i].name in self.object_to_cluster and task_objects[j].name in self.object_to_cluster:
                        continue
                    if task_objects[i].name in self.object_to_cluster:
                        group = self.object_to_cluster[task_objects[i].name]
                        self.object_to_cluster[task_objects[j].name] = group
                        self.cluster_to_objects[group].append(task_objects[j].name)
                    elif task_objects[j].name in self.object_to_cluster:
                        group = self.object_to_cluster[task_objects[j].name]
                        self.object_to_cluster[task_objects[i].name] = group
                        self.cluster_to_objects[group].append(task_objects[i].name)
                    else:
                        group = 'cluster' + str(num_clusters)
                        self.object_to_cluster[task_objects[i].name] = group
                        self.object_to_cluster[task_objects[j].name] = group
                        self.cluster_to_objects[group] = [task_objects[i].name, task_objects[j].name]
                        num_clusters += 1

        # add non-clustered objects to the cluster maps
        for obj in task_objects:
            if obj.name in self.object_to_cluster:
                continue
            self.object_to_cluster[obj.name] = obj.name
            self.cluster_to_objects[obj.name] = obj.name

        print '\nObject clusters: '
        print str(self.cluster_to_objects)

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
