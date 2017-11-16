#!/usr/bin/env python

# Python
import datetime
import glob
import os
import yaml

# ROS
import rosbag
import rospkg
import rospy
from task_sim.msg import Action


class DemoReader:

    def __init__(self):
        self.task = rospy.get_param('~task', 'task1')
        self.parse_modes = rospy.get_param('~parse_modes', 'all').split(',')
        self.output_suffix = rospy.get_param('~output_suffix', '_' + str(datetime.date.today()))
        supported_parse_modes = ['state-action']

        if 'all' in self.parse_modes:
            self.parse_modes = supported_parse_modes
        else:
            for parse_mode in self.parse_modes:
                if parse_mode not in supported_parse_modes:
                    usage = 'Unsupported parse mode: ' + parse_mode + '. Supported parse modes are:'
                    for parse_param_string in supported_parse_modes:
                        usage += '\n\t' + parse_param_string
                    print usage
                    return

        print 'Loading demonstrations for ' + self.task + '...'
        self.demo_list = glob.glob(rospkg.RosPack().get_path('task_sim') + '/data/' + self.task + "/demos/*.bag")
        print 'Found ' + str(len(self.demo_list)) + ' demonstrations.'

    def read_all(self):
        print 'Parsing demonstrations for:'
        for parse_mode in self.parse_modes:
            print '\t' + parse_mode

        # Initialize data structures based on parse modes
        if 'state-action' in self.parse_modes:
            state_action_pairs = []
            prev_state = None

        for demo_file in self.demo_list:
            print '\nReading ' + demo_file + '...'
            bag = rosbag.Bag(demo_file)

            for topic, msg, t in bag.read_messages(topics=['/table_sim/task_log']):

                # Parse messages based on parse modes
                if 'state-action' in self.parse_modes:
                    if prev_state is None:
                        prev_state = DemoReader.naive_state_vector(msg.state)
                    elif msg.action.action_type != Action.NOOP:
                        pair = {'state': prev_state, 'action': DemoReader.naive_action_vector(msg.action)}
                        prev_state = DemoReader.naive_state_vector(msg.state)
                        state_action_pairs.append(pair)

            bag.close()

        # Write out data files
        if 'state-action' in self.parse_modes:
            self.write_yaml(state_action_pairs, 'state-action')

    def write_yaml(self, data, parse_mode):
        filename = parse_mode + self.output_suffix + '.yaml'
        fullpath = rospkg.RosPack().get_path('task_sim') + '/data/' + self.task + '/training/' + filename
        print 'Writing ' + parse_mode + ' to ' + filename + ' in the data/training directory.'
        if os.path.exists(fullpath):
            f = open(fullpath, 'a')
        else:
            f = open(fullpath, 'w')
        yaml.dump(data, f)
        f.close()

    @staticmethod
    def naive_state_vector(state):
        """Convert a state message into a simple feature vector."""
        vector = []
        for obj in state.objects:
            vector.extend([obj.position.x, obj.position.y, obj.position.z, int(obj.in_drawer), int(obj.in_box),
                           int(obj.on_lid), int(obj.in_gripper), int(obj.occluded), int(obj.lost)])
        vector.extend([state.drawer_position.x, state.drawer_position.y, state.drawer_position.theta,
                       state.drawer_opening, state.box_position.x, state.box_position.y, state.box_position.z,
                       state.lid_position.x, state.lid_position.y, state.lid_position.z, state.gripper_position.x,
                       state.gripper_position.y, state.gripper_position.z, int(state.gripper_open),
                       DemoReader.name_to_int(state.object_in_gripper)])
        return vector

    @staticmethod
    def naive_action_vector(action):
        """Convert an action message into a simple feature vector."""
        vector = []
        vector.extend([action.action_type, DemoReader.name_to_int(action.object), action.position.x, action.position.y])
        return vector

    @staticmethod
    def name_to_int(name):
        if name.lower() == '':
            return 0
        if name.lower() == 'drawer':
            return 1
        elif name.lower() == 'lid':
            return 2
        elif name.lower() == 'apple':
            return 3
        elif name.lower() == 'batteries':
            return 4
        elif name.lower() == 'flashlight':
            return 5
        elif name.lower() == 'granola':
            return 6
        elif name.lower() == 'knife':
            return 7
        else:
            return -1


if __name__ == '__main__':
    rospy.init_node('demo_reader')
    demo_reader = DemoReader()
    demo_reader.read_all()