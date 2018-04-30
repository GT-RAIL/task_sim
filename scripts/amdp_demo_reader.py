#!/usr/bin/env python

# Python
import copy
import datetime
import glob
import os
import pickle
import yaml

# ROS
import rosbag
import rospkg
import rospy
from geometry_msgs.msg import Point
from task_sim.msg import Action

from task_sim import data_utils as DataUtils
from task_sim.oomdp.oo_state import OOState
from task_sim.str.amdp_state import AMDPState


class AMDPDemoReader:

    def __init__(self):
        self.task = rospy.get_param('~task', 'task4')
        self.output_suffix = rospy.get_param('~output_suffix', '_' + str(datetime.date.today()))
        self.amdp_id = rospy.get_param('~amdp_id', 2)

        print 'Loading demonstrations for ' + self.task + '...'
        self.demo_list = glob.glob(rospkg.RosPack().get_path('task_sim') + '/data/' + self.task + "/demos/*.bag")
        print 'Found ' + str(len(self.demo_list)) + ' demonstrations.'

    def read_all(self):
        print 'Parsing demonstrations for amdp_id ' + str(self.amdp_id)

        # Initialize data structures based on parse modes
        state_action_pairs = []
        prev_state = None
        prev_state_msg = None

        for demo_file in self.demo_list:
            print '\nReading ' + demo_file + '...'
            bag = rosbag.Bag(demo_file)
            for topic, msg, t in bag.read_messages(topics=['/table_sim/task_log']):
                # Parse messages based on parse modes
                state = AMDPState(amdp_id=self.amdp_id, state=OOState(state=msg.state))

                if prev_state_msg is None:
                    prev_state_msg = copy.deepcopy(msg.state)

                if prev_state is None:
                    prev_state = state.to_vector()
                elif msg.action.action_type != Action.NOOP:
                    a = msg.action
                    # convert action into something that fits into the new action list
                    if a.action_type == Action.PLACE:
                        a.object = DataUtils.get_task_frame(prev_state_msg, a.position)
                        a.position = Point()
                    elif a.action_type == Action.MOVE_ARM:
                        a.object = DataUtils.get_task_frame(prev_state_msg, a.position)
                        if a.object != 'stack' and a.object != 'drawer':
                            for o in prev_state_msg.objects:
                                if o.name != 'apple':
                                    continue
                                if a.position == o.position:
                                    a.object = 'apple'
                                    break
                            if a.object != 'apple':
                                x = prev_state_msg.gripper_position.x
                                y = prev_state_msg.gripper_position.y
                                px = a.position.x
                                py = a.position.y
                                if px == x and py > y:
                                    a.object = 'b'
                                elif px < x and py > y:
                                    a.object = 'bl'
                                elif px < x and py == y:
                                    a.object = 'l'
                                elif px < x and py < y:
                                    a.object = 'fl'
                                elif px == x and py < y:
                                    a.object = 'f'
                                elif px > x and py < y:
                                    a.object = 'fr'
                                elif px > x and py == y:
                                    a.object = 'r'
                                else:
                                    a.object = 'br'
                        a.position = Point()
                    elif a.action_type == Action.GRASP:
                        a.position = Point()
                    else:
                        a.position = Point()
                        a.object = ''

                    pair = {'state': copy.deepcopy(prev_state), 'action': str(a.action_type) + ':' + a.object}
                    state_action_pairs.append(pair)

                    # update stored data for next iteration
                    prev_state_msg = copy.deepcopy(msg.state)
                    prev_state = state.to_vector()

            bag.close()

        # Write out data files
        self.write_yaml(state_action_pairs, 'amdp_sa')

    def write_yaml(self, data, parse_mode):
        filename = parse_mode + '_' + str(self.amdp_id) + '.yaml'
        fullpath = rospkg.RosPack().get_path('task_sim') + '/data/' + self.task + '/training/' + filename
        print 'Writing ' + parse_mode + ' to ' + filename + ' in the data/training directory.'
        if os.path.exists(fullpath):
            f = open(fullpath, 'a')
        else:
            f = open(fullpath, 'w')
        yaml.dump(data, f)
        f.close()


if __name__ == '__main__':
    rospy.init_node('amdp_demo_reader')
    adr = AMDPDemoReader()
    adr.read_all()
