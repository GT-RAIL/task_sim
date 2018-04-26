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


class DemoReader:

    def __init__(self, verbose=False):
        """Creating a DemoReader requires a ROS master present"""
        self.task = rospy.get_param('~task', 'task4')
        self.output_suffix = rospy.get_param('~output_suffix', '_' + str(datetime.date.today()))
        self.original_messages = rospy.get_param('~original_messages', 'True')
        self.state_positions = rospy.get_param('~state_positions', 'True')
        self.state_semantics = rospy.get_param('~state_semantics', 'True')
        self.transform_frame = rospy.get_param('~transform_frame', 'False')
        self.action_centric_frames = rospy.get_param('~action_centric_frames', 'False')
        self.robot_centric_frames = rospy.get_param('~robot_centric_frames', 'False')
        self.robot_centric_state = rospy.get_param('~robot_centric_state', 'False')
        self.combined_actions = rospy.get_param('~combined_actions', 'False')
        self.history_buffer = rospy.get_param('~history_buffer', 0)

        if verbose:
            print 'Loading demonstrations for ' + self.task + '...'
        self.demo_list = glob.glob(rospkg.RosPack().get_path('task_sim') + '/data/' + self.task + "/demos/*.bag")
        if verbose:
            print 'Found ' + str(len(self.demo_list)) + ' demonstrations.'

    def read_all(self, write_yaml=False, write_pickle=False, verbose=False):
        """Return state-action pairs based on the bagfiles that the reader has
        been initialized to find"""

        state_action_pairs = []
        prev_state = None
        prev_state_msg = None

        for demo_file in self.demo_list:
            if verbose:
                print '\nReading ' + demo_file + '...'

            bag = rosbag.Bag(demo_file)
            for topic, msg, t in bag.read_messages(topics=['/table_sim/task_log']):
                state = msg.state
                if self.robot_centric_state:
                    state = DataUtils.change_frame(msg.state, 'gripper')

                if prev_state_msg is None:
                    prev_state_msg = msg.state
                if prev_state is None:
                    prev_state = DataUtils.naive_state_vector(
                        msg.state, self.state_positions,
                        self.state_semantics,
                        history_buffer=self.history_buffer
                    )

                elif msg.action.action_type != Action.NOOP:
                    if self.original_messages:
                        pair = (prev_state_msg, msg.action)
                    else:
                        if self.action_centric_frames:
                            if msg.action.action_type in [Action.PLACE]:
                                action_vector = DemoReader.task_object_centric_action_vector(prev_state_msg, msg.action, self.combined_actions)
                            elif msg.action.action_type in [Action.MOVE_ARM]:
                                action_vector = DemoReader.robot_centric_action_vector(prev_state_msg, msg.action, self.combined_actions)
                            else:
                                action_vector = DemoReader.naive_action_vector(prev_state_msg, msg.action, self.transform_frame, self.combined_actions)
                        elif self.robot_centric_frames:
                            if msg.action.action_type in [Action.PLACE, Action.MOVE_ARM]:
                                action_vector = DemoReader.robot_centric_action_vector(prev_state_msg, msg.action, self.combined_actions)
                            else:
                                action_vector = DemoReader.naive_action_vector(prev_state_msg, msg.action, self.transform_frame, self.combined_actions)
                        else:
                            action_vector = DemoReader.naive_action_vector(prev_state_msg, msg.action, self.transform_frame, self.combined_actions)

                        # append action history

                        pair = (prev_state, action_vector)

                    state_action_pairs.append(pair)

                    # update stored data for next iteration
                    prev_state_msg = msg.state
                    prev_state = DataUtils.naive_state_vector(
                        msg.state, self.state_positions, self.state_semantics,
                        history_buffer=self.history_buffer
                    )

                # Done parsing msg

            bag.close()

        # Write out data files if the flags are enabled
        if write_yaml:
            self.write_yaml(state_action_pairs, 'state-action')
        elif write_pickle:
            self.write_pickle(state_action_pairs, 'state-action')

        return state_action_pairs


    def write_yaml(self, data, parse_mode):
        # NOTE: This might be broken
        filename = parse_mode + self.output_suffix + '.yaml'
        fullpath = rospkg.RosPack().get_path('task_sim') + '/data/' + self.task + '/training/' + filename
        print 'Writing ' + parse_mode + ' to ' + filename + ' in the data/training directory.'
        if os.path.exists(fullpath):
            f = open(fullpath, 'a')
        else:
            f = open(fullpath, 'w')
        yaml.dump(data, f)
        f.close()

    def write_pickle(self, data, parse_mode):
        # NOTE: This might be broken
        filename = parse_mode + self.output_suffix + '.pkl'
        fullpath = rospkg.RosPack().get_path('task_sim') + '/data/' + self.task + '/training/' + filename
        print 'Writing ' + parse_mode + ' to ' + filename + ' in the data/training directory.'
        pickle.dump(data, file(fullpath, mode='w'))

    @staticmethod
    def naive_action_vector(state, action, transform_frame=False, combined_actions=False):
        """Convert an action message into a simple feature vector.

        Note that the position will be in the coordinate frame of the target object.
        If the target object is unspecified, it will be set as the closest object.
        (Place will only allow placeable objects to be set as the target frame.)
        """
        frame = action.object
        target = Point(action.position.x, action.position.y, 0)
        if action.action_type in [Action.PLACE]:
            frame = DataUtils.get_task_frame(state, target)
            if (frame != '' or frame.lower() != 'table') and transform_frame:
                target = DataUtils.change_frame_of_point(target, state, frame)
        elif action.action_type in [Action.MOVE_ARM]:
            frame = DataUtils.get_closest_frame(state, target)
            if (frame != '' or frame.lower() != 'table') and transform_frame:
                target = DataUtils.change_frame_of_point(target, state, frame)

        vector = []
        if combined_actions:
            vector.extend([DataUtils.create_combined_action(action.action_type, frame, action.position, state), target.x, target.y])
        else:
            vector.extend([action.action_type, DataUtils.name_to_int(frame), target.x, target.y])
        return vector

    @staticmethod
    def task_object_centric_action_vector(state, action, combined_actions=False):
        """Convert an action message into a simple feature vector in a task-relevant frame."""
        frame = action.object
        target = Point(action.position.x, action.position.y, 0)
        if action.action_type in [Action.PLACE, Action.MOVE_ARM]:
            frame = DataUtils.get_task_frame(state, target)
            if frame.lower() != 'table':
                target = DataUtils.change_frame_of_point(target, state, frame)

        vector = []
        if combined_actions:
            vector.extend([DataUtils.create_combined_action(action.action_type, frame, action.position, state), target.x, target.y])
        else:
            vector.extend([action.action_type, DataUtils.name_to_int(frame), target.x, target.y])
        return vector

    @staticmethod
    def robot_centric_action_vector(state, action, combined_actions=False):
        """Convert an action message into a simple feature vector in the gripper frame."""
        frame = 'Gripper'
        target = Point(action.position.x, action.position.y, 0)
        if action.action_type in [Action.PLACE, Action.MOVE_ARM]:
            target = DataUtils.change_frame_of_point(target, state, frame)

        vector = []
        if combined_actions:
            vector.extend([DataUtils.create_combined_action(action.action_type, frame, action.position, state), target.x, target.y])
        else:
            vector.extend([action.action_type, DataUtils.name_to_int(frame), target.x, target.y])
        return vector


if __name__ == '__main__':
    rospy.init_node('demo_reader')
    demo_reader = DemoReader()
    demo_reader.read_all()
