#!/usr/bin/env python

# Python
from math import floor
from random import random

# ROS
from task_sim.msg import Action
from task_sim.srv import SelectAction
from geometry_msgs.msg import Point
import numpy as np
import rospy
import rospkg

from data_utils import DataUtils

# scikit-learn
from sklearn.externals import joblib

class ClassifierNode:

    def __init__(self):
        """Initialize action classification pipeline as a service in a ROS node."""
        self.stochastic = rospy.get_param('~stochastic', False)

        self.task = rospy.get_param('~task', 'task1')

        classifier_path = self.cleanup_path(rospy.get_param('~classifier_name',
                                                            'random_forest_action_global_p+s_expert_combined.pkl'))
        place_regressor_path = self.cleanup_path(rospy.get_param('~classifier_name',
                                                                 'random_forest_place_target_global_p+s_expert_combined.pkl'))
        move_regressor_path = self.cleanup_path(rospy.get_param('~classifier_name',
                                                                'random_forest_move_target_global_p+s_expert_combined.pkl'))

        print classifier_path
        self.action_model = joblib.load(classifier_path)
        self.place_model = joblib.load(place_regressor_path)
        self.move_model = joblib.load(move_regressor_path)

        jobs = rospy.get_param('~n_jobs', 1)
        if 'n_jobs' in self.action_model.get_params().keys():
            self.action_model.set_params(n_jobs=jobs)
        if 'n_jobs' in self.place_model.get_params().keys():
            self.place_model.set_params(n_jobs=jobs)
        if 'n_jobs' in self.move_model.get_params().keys():
            self.move_model.set_params(n_jobs=jobs)

        self.service = rospy.Service('/table_sim/select_action', SelectAction, self.classify)

        print 'All models loaded.'

    def cleanup_path(self, path):
        if len(path) > 0 and path[0] != '/':
            path = rospkg.RosPack().get_path('task_sim') + '/data/' + self.task + '/models/' + path
        return path

    def classify(self, req):
        """Return binary classification of an ordered grasp pair feature vector."""

        action = Action()

        # Convert state to feature vector
        features = DataUtils.naive_state_vector(req.state, True, True)

        # Classify action
        if self.stochastic:
            probs = self.action_model.predict_proba(np.asarray(features).reshape(1, -1)).flatten().tolist()
            selection = random()
            cprob = 0
            action_label = 0
            for i in range(1, len(probs)):
                cprob += probs[i]
                if cprob >= selection:
                    action_label = self.action_model.classes_[i]
                    break
        else:
            action_label = self.action_model.predict(np.asarray(features).reshape(1, -1))
        action_type = DataUtils.get_action_from_label(action_label)
        action_modifier = DataUtils.get_action_modifier_from_label(action_label)
        action.action_type = action_type
        if action_type in [Action.GRASP]:
            action.object = DataUtils.int_to_name(action_modifier)

        # Augment state with action
        features.extend([action_type, action_modifier])

        # Regress parameters where necessary
        if action_type in [Action.PLACE]:
            target = self.place_model.predict(np.asarray(features).reshape(1, -1))
            # Convert coordinates to global frame
            action.position = DataUtils.get_point_in_global_frame(req.state, Point(int(floor(target[0][0] + .5)), int(floor(target[0][1] + .5)), 0),
                                                DataUtils.int_to_name(action_modifier))

        if action_type in [Action.MOVE_ARM]:
            target = self.move_model.predict(np.asarray(features).reshape(1, -1))
            # Convert coordinates to global frame
            action.position = DataUtils.get_point_in_global_frame(req.state, Point(int(floor(target[0][0] + .5)), int(floor(target[0][1] + .5)), 0), 'Gripper')

        return action


if __name__ == '__main__':
    rospy.init_node('classifier_node')

    classifier_node = ClassifierNode()
    print 'Ready to classify.'

    rospy.spin()
