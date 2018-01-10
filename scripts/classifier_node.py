#!/usr/bin/env python

# Python
import copy
from math import floor, sqrt
from random import random, randint

# ROS
from task_sim.msg import Action, Status
from task_sim.srv import QueryStatus, SelectAction
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
        self.history_buffer = rospy.get_param('~history_buffer', 0)
        self.stochastic = rospy.get_param('~stochastic', False)
        self.semantic_place = rospy.get_param('~semantic_place', False)
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

        self.state_history = []

        self.service = rospy.Service('/table_sim/select_action', SelectAction, self.classify)
        self.status_service = rospy.Service('/table_sim/query_status', QueryStatus, self.query_status)

        print 'All models loaded.'

    def cleanup_path(self, path):
        if len(path) > 0 and path[0] != '/':
            path = rospkg.RosPack().get_path('task_sim') + '/data/' + self.task + '/models/' + path
        return path

    def classify(self, req):
        """Return binary classification of an ordered grasp pair feature vector."""

        action = Action()

        # Convert state to feature vector
        features = DataUtils.naive_state_vector(req.state, True, True, history_buffer=self.history_buffer)

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
            if self.semantic_place:
                print 'Placing in ' + DataUtils.int_to_name(action_modifier)
                if DataUtils.int_to_name(action_modifier) == 'Stack':
                    # Pick a random free point on top of the stack of drawers
                    points = []
                    if req.state.drawer_position.theta == 0 or req.state.drawer_position.theta == 180:
                        for x in range(int(req.state.drawer_position.x - 3), int(req.state.drawer_position.x + 4)):
                            for y in range(int(req.state.drawer_position.y - 2), int(req.state.drawer_position.y + 3)):
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z == 3:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 3))
                    else:
                        for x in range(int(req.state.drawer_position.x - 2), int(req.state.drawer_position.x + 3)):
                            for y in range(int(req.state.drawer_position.y - 3), int(req.state.drawer_position.y + 4)):
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z == 3:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 3))
                    if len(points) > 0:
                        action.position = points[randint(0, len(points) - 1)]
                    else:  # Regress parameters for table or unexpected place surfaces
                        target = self.place_model.predict(np.asarray(features).reshape(1, -1))
                        # Convert coordinates to global frame
                        action.position = DataUtils.get_point_in_global_frame(req.state,
                            Point(int(floor(target[0][0] + .5)), int(floor(target[0][1] + .5)), 0), DataUtils.int_to_name(action_modifier))
                elif DataUtils.int_to_name(action_modifier) == 'Drawer':
                    # Pick a random free point in the drawer that's also not in the drawer stack footprint
                    points = []
                    if req.state.drawer_position.theta == 0:
                        for x in range(int(req.state.drawer_position.x + 4), int(req.state.drawer_position.x + req.state.drawer_opening + 3)):
                            for y in range(int(req.state.drawer_position.y - 1), int(req.state.drawer_position.y + 2)):
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z > 0:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 2))
                    elif req.state.drawer_position.theta == 180:
                        for x in range(int(req.state.drawer_position.x - req.state.drawer_opening - 2), int(req.state.drawer_position.x - 3)):
                            for y in range(int(req.state.drawer_position.y - 1), int(req.state.drawer_position.y + 2)):
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z > 0:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 2))
                    elif req.state.drawer_position.theta == 90:
                        for x in range(int(req.state.drawer_position.x - 1), int(req.state.drawer_position.x + 2)):
                            for y in range(int(req.state.drawer_position.y + 4), int(req.state.drawer_position.y + req.state.drawer_opening + 3)):
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z > 0:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 2))
                    else:
                        for x in range(int(req.state.drawer_position.x - 1), int(req.state.drawer_position.x + 2)):
                            for y in range(int(req.state.drawer_position.y - req.state.drawer_opening - 2), int(req.state.drawer_position.y - 3)):
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z > 0:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 2))
                    if len(points) > 0:
                        action.position = points[randint(0, len(points) - 1)]
                    else:  # Regress parameters for table or unexpected place surfaces
                        target = self.place_model.predict(np.asarray(features).reshape(1, -1))
                        # Convert coordinates to global frame
                        action.position = DataUtils.get_point_in_global_frame(req.state,
                            Point(int(floor(target[0][0] + .5)), int(floor(target[0][1] + .5)), 0), DataUtils.int_to_name(action_modifier))
                elif DataUtils.int_to_name(action_modifier) == 'Box':
                    # Special case: holding lid
                    if req.state.object_in_gripper.lower() == 'lid':
                        action.position = req.state.box_position
                    else:
                        # Pick a random free point in the box that's also not in the lid footprint
                        points = []
                        for x in range(int(req.state.box_position.x - 1), int(req.state.box_position.x + 2)):
                            for y in range(int(req.state.box_position.y - 1), int(req.state.box_position.y + 2)):
                                if (x >= req.state.lid_position.x - 2 and x <= req.state.lid_position.x + 2
                                    and y >= req.state.lid_position.y - 2 and y <= req.state.lid_position.y + 2):
                                    continue
                                clear = True
                                for obj in req.state.objects:
                                    if obj.position.x == x and obj.position.y == y and obj.position.z <= 1:
                                        clear = False
                                        break
                                if clear:
                                    points.append(Point(x, y, 2))
                        if len(points) > 0:
                            action.position = points[randint(0, len(points) - 1)]
                        else:  # Regress parameters for table or unexpected place surfaces
                            target = self.place_model.predict(np.asarray(features).reshape(1, -1))
                            # Convert coordinates to global frame
                            action.position = DataUtils.get_point_in_global_frame(req.state,
                                Point(int(floor(target[0][0] + .5)), int(floor(target[0][1] + .5)), 0), DataUtils.int_to_name(action_modifier))
                elif DataUtils.int_to_name(action_modifier) == 'Lid':
                    # Pick a random free point on the lid
                    points = []
                    for x in range(int(req.state.lid_position.x - 2), int(req.state.lid_position.x + 3)):
                        for y in range(int(req.state.lid_position.y - 2), int(req.state.lid_position.y + 3)):
                            clear = True
                            for obj in req.state.objects:
                                if obj.position.x == x and obj.position.y == y and obj.position.z == req.state.lid_position.z:
                                    clear = False
                                    break
                            if clear:
                                points.append(Point(x, y, 2))
                    if len(points) > 0:
                        action.position = points[randint(0, len(points) - 1)]
                    else:  # Regress parameters for table or unexpected place surfaces
                        target = self.place_model.predict(np.asarray(features).reshape(1, -1))
                        # Convert coordinates to global frame
                        action.position = DataUtils.get_point_in_global_frame(req.state,
                            Point(int(floor(target[0][0] + .5)), int(floor(target[0][1] + .5)), 0), DataUtils.int_to_name(action_modifier))
                else:  # Regress parameters for table or unexpected place surfaces
                    target = self.place_model.predict(np.asarray(features).reshape(1, -1))
                    # Convert coordinates to global frame
                    action.position = DataUtils.get_point_in_global_frame(req.state,
                        Point(int(floor(target[0][0] + .5)), int(floor(target[0][1] + .5)), 0), DataUtils.int_to_name(action_modifier))
            else:
                target = self.place_model.predict(np.asarray(features).reshape(1, -1))
                # Convert coordinates to global frame
                action.position = DataUtils.get_point_in_global_frame(req.state,
                    Point(int(floor(target[0][0] + .5)), int(floor(target[0][1] + .5)), 0), DataUtils.int_to_name(action_modifier))

            print 'Place position: ' + str(action.position)

        if action_type in [Action.MOVE_ARM]:
            target = self.move_model.predict(np.asarray(features).reshape(1, -1))
            # Convert coordinates to global frame
            action.position = DataUtils.get_point_in_global_frame(req.state, Point(int(floor(target[0][0] + .5)), int(floor(target[0][1] + .5)), 0), 'Gripper')

        return action


    def query_status(self, req):
        # Check termination criteria
        completed = True
        failed = False
        status = Status()
        status.status_code = Status.IN_PROGRESS
        for object in req.state.objects:
            if object.name.lower() == 'apple':
                dst = sqrt(pow(20 - object.position.x, 2) + pow(1 - object.position.y, 2))
                if object.lost or dst <= 3 or dst >= 20:
                    failed = True
                    completed = False
                    break
                if not object.in_box:
                    completed = False
            elif object.name.lower() == 'flashlight':
                dst = sqrt(pow(20 - object.position.x, 2) + pow(1 - object.position.y, 2))
                if object.lost or dst <= 3 or dst >= 20:
                    failed = True
                    completed = False
                    break
                if not object.in_drawer:
                    completed = False
            elif object.name.lower() == 'batteries':
                dst = sqrt(pow(20 - object.position.x, 2) + pow(1 - object.position.y, 2))
                if object.lost or dst <= 3 or dst >= 20:
                    failed = True
                    completed = False
                    break
                if not object.in_drawer:
                    completed = False
        if req.state.drawer_opening > 0:
            completed = False
        if req.state.lid_position.x != req.state.box_position.x or req.state.lid_position.y != req.state.box_position.y:
            completed = False

        if failed:
            status.status_code = Status.FAILED
            return status
        if completed:
            status.status_code = Status.COMPLETED
            return status

        # Check if intervention is required (state repeated 5 times in last 50 actions)
        self.state_history.append(copy.deepcopy(req.state))
        self.state_history = self.state_history[-50:]
        repeat = 0
        for state in self.state_history:
            if state == req.state:
                repeat += 1
        if repeat >= 5:
            status.status_code = Status.INTERVENTION_REQUESTED
            # TODO: Clear history for future loop detection?

        return status


if __name__ == '__main__':
    rospy.init_node('classifier_node')

    classifier_node = ClassifierNode()
    print 'Ready to classify.'

    rospy.spin()
