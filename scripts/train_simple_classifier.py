#!/usr/bin/env python

# task_sim
from data_utils import DataUtils
from task_sim.msg import Action

# ROS
import rospy
import rospkg

# numpy
import numpy

# yaml
import yaml

# scikit-learn
from sklearn.ensemble import AdaBoostClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.externals import joblib
from sklearn.linear_model import LogisticRegression
from sklearn.neighbors import KNeighborsClassifier
from sklearn.neural_network import MLPClassifier
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.utils import shuffle


def train_classifier():
    """Train a classifier and save it as a .pkl for later use."""
    rospy.init_node('train_simple_classifiers')

    mode = rospy.get_param('~mode', 'evaluate')
    supported_modes = ['evaluate', 'train']
    if mode not in supported_modes:
        usage = 'Unsupported mode: ' + mode + '. Supported modes are:'
        for mode_string in supported_modes:
            usage += '\n\t' + mode_string
        print usage
        return

    print 'Fitting simple classifiers to ' + mode + ' models.'

    types = rospy.get_param('~classifier_types', 'decision_tree').split(',')
    supported_classifiers = ['decision_tree', 'random_forest', 'ada_boost', 'knn', 'svm', 'logistic_regression']
    normalize_classifiers = ['knn']

    if 'all' in types:
        types = supported_classifiers

    for classifier_type in types:
        if classifier_type not in supported_classifiers:
            usage = 'Unsupported classifier type: ' + classifier_type + '. Supported classifiers are:'
            for classifier_string in supported_classifiers:
                usage += '\n\t' + classifier_string
            print usage
            return

    task = rospy.get_param('~task', 'task1')
    filepath = rospy.get_param('~file_name', 'state-action_2017-11-28.yaml')
    if len(filepath) > 0 and filepath[0] != '/':
        rospack = rospkg.RosPack()
        filepath = rospack.get_path('task_sim') + '/data/' + task + '/training/' + filepath

    # Load with YAML
    stream = file(filepath, 'r')
    raw_data = yaml.load(stream)

    data_select_action = []
    label_select_action = []
    data_select_frame = []
    label_select_frame = []
    data_position = []
    label_position = []
    parse_data(raw_data,
               data_select_action, label_select_action,
               data_select_frame, label_select_frame,
               data_position, label_position)

    data_act = numpy.asarray(data_select_action)
    label_act = numpy.asarray(label_select_action)
    data_frame = numpy.asarray(data_select_frame)
    label_frame = numpy.asarray(label_select_frame)
    data_pos = numpy.asarray(data_position)
    label_pos = numpy.asarray(label_position)

    # normalize feature vectors (for use in approaches that require this)
    data_act_n = DataUtils.normalize_vector(data_act)
    data_frame_n = DataUtils.normalize_vector(data_frame)
    data_pos_n = DataUtils.normalize_vector(data_pos)

    print '\n'
    print str(data_act[0])
    print str(data_act_n[0])
    print '\n'


    print '\nImported', data_act.shape[0], 'training instances for action selection'
    print '\nImported', data_frame.shape[0], 'training instances for target frame selection'
    print '\nImported', data_pos.shape[0], 'training instances for target position regression'

    data_act, label_act = shuffle(data_act, label_act)
    data_frame, label_frame = shuffle(data_frame, label_frame)
    data_pos, label_pos = shuffle(data_pos, label_pos)

    return

    for classifier_type in types:
        print '\n----------------------------------------------------'
        print 'Training classifier: ' + classifier_type + ' for action selection.'

        classifier = prepare_classifier(classifier_type)

        print('Training on the full dataset...')
        if classifier_type in normalize_classifiers:
            classifier.fit(data_act_n, label_act)
        else:
            classifier.fit(data_act, label_act)

        joblib.dump(classifier, classifier_type + '.pkl')
        print('Saved model ' + classifier_type + '.pkl to current directory.')


def parse_data(data, data_select_action, label_select_action, data_select_frame, label_select_frame, data_position,
               label_position):
    """Extract data to train each classifier/regressor.

    data_select_action, label_select_action:
        full state vector and associated action type performed
    data_select_frame, label_select_frame:
        state vector with action type appended (excluding actions without frame parameter) and target coordinate frame
    data_position, label_position:
        state vector with action type and frame appended (excluding actions without position parameter) and target position
    """
    actions_with_frames = [Action.GRASP, Action.PLACE, Action.MOVE_ARM]
    actions_with_positions = [Action.PLACE, Action.MOVE_ARM]
    for entry in data:
        action_type = entry['action'][0]
        data_select_action.append(entry['state'])
        label_select_action.append(action_type)

        state_with_action = entry['state'][:]
        state_with_action.append(action_type)

        if action_type in actions_with_frames:
            data_select_frame.append(state_with_action)
            label_select_frame.append(entry['action'][1])

        if action_type in actions_with_positions:
            state_with_action_frame = state_with_action[:]
            state_with_action_frame.append(entry['action'][1])
            data_position.append(state_with_action_frame)
            label_position.append(entry['action'][2:])


def prepare_classifier(classifier_type):
    """Return a classifier object for any supported classifier type."""
    # Note: values below were chosen from analysis of cross validation
    if classifier_type == 'decision_tree':
        return DecisionTreeClassifier()
    elif classifier_type == 'random_forest':
        return RandomForestClassifier(n_estimators=50)
    elif classifier_type == 'ada_boost':
        return AdaBoostClassifier(n_estimators=50)
    elif classifier_type == 'knn':
        return KNeighborsClassifier(weights='distance')
    elif classifier_type == 'svm':
        return SVC(C=10, probability=True)
    elif classifier_type == 'logistic_regression':
        return LogisticRegression()


def load_data(filepath):
    """Parse training data and labels from a .csv file."""
    data = numpy.loadtxt(filepath, delimiter=',')
    x = data[:, :data.shape[1] - 1]
    y = data[:, data.shape[1] - 1]
    return x, y


if __name__ == '__main__':
    try:
        train_classifier()
    except rospy.ROSInterruptException:
        pass
