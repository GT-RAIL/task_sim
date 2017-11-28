#!/usr/bin/env python

# task_sim
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

    types = rospy.get_param('~classifier_types', 'decision_tree').split(',')
    supported_classifiers = ['decision_tree', 'random_forest', 'ada_boost', 'knn', 'svm', 'logistic_regression']

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
    filepath = rospy.get_param('~file_name', 'state-action.yaml')
    if len(filepath) > 0 and filepath[0] != '/':
        rospack = rospkg.RosPack()
        filepath = rospack.get_path('task_sim') + '/data/' + task + 'training' + filepath

    # Load with YAML
    stream = file(filepath, 'r')
    raw_data = yaml.load(stream)

    print '\nImported', data.shape[0], 'training instances'

    data, label = shuffle(data, label)

    for classifier_type in types:
        print '\n----------------------------------------------------'
        print 'Training classifier: ' + classifier_type

        classifier = prepare_classifier(classifier_type)

        print('Training on the full dataset...')
        if classifier_type in normalize_classifiers:
            classifier.fit(data_normalized, label)
        else:
            classifier.fit(data, label)

        joblib.dump(classifier, classifier_type + '.pkl')
        print('Saved model ' + classifier_type + '.pkl to current directory.')


def parse_data(data, data_select_action, label_select_action, data_select_frame, label_select_frame, data_position, label_position):
    """Extract data to train each classifier/regressor.

    data_select_action, label_select_action:
        full state vector and associated action type performed
    data_select_frame, label_select_frame:
        state vector with action type appended (excluding actions without frame parameter) and target coordinate frame
    data_position, label_position:
        state vector with action type and frame appended (excluding actions without position parameter) and target position
    """
    actions_with_frames = [Action.Grasp, Action.PLACE, Action.MOVE_ARM]
    actions_with_positions = [Action.PLACE, Action.MOVE_ARM]
    for entry in data:
        action_type = entry.action[0]
        data_select_action.append(entry.state)
        label_select_action.append(action_type)

        state_with_action = entry.state[:].append(action_type)

        if action_type in actions_with_frames:
            data_select_frame.append(state_with_action)
            label_select_frame.append(entry.action[1])

        if action_type in actions_with_positions:
            data_position.append(state_with_action[:].append(entry.action[1]))
            label_position.append(entry.action[2:])


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
    elif classifier_type == 'nn1':
        return MLPClassifier(learning_rate='adaptive', max_iter=500)
    elif classifier_type == 'nn2':
        return MLPClassifier(learning_rate='adaptive', hidden_layer_sizes=(100, 50), max_iter=1000)


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
