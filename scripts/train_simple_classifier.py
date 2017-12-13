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
from sklearn.ensemble import AdaBoostClassifier, RandomForestClassifier, RandomForestRegressor
from sklearn.externals import joblib
from sklearn.linear_model import LinearRegression, LogisticRegression, Ridge, Lasso
from sklearn import metrics
from sklearn.model_selection import cross_val_score, learning_curve, train_test_split, GridSearchCV
from sklearn.neighbors import KNeighborsClassifier
from sklearn.svm import SVC, SVR
from sklearn.tree import DecisionTreeClassifier, DecisionTreeRegressor
from sklearn.utils import shuffle

# pyplot
import matplotlib.pyplot as pyplot


def train_classifier():
    """Train a classifier and save it as a .pkl for later use."""
    rospy.init_node('train_simple_classifiers')

    plot = rospy.get_param('~plot', True)

    file_suffix = rospy.get_param('~file_suffix', '')

    mode = rospy.get_param('~mode', 'evaluate')
    supported_modes = ['evaluate', 'train', 'cross-validate']
    if mode not in supported_modes:
        usage = 'Unsupported mode: ' + mode + '. Supported modes are:'
        for mode_string in supported_modes:
            usage += '\n\t' + mode_string
        print usage
        return

    print 'Fitting simple classifiers to ' + mode + ' models.'

    classifiers = rospy.get_param('~classifier_types', 'random_forest').split(',')
    supported_classifiers = ['decision_tree', 'random_forest', 'ada_boost', 'knn', 'svm', 'logistic_regression']
    normalize_classifiers = ['knn']
    if 'all' in classifiers:
        classifiers = supported_classifiers

    for classifier_type in classifiers:
        if classifier_type not in supported_classifiers:
            usage = 'Unsupported classifier type: ' + classifier_type + '. Supported classifiers are:'
            for classifier_string in supported_classifiers:
                usage += '\n\t' + classifier_string
            print usage
            return

    regressors = rospy.get_param('~regressor_types', 'ridge').split(',')
    supported_regressors = ['linear', 'lasso', 'ridge', 'decision_tree', 'random_forest']
    normalize_regressors = ['lasso', 'ridge']
    if 'all' in regressors:
        regressors = supported_regressors

    for regressor_type in regressors:
        if regressor_type not in supported_regressors:
            usage = 'Unsupported regressor type: ' + regressor_type + '. Supported regressors are:'
            for regressor_string in supported_regressors:
                usage += '\n\t' + regressor_string
            print usage
            return

    task = rospy.get_param('~task', 'task1')
    filepath = rospy.get_param('~file_name', 'state-action_global_p+s_expert_combined.yaml')
    if len(filepath) > 0 and filepath[0] != '/':
        rospack = rospkg.RosPack()
        filepath = rospack.get_path('task_sim') + '/data/' + task + '/training/' + filepath

    # Load with YAML
    stream = file(filepath, 'r')
    raw_data = yaml.load(stream)

    # # Setup for tiered classifier method (action -> frame -> regression)
    # data_select_action = []
    # label_select_action = []
    # data_select_frame = []
    # label_select_frame = []
    # data_position = []
    # label_position = []

    # data_act = numpy.asarray(data_select_action)
    # label_act = numpy.asarray(label_select_action)
    # data_frame = numpy.asarray(data_select_frame)
    # label_frame = numpy.asarray(label_select_frame)
    # data_pos = numpy.asarray(data_position)
    # label_pos = numpy.asarray(label_position)
    #
    # # normalize feature vectors (for use in approaches that require this)
    # data_act_n = DataUtils.normalize_vector(data_act)
    # data_frame_n = DataUtils.normalize_vector(data_frame)
    # data_pos_n = DataUtils.normalize_vector(data_pos)
    #
    # print '\nImported', data_act.shape[0], 'training instances for action selection'
    # print '\nImported', data_frame.shape[0], 'training instances for target frame selection'
    # print '\nImported', data_pos.shape[0], 'training instances for target position regression'
    #
    # data_act, label_act = shuffle(data_act, label_act)
    # data_frame, label_frame = shuffle(data_frame, label_frame)
    # data_pos, label_pos = shuffle(data_pos, label_pos)
    #
    # # train/test splits for feature vectors
    # split = rospy.get_param('~split', 0.4)
    # data_act_train, data_act_test, label_act_train, label_act_test = train_test_split(data_act, label_act, test_size=split)
    # data_frame_train, data_frame_test, label_frame_train, label_frame_test = train_test_split(data_frame, label_frame, test_size=split)
    # data_pos_train, data_pos_test, label_pos_train, label_pos_test = train_test_split(data_pos, label_pos, test_size=split)
    #
    # # train/test splits for normalized feature vectors
    # data_act_train_n, data_act_test_n, label_act_train_n, label_act_test_n = train_test_split(data_act_n, label_act, test_size=split)
    # data_frame_train_n, data_frame_test_n, label_frame_train_n, label_frame_test_n = train_test_split(data_frame_n, label_frame, test_size=split)
    # data_pos_train_n, data_pos_test_n, label_pos_train_n, label_pos_test_n= train_test_split(data_pos_n, label_pos, test_size=split)

    # Setup for combined action classifier method (action+frame classifier -> action-specific regressor)
    data_select_action = []
    label_select_action = []
    data_place_position = []
    label_place_position = []
    data_move_position = []
    label_move_position = []

    parse_data2(raw_data,
               data_select_action, label_select_action,
               data_place_position, label_place_position,
               data_move_position, label_move_position)

    data_act = numpy.asarray(data_select_action)
    label_act = numpy.asarray(label_select_action)
    data_place = numpy.asarray(data_place_position)
    label_place = numpy.asarray(label_place_position)
    data_move = numpy.asarray(data_move_position)
    label_move = numpy.asarray(label_move_position)

    # normalize feature vectors (for use in approaches that require this)
    data_act_n = DataUtils.normalize_vector(data_act)
    data_place_n = DataUtils.normalize_vector(data_place)
    data_move_n = DataUtils.normalize_vector(data_move)

    print '\nImported', data_act.shape[0], 'training instances for action selection'
    print '\nImported', data_place.shape[0], 'training instances for place-style action target regression'
    print '\nImported', data_move.shape[0], 'training instances for move-style action target regression'

    data_act, label_act = shuffle(data_act, label_act)
    data_place, label_place = shuffle(data_place, label_place)
    data_move, label_move = shuffle(data_move, label_move)

    # train/test splits for feature vectors
    split = rospy.get_param('~split', 0.4)
    data_act_train, data_act_test, label_act_train, label_act_test = train_test_split(data_act, label_act, test_size=split)
    data_place_train, data_place_test, label_place_train, label_place_test = train_test_split(data_place, label_place, test_size=split)
    data_move_train, data_move_test, label_move_train, label_move_test = train_test_split(data_move, label_move, test_size=split)

    # train/test splits for normalized feature vectors
    data_act_train_n, data_act_test_n, label_act_train_n, label_act_test_n = train_test_split(data_act_n, label_act, test_size=split)
    data_place_train_n, data_place_test_n, label_place_train_n, label_place_test_n = train_test_split(data_place_n, label_place, test_size=split)
    data_move_train_n, data_move_test_n, label_move_train_n, label_move_test_n= train_test_split(data_move_n, label_move, test_size=split)


    # # Training for tiered classifier method (action -> frame -> regression)
    # for classifier_type in classifiers:
    #     print '\n----------------------------------------------------'
    #     print 'Training classifier: ' + classifier_type + ' for action selection.'
    # 
    #     if mode == 'train':
    #         if classifier_type in normalize_classifiers:
    #             save_classifier(classifier_type, data_act_n, label_act, '_action')
    #         else:
    #             save_classifier(classifier_type, data_act, label_act, '_action')
    #     elif mode == 'evaluate':
    #         if classifier_type in normalize_classifiers:
    #             evaluate_classifier(classifier_type, data_act_n, label_act, data_act_train_n, data_act_test_n,
    #                                 label_act_train, label_act_test, split, plot, title_mod=' Action Selection')
    #         else:
    #             evaluate_classifier(classifier_type, data_act, label_act, data_act_train, data_act_test,
    #                                 label_act_train, label_act_test, split, plot, title_mod=' Action Selection')
    #     elif mode == 'cross-validate':
    #         if classifier_type in normalize_classifiers:
    #             cross_validate_classifier(classifier_type, data_act, label_act)
    #         else:
    #             cross_validate_classifier(classifier_type, data_act, label_act)
    # 
    # 
    # for classifier_type in classifiers:
    #     print '\n----------------------------------------------------'
    #     print 'Training classifier: ' + classifier_type + ' for target frame selection.'
    # 
    #     if mode == 'train':
    #         if classifier_type in normalize_classifiers:
    #             save_classifier(classifier_type, data_frame_n, label_frame, '_frame')
    #         else:
    #             save_classifier(classifier_type, data_frame, label_frame, '_frame')
    #     elif mode == 'evaluate':
    #         if classifier_type in normalize_classifiers:
    #             evaluate_classifier(classifier_type, data_frame_n, label_frame, data_frame_train_n, data_frame_test_n,
    #                                 label_frame_train, label_frame_test, split, plot, title_mod=' Frame Selection')
    #         else:
    #             evaluate_classifier(classifier_type, data_frame, label_frame, data_frame_train, data_frame_test,
    #                                 label_frame_train, label_frame_test, split, plot, title_mod=' Frame Selection')
    #     elif mode == 'cross-validate':
    #         if classifier_type in normalize_classifiers:
    #             cross_validate_classifier(classifier_type, data_frame, label_frame)
    #         else:
    #             cross_validate_classifier(classifier_type, data_frame, label_frame)
    # 
    # 
    # for regressor_type in regressors:
    #     print '\n----------------------------------------------------'
    #     print 'Training regressor: ' + regressor_type + ' for target position calculation.'
    # 
    #     if mode == 'train':
    #         if regressor_type in normalize_regressors:
    #             save_regressor(regressor_type, data_pos_n, label_pos, '_target')
    #         else:
    #             save_regressor(regressor_type, data_pos, label_pos, '_target')
    #     elif mode == 'evaluate':
    #         if regressor_type in normalize_regressors:
    #             evaluate_regressor(regressor_type, data_pos_n, label_pos, data_pos_train_n, data_pos_test_n,
    #                                 label_pos_train, label_pos_test, split, plot, title_mod=' Target Calculation')
    #         else:
    #             evaluate_regressor(regressor_type, data_pos, label_pos, data_pos_train, data_pos_test,
    #                                 label_pos_train, label_pos_test, split, plot, title_mod=' Target Calculation')
    #     elif mode == 'cross-validate':
    #         pass
    # 
    #         '''
    #         if regressor_type in normalize_regressors:
    #             cross_validate_regressor(regressor_type, data_pos, label_pos, plot)
    #         else:
    #             cross_validate_regressor(regressor_type, data_pos, label_pos, plot)
    #         
    
    # Training for combined action classifier method (action+frame classifier -> action-specific regressor)
    for classifier_type in classifiers:
        print '\n----------------------------------------------------'
        print 'Training classifier: ' + classifier_type + ' for action selection.'

        if mode == 'train':
            if classifier_type in normalize_classifiers:
                save_classifier(classifier_type, data_act_n, label_act, task, '_action' + file_suffix)
            else:
                save_classifier(classifier_type, data_act, label_act, task, '_action' + file_suffix)
        elif mode == 'evaluate':
            if classifier_type in normalize_classifiers:
                evaluate_classifier(classifier_type, data_act_n, label_act, data_act_train_n, data_act_test_n,
                                    label_act_train, label_act_test, split, plot, title_mod=' Action Selection')
            else:
                evaluate_classifier(classifier_type, data_act, label_act, data_act_train, data_act_test,
                                    label_act_train, label_act_test, split, plot, title_mod=' Action Selection')
        elif mode == 'cross-validate':
            if classifier_type in normalize_classifiers:
                cross_validate_classifier(classifier_type, data_act, label_act)
            else:
                cross_validate_classifier(classifier_type, data_act, label_act)

    
    for regressor_type in regressors:
        print '\n----------------------------------------------------'
        print 'Training regressor: ' + regressor_type + ' for place target calculation.'

        if mode == 'train':
            if regressor_type in normalize_regressors:
                save_regressor(regressor_type, data_place_n, label_place, task, '_place_target' + file_suffix)
            else:
                save_regressor(regressor_type, data_place, label_place, task, '_place_target' + file_suffix)
        elif mode == 'evaluate':
            if regressor_type in normalize_regressors:
                evaluate_regressor(regressor_type, data_place_n, label_place, data_place_train_n, data_place_test_n,
                                   label_place_train, label_place_test, split, plot, 
                                   title_mod=' Place Target Calculation')
            else:
                evaluate_regressor(regressor_type, data_place, label_place, data_place_train, data_place_test,
                                   label_place_train, label_place_test, split, plot, 
                                   title_mod=' Place Target Calculation')
        elif mode == 'cross-validate':
            print 'Cross-validation is not set up for regressors.'
            
    for regressor_type in regressors:
        print '\n----------------------------------------------------'
        print 'Training regressor: ' + regressor_type + ' for move target calculation.'

        if mode == 'train':
            if regressor_type in normalize_regressors:
                save_regressor(regressor_type, data_move_n, label_move, task, '_move_target' + file_suffix)
            else:
                save_regressor(regressor_type, data_move, label_move, task, '_move_target' + file_suffix)
        elif mode == 'evaluate':
            if regressor_type in normalize_regressors:
                evaluate_regressor(regressor_type, data_move_n, label_move, data_move_train_n, data_move_test_n,
                                   label_move_train, label_move_test, split, plot,
                                   title_mod=' Move Target Calculation')
            else:
                evaluate_regressor(regressor_type, data_move, label_move, data_move_train, data_move_test,
                                   label_move_train, label_move_test, split, plot, 
                                   title_mod=' Move Target Calculation')
        elif mode == 'cross-validate':
            print 'Cross-validation is not set up for regressors.'


    if plot:
        raw_input('Press [enter] to end program.  Note: this will close all plots!')


def save_classifier(classifier_type, data, labels, task, title_modifier):
    classifier = prepare_classifier(classifier_type)

    print('Training on the full dataset...')

    classifier.fit(data, labels)

    path = rospkg.RosPack().get_path('task_sim') + '/data/' + task + '/models/' + \
           classifier_type + title_modifier + '.pkl'

    joblib.dump(classifier, path)
    print('Saved model ' + classifier_type + title_modifier + '.pkl to data/' + task + '/models directory.')


def evaluate_classifier(classifier_type, data, labels, data_train, data_test, labels_train, labels_test, split, plot,
                        title_mod=''):
    classifier = prepare_classifier(classifier_type)

    print('Performing 10-fold cross validation...')
    scores = cross_val_score(classifier, data, labels, cv=10)
    print('Accuracy: %0.2f +/- %0.2f\n' % (scores.mean(), scores.std()))

    print('Detailed results on a %0.0f/%0.0f train/test split:' % ((1 - split)*100, split*100))
    classifier.fit(data_train, labels_train)
    predicted = classifier.predict(data_test)
    print(metrics.classification_report(labels_test, predicted))
    print(metrics.confusion_matrix(labels_test, predicted))

    if plot:
        print('\nGenerating learning curve plot...')
        cross_val_size = 10
        step_size = data.shape[0]//20
        train_sizes = range(step_size, data.shape[0] - data.shape[0]//cross_val_size, step_size)
        train_sizes, train_scores, valid_scores = learning_curve(prepare_classifier(classifier_type), data, labels,
                                                                 train_sizes=train_sizes, cv=cross_val_size)
        y1_mean, y1_lower, y1_upper = calculate_means_with_bounds(train_scores)
        y2_mean, y2_lower, y2_upper = calculate_means_with_bounds(valid_scores)
        pyplot.figure()
        pyplot.ion()
        pyplot.fill_between(train_sizes, y1_lower, y1_upper, color='r', alpha=0.2)
        pyplot.plot(train_sizes, y1_mean, color='r', label='Training score')
        pyplot.fill_between(train_sizes, y2_lower, y2_upper, color='b', alpha=0.2)
        pyplot.plot(train_sizes, y2_mean, color='b', label='Cross-validation score')
        pyplot.axis([0, data.shape[0], 0, 1.05])
        pyplot.title(get_classifier_string(classifier_type) + title_mod)
        pyplot.xlabel('Training examples')
        pyplot.ylabel('Score')
        pyplot.legend(loc='lower right')
        pyplot.pause(0.05)


def save_regressor(regressor_type, data, labels, task, title_mod=''):
    regressor = prepare_regressor(regressor_type)

    print('Training on the full dataset...')

    regressor.fit(data, labels)

    path = rospkg.RosPack().get_path('task_sim') + '/data/' + task + '/models/' + \
           regressor_type + title_mod + '.pkl'

    joblib.dump(regressor, path)
    print('Saved model ' + regressor_type + title_mod + '.pkl to data/' + task + '/models directory.')


def evaluate_regressor(regressor_type, data, labels, data_train, data_test, labels_train, labels_test, split, plot,
                       title_mod=''):
    regressor = prepare_regressor(regressor_type)

    print('Performing 10-fold cross validation...')
    scores = cross_val_score(regressor, data, labels, cv=10)
    print('R2 Score: %0.2f +/- %0.2f\n' % (scores.mean(), scores.std()))

    print('Detailed results on a %0.0f/%0.0f train/test split:' % ((1 - split)*100, split*100))
    regressor.fit(data_train, labels_train)
    predicted = regressor.predict(data_test)
    print('Mean squared error: ' + str(metrics.mean_squared_error(labels_test, predicted)))
    print('Explained variance score: ' + str(metrics.explained_variance_score(labels_test, predicted)))
    print('R2 Score: ' + str(metrics.r2_score(labels_test, predicted)))

    if plot:
        print('\nGenerating learning curve plot...')
        cross_val_size = 10
        step_size = data.shape[0]//20
        train_sizes = range(step_size, data.shape[0] - data.shape[0]//cross_val_size, step_size)
        train_sizes, train_scores, valid_scores = learning_curve(prepare_regressor(regressor_type), data, labels,
                                                                 train_sizes=train_sizes, cv=cross_val_size)
        y1_mean, y1_lower, y1_upper = calculate_means_with_bounds(train_scores)
        y2_mean, y2_lower, y2_upper = calculate_means_with_bounds(valid_scores)
        pyplot.figure()
        pyplot.ion()
        pyplot.fill_between(train_sizes, y1_lower, y1_upper, color='r', alpha=0.2)
        pyplot.plot(train_sizes, y1_mean, color='r', label='Training score')
        pyplot.fill_between(train_sizes, y2_lower, y2_upper, color='b', alpha=0.2)
        pyplot.plot(train_sizes, y2_mean, color='b', label='Cross-validation score')
        pyplot.axis([0, data.shape[0], 0, 1.05])
        pyplot.title(get_regressor_string(regressor_type) + title_mod)
        pyplot.xlabel('Training examples')
        pyplot.ylabel('Score')
        pyplot.legend(loc='lower right')
        pyplot.pause(0.05)


def cross_validate_classifier(classifier_type, data, labels):
    parameters = prepare_parameter_grid(classifier_type)
    classifier = GridSearchCV(prepare_classifier(classifier_type), parameters, cv=5)
    classifier.fit(data, labels)

    means = classifier.cv_results_['mean_test_score']
    stds = classifier.cv_results_['std_test_score']
    print("\nDetailed breakdown:")
    for mean, std, params in zip(means, stds, classifier.cv_results_['params']):
        print("%0.3f +/-%0.03f for %r" % (mean, std*2, params))

    print("\nBest params:")
    print(classifier.best_params_)


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


def parse_data2(data, data_select_action, label_select_action, data_place_position, label_place_position, data_move_position,
               label_move_position):
    """Extract data to train each classifier/regressor.

    data_select_action, label_select_action:
        full state vector and associated action type performed
    data_select_frame, label_select_frame:
        state vector with action type appended (excluding actions without frame parameter) and target coordinate frame
    data_position, label_position:
        state vector with action type and frame appended (excluding actions without position parameter) and target position
    """
    actions_with_place_position = [Action.PLACE]
    actions_with_move_positions = [Action.MOVE_ARM]
    for entry in data:
        action_label = entry['action'][0]
        action_type = DataUtils.get_action_from_label(action_label)
        action_modifier = DataUtils.get_action_modifier_from_label(action_label)
        data_select_action.append(entry['state'])
        label_select_action.append(action_label)

        state_with_action = entry['state'][:]
        state_with_action.extend([action_label, action_modifier])

        if action_type in actions_with_place_position:
            data_place_position.append(state_with_action)
            label_place_position.append(entry['action'][1:])

        if action_type in actions_with_move_positions:
            data_move_position.append(state_with_action)
            label_move_position.append(entry['action'][1:])


def prepare_classifier(classifier_type):
    """Return a classifier object for any supported classifier type."""
    # Note: values below were chosen from analysis of cross validation
    if classifier_type == 'decision_tree':
        return DecisionTreeClassifier(min_samples_split=24)
    elif classifier_type == 'random_forest':
        return RandomForestClassifier(n_estimators=50, max_features=None, max_depth=4)
    elif classifier_type == 'ada_boost':
        return AdaBoostClassifier(base_estimator=DecisionTreeClassifier(max_depth=4, min_samples_leaf=2, splitter='random'), n_estimators=50)
    elif classifier_type == 'knn':
        return KNeighborsClassifier(weights='distance')
    elif classifier_type == 'svm':
        return SVC(kernel='linear', C=1, probability=True)  # Kernels: linear, poly seem to work well
    elif classifier_type == 'logistic_regression':
        return LogisticRegression()


def prepare_regressor(regressor_type):
    """Return a classifier object for any supported classifier type."""
    # Note: values below were chosen from analysis of cross validation
    if regressor_type == 'linear':
        return LinearRegression()
    elif regressor_type == 'lasso':
        return Lasso()
    elif regressor_type == 'ridge':
        return Ridge()
    elif regressor_type == 'decision_tree':
        return DecisionTreeRegressor()
    elif regressor_type == 'random_forest':
        return RandomForestRegressor(n_estimators=50, max_features=None, max_depth=4)


def calculate_means_with_bounds(score_list):
    """Calculate mean, lower bound (-1 stdev), and upper bound (+1 stdev) for a list of classification scores."""
    s_mean = []
    s_lower = []
    s_upper = []
    for scores in score_list:
        mean = numpy.mean(scores)
        stdev = numpy.std(scores)
        s_mean.append(mean)
        s_lower.append(mean - stdev)
        s_upper.append(mean + stdev)
    return s_mean, s_lower, s_upper


def prepare_parameter_grid(classifier_type):
    """Return a parameter grid for cross-validation of any supported classifier type."""
    # TODO(enhancement): grid parameters are hardcoded for each classifier
    if classifier_type == 'decision_tree':
        return [{'min_samples_leaf': [1, 2, 4, 8],
                 'min_samples_split': [2, 4, 8, 12, 16, 20, 24],
                 'max_depth': [1, 2, 4, 8, 16, 32, None]}]
    elif classifier_type == 'random_forest':
        return [{'n_estimators': [10, 20, 50, 100], 'max_features': [None], 'max_depth': [1, 2, 4, 8, None],
                 'min_samples_split': [2, 4, 8, 16]}]
    elif classifier_type == 'ada_boost':
        return [{'n_estimators': [25, 50, 100, 200], 'learning_rate': [.01, .1, 1, 10, 100]}]
    elif classifier_type == 'knn':
        return [{'n_neighbors': [1, 2, 5, 10, 20, 50], 'weights': ['uniform', 'distance']}]
    elif classifier_type == 'svm':
        return [{'C': [0.1, 1.0, 10.0], 'kernel': ['linear', 'rbf']}]
    elif classifier_type == 'logistic_regression':
        return [{'penalty': ['l1', 'l2'], 'C': [.01, 0.1, 1.0, 10.0], 'solver': ['liblinear']},
                {'penalty': ['l2'], 'C': [.01, 0.1, 1.0, 10.0], 'solver': ['lbfgs']}]


def get_classifier_string(classifier_type):
    """Convert the classifier_type parameter into a more human-readable string."""
    if classifier_type == 'decision_tree':
        return 'Decision Tree'
    elif classifier_type == 'random_forest':
        return 'Random Forest'
    elif classifier_type == 'ada_boost':
        return 'AdaBoost'
    elif classifier_type == 'knn':
        return 'KNN'
    elif classifier_type == 'svm':
        return 'SVM'
    elif classifier_type == 'logistic_regression':
        return 'Logistic Regression'


def get_regressor_string(regressor_type):
    """Convert the classifier_type parameter into a more human-readable string."""
    if regressor_type == 'linear':
        return 'Ordinary Least Squares'
    elif regressor_type == 'lasso':
        return 'Lasso'
    elif regressor_type == 'ridge':
        return 'Ridge'
    elif regressor_type == 'decision_tree':
        return 'Decision Tree'
    elif regressor_type == 'random_forest':
        return 'Random Forest'


if __name__ == '__main__':
    try:
        train_classifier()
    except rospy.ROSInterruptException:
        pass
