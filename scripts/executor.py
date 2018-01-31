#!/usr/bin/env python

# Numpy
from numpy import mean, std

# ROS
import rospy
from std_srvs.srv import Empty
from task_sim.msg import Action, State, Status
from task_sim.srv import Execute, QueryState, QueryStatus, RequestIntervention, SelectAction

class Executor:

    def __init__(self):
        self.allow_interventions = rospy.get_param('~allow_interventions', True)
        self.trials = rospy.get_param('~trials', 1)
        self.trial = 0
        self.successes = 0
        self.failures = 0
        self.timeouts = 0

        self.last_action = Action()
        self.last_action.action_type = Action.NOOP

        if self.trials > 1:
            self.interventions = []
            self.action_counts = []
            self.temp_action_counts = [0, 0, 0, 0, 0, 0, 0, 0]
            self.successful_action_counts = []
            self.failed_action_counts = []
            for i in range(self.trials):
                self.interventions.append(0)
                self.action_counts.append([0, 0, 0, 0, 0, 0, 0, 0])
        else:
            self.interventions = 0
            self.action_counts = [0, 0, 0, 0, 0, 0, 0, 0]
        self.step_count = 0

        self.query_state = rospy.ServiceProxy('table_sim/query_state', QueryState)
        self.query_status = rospy.ServiceProxy('table_sim/query_status', QueryStatus)
        self.select_action = rospy.ServiceProxy('table_sim/select_action', SelectAction)
        self.execute = rospy.ServiceProxy('table_sim/execute_action', Execute)
        self.request_intervention = rospy.ServiceProxy('table_sim/request_intervention', RequestIntervention)
        self.reset = rospy.ServiceProxy('table_sim/reset_simulation', Empty)

        print 'Starting trial 1...'

    def step(self):
        if self.step_count > 1000:
            return Status.TIMEOUT

        state = self.query_state().state
        action = self.select_action(state, self.last_action).action
        self.last_action = action
        if self.trials == 1:
            if action.action_type != Action.NOOP:
                self.action_counts[action.action_type] += 1
        else:
            if action.action_type != Action.NOOP:
                self.action_counts[self.trial][action.action_type] += 1
                self.temp_action_counts[action.action_type] += 1
        self.step_count += 1
        result_state = self.execute(action)
        status_code = self.query_status(result_state.state).status.status_code

        if status_code == Status.INTERVENTION_REQUESTED:
            if self.allow_interventions:
                if self.trials == 1:
                    self.interventions += 1
                else:
                    self.interventions[self.trial] += 1
                actions = self.request_intervention().actions
                for action in actions:
                    if action.action_type != Action.NOOP:
                        if self.trials == 1:
                            self.action_counts[action.action_type] += 1
                        else:
                            self.action_counts[self.trial][action.action_type] += 1
                            self.temp_action_counts[action.action_type] += 1
                        self.step_count += 1
                        self.last_action = action
            else:
                status_code = Status.IN_PROGRESS

        return status_code

    def init_trial(self):
        self.temp_action_counts = [0, 0, 0, 0, 0, 0, 0, 0]
        self.step_count = 0
        self.trial += 1
        self.reset()
        if self.trial < self.trials:
            print 'Starting trial ' + str(self.trial + 1) + '...'


if __name__ == '__main__':
    rospy.init_node('executor')

    executor = Executor()

    loop_rate = rospy.Rate(100)

    if executor.trials == 1:
        while not rospy.is_shutdown():
            loop_rate.sleep()
            status_code = executor.step()
            if status_code == Status.FAILED or status_code == Status.COMPLETED or status_code == Status.TIMEOUT:
                break

        print '\n\n--------------------------------------------------------------------'
        print 'Execution complete.  Results:'
        if status_code == Status.FAILED:
            print '\n\tTask failed.'
        elif status_code == Status.TIMEOUT:
            print '\n\tTask failed (Timeout).'
        else:
            print '\n\tTask succeeded.'
        print '\n\tNumber of interventions: ' + str(executor.interventions)
        print '\n\tAction counts:'
        print '\t\t' + str(executor.action_counts[0]) + '\t- grasp'
        print '\t\t' + str(executor.action_counts[1]) + '\t- place'
        print '\t\t' + str(executor.action_counts[2]) + '\t- open gripper'
        print '\t\t' + str(executor.action_counts[3]) + '\t- close gripper'
        print '\t\t' + str(executor.action_counts[4]) + '\t- move arm'
        print '\t\t' + str(executor.action_counts[5]) + '\t- raise arm'
        print '\t\t' + str(executor.action_counts[6]) + '\t- lower arm'
        print '\t\t' + str(executor.action_counts[7]) + '\t- reset arm'
    else:
        for i in range(executor.trials):
            while not rospy.is_shutdown():
                loop_rate.sleep()
                status_code = executor.step()
                if status_code == Status.FAILED or status_code == Status.COMPLETED or status_code == Status.TIMEOUT:
                    break

            print 'Completed trial' + str(executor.trial + 1) + ':'
            if status_code == Status.FAILED:
                print '\tTask failed.'
                executor.failures += 1
                executor.failed_action_counts.append(executor.temp_action_counts)
            elif status_code == Status.TIMEOUT:
                print '\tTask failed (Timeout).'
                executor.timeouts += 1
            else:
                print '\tTask succeeded.'
                executor.successes += 1
                executor.successful_action_counts.append(executor.temp_action_counts)
            if i != executor.trials - 1:
                executor.init_trial()

        print '\n\n--------------------------------------------------------------------'
        print 'Execution complete.  Results:'

        print '\n\t Success rate: ' + str(executor.successes / float(executor.successes + executor.failures + executor.timeouts))
        print '\t Timeout rate: ' + str(executor.timeouts / float(executor.successes + executor.failures + executor.timeouts))

        if executor.allow_interventions:
            print '\n\tNumber of interventions: ' + str(mean(executor.interventions)) + ' +/- ' \
                  + str(std(executor.interventions))
        print '\n\tAction counts (total):'
        action_means = mean(executor.action_counts, axis=0)
        action_stds = std(executor.action_counts, axis=0)
        print '\t\t' + str(action_means[0]) + ' +/- ' + str(action_stds[0]) + '\t- grasp'
        print '\t\t' + str(action_means[1]) + ' +/- ' + str(action_stds[1]) + '\t- place'
        print '\t\t' + str(action_means[2]) + ' +/- ' + str(action_stds[2]) + '\t- open gripper'
        print '\t\t' + str(action_means[3]) + ' +/- ' + str(action_stds[3]) + '\t- close gripper'
        print '\t\t' + str(action_means[4]) + ' +/- ' + str(action_stds[4]) + '\t- move arm'
        print '\t\t' + str(action_means[5]) + ' +/- ' + str(action_stds[5]) + '\t- raise arm'
        print '\t\t' + str(action_means[6]) + ' +/- ' + str(action_stds[6]) + '\t- lower arm'
        print '\t\t' + str(action_means[7]) + ' +/- ' + str(action_stds[7]) + '\t- reset arm'

        if len(executor.successful_action_counts) > 0:
            print '\n\tAction counts (successful executions, n=' + str(executor.successes) + '):'
            action_means = mean(executor.successful_action_counts, axis=0)
            if len(executor.successful_action_counts) > 1:
                action_stds = std(executor.successful_action_counts, axis=0)
            else:
                action_stds = [0, 0, 0, 0, 0, 0, 0, 0]
            print '\t\t' + str(action_means[0]) + ' +/- ' + str(action_stds[0]) + '\t- grasp'
            print '\t\t' + str(action_means[1]) + ' +/- ' + str(action_stds[1]) + '\t- place'
            print '\t\t' + str(action_means[2]) + ' +/- ' + str(action_stds[2]) + '\t- open gripper'
            print '\t\t' + str(action_means[3]) + ' +/- ' + str(action_stds[3]) + '\t- close gripper'
            print '\t\t' + str(action_means[4]) + ' +/- ' + str(action_stds[4]) + '\t- move arm'
            print '\t\t' + str(action_means[5]) + ' +/- ' + str(action_stds[5]) + '\t- raise arm'
            print '\t\t' + str(action_means[6]) + ' +/- ' + str(action_stds[6]) + '\t- lower arm'
            print '\t\t' + str(action_means[7]) + ' +/- ' + str(action_stds[7]) + '\t- reset arm'

        if len(executor.failed_action_counts) > 0:
            print '\n\tAction counts (failed executions, n=' + str(executor.failures) + '):'
            action_means = mean(executor.failed_action_counts, axis=0)
            if len(executor.failed_action_counts) > 1:
                action_stds = std(executor.failed_action_counts, axis=0)
            else:
                action_stds = [0, 0, 0, 0, 0, 0, 0, 0]
            print '\t\t' + str(action_means[0]) + ' +/- ' + str(action_stds[0]) + '\t- grasp'
            print '\t\t' + str(action_means[1]) + ' +/- ' + str(action_stds[1]) + '\t- place'
            print '\t\t' + str(action_means[2]) + ' +/- ' + str(action_stds[2]) + '\t- open gripper'
            print '\t\t' + str(action_means[3]) + ' +/- ' + str(action_stds[3]) + '\t- close gripper'
            print '\t\t' + str(action_means[4]) + ' +/- ' + str(action_stds[4]) + '\t- move arm'
            print '\t\t' + str(action_means[5]) + ' +/- ' + str(action_stds[5]) + '\t- raise arm'
            print '\t\t' + str(action_means[6]) + ' +/- ' + str(action_stds[6]) + '\t- lower arm'
            print '\t\t' + str(action_means[7]) + ' +/- ' + str(action_stds[7]) + '\t- reset arm'