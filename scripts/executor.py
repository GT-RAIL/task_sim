#!/usr/bin/env python

# ROS
from task_sim.msg import Action, State, Status
from task_sim.srv import Execute, QueryState, QueryStatus, RequestIntervention, SelectAction
import rospy

class Executor:

    def __init__(self):
        self.interventions = 0
        self.action_counts = [0, 0, 0, 0, 0, 0, 0, 0]

        self.query_state = rospy.ServiceProxy('table_sim/query_state', QueryState)
        self.query_status = rospy.ServiceProxy('table_sim/query_status', QueryStatus)
        self.select_action = rospy.ServiceProxy('table_sim/select_action', SelectAction)
        self.execute = rospy.ServiceProxy('table_sim/execute_action', Execute)
        self.request_intervention = rospy.ServiceProxy('table_sim/request_intervention', RequestIntervention)

    def step(self):
        state = self.query_state().state
        action = self.select_action(state).action
        print str(action)
        self.action_counts[action.action_type] += 1
        result_state = self.execute(action)
        status_code = self.query_status(result_state.state).status.status_code

        if status_code == Status.FAILED:
            print 'Task failed.'
        elif status_code == Status.COMPLETED:
            print 'Task completed.'
        elif status_code == Status.INTERVENTION_REQUESTED:
            print 'Requesting intervention...'
            self.interventions += 1
            actions = self.request_intervention().actions
            for action in actions:
                if action.action_type != Action.NOOP:
                    self.action_counts[action.action_type] += 1

        return status_code



if __name__ == '__main__':
    rospy.init_node('executor')

    executor = Executor()

    loop_rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        loop_rate.sleep()
        status_code = executor.step()
        if status_code == Status.FAILED or status_code == Status.COMPLETED:
            break

    print '\n\n--------------------------------------------------------------------'
    print 'Execution complete.  Results:'
    if status_code == Status.FAILED:
        print '\n\tTask failed.'
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