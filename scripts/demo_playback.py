#!/usr/bin/env python
# Replays the demonstration by listening in on the bag topic and sending a
# service call on execute_action

import rospy

from task_sim.msg import Log, Action
from task_sim.srv import Execute

# Simply a pass through that sends the action messages on to the table_sim

class DemoPlayback(object):
    '''
    Class to playback the actions used in the demo. Does not consider the end
    state observed in the demo. If the state after an action differs from the
    one observed in the demo, a log message will be printed.

    Expects: /table_sim/replay_log (remap the topic in the bag file)
    Outputs: /table_sim/execute_action calls on the service
    '''

    def __init__(self):
        self._subscriber = rospy.Subscriber(
            "/table_sim/replay_log",
            Log,
            self.on_replay_log
        )
        self.execute = rospy.ServiceProxy('/table_sim/execute_action', Execute)
        self.action_to_str = {
            Action.NOOP: "NOOP",
            Action.GRASP: "GRASP",
            Action.PLACE: "PLACE",
            Action.OPEN_GRIPPER: "OPEN_GRIPPER",
            Action.CLOSE_GRIPPER: "CLOSE_GRIPPER",
            Action.MOVE_ARM: "MOVE_ARM",
            Action.RAISE_ARM: "RAISE_ARM",
            Action.LOWER_ARM: "LOWER_ARM",
            Action.RESET_ARM: "RESET_ARM",
        }

    def on_replay_log(self, msg):
        demo_new_state, action = msg.state, msg.action

        rospy.loginfo(
            "Executing action {} on object {} at coord ({}, {}, {})"
            .format(
                self.action_to_str[action.action_type],
                action.object,
                action.position.x, action.position.y, action.position.z
            )
        )
        play_new_state = self.execute(action)

        # TODO: For each aspect of the state, output if it has remained the same
        # as in the demonstration or not. Of course, this has not been
        # implemented yet

if __name__ == '__main__':
    rospy.init_node('demo_playback')
    playback_node = DemoPlayback()
    rospy.spin()
