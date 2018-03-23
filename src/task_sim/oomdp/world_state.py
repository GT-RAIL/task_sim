#!/usr/bin/env python
# Create a state object that encapsulates the OOMDP state and the regular state.
# The API of this state is the same as that of the regular state, but it also
# updates the OOMDP when attributes are accessed.

from __future__ import print_function, division

import string

from geometry_msgs.msg import Point

from task_sim.msg import State as StateMsg, Object as ObjectMsg, SmallContainer as ContainerMsg
from task_sim.msg import OOState as OOStateMsg
from task_sim.oomdp.oo_state import OOState
from task_sim.oomdp.oomdp_classes import Box, Container, Drawer, Gripper, Item, Lid, Stack
from task_sim.oomdp.oomdp_relations import Relation, REVERSE_RELATIONS, RELATIONS

# Helper classes that should not be used outside of this file

class OOMDPPoint(object):
    """
    Intercept the incoming x,y,z modifications and propagate them to the
    _state_msg and the _oo_state sections
    """
    def __init__(self, point, oomdp_classes, oo_state):
        self.point = point
        self.oomdp_classes = oomdp_classes
        self.oo_state = oo_state

    def __eq__(self, other):
        return self.point == other

    def __hash__(self):
        return hash(self.point)

    @property
    def x(self):
        return self.point.x

    @x.setter
    def x(self, val):
        self.point.x = val
        for oomdp_class in self.oomdp_classes:
            if hasattr(oomdp_class, 'x'):
                oomdp_class.x = val
                self.oo_state.update_set.add(oomdp_class)

    @property
    def y(self):
        return self.point.y

    @y.setter
    def y(self, val):
        self.point.y = val
        for oomdp_class in self.oomdp_classes:
            if hasattr(oomdp_class, 'y'):
                oomdp_class.y = val
                self.oo_state.update_set.add(oomdp_class)

    @property
    def z(self):
        return self.point.z

    @z.setter
    def z(self, val):
        self.point.z = val
        for oomdp_class in self.oomdp_classes:
            if hasattr(oomdp_class, 'z'):
                oomdp_class.z = val
                self.oo_state.update_set.add(oomdp_class)


class OOMDPPose2D(object):
    """
    Intercept the incoming x,y,theta modifications and propagate them to the
    _state_msg and the _oo_state sections
    """
    def __init__(self, pose, oomdp_classes, oo_state):
        self.pose = pose
        self.oomdp_classes = oomdp_classes
        self.oo_state = oo_state

    def __eq__(self, other):
        return self.pose == other

    def __hash__(self):
        return hash(self.pose)

    @property
    def x(self):
        return self.pose.x

    @x.setter
    def x(self, val):
        self.pose.x = val
        for oomdp_class in self.oomdp_classes:
            if hasattr(oomdp_class, 'x'):
                oomdp_class.x = val
                self.oo_state.update_set.add(oomdp_class)

    @property
    def y(self):
        return self.pose.y

    @y.setter
    def y(self, val):
        self.pose.y = val
        for oomdp_class in self.oomdp_classes:
            if hasattr(oomdp_class, 'y'):
                oomdp_class.y = val
                self.oo_state.update_set.add(oomdp_class)

    @property
    def theta(self):
        return self.pose.theta

    @theta.setter
    def theta(self, val):
        self.pose.theta = val


class OOMDPObject(object):
    """Abstract away the setting of objects in the old state message"""
    pass

# Define the World State

class WorldState(object):
    """
    I'm tempted to just derive from the original state message, but that is
    ultimately not as extensible. So here we are reimplementing state. Joy :|
    """

    def __init__(self,
        gripper_name="gripper", box_name="box", lid_name="lid",
        drawer_name="drawer", stack_name="stack",
        *args, **kwargs
    ):
        """For initialization, pretend as if it is the state message"""
        self._state_msg = StateMsg(*args, **kwargs)
        self._oo_state = None
        self.reinit_oo_state(gripper_name, box_name, lid_name, drawer_name, stack_name)

    # First the accessors to the two states
    def get_state(self):
        return self._state_msg

    def get_oo_state(self):
        return self._oo_state

    def reinit_oo_state(
        self,
        gripper_name="gripper", box_name="box", lid_name="lid",
        drawer_name="drawer", stack_name="stack"
    ):
        self._oo_state = OOState(
            state=self._state_msg,
            gripper_name=gripper_name, box_name=box_name, lid_name=lid_name,
            drawer_name=drawer_name, stack_name=stack_name
        )

        # Save the objects to speed up lookup
        self._gripper = self._oo_state.grippers[gripper_name]
        self._drawer = self._oo_state.drawers[drawer_name]
        self._box = self._oo_state.boxes[box_name]
        self._lid = self._oo_state.lids[lid_name]
        self._stack = self._oo_state.stacks[stack_name]

        # Create the position/pose objects for the gripper, box, lid and drawer
        self._gripper_position = OOMDPPoint(
            self._state_msg.gripper_position,
            [self._gripper],
            self._oo_state
        )
        self._lid_position = OOMDPPoint(
            self._state_msg.lid_position,
            [self._lid],
            self._oo_state
        )
        self._box_position = OOMDPPoint(
            self._state_msg.box_position,
            [self._box],
            self._oo_state
        )
        self._drawer_position = OOMDPPose2D(
            self._state_msg.drawer_position,
            [self._drawer], # TODO: Should we add stack to this?
            self._oo_state
        )

    # Now the adapted properties

    @property
    def action_history(self):
        return self._state_msg.action_history

    @action_history.setter
    def action_history(self, val):
        self._state_msg.action_history = val

    @property
    def result_history(self):
        return self._state_msg.result_history

    @result_history.setter
    def result_history(self, val):
        self._state_msg.result_history = val


    @property
    def drawer_opening(self):
        return self._state_msg.drawer_opening

    @drawer_opening.setter
    def drawer_opening(self, val):
        self._state_msg.drawer_opening = val
        self._drawer._modified = True
        self._oo_state.update_set.add(self._drawer)


    @property
    def gripper_open(self):
        return self._state_msg.gripper_open

    @gripper_open.setter
    def gripper_open(self, val):
        self._state_msg.gripper_open = val
        self._gripper.closed = val
        self._oo_state.update_set.add(self._gripper)


    @property
    def object_in_gripper(self):
        return self._state_msg.object_in_gripper

    @object_in_gripper.setter
    def object_in_gripper(self, val):
        self._state_msg.object_in_gripper = val
        self._gripper.holding = val.translate(None, string.digits)
        self._oo_state.update_set.add(self._gripper)


    @property
    def gripper_position(self):
        return self._gripper_position

    @gripper_position.setter
    def gripper_position(self, val):
        self._gripper_position.x = val.x
        self._gripper_position.y = val.y
        self._gripper_position.z = val.z


    @property
    def lid_position(self):
        return self._lid_position

    @lid_position.setter
    def lid_position(self, val):
        self._lid_position.x = val.x
        self._lid_position.y = val.y
        self._lid_position.z = val.z


    @property
    def box_position(self):
        return self._box_position

    @box_position.setter
    def box_position(self, val):
        self._box_position.x = val.x
        self._box_position.y = val.y
        self._box_position.z = val.z


    @property
    def drawer_position(self):
        return self._drawer_position

    @drawer_position.setter
    def drawer_position(self, val):
        self._drawer_position.x = val.x
        self._drawer_position.y = val.y
        if hasattr(val, 'theta'):
            self._drawer_position.theta = val.theta


    # TODO: After testing, abstract away the objects and containers too
    @property
    def objects(self):
        return self._state_msg.objects

    @objects.setter
    def objects(self, val):
        self._state_msg.objects = val

    @property
    def containers(self):
        return self._state_msg.containers

    @containers.setter
    def containers(self, val):
        self._state_msg.containers = val
