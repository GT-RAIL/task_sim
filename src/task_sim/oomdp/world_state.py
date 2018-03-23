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
    def __init__(self, point, oomdp_objs, oo_state):
        self.point = point
        self.oomdp_objs = oomdp_objs
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
        if self.point.x != val:
            self.point.x = val
            for oomdp_obj in self.oomdp_objs:
                if hasattr(oomdp_obj, 'x'):
                    oomdp_obj.x = val
                    self.oo_state.update_set.add(oomdp_obj)

    @property
    def y(self):
        return self.point.y

    @y.setter
    def y(self, val):
        if self.point.y != val:
            self.point.y = val
            for oomdp_obj in self.oomdp_objs:
                if hasattr(oomdp_obj, 'y'):
                    oomdp_obj.y = val
                    self.oo_state.update_set.add(oomdp_obj)

    @property
    def z(self):
        return self.point.z

    @z.setter
    def z(self, val):
        if self.point.z != val:
            self.point.z = val
            for oomdp_obj in self.oomdp_objs:
                if hasattr(oomdp_obj, 'z'):
                    oomdp_obj.z = val
                    self.oo_state.update_set.add(oomdp_obj)


class OOMDPPose2D(object):
    """
    Intercept the incoming x,y,theta modifications and propagate them to the
    _state_msg and the _oo_state sections
    """
    def __init__(self, pose, oomdp_objs, oo_state):
        self.pose = pose
        self.oomdp_objs = oomdp_objs
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
        if self.pose.x != val:
            self.pose.x = val
            for oomdp_obj in self.oomdp_objs:
                if hasattr(oomdp_obj, 'x'):
                    oomdp_obj.x = val
                    self.oo_state.update_set.add(oomdp_obj)

    @property
    def y(self):
        return self.pose.y

    @y.setter
    def y(self, val):
        if self.pose.y != val:
            self.pose.y = val
            for oomdp_obj in self.oomdp_objs:
                if hasattr(oomdp_obj, 'y'):
                    oomdp_obj.y = val
                    self.oo_state.update_set.add(oomdp_obj)

    @property
    def theta(self):
        return self.pose.theta

    @theta.setter
    def theta(self, val):
        self.pose.theta = val


class OOMDPObject(object):
    """Abstract away the setting of objects in the old state message"""

    def __init__(self, obj, oomdp_obj, oo_state):
        self.obj = obj
        self.oomdp_obj = oomdp_obj
        self.oo_state = oo_state

        self._obj_position = OOMDPPoint(
            obj.position,
            [oomdp_obj],
            oo_state
        )

    def __eq__(self, other):
        return self.obj == other

    def __hash__(self):
        return hash(self.obj)

    @property
    def name(self):
        return self.obj.name

    @property
    def unique_name(self):
        return self.obj.unique_name

    @property
    def position(self):
        return self._obj_position

    @position.setter
    def position(self, val):
        self._obj_position.x = val.x
        self._obj_position.y = val.y
        self._obj_position.z = val.z

    @property
    def in_drawer(self):
        return self.obj.in_drawer

    @in_drawer.setter
    def in_drawer(self, val):
        if self.obj.in_drawer != val:
            self.obj.in_drawer = val
            self.oomdp_obj._modified = True
            self.oo_state.update_set.add(self.oomdp_obj)

    @property
    def in_box(self):
        return self.obj.in_box

    @in_box.setter
    def in_box(self, val):
        if self.obj.in_box != val:
            self.obj.in_box = val
            self.oomdp_obj._modified = True
            self.oo_state.update_set.add(self.oomdp_obj)

    @property
    def on_lid(self):
        return self.obj.on_lid

    @on_lid.setter
    def on_lid(self, val):
        if self.obj.on_lid != val:
            self.obj.on_lid = val
            self.oomdp_obj._modified = True
            self.oo_state.update_set.add(self.oomdp_obj)

    @property
    def on_stack(self):
        return self.obj.on_stack

    @on_stack.setter
    def on_stack(self, val):
        if self.obj.on_stack != val:
            self.obj.on_stack = val
            self.oomdp_obj._modified = True
            self.oo_state.update_set.add(self.oomdp_obj)

    @property
    def in_gripper(self):
        return self.obj.in_gripper

    @in_gripper.setter
    def in_gripper(self, val):
        if self.obj.in_gripper != val:
            self.obj.in_gripper = val
            self.oomdp_obj._modified = True
            self.oo_state.update_set.add(self.oomdp_obj)

    @property
    def occluded(self):
        return self.obj.occluded

    @occluded.setter
    def occluded(self, val):
        if self.obj.occluded != val:
            self.obj.occluded = val
            self.oomdp_obj._modified = True
            self.oo_state.update_set.add(self.oomdp_obj)

    @property
    def lost(self):
        return self.obj.lost

    @lost.setter
    def lost(self, val):
        if self.obj.occluded != val:
            self.obj.lost = val
            self.oomdp_obj._modified = True
            self.oo_state.update_set.add(self.oomdp_obj)


class OOMDPContainer(object):
    """Abstract away the settings of the containers in the old state message"""

    def __init__(self, container, oomdp_container, oo_state):
        self.container = container
        self.oomdp_container = oomdp_container
        self.oo_state = oo_state

        self._container_position = OOMDPPoint(
            container.position,
            [oomdp_container],
            oo_state
        )

    def __eq__(self, other):
        return self.container == other

    def __hash__(self):
        return hash(self.container)

    @property
    def name(self):
        return self.container.name

    @property
    def unique_name(self):
        return self.container.unique_name

    @property
    def width(self):
        return self.container.width

    @property
    def height(self):
        return self.container.height

    @property
    def position(self):
        return self._container_position

    @position.setter
    def position(self, val):
        self._container_position.x = val.x
        self._container_position.y = val.y
        self._container_position.z = val.z

    @property
    def contains(self):
        return self.container.contains

    @contains.setter
    def contains(self, val):
        if self.container.contains != val:
            self.container.contains = val
            self.oomdp_container._modified = True
            self.oo_state.update_set.add(self.oomdp_container)

    @property
    def in_drawer(self):
        return self.container.in_drawer

    @in_drawer.setter
    def in_drawer(self, val):
        if self.container.in_drawer != val:
            self.container.in_drawer = val
            self.oomdp_container._modified = True
            self.oo_state.update_set.add(self.oomdp_container)

    @property
    def in_box(self):
        return self.container.in_box

    @in_box.setter
    def in_box(self, val):
        if self.container.in_box != val:
            self.container.in_box = val
            self.oomdp_container._modified = True
            self.oo_state.update_set.add(self.oomdp_container)

    @property
    def on_lid(self):
        return self.container.on_lid

    @on_lid.setter
    def on_lid(self, val):
        if self.container.on_lid != val:
            self.container.on_lid = val
            self.oomdp_container._modified = True
            self.oo_state.update_set.add(self.oomdp_container)

    @property
    def on_stack(self):
        return self.container.on_stack

    @on_stack.setter
    def on_stack(self, val):
        if self.container.on_stack != val:
            self.container.on_stack = val
            self.oomdp_container._modified = True
            self.oo_state.update_set.add(self.oomdp_container)

    @property
    def in_gripper(self):
        return self.container.in_gripper

    @in_gripper.setter
    def in_gripper(self, val):
        if self.container.in_gripper != val:
            self.container.in_gripper = val
            self.oomdp_container._modified = True
            self.oo_state.update_set.add(self.oomdp_container)

    @property
    def occluded(self):
        return self.container.occluded

    @occluded.setter
    def occluded(self, val):
        if self.container.occluded != val:
            self.container.occluded = val
            self.oomdp_container._modified = True
            self.oo_state.update_set.add(self.oomdp_container)

    @property
    def lost(self):
        return self.container.lost

    @lost.setter
    def lost(self, val):
        if self.container.lost != val:
            self.container.lost = val
            self.oomdp_container._modified = True
            self.oo_state.update_set.add(self.oomdp_container)


# Define the World State

class WorldState(object):
    """
    I'm tempted to just derive from the original state message, but that is
    ultimately not as extensible. So here we are reimplementing state. Joy :|
    """

    def __init__(self, init_oo_state=False,
        gripper_name="gripper", box_name="box", lid_name="lid",
        drawer_name="drawer", stack_name="stack",
        *args, **kwargs
    ):
        """For initialization, pretend as if it is the state message"""
        self._state_msg = StateMsg(*args, **kwargs)
        self._oo_state = None
        if init_oo_state:
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

        # Adapt the objects and containers
        self._objects = tuple([
            OOMDPObject(obj, self._oo_state.items[obj.unique_name], self._oo_state)
            for obj in self._state_msg.objects
        ])

        self._containers = tuple([
            OOMDPContainer(
                container, self._oo_state.containers[container.unique_name], self._oo_state
            )
            for container in self._state_msg.containers
        ])


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
        if self._state_msg.drawer_opening != val:
            self._state_msg.drawer_opening = val
            if self._oo_state is not None:
                self._drawer._modified = True
                self._oo_state.update_set.add(self._drawer)


    @property
    def gripper_open(self):
        return self._state_msg.gripper_open

    @gripper_open.setter
    def gripper_open(self, val):
        if self._state_msg.gripper_open != val:
            self._state_msg.gripper_open = val
            if self._oo_state is not None:
                self._gripper.closed = val
                self._oo_state.update_set.add(self._gripper)


    @property
    def object_in_gripper(self):
        return self._state_msg.object_in_gripper

    @object_in_gripper.setter
    def object_in_gripper(self, val):
        if self._state_msg.object_in_gripper != val:
            self._state_msg.object_in_gripper = val
            if self._oo_state is not None:
                self._gripper.holding = val.translate(None, string.digits)
                self._oo_state.update_set.add(self._gripper)


    @property
    def gripper_position(self):
        if self._oo_state is None:
            return self._state_msg.gripper_position
        else:
            return self._gripper_position

    @gripper_position.setter
    def gripper_position(self, val):
        if self._oo_state is None:
            self._state_msg.gripper_position = val
        else:
            self._gripper_position.x = val.x
            self._gripper_position.y = val.y
            self._gripper_position.z = val.z


    @property
    def lid_position(self):
        if self._oo_state is None:
            return self._state_msg.lid_position
        else:
            return self._lid_position

    @lid_position.setter
    def lid_position(self, val):
        if self._oo_state is None:
            self._state_msg.lid_position = val
        else:
            self._lid_position.x = val.x
            self._lid_position.y = val.y
            self._lid_position.z = val.z


    @property
    def box_position(self):
        if self._oo_state is None:
            return self._state_msg.box_position
        else:
            return self._box_position

    @box_position.setter
    def box_position(self, val):
        if self._oo_state is None:
            self._state_msg.box_position = val
        else:
            self._box_position.x = val.x
            self._box_position.y = val.y
            self._box_position.z = val.z


    @property
    def drawer_position(self):
        if self._oo_state is None:
            return self._state_msg.drawer_position
        else:
            return self._drawer_position

    @drawer_position.setter
    def drawer_position(self, val):
        if self._oo_state is None:
            return self._state_msg.drawer_position
        else:
            self._drawer_position.x = val.x
            self._drawer_position.y = val.y
            if hasattr(val, 'theta'):
                self._drawer_position.theta = val.theta


    # This lists are mutable until we have initialized and OOState. Once that
    # happens, then we freeze this so that relations don't go wonky
    @property
    def objects(self):
        if self._oo_state is None:
            return self._state_msg.objects
        else:
            return self._objects

    @objects.setter
    def objects(self, val):
        if self._oo_state is None:
            self._state_msg.objects = val
        else:
            raise NotImplementedError("Once relations are initialized, objects cannot be changed")

    @property
    def containers(self):
        if self._oo_state is None:
            return self._state_msg.containers
        else:
            return self._containers

    @containers.setter
    def containers(self, val):
        if self._oo_state is None:
            self._state_msg.containers = val
        else:
            raise NotImplementedError("Once relations are initialized, containers cannot be changed")
