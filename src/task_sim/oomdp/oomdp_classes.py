#!/usr/bin/env python

from task_sim.msg import Box as BoxMsg
from task_sim.msg import Container as ContainerMsg
from task_sim.msg import Drawer as DrawerMsg
from task_sim.msg import Gripper as GripperMsg
from task_sim.msg import Item as ItemMsg
from task_sim.msg import Lid as LidMsg
from task_sim.msg import Stack as StackMsg

# TODO:
"""
Relations:
    Gripper occlusion

Occlusion?
Edges?
"""


class Item(object):

    def __init__(self, x=0, y=0, z=0, name='', unique_name='', msg=None):
        self.msg = msg or ItemMsg(x=x, y=y, z=z, name=name, unique_name=unique_name)
        self.relations = set()

        # Private variable to keep track of if relations need to be updated
        self._modified = False

    def to_ros(self):
        return self.msg

    def from_ros(self, msg):
        self.msg = msg
        return self

    def relations_updated(self):
        """Mark the relations as having been updated"""
        self._modified = False

    @property
    def x(self):
        return int(self.msg.x)

    @x.setter
    def x(self, value):
        self._modified = True
        self.msg.x = value

    @property
    def y(self):
        return int(self.msg.y)

    @y.setter
    def y(self, value):
        self._modified = True
        self.msg.y = value

    @property
    def z(self):
        return int(self.msg.z)

    @z.setter
    def z(self, value):
        self._modified = True
        self.msg.z = value

    @property
    def name(self):
        return self.msg.name

    @property
    def unique_name(self):
        return self.msg.unique_name

    # Relations
    def atop(self, obj):
        """True if an object is resting on top of another object

        Valid classes: Stack, Lid
        """
        if obj.__class__ == Stack:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z = 3
        elif obj.__class__ == Lid:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z = obj.z
        else:
            return False

        return x_min <= self.x <= x_max and y_min <= self.y <= y_max and self.z == z

    def inside(self, obj):
        """True if an object is contained within another object

        Valid classes: Box, Container, Drawer
        """
        if obj.__class__ == Box:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z_min = 0
            z_max = 1
        elif obj.__class__ == Container:
            x_min = obj.x
            x_max = obj.x + obj.width - 1
            y_min = obj.y
            y_max = obj.y + obj.depth - 1
            z_min = obj.z
            z_max = obj.z
        elif obj.__class__ == Drawer:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z_min = 2
            z_max = 2
        else:
            return False

        return x_min <= self.x <= x_max and y_min <= self.y <= y_max and z_min <= self.z <= z_max

    def touching(self, obj):
        """True if an object is touching (x-y plane, 'atop' covers z) another object

        Valid classes: All"""
        if obj.__class__ == Box:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z_min = 0
            z_max = 1
        elif obj.__class__ == Container:
            x_min = obj.x
            x_max = obj.x + obj.width - 1
            y_min = obj.y
            y_max = obj.y + obj.depth - 1
            z_min = obj.z
            z_max = obj.z
        elif obj.__class__ == Drawer:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z_min = 1
            z_max = 2
        elif obj.__class__ == Lid:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z_min = obj.z
            z_max = obj.z
        elif obj.__class__ == Stack:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z_min = 0
            z_max = 2
        else:
            x_min = obj.x
            x_max = obj.x
            y_min = obj.y
            y_max = obj.y
            z_min = obj.z
            z_max = obj.z

        for i in range(-1, 2):
            for j in range(-1, 2):
                if x_min <= self.x + i <= x_max and y_min <= self.y + j <= y_max and z_min <= self.z <= z_max:
                    return True
        return False


    def left_of(self, obj):
        """True if an object is to the left of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.x - obj.radius
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_x = obj.x - (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x < check_x

    def right_of(self, obj):
        """True if an object is to the right of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.x + obj.radius
        elif obj.__class__ == Container:
            check_x = obj.x + obj.width - 1
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_x = obj.x + (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x > check_x

    def behind(self, obj):
        """True if an object is behind another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.y + obj.radius
        elif obj.__class__ == Container:
            check_y = obj.y + obj.depth - 1
        elif obj.__class__ == Drawer:
            check_y = obj.y + (obj.depth - 1)/2
        else:
            check_y = obj.y

        return self.y > check_y

    def in_front_of(self, obj):
        """True if an object is in front of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.y - obj.radius
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_y = obj.y - (obj.depth - 1)/2
        else:
            check_y = obj.y

        return self.y < check_y

    def on(self, obj):
        """True if an object is x-y-aligned with another object

        Valid classes: All
        """
        return not(self.left_of(obj) or self.right_of(obj) or self.behind(obj) or self.in_front_of(obj))

    def above(self, obj):
        """True if an object is higher than another object

        Valid classes: All
        """
        if obj.__class__ == Box:
            check_z = 1
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_z = 2
        else:
            check_z = obj.z

        return self.z > check_z

    def below(self, obj):
        """True if an object is lower than another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Stack:
            check_z = 0
        elif obj.__class__ == Drawer:
            check_z = 1
        else:
            check_z = obj.z

        return self.z < check_z

    def level_with(self, obj):
        """True if an object is z-aligned with another object

        Valid classes: All
        """
        return not(self.above(obj) or self.below(obj))


class Container(object):

    def __init__(self, x=0, y=0, z=0, width=2, depth=2, name='', unique_name='', msg=None):
        self.msg = msg or ContainerMsg(x=x, y=y, z=z, width=width, depth=depth, name=name, unique_name=unique_name)
        self.relations = set()

        # Private variable to keep track of if relations need to be updated
        self._modified = False

    def to_ros(self):
        return self.msg

    def from_ros(self, msg):
        self.msg = msg
        return self

    def relations_updated(self):
        """Mark the relations as having been updated"""
        self._modified = False

    @property
    def x(self):
        return int(self.msg.x)

    @x.setter
    def x(self, value):
        self._modified = True
        self.msg.x = value

    @property
    def y(self):
        return int(self.msg.y)

    @y.setter
    def y(self, value):
        self._modified = True
        self.msg.y = value

    @property
    def z(self):
        return int(self.msg.z)

    @z.setter
    def z(self, value):
        self._modified = True
        self.msg.z = value

    @property
    def width(self):
        return int(self.msg.width)

    @property
    def depth(self):
        return int(self.msg.depth)

    @property
    def name(self):
        return self.msg.name

    @property
    def unique_name(self):
        return self.msg.unique_name

    # Relations
    def atop(self, obj):
        """True if on object is resting on top of another object

        Valid classes: Box, Container, Drawer, Item, Lid, Stack
        """
        if obj.__class__ == Box:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z = 2
        elif obj.__class__ == Container:
            x_min = obj.x
            x_max = obj.x + obj.width
            y_min = obj.y
            y_max = obj.y + obj.depth
            z = obj.z + 1
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z = 3
        elif obj.__class__ == Item:
            x_min = obj.x
            x_max = obj.x
            y_min = obj.y
            y_max = obj.y
            z = obj.z
        elif obj.__class__ == Lid:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z = obj.z
        else:
            return False

        # atop if any part of the container is on top of any part of the object
        for x in range(self.x, self.x + self.width):
            for y in range(self.y, self.y + self.depth):
                if x_min <= x <= x_max and y_min <= y <= y_max and self.z == z:
                    return True
        return False

    def inside(self, obj):
        """True if an object is contained within another object

        Valid classes: Box, Drawer
        """
        if obj.__class__ == Box:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z_min = 0
            z_max = 1
        elif obj.__class__ == Drawer:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z_min = 2
            z_max = 2
        else:
            return False

        # atop if any part of the container is on top of any part of the object
        for x in range(self.x, self.x + self.width):
            for y in range(self.y, self.y + self.depth):
                if not(x_min <= x <= x_max and y_min <= y <= y_max and z_min <= self.z <= z_max):
                    return False
        return True

    def touching(self, obj):
        """True if an object is touching (x-y plane, 'atop' covers z) another object

        Valid classes: All"""
        if obj.__class__ == Box:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z_min = 0
            z_max = 1
        elif obj.__class__ == Container:
            x_min = obj.x
            x_max = obj.x + obj.width - 1
            y_min = obj.y
            y_max = obj.y + obj.depth - 1
            z_min = obj.z
            z_max = obj.z
        elif obj.__class__ == Drawer:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z_min = 1
            z_max = 2
        elif obj.__class__ == Lid:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z_min = obj.z
            z_max = obj.z
        elif obj.__class__ == Stack:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z_min = 0
            z_max = 2
        else:
            x_min = obj.x
            x_max = obj.x
            y_min = obj.y
            y_max = obj.y
            z_min = obj.z
            z_max = obj.z

        for x in range(self.x - 1, self.x + self.width + 1):
            for y in [self.y - 1, self.y + self.depth]:
                if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= self.z <= z_max:
                    return True
        for y in range(self.y, self.y + self.depth):
            for x in [self.x - 1, self.x + self.width]:
                if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= self.z <= z_max:
                    return True
        return False

    def left_of(self, obj):
        """True if an object is to the left of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.x - obj.radius
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_x = obj.x - (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x + self.width - 1 < check_x

    def right_of(self, obj):
        """True if an object is to the right of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.x + obj.radius
        elif obj.__class__ == Container:
            check_x = obj.x + obj.width - 1
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_x = obj.x + (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x > check_x

    def behind(self, obj):
        """True if an object is behind another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.y + obj.radius
        elif obj.__class__ == Container:
            check_y = obj.y + obj.depth - 1
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_y = obj.y + (obj.depth - 1)/2
        else:
            check_y = obj.y

        return self.y > check_y

    def in_front_of(self, obj):
        """True if an object is in front of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.y - obj.radius
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_y = obj.y - (obj.depth - 1)/2
        else:
            check_y = obj.y

        return self.y + self.depth - 1 < check_y

    def on(self, obj):
        """True if an object is x-y-aligned with another object

        Valid classes: All
        """
        return not(self.left_of(obj) or self.right_of(obj) or self.behind(obj) or self.in_front_of(obj))

    def above(self, obj):
        """True if an object is higher than another object

        Valid classes: All
        """
        if obj.__class__ == Box:
            check_z = 1
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_z = 2
        else:
            check_z = obj.z

        return self.z > check_z

    def below(self, obj):
        """True if an object is lower than another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Stack:
            check_z = 0
        elif obj.__class__ == Drawer:
            check_z = 1
        else:
            check_z = obj.z

        return self.z < check_z

    def level_with(self, obj):
        """True if an object is z-aligned with another object

        Valid classes: All
        """
        return not(self.above(obj) or self.below(obj))


class Gripper(object):

    def __init__(self, x=0, y=0, z=0, closed=False, holding='', name='', unique_name='', msg=None):
        self.msg = msg or GripperMsg(x=x, y=y, z=z, closed=closed, holding=holding, name=name, unique_name=unique_name)
        self.relations = set()

        # Private variable to keep track of if relations need to be updated
        self._modified = False

    def to_ros(self):
        return self.msg

    def from_ros(self, msg):
        self.msg = msg
        return self

    def relations_updated(self):
        """Mark the relations as having been updated"""
        self._modified = False

    @property
    def x(self):
        return int(self.msg.x)

    @x.setter
    def x(self, value):
        self._modified = True
        self.msg.x = value

    @property
    def y(self):
        return int(self.msg.y)

    @y.setter
    def y(self, value):
        self._modified = True
        self.msg.y = value

    @property
    def z(self):
        return int(self.msg.z)

    @z.setter
    def z(self, value):
        self._modified = True
        self.msg.z = value

    @property
    def closed(self):
        return self.msg.closed

    @closed.setter
    def closed(self, value):
        self._modified = True
        self.msg.closed = value

    @property
    def holding(self):
        return self.msg.holding

    @holding.setter
    def holding(self, value):
        self._modified = True
        self.msg.holding = value

    @property
    def name(self):
        return self.msg.name

    @property
    def unique_name(self):
        return self.msg.unique_name

    # Relations
    def inside(self, obj):
        """True if an object is contained within another object

        Valid classes: Box, Container, Drawer
        """
        if obj.__class__ == Box:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z_min = 0
            z_max = 1
        elif obj.__class__ == Container:
            x_min = obj.x
            x_max = obj.x + obj.width
            y_min = obj.y
            y_max = obj.y + obj.depth
            z_min = obj.z
            z_max = obj.z
        elif obj.__class__ == Drawer:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z_min = 2
            z_max = 2
        else:
            return False

        return x_min <= self.x <= x_max and y_min <= self.y <= y_max and z_min <= self.z <= z_max

    def touching(self, obj):
        """True if an object is touching (x-y plane, 'atop' covers z) another object

        Valid classes: All"""
        if obj.__class__ == Box:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z_min = 0
            z_max = 1
        elif obj.__class__ == Container:
            x_min = obj.x
            x_max = obj.x + obj.width - 1
            y_min = obj.y
            y_max = obj.y + obj.depth - 1
            z_min = obj.z
            z_max = obj.z
        elif obj.__class__ == Drawer:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z_min = 1
            z_max = 2
        elif obj.__class__ == Lid:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z_min = obj.z
            z_max = obj.z
        elif obj.__class__ == Stack:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z_min = 0
            z_max = 2
        else:
            x_min = obj.x
            x_max = obj.x
            y_min = obj.y
            y_max = obj.y
            z_min = obj.z
            z_max = obj.z

        for i in range(-2, 3):
            for j in range(-2, 3):
                if x_min <= self.x + i <= x_max and y_min <= self.y + j <= y_max and z_min <= self.z <= z_max:
                    return True
        return False

    def left_of(self, obj):
        """True if an object is to the left of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.x - obj.radius
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_x = obj.x - (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x < check_x

    def right_of(self, obj):
        """True if an object is to the right of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.x + obj.radius
        elif obj.__class__ == Container:
            check_x = obj.x + obj.width - 1
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_x = obj.x + (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x > check_x

    def behind(self, obj):
        """True if an object is behind another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.y + obj.radius
        elif obj.__class__ == Container:
            check_y = obj.y + obj.depth - 1
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_y = obj.y + (obj.depth - 1)/2
        else:
            check_y = obj.y

        return self.y > check_y

    def in_front_of(self, obj):
        """True if an object is in front of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.y - obj.radius
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_y = obj.y - (obj.depth - 1)/2
        else:
            check_y = obj.y

        return self.y < check_y

    def on(self, obj):
        """True if an object is x-y-aligned with another object

        Valid classes: All
        """
        return not(self.left_of(obj) or self.right_of(obj) or self.behind(obj) or self.in_front_of(obj))

    def above(self, obj):
        """True if an object is higher than another object

        Valid classes: All
        """
        if obj.__class__ == Box:
            check_z = 1
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_z = 2
        else:
            check_z = obj.z

        return self.z > check_z

    def below(self, obj):
        """True if an object is lower than another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Stack:
            check_z = 0
        elif obj.__class__ == Drawer:
            check_z = 1
        else:
            check_z = obj.z

        return self.z < check_z

    def level_with(self, obj):
        """True if an object is z-aligned with another object

        Valid classes: All
        """
        return not(self.above(obj) or self.below(obj))


class Drawer(object):

    def __init__(self, x=0, y=0, width=7, depth=5, name='', unique_name='', msg=None):
        self.msg = msg or DrawerMsg(x=x, y=y, width=width, depth=depth, name=name, unique_name=unique_name)
        self.relations = set()

        # Private variable to keep track of if relations need to be updated
        self._modified = False

    def to_ros(self):
        return self.msg

    def from_ros(self, msg):
        self.msg = msg
        return self

    def relations_updated(self):
        """Mark the relations as having been updated"""
        self._modified = False

    @property
    def x(self):
        return int(self.msg.x)

    @x.setter
    def x(self, value):
        self._modified = True
        self.msg.x = value

    @property
    def y(self):
        return int(self.msg.y)

    @y.setter
    def y(self, value):
        self._modified = True
        self.msg.y = value

    @property
    def width(self):
        return int(self.msg.width)

    @property
    def depth(self):
        return int(self.msg.depth)

    @property
    def name(self):
        return self.msg.name

    @property
    def unique_name(self):
        return self.msg.unique_name

    # Relations
    def closing(self, obj):
        """True if a drawer is pushed into a stack

        Valid classes: Stack
        """
        return abs(self.x - obj.x) <= 1 and abs(self.y - obj.y) <= 1

    def touching(self, obj):
        """True if an object is touching (x-y plane, 'atop' covers z) another object

        Valid classes: All"""
        if obj.__class__ == Box:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z_min = 0
            z_max = 1
        elif obj.__class__ == Container:
            x_min = obj.x
            x_max = obj.x + obj.width - 1
            y_min = obj.y
            y_max = obj.y + obj.depth - 1
            z_min = obj.z
            z_max = obj.z
        elif obj.__class__ == Drawer:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z_min = 1
            z_max = 2
        elif obj.__class__ == Lid:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z_min = obj.z
            z_max = obj.z
        elif obj.__class__ == Stack:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z_min = 0
            z_max = 2
        else:
            x_min = obj.x
            x_max = obj.x
            y_min = obj.y
            y_max = obj.y
            z_min = obj.z
            z_max = obj.z

        w = (self.width - 1)/2
        d = (self.width - 1)/2
        for x in range(self.x - w - 1, self.x + w + 2):
            for y in [self.y - d - 1, self.y + d + 1]:
                for z in [1, 2]:
                    if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max:
                        return True
        for y in range(self.y - d, self.y + d + 1):
            for x in [self.x - w - 1, self.x + w + 1]:
                for z in [1, 2]:
                    if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max:
                        return True
        return False


class Stack(object):

    def __init__(self, x=0, y=0, width=7, depth=5, name='', unique_name='', msg=None):
        self.msg = msg or StackMsg(x=x, y=y, width=width, depth=depth, name=name, unique_name=unique_name)
        self.relations = set()

        # Private variable to keep track of if relations need to be updated
        self._modified = False

    def to_ros(self):
        return self.msg

    def from_ros(self, msg):
        self.msg = msg
        return self

    def relations_updated(self):
        """Mark the relations as having been updated"""
        self._modified = False

    @property
    def x(self):
        return int(self.msg.x)

    @x.setter
    def x(self, value):
        self._modified = True
        self.msg.x = value

    @property
    def y(self):
        return int(self.msg.y)

    @y.setter
    def y(self, value):
        self._modified = True
        self.msg.y = value

    @property
    def width(self):
        return int(self.msg.width)

    @property
    def depth(self):
        return int(self.msg.depth)

    @property
    def name(self):
        return self.msg.name

    @property
    def unique_name(self):
        return self.msg.unique_name


class Box(object):

    def __init__(self, x=0, y=0, radius=2, name='', unique_name='', msg=None):
        self.msg = msg or BoxMsg(x=x, y=y, radius=radius, name=name, unique_name=unique_name)
        self.relations = set()

        # Private variable to keep track of if relations need to be updated
        self._modified = False

    def to_ros(self):
        return self.msg

    def from_ros(self, msg):
        self.msg = msg
        return self

    def relations_updated(self):
        """Mark the relations as having been updated"""
        self._modified = False

    @property
    def x(self):
        return int(self.msg.x)

    @x.setter
    def x(self, value):
        self._modified = True
        self.msg.x = value

    @property
    def y(self):
        return int(self.msg.y)

    @y.setter
    def y(self, value):
        self._modified = True
        self.msg.y = value

    @property
    def radius(self):
        return int(self.msg.radius)

    @property
    def name(self):
        return self.msg.name

    @property
    def unique_name(self):
        return self.msg.unique_name


class Lid(object):

    def __init__(self, x=0, y=0, z=0, radius=2, name='', unique_name='', msg=None):
        self.msg = msg or LidMsg(x=x, y=y, z=z, radius=radius, name=name, unique_name=unique_name)
        self.relations = set()

        # Private variable to keep track of if relations need to be updated
        self._modified = False

    def to_ros(self):
        return self.msg

    def from_ros(self, msg):
        self.msg = msg
        return self

    def relations_updated(self):
        """Mark the relations as having been updated"""
        self._modified = False

    @property
    def x(self):
        return int(self.msg.x)

    @x.setter
    def x(self, value):
        self._modified = True
        self.msg.x = value

    @property
    def y(self):
        return int(self.msg.y)

    @y.setter
    def y(self, value):
        self._modified = True
        self.msg.y = value

    @property
    def z(self):
        return int(self.msg.z)

    @z.setter
    def z(self, value):
        self._modified = True
        self.msg.z = value

    @property
    def radius(self):
        return int(self.msg.radius)

    @property
    def name(self):
        return self.msg.name

    @property
    def unique_name(self):
        return self.msg.unique_name

    # Relations
    def closing(self, obj):
        """True if an object is closing a container

        Valid classes: Box
        """
        return self.x == obj.x and self.y == obj.y and self.z == 1

    def atop(self, obj):
        """True if on object is resting on top of another object

        Valid classes: Container, Drawer, Item, Lid, Stack
        """
        if obj.__class__ == Container:
            x_min = obj.x
            x_max = obj.x + obj.width
            y_min = obj.y
            y_max = obj.y + obj.depth
            z = obj.z + 1
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z = 3
        elif obj.__class__ == Item:
            x_min = obj.x
            x_max = obj.x
            y_min = obj.y
            y_max = obj.y
            z = obj.z
        elif obj.__class__ == Lid:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z = obj.z
        else:
            return False

        # atop if any part of the lid is on top of any part of the object
        for x in range(self.x - self.radius, self.x + self.radius + 1):
            for y in range(self.y - self.radius, self.y + self.radius + 1):
                if x_min <= x <= x_max and y_min <= y <= y_max and self.z == z:
                    return True
        return False

    def touching(self, obj):
        """True if an object is touching (x-y plane, 'atop' covers z) another object

        Valid classes: All"""
        if obj.__class__ == Box:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z_min = 0
            z_max = 1
        elif obj.__class__ == Container:
            x_min = obj.x
            x_max = obj.x + obj.width - 1
            y_min = obj.y
            y_max = obj.y + obj.depth - 1
            z_min = obj.z
            z_max = obj.z
        elif obj.__class__ == Drawer:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z_min = 1
            z_max = 2
        elif obj.__class__ == Lid:
            x_min = obj.x - obj.radius
            x_max = obj.x + obj.radius
            y_min = obj.y - obj.radius
            y_max = obj.y + obj.radius
            z_min = obj.z
            z_max = obj.z
        elif obj.__class__ == Stack:
            w = (obj.width - 1)/2
            d = (obj.depth - 1)/2
            x_min = obj.x - w
            x_max = obj.x + w
            y_min = obj.y - d
            y_max = obj.y + d
            z_min = 0
            z_max = 2
        else:
            x_min = obj.x
            x_max = obj.x
            y_min = obj.y
            y_max = obj.y
            z_min = obj.z
            z_max = obj.z

        for x in range(self.x - self.radius - 1, self.x + self.radius + 2):
            for y in [self.y - self.radius - 1, self.y + self.radius + 1]:
                if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= self.z <= z_max:
                    return True
        for y in range(self.y - self.radius - 1, self.y + self.radius + 1):
            for x in [self.x - self.radius - 1, self.x + self.radius + 1]:
                if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= self.z <= z_max:
                    return True
        return False

    def left_of(self, obj):
        """True if an object is to the left of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.x - obj.radius
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_x = obj.x - (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x + self.radius < check_x

    def right_of(self, obj):
        """True if an object is to the right of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.x + obj.radius
        elif obj.__class__ == Container:
            check_x = obj.x + obj.width - 1
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_x = obj.x + (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x - self.radius > check_x

    def behind(self, obj):
        """True if an object is behind another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.y + obj.radius
        elif obj.__class__ == Container:
            check_y = obj.y + obj.depth - 1
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_y = obj.y + (obj.depth - 1)/2
        else:
            check_y = obj.y

        return self.y - self.radius > check_y

    def in_front_of(self, obj):
        """True if an object is in front of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.y - obj.radius
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_y = obj.y - (obj.depth - 1)/2
        else:
            check_y = obj.y

        return self.y + self.radius < check_y

    def on(self, obj):
        """True if an object is x-y-aligned with another object

        Valid classes: All
        """
        return not(self.left_of(obj) or self.right_of(obj) or self.behind(obj) or self.in_front_of(obj))

    def above(self, obj):
        """True if an object is higher than another object

        Valid classes: All
        """
        if obj.__class__ == Box:
            check_z = 1
        elif obj.__class__ == Drawer or obj.__class__ == Stack:
            check_z = 2
        else:
            check_z = obj.z

        return self.z > check_z

    def below(self, obj):
        """True if an object is lower than another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Stack:
            check_z = 0
        elif obj.__class__ == Drawer:
            check_z = 1
        else:
            check_z = obj.z

        return self.z < check_z

    def level_with(self, obj):
        """True if an object is z-aligned with another object

        Valid classes: All
        """
        return not(self.above(obj) or self.below(obj))
