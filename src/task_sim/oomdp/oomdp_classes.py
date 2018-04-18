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


class Item:

    def __init__(self, x=0, y=0, z=0, name='', unique_name='', msg=None):
        if msg is not None:
            self.from_ros(msg)
        else:
            # mutable
            self.x = x
            self.y = y
            self.z = z

            # immutable
            self.name = name
            self.unique_name = unique_name

    def to_ros(self):
        msg = ItemMsg()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.name = self.name
        msg.unique_name = self.unique_name
        return msg

    def from_ros(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.name = msg.name
        self.unique_name = msg.unique_name

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


class Container:

    def __init__(self, x=0, y=0, z=0, width=2, depth=2, name='', unique_name='', msg=None):
        if msg is not None:
            self.from_ros(msg)
        else:
            # mutable
            self.x = x
            self.y = y
            self.z = z

            # immutable
            self.width = width
            self.depth = depth
            self.name = name
            self.unique_name = unique_name

    def to_ros(self):
        msg = ContainerMsg()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.width = self.width
        msg.depth = self.depth
        msg.name = self.name
        msg.unique_name = self.unique_name
        return msg

    def from_ros(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.width = msg.width
        self.depth = msg.depth
        self.name = msg.name
        self.unique_name = msg.unique_name

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
        for x in range(int(self.x), int(self.x + self.width)):
            for y in range(int(self.y), int(self.y + self.depth)):
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
        for x in range(int(self.x), int(self.x + self.width)):
            for y in range(int(self.y), int(self.y + self.depth)):
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

        for x in range(int(self.x - 1), int(self.x + self.width + 1)):
            for y in [self.y - 1, self.y + self.depth]:
                if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= self.z <= z_max:
                    return True
        for y in range(int(self.y), int(self.y + self.depth)):
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


class Gripper:

    def __init__(self, x=0, y=0, z=0, closed=False, holding='', name='', unique_name='', msg=None):
        if msg is not None:
            self.from_ros(msg)
        else:
            # mutable
            self.x = x
            self.y = y
            self.z = z
            self.closed = closed
            self.holding = holding

            # immutable
            self.name = name
            self.unique_name = unique_name

    def to_ros(self):
        msg = GripperMsg()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.closed = self.closed
        msg.holding = self.holding
        msg.name = self.name
        msg.unique_name = self.unique_name
        return msg

    def from_ros(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.closed = msg.closed
        self.holding = msg.holding
        self.name = msg.name
        self.unique_name = msg.unique_name

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


class Drawer:

    def __init__(self, x=0, y=0, width=7, depth=5, name='', unique_name='', msg=None):
        if msg is not None:
            self.from_ros(msg)
        else:
            # mutable
            self.x = x
            self.y = y

            # immutable
            self.width = width
            self.depth = depth
            self.name = name
            self.unique_name = unique_name

    def to_ros(self):
        msg = DrawerMsg()
        msg.x = self.x
        msg.y = self.y
        msg.width = self.width
        msg.depth = self.depth
        msg.name = self.name
        msg.unique_name = self.unique_name
        return msg

    def from_ros(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.width = msg.width
        self.depth = msg.depth
        self.name = msg.name
        self.unique_name = msg.unique_name

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
        for x in range(int(self.x - w - 1), int(self.x + w + 2)):
            for y in [self.y - d - 1, self.y + d + 1]:
                for z in [1, 2]:
                    if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max:
                        return True
        for y in range(int(self.y - d), int(self.y + d + 1)):
            for x in [self.x - w - 1, self.x + w + 1]:
                for z in [1, 2]:
                    if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max:
                        return True
        return False


class Stack:

    def __init__(self, x=0, y=0, width=7, depth=5, name='', unique_name='', msg=None):
        if msg is not None:
            self.from_ros(msg)
        else:
            # mutable
            self.x = x
            self.y = y

            # immutable
            self.width = width
            self.depth = depth
            self.name = name
            self.unique_name = unique_name

    def to_ros(self):
        msg = StackMsg()
        msg.x = self.x
        msg.y = self.y
        msg.width = self.width
        msg.depth = self.depth
        msg.name = self.name
        msg.unique_name = self.unique_name
        return msg

    def from_ros(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.width = msg.width
        self.depth = msg.depth
        self.name = msg.name
        self.unique_name = msg.unique_name


class Box:

    def __init__(self, x=0, y=0, radius=2, name='', unique_name='', msg=None):
        if msg is not None:
            self.from_ros(msg)
        else:
            # mutable
            self.x = x
            self.y = y

            # immutable
            self.radius = radius
            self.name = name
            self.unique_name = unique_name

    def to_ros(self):
        msg = BoxMsg()
        msg.x = self.x
        msg.y = self.y
        msg.radius = self.radius
        msg.name = self.name
        msg.unique_name = self.unique_name
        return msg

    def from_ros(self, msg):
        self.x = int(msg.x)
        self.y = int(msg.y)
        self.radius = int(msg.radius)
        self.name = msg.name
        self.unique_name = msg.unique_name


class Lid:

    def __init__(self, x=0, y=0, z=0, radius=2, name='', unique_name='', msg=None):
        if msg is not None:
            self.from_ros(msg)
        else:
            # mutable
            self.x = x
            self.y = y
            self.z = z

            # immutable
            self.radius = radius
            self.name = name
            self.unique_name = unique_name

    def to_ros(self):
        msg = LidMsg()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.radius = self.radius
        msg.name = self.name
        msg.unique_name = self.unique_name
        return msg

    def from_ros(self, msg):
        self.x = int(msg.x)
        self.y = int(msg.y)
        self.z = int(msg.z)
        self.radius = int(msg.radius)
        self.name = msg.name
        self.unique_name = msg.unique_name

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
        for x in range(int(self.x - self.radius), int(self.x + self.radius + 1)):
            for y in range(int(self.y - self.radius), int(self.y + self.radius + 1)):
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

        for x in range(int(self.x - self.radius - 1), int(self.x + self.radius + 2)):
            for y in [self.y - self.radius - 1, self.y + self.radius + 1]:
                if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= self.z <= z_max:
                    return True
        for y in range(int(self.y - self.radius - 1), int(self.y + self.radius + 1)):
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
