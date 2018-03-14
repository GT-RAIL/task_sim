#!/usr/bin/env python

# TODO:
"""
Separate Drawer from Stack

Relations:
    Touching
    Closing (Drawer)
    Holding (Gripper)

Occlusion?
Edges?
"""


class Item:

    def __init__(self, x=0, y=0, z=0, name='', unique_name=''):
        # mutable
        self.x = x
        self.y = y
        self.z = z

        # immutable
        self.name = name
        self.unique_name = unique_name

    # Relations
    def atop(self, obj):
        """True if an object is resting on top of another object

        Valid classes: Drawer, Lid
        """
        if obj.__class__ == Drawer:
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

    def left_of(self, obj):
        """True if an object is to the left of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.position.x - obj.radius
        elif obj.__class__ == Drawer:
            check_x = obj.position.x - (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x < check_x

    def right_of(self, obj):
        """True if an object is to the right of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.position.x + obj.radius
        elif obj.__class__ == Container:
            check_x = obj.x + obj.width - 1
        elif obj.__class__ == Drawer:
            check_x = obj.position.x + (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x > check_x

    def behind(self, obj):
        """True if an object is behind another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.position.y + obj.radius
        elif obj.__class__ == Container:
            check_y = obj.position.y + obj.depth - 1
        elif obj.__class__ == Drawer:
            check_y = obj.position.y + (obj.depth - 1)/2
        else:
            check_y = obj.y

        return self.y > check_y

    def in_front_of(self, obj):
        """True if an object is in front of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.position.y
        elif obj.__class__ == Drawer:
            check_y = obj.position.y
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
        elif obj.__class__ == Drawer:
            check_z = 2
        else:
            check_z = obj.z

        return self.z > check_z

    def below(self, obj):
        """True if an object is lower than another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Drawer:
            check_z = 0
        else:
            check_z = obj.z

        return self.z < check_z

    def level_with(self, obj):
        """True if an object is z-aligned with another object

        Valid classes: All
        """
        return not(self.above(obj) or self.below(obj))


class Container:

    def __init__(self, x=0, y=0, z=0, width=2, depth=2, name='', unique_name=''):
        # mutable
        self.x = x
        self.y = y
        self.z = z

        # immutable
        self.width = width
        self.depth = depth
        self.name = name
        self.unique_name = unique_name

    # Relations
    def atop(self, obj):
        """True if on object is resting on top of another object

        Valid classes: Container, Drawer, Item, Lid
        """
        if obj.__class__ == Container:
            x_min = obj.x
            x_max = obj.x + obj.width
            y_min = obj.y
            y_max = obj.y + obj.depth
            z = obj.z + 1
        elif obj.__class__ == Drawer:
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

    def left_of(self, obj):
        """True if an object is to the left of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.position.x - obj.radius
        elif obj.__class__ == Drawer:
            check_x = obj.position.x - (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x + obj.width - 1 < check_x

    def right_of(self, obj):
        """True if an object is to the right of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.position.x + obj.radius
        elif obj.__class__ == Container:
            check_x = obj.x + obj.width - 1
        elif obj.__class__ == Drawer:
            check_x = obj.position.x + (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x > check_x

    def behind(self, obj):
        """True if an object is behind another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.position.y + obj.radius
        elif obj.__class__ == Container:
            check_y = obj.position.y + obj.depth - 1
        elif obj.__class__ == Drawer:
            check_y = obj.position.y + (obj.depth - 1)/2
        else:
            check_y = obj.y

        return self.y > check_y

    def in_front_of(self, obj):
        """True if an object is in front of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.position.y
        elif obj.__class__ == Drawer:
            check_y = obj.position.y
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
        elif obj.__class__ == Drawer:
            check_z = 2
        else:
            check_z = obj.z

        return self.z > check_z

    def below(self, obj):
        """True if an object is lower than another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Drawer:
            check_z = 0
        else:
            check_z = obj.z

        return self.z < check_z

    def level_with(self, obj):
        """True if an object is z-aligned with another object

        Valid classes: All
        """
        return not(self.above(obj) or self.below(obj))


class Gripper:

    def __init__(self, x=0, y=0, z=0, closed=False, name='', unique_name=''):
        # mutable
        self.x = x
        self.y = y
        self.z = z
        self.closed = closed

        # immutable
        self.name = name
        self.unique_name = unique_name

    # Relations
    def left_of(self, obj):
        """True if an object is to the left of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.position.x - obj.radius
        elif obj.__class__ == Drawer:
            check_x = obj.position.x - (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x < check_x

    def right_of(self, obj):
        """True if an object is to the right of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.position.x + obj.radius
        elif obj.__class__ == Container:
            check_x = obj.x + obj.width - 1
        elif obj.__class__ == Drawer:
            check_x = obj.position.x + (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x > check_x

    def behind(self, obj):
        """True if an object is behind another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.position.y + obj.radius
        elif obj.__class__ == Container:
            check_y = obj.position.y + obj.depth - 1
        elif obj.__class__ == Drawer:
            check_y = obj.position.y + (obj.depth - 1)/2
        else:
            check_y = obj.y

        return self.y > check_y

    def in_front_of(self, obj):
        """True if an object is in front of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.position.y
        elif obj.__class__ == Drawer:
            check_y = obj.position.y
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
        elif obj.__class__ == Drawer:
            check_z = 2
        else:
            check_z = obj.z

        return self.z > check_z

    def below(self, obj):
        """True if an object is lower than another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Drawer:
            check_z = 0
        else:
            check_z = obj.z

        return self.z < check_z

    def level_with(self, obj):
        """True if an object is z-aligned with another object

        Valid classes: All
        """
        return not(self.above(obj) or self.below(obj))


class Drawer:

    def __init__(self, x=0, y=0, opening=0, width=5, depth=7, name='', unique_name=''):
        # mutable
        self.x = x
        self.y = y
        self.opening = opening

        # immutable
        self.width = width
        self.depth = depth
        self.name = name
        self.unique_name = unique_name


class Box:

    def __init__(self, x=0, y=0, radius=2, name='', unique_name=''):
        # mutable
        self.x = x
        self.y = y

        # immutable
        self.radius = radius
        self.name = name
        self.unique_name = unique_name


class Lid:

    def __init__(self, x=0, y=0, z=0, radius=2, name='', unique_name=''):
        # mutable
        self.x = x
        self.y = y
        self.z = z

        # immutable
        self.radius = radius
        self.name = name
        self.unique_name = unique_name

    # Relations
    def closing(self, obj):
        """True if an object is closing a container

        Valid classes: Box
        """
        return self.x == obj.x and self.y == obj.y and self.z == 2

    def atop(self, obj):
        """True if on object is resting on top of another object

        Valid classes: Container, Drawer, Item, Lid
        """
        if obj.__class__ == Container:
            x_min = obj.x
            x_max = obj.x + obj.width
            y_min = obj.y
            y_max = obj.y + obj.depth
            z = obj.z + 1
        elif obj.__class__ == Drawer:
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

    def left_of(self, obj):
        """True if an object is to the left of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.position.x - obj.radius
        elif obj.__class__ == Drawer:
            check_x = obj.position.x - (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x + self.radius < check_x

    def right_of(self, obj):
        """True if an object is to the right of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_x = obj.position.x + obj.radius
        elif obj.__class__ == Container:
            check_x = obj.x + obj.width - 1
        elif obj.__class__ == Drawer:
            check_x = obj.position.x + (obj.width - 1)/2
        else:
            check_x = obj.x

        return self.x - self.radius > check_x

    def behind(self, obj):
        """True if an object is behind another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.position.y + obj.radius
        elif obj.__class__ == Container:
            check_y = obj.position.y + obj.depth - 1
        elif obj.__class__ == Drawer:
            check_y = obj.position.y + (obj.depth - 1)/2
        else:
            check_y = obj.y

        return self.y - self.radius > check_y

    def in_front_of(self, obj):
        """True if an object is in front of another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Lid:
            check_y = obj.position.y
        elif obj.__class__ == Drawer:
            check_y = obj.position.y
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
        elif obj.__class__ == Drawer:
            check_z = 2
        else:
            check_z = obj.z

        return self.z > check_z

    def below(self, obj):
        """True if an object is lower than another object

        Valid classes: All
        """
        if obj.__class__ == Box or obj.__class__ == Drawer:
            check_z = 0
        else:
            check_z = obj.z

        return self.z < check_z

    def level_with(self, obj):
        """True if an object is z-aligned with another object

        Valid classes: All
        """
        return not(self.above(obj) or self.below(obj))
