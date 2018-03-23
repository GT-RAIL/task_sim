#!/usr/bin/env python
# File that defines relations

from __future__ import print_function, division

import numpy as np

from task_sim.oomdp.oomdp_classes import Box, Container, Drawer, Gripper, Item, Lid, Stack

# Type of relations that we use at the moment
relation_kinds = np.array([
    'touching',     # 0, start spatial
    'left_of',      # 1
    'right_of',
    'behind',
    'in_front_of',  # 4
    'on',
    'above',
    'below',
    'level_with',   # 8, end spatial
    'inside',       # 9
    'atop',
    'closing'
])

# Format:
#   {Class1: {Class2: [kind, ...]}}
# Class1 and Class2 are classes in the OOMDP
# kind is the type of relation
RELATIONS = {
    Item: {
        Item: list(relation_kinds[ :9 ]), # spatial
        Container: list(relation_kinds[ [9]+range(9) ]), # inside + spatial
        Box: list(relation_kinds[ [9]+range(9) ]), # inside + spatial
        Drawer: list(relation_kinds[ [9]+range(9) ]), # inside + spatial
        Lid: list(relation_kinds[ [10]+range(9) ]), # atop + spatial
        Stack: list(relation_kinds[ [10]+range(9) ]), # atop + spatial
    },

    Container: {
        Container: list(relation_kinds[ [10]+range(9) ]), # atop + spatial
        Box: list(relation_kinds[ [10,9]+range(9) ]), # inside,atop + spatial
        Drawer: list(relation_kinds[ [10,9]+range(9) ]), # inside,atop + spatial
        Lid: list(relation_kinds[ [10]+range(9) ]), # atop + spatial
        Stack: list(relation_kinds[ [10]+range(9) ]), # atop + spatial
    },

    Lid: {
        Lid: list(relation_kinds[ [10]+range(9) ]), # atop + spatial
        Box: list(relation_kinds[ [11,10]+range(9) ]), # closing,atop + spatial
        Drawer: list(relation_kinds[ [10]+range(9) ]), # atop + spatial
        Stack: list(relation_kinds[ [10]+range(9) ]), # atop + spatial
    },

    Drawer: {
        Drawer: [relation_kinds[0]], # touching
        Box: [relation_kinds[0]], # touching
        Stack: list(relation_kinds[ [11,0] ]), # closing,touching
    },

    Gripper: {
        Gripper: list(relation_kinds[ :9 ]), # spatial
        Item: list(relation_kinds[ range(9) ]), # spatial
        Container: list(relation_kinds[ [9]+range(9) ]), # inside + spatial
        Box: list(relation_kinds[ [9]+range(9) ]), # inside + spatial
        Drawer: list(relation_kinds[ [9]+range(9) ]), # inside + spatial
        Lid: list(relation_kinds[ :9 ]), # spatial
        Stack: list(relation_kinds[ :9 ]), # spatial
    },
}

REVERSE_RELATIONS = {
    'left_of': 'right_of',
    'right_of': 'left_of',
    'behind': 'in_front_of',
    'in_front_of': 'behind',
    'above': 'below',
    'below': 'above'
}

# Define the class of relation

class Relation(object):
    """Defines the relation between two objects"""

    def __init__(self, kind, obj1, obj2):
        self._sanity_check(kind, obj1, obj2)

        self.kind = kind
        self.obj1 = obj1
        self.obj2 = obj2
        self.name = "{0}_{1}_{2}".format(obj1.name, kind, obj2.name)

        self._value = None
        # self._recalculate_value(True)

    def __str__(self):
        return "{}({},{}): {}".format(
            self.kind,
            self.obj1.unique_name,
            self.obj2.unique_name,
            self.value
        )

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        sanity = type(self) == type(other)
        return sanity and (
            self.obj1.unique_name == other.obj1.unique_name
            and self.obj2.unique_name == other.obj2.unique_name
            and self.kind == other.kind
        )

    def __hash__(self):
        return (hash(self.kind) ^ hash(self.obj1.unique_name) ^ hash(self.obj2.unique_name))

    def _sanity_check(self, kind, obj1, obj2):
        if not hasattr(obj1, kind):
            raise ValueError("{} does not have relation {}".format(obj1.__class__, kind))

    @property
    def value(self):
        self._recalculate_value()
        return self._value

    def _recalculate_value(self, force=False):
        # TODO: Check the force flag and some other properties of objects that
        # we can use to cache the relations
        self._value = getattr(self.obj1, self.kind)(self.obj2)
