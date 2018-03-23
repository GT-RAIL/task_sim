#!/usr/bin/env python

from __future__ import print_function

import numpy as np

from string import digits
from collections import defaultdict
from pprint import pprint

from task_sim.msg import OOState as OOStateMsg
from task_sim.oomdp.oomdp_classes import Box, Container, Drawer, Gripper, Item, Lid, Stack
from task_sim.oomdp.oomdp_relations import Relation, REVERSE_RELATIONS, RELATIONS


class OOState:

    def __init__(self, state=None):
        self.clear_state()
        if state is not None:
            self.init_from_state(state)

    def clear_state(self):
        """
        Since this is called in init, this also defines the member variables.
        The names lists have been included to force a predefined ordering that
        is consistent across systems and runs
        """
        self.boxes = {}

        self.containers = {}
        self.container_names = []

        self.drawers = {}
        self.drawer_names = []

        self.grippers = {}
        self.gripper_names = []

        self.items = {}
        self.item_names = []

        self.lids = {}
        self.lid_names = []

        self.stacks = {}

        self.clear_relations()


    def init_from_state(self, state):
        self.clear_state()

        for o in state.objects:
            item = Item(o.position.x, o.position.y, o.position.z, o.name, o.unique_name)
            self.items[item.unique_name] = item
        self.item_names = sorted(self.items.keys())

        for c in state.containers:
            container = Container(c.position.x, c.position.y, c.position.z, c.width, c.height, c.name, c.unique_name)
            self.containers[c.unique_name] = container
        self.container_names = sorted(self.containers.keys())

        box = Box(state.box_position.x, state.box_position.y, name='box', unique_name='box')
        self.boxes[box.unique_name] = box

        drawer = Drawer(state.drawer_position.x + state.drawer_opening, state.drawer_position.y, name='drawer',
                        unique_name='drawer')
        self.drawers[drawer.unique_name] = drawer
        self.drawer_names = sorted(self.drawers.keys())

        holding = state.object_in_gripper.translate(None, digits)
        gripper = Gripper(state.gripper_position.x, state.gripper_position.y, state.gripper_position.z,
                          not state.gripper_open, holding, 'gripper', 'gripper')
        self.grippers[gripper.unique_name] = gripper
        self.gripper_names = sorted(self.grippers.keys())

        lid = Lid(state.lid_position.x, state.lid_position.y, state.lid_position.z, name="lid", unique_name="lid")
        self.lids[lid.unique_name] = lid
        self.lid_names = sorted(self.lids.keys())

        stack = Stack(state.drawer_position.x, state.drawer_position.y, name='stack', unique_name='stack')
        self.stacks[stack.unique_name] = stack

        self.reinit_relations()


    def reinit_relations(self):
        """
        Initialize the set of all the relations
        """
        self.clear_relations()

        # Helper function to add the relation to the set known relations
        def add_relation(relation_kind, obj1, obj2):
            relation = Relation(relation_kind, obj1, obj2)
            self.relations[relation.name].add(relation)
            obj1.relations.add(relation)
            obj2.relations.add(relation)

        class_to_objdict_map = {
            Item: (self.items, self.item_names),
            Container: (self.containers, self.container_names),
            Box: (self.boxes, self.boxes.keys()),
            Drawer: (self.drawers, self.drawer_names),
            Lid: (self.lids, self.lid_names),
            Stack: (self.stacks, self.stacks.keys()),
            Gripper: (self.grippers, self.gripper_names),
        }

        # For each type of RELATIONS pairs, define the relevant relations
        # WITH the wonders of quintuple for-loops. This is fun
        for class_type, rel_class_types in RELATIONS.iteritems():
            obj_dict, obj_keys = class_to_objdict_map[class_type]

            for i, key in enumerate(obj_keys):
                obj = obj_dict[key]

                for rel_class_type, relation_kinds in rel_class_types.iteritems():
                    rel_obj_dict, rel_obj_keys = class_to_objdict_map[rel_class_type]
                    rel_obj_keys = (
                        obj_keys[i+1:] if class_type == rel_class_type
                        else rel_obj_keys
                    )

                    for rel_key in rel_obj_keys:
                        rel_obj = rel_obj_dict[rel_key]

                        for relation_kind in relation_kinds:
                            add_relation(relation_kind, obj, rel_obj)

        # Update the relation names and relation values
        self.relation_names = { x: i for i, x in enumerate(sorted(self.relations.keys())) }
        self.relation_values = [
            any([relation.value for relation in self.relations[name]])
            for name in sorted(self.relation_names.keys())
        ]

        # Return
        return self


    def calculate_relations(self):
        """Calculate all relations between all objects. TODO: Be
        smarter about this
        """
        # TODO: Maybe parallel for loops will solve speed here?
        pass


    def clear_relations(self):
        self.relations = defaultdict(set)
        self.relation_names = {}
        # Speed up ROS message construction by caching the values?
        self.relation_values = []

        for b in self.boxes.values():
            b.relations.clear()
        for c in self.containers.values():
            c.relations.clear()
        for d in self.drawers.values():
            d.relations.clear()
        for g in self.grippers.values():
            g.relations.clear()
        for i in self.items.values():
            i.relations.clear()
        for l in self.lids.values():
            l.relations.clear()
        for s in self.stacks.values():
            s.relations.clear()

    def to_ros(self):
        msg = OOStateMsg()

        for obj in self.boxes.values():
            msg.boxes.append(obj.to_ros())

        for obj in self.containers.values():
            msg.containers.append(obj.to_ros())

        for obj in self.drawers.values():
            msg.drawers.append(obj.to_ros())

        for obj in self.grippers.values():
            msg.grippers.append(obj.to_ros())

        for obj in self.items.values():
            msg.items.append(obj.to_ros())

        for obj in self.lids.values():
            msg.lids.append(obj.to_ros())

        for obj in self.stacks.values():
            msg.stacks.append(obj.to_ros())

        msg.relations = self.relation_values
        return msg

    def from_ros(self, msg):
        self.clear_state()

        for obj in msg.boxes:
            self.boxes[obj.unique_name] = Box(msg=obj)

        for obj in msg.containers:
            self.containers[obj.unique_name] = Container(msg=obj)

        for obj in msg.drawers:
            self.drawers[obj.unique_name] = Drawer(msg=obj)

        for obj in msg.grippers:
            self.grippers[obj.unique_name] = Gripper(msg=obj)

        for obj in msg.items:
            self.items[obj.unique_name] = Item(msg=obj)

        for obj in msg.lids:
            self.lids[obj.unique_name] = Lid(msg=obj)

        for obj in msg.stacks:
            self.stacks[obj.unique_name] = Stack(msg=obj)

        self.reinit_relations()
        return self
