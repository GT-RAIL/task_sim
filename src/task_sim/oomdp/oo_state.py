#!/usr/bin/env python

from task_sim.oomdp.oomdp_classes import Box, Container, Drawer, Gripper, Item, Lid, Stack

reverse_relation = {
        'left_of': 'right_of',
        'right_of': 'left_of',
        'behind': 'in_front_of',
        'in_front_of': 'behind',
        'above': 'below',
        'below': 'above'
    }


class OOState:

    def __init__(self, state=None):
        if state is None:
            self.boxes = {}
            self.containers = {}
            self.drawers = {}
            self.grippers = {}
            self.items = {}
            self.lids = {}
            self.stacks = {}
            self.relations = []
        else:
            self.init_from_state(state)

    def init_from_state(self, state):
        for o in state.objects:
            item = Item(o)
            self.items[item.unique_name] = item

        for c in state.containers:
            container = Container(c)
            self.items[c.unique_name] = container

        box = Box(state.box_position.x, state.box.position.y, name='box', unique_name='box')
        self.boxes[box.unique_name] = box

        drawer = Drawer(state.drawer_position.x + state.drawer_opening, state.drawer_position.y, name='drawer',
                        unique_name='drawer')
        self.drawers[drawer.unique_name] = drawer

        holding = state.object_in_gripper
        gripper = Gripper(state.gripper_position.x, state.gripper_position.y, state.gripper_position.z,
                          not state.gripper_open, holding, 'gripper', 'gripper')
        self.grippers[gripper.unique_name] = gripper

        lid = Lid(state.lid_position.x, state.lid_position.y, state.lid_position.z, name="lid", unique_name="lid")
        self.lids[lid.unique_name] = lid

        stack = Stack(state.drawer_position.x, state.drawer_position.y, name='stack', unique_name='stack')
        self.stacks[stack.unique_name] = stack

        self.calculate_relations()

    def calculate_relations(self):
        """Calculate all relations between all objects

        Note: this will clear any current relations and recalculate
        """
        self.clear_relations()

        def add_relation(obj1, obj2, result, relation_name, sort=False):
            reverse = None
            if sort:
                name_order = [obj1.unique_name, obj2.unique_name]
                name_order.sort()
                if result:
                    rel = name_order[0] + '_' + relation_name + '_' + name_order[1]
                else:
                    rel = 'not_' + name_order[0] + '_' + relation_name + '_' + name_order[1]
            else:
                if result:
                    rel = obj1.unique_name + '_' + relation_name + '_' + obj2.unique_name
                    if relation_name in reverse_relation:
                        reverse = obj2.unique_name + '_' + reverse_relation[relation_name] + '_' + obj1.unique_name
                else:
                    rel = 'not_' + obj1.unique_name + '_' + relation_name + '_' + obj2.unique_name
                    if relation_name in reverse_relation:
                        reverse = 'not_' + obj2.unique_name + '_' + reverse_relation[relation_name] + '_' + obj1.unique_name

            if rel not in self.relations:
                self.relations.append(rel)

            if rel not in obj1.relations:
                obj1.relations.append(rel)

            if reverse is not None:
                if reverse not in self.relations:
                    self.relations.append(reverse)
                if reverse not in obj2.relations:
                    obj2.relations.append(reverse)
            else:
                if rel not in obj2.relations:
                    obj2.relations.append(rel)

        for i in range(len(self.items)):
            item = self.items[self.items.keys()[i]]

            # item-item relations
            for j in range(i + 1, len(self.items)):
                item2 = self.items[self.items.keys()[j]]
                add_relation(item, item2, item.touching(item2), 'touching', sort=True)
                add_relation(item, item2, item.left_of(item2), 'left_of')
                add_relation(item, item2, item.right_of(item2), 'right_of')
                add_relation(item, item2, item.behind(item2), 'behind')
                add_relation(item, item2, item.in_front_of(item2), 'in_front_of')
                add_relation(item, item2, item.on(item2), 'on', sort=True)
                add_relation(item, item2, item.above(item2), 'above')
                add_relation(item, item2, item.below(item2), 'below')
                add_relation(item, item2, item.level_with(item2), 'level_with', sort=True)

            # item-container relations
            for c in self.containers.values():
                add_relation(item, c, item.inside(c), 'inside')
                add_relation(item, c, item.touching(c), 'touching')
                add_relation(item, c, item.left_of(c), 'left_of')
                add_relation(item, c, item.right_of(c), 'right_of')
                add_relation(item, c, item.behind(c), 'behind')
                add_relation(item, c, item.in_front_of(c), 'in_front_of')
                add_relation(item, c, item.on(c), 'on')
                add_relation(item, c, item.above(c), 'above')
                add_relation(item, c, item.below(c), 'below')
                add_relation(item, c, item.level_with(c), 'level_with')

            # item-box relations
            for b in self.boxes.values():
                add_relation(item, b, item.inside(b), 'inside')
                add_relation(item, b, item.touching(b), 'touching')
                add_relation(item, b, item.left_of(b), 'left_of')
                add_relation(item, b, item.right_of(b), 'right_of')
                add_relation(item, b, item.behind(b), 'behind')
                add_relation(item, b, item.in_front_of(b), 'in_front_of')
                add_relation(item, b, item.on(b), 'on')
                add_relation(item, b, item.above(b), 'above')
                add_relation(item, b, item.below(b), 'below')
                add_relation(item, b, item.level_with(b), 'level_with')

            # item-drawer relations
            for d in self.drawers.values():
                add_relation(item, d, item.inside(d), 'inside')
                add_relation(item, d, item.touching(d), 'touching')
                add_relation(item, d, item.left_of(d), 'left_of')
                add_relation(item, d, item.right_of(d), 'right_of')
                add_relation(item, d, item.behind(d), 'behind')
                add_relation(item, d, item.in_front_of(d), 'in_front_of')
                add_relation(item, d, item.on(d), 'on')
                add_relation(item, d, item.above(d), 'above')
                add_relation(item, d, item.below(d), 'below')
                add_relation(item, d, item.level_with(d), 'level_with')

            # item-lid relations
            for l in self.lids.values():
                add_relation(item, l, item.atop(l), 'atop')
                add_relation(item, l, item.touching(l), 'touching')
                add_relation(item, l, item.left_of(l), 'left_of')
                add_relation(item, l, item.right_of(l), 'right_of')
                add_relation(item, l, item.behind(l), 'behind')
                add_relation(item, l, item.in_front_of(l), 'in_front_of')
                add_relation(item, l, item.on(l), 'on')
                add_relation(item, l, item.above(l), 'above')
                add_relation(item, l, item.below(l), 'below')
                add_relation(item, l, item.level_with(l), 'level_with')

            # item-stack relations
            for s in self.stacks.values():
                add_relation(item, s, item.atop(s), 'atop')
                add_relation(item, s, item.touching(s), 'touching')
                add_relation(item, s, item.left_of(s), 'left_of')
                add_relation(item, s, item.right_of(s), 'right_of')
                add_relation(item, s, item.behind(s), 'behind')
                add_relation(item, s, item.in_front_of(s), 'in_front_of')
                add_relation(item, s, item.on(s), 'on')
                add_relation(item, s, item.above(s), 'above')
                add_relation(item, s, item.below(s), 'below')
                add_relation(item, s, item.level_with(s), 'level_with')

        for i in range(len(self.containers)):
            container = self.containers[self.containers.keys()[i]]

            # container-container relations
            for j in range(i + 1, len(self.containers)):
                container2 = self.containers[self.containers.keys()[j]]
                add_relation(container, container2, item.inside(container2), 'atop')
                add_relation(container, container2, item.touching(container2), 'touching', sort=True)
                add_relation(container, container2, item.left_of(container2), 'left_of')
                add_relation(container, container2, item.right_of(container2), 'right_of')
                add_relation(container, container2, item.behind(container2), 'behind')
                add_relation(container, container2, item.in_front_of(container2), 'in_front_of')
                add_relation(container, container2, item.on(container2), 'on', sort=True)
                add_relation(container, container2, item.above(container2), 'above')
                add_relation(container, container2, item.below(container2), 'below')
                add_relation(container, container2, item.level_with(container2), 'level_with', sort=True)

            # item-box relations
            for b in self.boxes.values():
                add_relation(container, b, item.atop(b), 'atop')
                add_relation(container, b, item.inside(b), 'inside')
                add_relation(container, b, item.touching(b), 'touching')
                add_relation(container, b, item.left_of(b), 'left_of')
                add_relation(container, b, item.right_of(b), 'right_of')
                add_relation(container, b, item.behind(b), 'behind')
                add_relation(container, b, item.in_front_of(b), 'in_front_of')
                add_relation(container, b, item.on(b), 'on')
                add_relation(container, b, item.above(b), 'above')
                add_relation(container, b, item.below(b), 'below')
                add_relation(container, b, item.level_with(b), 'level_with')

            # item-drawer relations
            for d in self.drawers.values():
                add_relation(container, d, item.atop(d), 'atop')
                add_relation(container, d, item.inside(d), 'inside')
                add_relation(container, d, item.touching(d), 'touching')
                add_relation(container, d, item.left_of(d), 'left_of')
                add_relation(container, d, item.right_of(d), 'right_of')
                add_relation(container, d, item.behind(d), 'behind')
                add_relation(container, d, item.in_front_of(d), 'in_front_of')
                add_relation(container, d, item.on(d), 'on')
                add_relation(container, d, item.above(d), 'above')
                add_relation(container, d, item.below(d), 'below')
                add_relation(container, d, item.level_with(d), 'level_with')

            # item-lid relations
            for l in self.lids.values():
                add_relation(container, l, item.atop(l), 'atop')
                add_relation(container, l, item.touching(l), 'touching')
                add_relation(container, l, item.left_of(l), 'left_of')
                add_relation(container, l, item.right_of(l), 'right_of')
                add_relation(container, l, item.behind(l), 'behind')
                add_relation(container, l, item.in_front_of(l), 'in_front_of')
                add_relation(container, l, item.on(l), 'on')
                add_relation(container, l, item.above(l), 'above')
                add_relation(container, l, item.below(l), 'below')
                add_relation(container, l, item.level_with(l), 'level_with')

            # item-stack relations
            for s in self.stacks.values():
                add_relation(container, s, item.atop(s), 'atop')
                add_relation(container, s, item.touching(s), 'touching')
                add_relation(container, s, item.left_of(s), 'left_of')
                add_relation(container, s, item.right_of(s), 'right_of')
                add_relation(container, s, item.behind(s), 'behind')
                add_relation(container, s, item.in_front_of(s), 'in_front_of')
                add_relation(container, s, item.on(s), 'on')
                add_relation(container, s, item.above(s), 'above')
                add_relation(container, s, item.below(s), 'below')
                add_relation(container, s, item.level_with(s), 'level_with')

    def clear_relations(self):
        self.relations = []

        for b in self.boxes:
            b.relations = []
        for c in self.containers:
            c.relations = []
        for d in self.drawers:
            d.relations = []
        for g in self.grippers:
            g.relations = []
        for i in self.items:
            i.relations = []
        for l in self.lids:
            l.relations = []
        for s in self.stacks:
            s.relations = []
