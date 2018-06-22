#!/usr/bin/env python

from string import digits

from task_sim.msg import OOState as OOStateMsg
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

    def __init__(self, state=None, continuous=False):
        self.clear_state()
        if state is not None:
            self.init_from_state(state, continuous)

    def clear_state(self):
        self.boxes = {}
        self.containers = {}
        self.drawers = {}
        self.grippers = {}
        self.items = {}
        self.lids = {}
        self.stacks = {}
        self.relations = []

    def init_from_state(self, state, continuous=False):
        self.clear_state()

        for o in state.objects:
            item = Item(o.position.x, o.position.y, o.position.z, o.name, o.unique_name, continuous=continuous)
            self.items[item.unique_name] = item

        for c in state.containers:
            container = Container(c.position.x, c.position.y, c.position.z, c.width, c.height, c.name, c.unique_name)
            self.containers[c.unique_name] = container

        box = Box(state.box_position.x, state.box_position.y, name='box', unique_name='box', continuous=continuous)
        self.boxes[box.unique_name] = box

        if continuous:
            drawer = Drawer(state.drawer_position.x + state.drawer_opening, state.drawer_position.y, name='drawer',
                            unique_name='drawer', width=0.2286, depth=0.3302, continuous=True)
        else:
            drawer = Drawer(state.drawer_position.x + state.drawer_opening, state.drawer_position.y, name='drawer',
                            unique_name='drawer')
        self.drawers[drawer.unique_name] = drawer

        holding = state.object_in_gripper.translate(None, digits)
        gripper = Gripper(state.gripper_position.x, state.gripper_position.y, state.gripper_position.z,
                          not state.gripper_open, holding, 'gripper', 'gripper', continuous=continuous)
        self.grippers[gripper.unique_name] = gripper

        lid = Lid(state.lid_position.x, state.lid_position.y, state.lid_position.z, name="lid", unique_name="lid",
                  continuous=continuous)
        self.lids[lid.unique_name] = lid

        if continuous:
            stack = Stack(state.drawer_position.x, state.drawer_position.y, name='stack', unique_name='stack',
                          width=0.2486, depth=0.3302, continuous=True)
        else:
            stack = Stack(state.drawer_position.x, state.drawer_position.y, name='stack', unique_name='stack')
        self.stacks[stack.unique_name] = stack

        self.calculate_relations()

    def calculate_relations(self):
        """Calculate all relations between all objects

        Note: this will clear any current relations and recalculate
        """
        self.clear_relations()

        self.relation_count = 0

        def add_relation(obj1, obj2, result, relation_name, sort=False):
            self.relation_count += 1
            reverse = None
            rel = None
            if sort:
                name_order = sorted([obj1.name, obj2.name])
                if result:
                    rel = name_order[0] + '_' + relation_name + '_' + name_order[1]
            else:
                if result:
                    rel = obj1.name + '_' + relation_name + '_' + obj2.name
                    if relation_name in reverse_relation:
                        reverse = obj2.name + '_' + reverse_relation[relation_name] + '_' + obj1.name

            if rel is not None and rel not in self.relations:
                self.relations.append(rel)

            if rel is not None and rel not in obj1.relations:
                obj1.relations.append(rel)

            if reverse is not None:
                if reverse not in self.relations:
                    self.relations.append(reverse)
                if reverse not in obj2.relations:
                    obj2.relations.append(reverse)
            else:
                if rel is not None and rel not in obj2.relations:
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
                add_relation(container, container2, container.inside(container2), 'atop')
                add_relation(container, container2, container.touching(container2), 'touching', sort=True)
                add_relation(container, container2, container.left_of(container2), 'left_of')
                add_relation(container, container2, container.right_of(container2), 'right_of')
                add_relation(container, container2, container.behind(container2), 'behind')
                add_relation(container, container2, container.in_front_of(container2), 'in_front_of')
                add_relation(container, container2, container.on(container2), 'on', sort=True)
                add_relation(container, container2, container.above(container2), 'above')
                add_relation(container, container2, container.below(container2), 'below')
                add_relation(container, container2, container.level_with(container2), 'level_with', sort=True)

            # container-box relations
            for b in self.boxes.values():
                add_relation(container, b, container.atop(b), 'atop')
                add_relation(container, b, container.inside(b), 'inside')
                add_relation(container, b, container.touching(b), 'touching')
                add_relation(container, b, container.left_of(b), 'left_of')
                add_relation(container, b, container.right_of(b), 'right_of')
                add_relation(container, b, container.behind(b), 'behind')
                add_relation(container, b, container.in_front_of(b), 'in_front_of')
                add_relation(container, b, container.on(b), 'on')
                add_relation(container, b, container.above(b), 'above')
                add_relation(container, b, container.below(b), 'below')
                add_relation(container, b, container.level_with(b), 'level_with')

            # container-drawer relations
            for d in self.drawers.values():
                add_relation(container, d, container.atop(d), 'atop')
                add_relation(container, d, container.inside(d), 'inside')
                add_relation(container, d, container.touching(d), 'touching')
                add_relation(container, d, container.left_of(d), 'left_of')
                add_relation(container, d, container.right_of(d), 'right_of')
                add_relation(container, d, container.behind(d), 'behind')
                add_relation(container, d, container.in_front_of(d), 'in_front_of')
                add_relation(container, d, container.on(d), 'on')
                add_relation(container, d, container.above(d), 'above')
                add_relation(container, d, container.below(d), 'below')
                add_relation(container, d, container.level_with(d), 'level_with')

            # container-lid relations
            for l in self.lids.values():
                add_relation(container, l, container.atop(l), 'atop')
                add_relation(container, l, container.touching(l), 'touching')
                add_relation(container, l, container.left_of(l), 'left_of')
                add_relation(container, l, container.right_of(l), 'right_of')
                add_relation(container, l, container.behind(l), 'behind')
                add_relation(container, l, container.in_front_of(l), 'in_front_of')
                add_relation(container, l, container.on(l), 'on')
                add_relation(container, l, container.above(l), 'above')
                add_relation(container, l, container.below(l), 'below')
                add_relation(container, l, container.level_with(l), 'level_with')

            # container-stack relations
            for s in self.stacks.values():
                add_relation(container, s, container.atop(s), 'atop')
                add_relation(container, s, container.touching(s), 'touching')
                add_relation(container, s, container.left_of(s), 'left_of')
                add_relation(container, s, container.right_of(s), 'right_of')
                add_relation(container, s, container.behind(s), 'behind')
                add_relation(container, s, container.in_front_of(s), 'in_front_of')
                add_relation(container, s, container.on(s), 'on')
                add_relation(container, s, container.above(s), 'above')
                add_relation(container, s, container.below(s), 'below')
                add_relation(container, s, container.level_with(s), 'level_with')

        for i in range(len(self.lids)):
            lid = self.lids[self.lids.keys()[i]]

            # lid-lid relations
            for j in range(i + 1, len(self.lids)):
                lid2 = self.lids[self.lids.keys()[j]]
                add_relation(lid, lid2, lid.inside(lid2), 'atop')
                add_relation(lid, lid2, lid.touching(lid2), 'touching', sort=True)
                add_relation(lid, lid2, lid.left_of(lid2), 'left_of')
                add_relation(lid, lid2, lid.right_of(lid2), 'right_of')
                add_relation(lid, lid2, lid.behind(lid2), 'behind')
                add_relation(lid, lid2, lid.in_front_of(lid2), 'in_front_of')
                add_relation(lid, lid2, lid.on(lid2), 'on', sort=True)
                add_relation(lid, lid2, lid.above(lid2), 'above')
                add_relation(lid, lid2, lid.below(lid2), 'below')
                add_relation(lid, lid2, lid.level_with(lid2), 'level_with', sort=True)

            # lid-box relations
            for b in self.boxes.values():
                add_relation(lid, b, lid.closing(b), 'closing')
                add_relation(lid, b, lid.atop(b), 'atop')
                add_relation(lid, b, lid.touching(b), 'touching')
                add_relation(lid, b, lid.left_of(b), 'left_of')
                add_relation(lid, b, lid.right_of(b), 'right_of')
                add_relation(lid, b, lid.behind(b), 'behind')
                add_relation(lid, b, lid.in_front_of(b), 'in_front_of')
                add_relation(lid, b, lid.on(b), 'on')
                add_relation(lid, b, lid.above(b), 'above')
                add_relation(lid, b, lid.below(b), 'below')
                add_relation(lid, b, lid.level_with(b), 'level_with')

            # lid-drawer relations
            for d in self.drawers.values():
                add_relation(lid, d, lid.atop(d), 'atop')
                add_relation(lid, d, lid.touching(d), 'touching')
                add_relation(lid, d, lid.left_of(d), 'left_of')
                add_relation(lid, d, lid.right_of(d), 'right_of')
                add_relation(lid, d, lid.behind(d), 'behind')
                add_relation(lid, d, lid.in_front_of(d), 'in_front_of')
                add_relation(lid, d, lid.on(d), 'on')
                add_relation(lid, d, lid.above(d), 'above')
                add_relation(lid, d, lid.below(d), 'below')
                add_relation(lid, d, lid.level_with(d), 'level_with')

            # lid-stack relations
            for s in self.stacks.values():
                add_relation(lid, s, lid.atop(s), 'atop')
                add_relation(lid, s, lid.touching(s), 'touching')
                add_relation(lid, s, lid.left_of(s), 'left_of')
                add_relation(lid, s, lid.right_of(s), 'right_of')
                add_relation(lid, s, lid.behind(s), 'behind')
                add_relation(lid, s, lid.in_front_of(s), 'in_front_of')
                add_relation(lid, s, lid.on(s), 'on')
                add_relation(lid, s, lid.above(s), 'above')
                add_relation(lid, s, lid.below(s), 'below')
                add_relation(lid, s, lid.level_with(s), 'level_with')

        for i in range(len(self.drawers)):
            drawer = self.drawers[self.drawers.keys()[i]]

            # drawer-drawer relations
            for j in range(i + 1, len(self.drawers)):
                drawer2 = self.drawers[self.drawers.keys()[j]]
                add_relation(drawer, drawer2, drawer.touching(drawer2), 'touching', sort=True)

            # drawer-box relations
            for b in self.boxes.values():
                add_relation(drawer, b, drawer.touching(b), 'touching')

            # drawer-stack relations
            for s in self.stacks.values():
                add_relation(drawer, s, drawer.closing(s), 'closing')
                add_relation(drawer, s, drawer.touching(s), 'touching')

        for i in range(len(self.grippers)):
            gripper = self.grippers[self.grippers.keys()[i]]

            # gripper-gripper relations
            for j in range(i + 1, len(self.grippers)):
                gripper2 = self.grippers[self.grippers.keys()[j]]
                add_relation(gripper, gripper2, gripper.touching(gripper2), 'touching', sort=True)
                add_relation(gripper, gripper2, gripper.left_of(gripper2), 'left_of')
                add_relation(gripper, gripper2, gripper.right_of(gripper2), 'right_of')
                add_relation(gripper, gripper2, gripper.behind(gripper2), 'behind')
                add_relation(gripper, gripper2, gripper.in_front_of(gripper2), 'in_front_of')
                add_relation(gripper, gripper2, gripper.on(gripper2), 'on', sort=True)
                add_relation(gripper, gripper2, gripper.above(gripper2), 'above')
                add_relation(gripper, gripper2, gripper.below(gripper2), 'below')
                add_relation(gripper, gripper2, gripper.level_with(gripper2), 'level_with', sort=True)

            # gripper-item relations
            for item in self.items.values():
                add_relation(gripper, item, gripper.inside(item), 'inside')
                add_relation(gripper, item, gripper.touching(item), 'touching')
                add_relation(gripper, item, gripper.left_of(item), 'left_of')
                add_relation(gripper, item, gripper.right_of(item), 'right_of')
                add_relation(gripper, item, gripper.behind(item), 'behind')
                add_relation(gripper, item, gripper.in_front_of(item), 'in_front_of')
                add_relation(gripper, item, gripper.on(item), 'on')
                add_relation(gripper, item, gripper.above(item), 'above')
                add_relation(gripper, item, gripper.below(item), 'below')
                add_relation(gripper, item, gripper.level_with(item), 'level_with')

            # gripper-container relations
            for c in self.containers.values():
                add_relation(gripper, c, gripper.inside(c), 'inside')
                add_relation(gripper, c, gripper.touching(c), 'touching')
                add_relation(gripper, c, gripper.left_of(c), 'left_of')
                add_relation(gripper, c, gripper.right_of(c), 'right_of')
                add_relation(gripper, c, gripper.behind(c), 'behind')
                add_relation(gripper, c, gripper.in_front_of(c), 'in_front_of')
                add_relation(gripper, c, gripper.on(c), 'on')
                add_relation(gripper, c, gripper.above(c), 'above')
                add_relation(gripper, c, gripper.below(c), 'below')
                add_relation(gripper, c, gripper.level_with(c), 'level_with')

            # gripper-box relations
            for b in self.boxes.values():
                add_relation(gripper, b, gripper.inside(b), 'inside')
                add_relation(gripper, b, gripper.touching(b), 'touching')
                add_relation(gripper, b, gripper.left_of(b), 'left_of')
                add_relation(gripper, b, gripper.right_of(b), 'right_of')
                add_relation(gripper, b, gripper.behind(b), 'behind')
                add_relation(gripper, b, gripper.in_front_of(b), 'in_front_of')
                add_relation(gripper, b, gripper.on(b), 'on')
                add_relation(gripper, b, gripper.above(b), 'above')
                add_relation(gripper, b, gripper.below(b), 'below')
                add_relation(gripper, b, gripper.level_with(b), 'level_with')

            # item-drawer relations
            for d in self.drawers.values():
                add_relation(gripper, d, gripper.inside(d), 'inside')
                add_relation(gripper, d, gripper.touching(d), 'touching')
                add_relation(gripper, d, gripper.left_of(d), 'left_of')
                add_relation(gripper, d, gripper.right_of(d), 'right_of')
                add_relation(gripper, d, gripper.behind(d), 'behind')
                add_relation(gripper, d, gripper.in_front_of(d), 'in_front_of')
                add_relation(gripper, d, gripper.on(d), 'on')
                add_relation(gripper, d, gripper.above(d), 'above')
                add_relation(gripper, d, gripper.below(d), 'below')
                add_relation(gripper, d, gripper.level_with(d), 'level_with')

            # item-lid relations
            for l in self.lids.values():
                add_relation(gripper, l, gripper.touching(l), 'touching')
                add_relation(gripper, l, gripper.left_of(l), 'left_of')
                add_relation(gripper, l, gripper.right_of(l), 'right_of')
                add_relation(gripper, l, gripper.behind(l), 'behind')
                add_relation(gripper, l, gripper.in_front_of(l), 'in_front_of')
                add_relation(gripper, l, gripper.on(l), 'on')
                add_relation(gripper, l, gripper.above(l), 'above')
                add_relation(gripper, l, gripper.below(l), 'below')
                add_relation(gripper, l, gripper.level_with(l), 'level_with')

            # item-stack relations
            for s in self.stacks.values():
                add_relation(gripper, s, gripper.touching(s), 'touching')
                add_relation(gripper, s, gripper.left_of(s), 'left_of')
                add_relation(gripper, s, gripper.right_of(s), 'right_of')
                add_relation(gripper, s, gripper.behind(s), 'behind')
                add_relation(gripper, s, gripper.in_front_of(s), 'in_front_of')
                add_relation(gripper, s, gripper.on(s), 'on')
                add_relation(gripper, s, gripper.above(s), 'above')
                add_relation(gripper, s, gripper.below(s), 'below')
                add_relation(gripper, s, gripper.level_with(s), 'level_with')

    def clear_relations(self):
        self.relations = []

        for b in self.boxes.values():
            b.relations = []
        for c in self.containers.values():
            c.relations = []
        for d in self.drawers.values():
            d.relations = []
        for g in self.grippers.values():
            g.relations = []
        for i in self.items.values():
            i.relations = []
        for l in self.lids.values():
            l.relations = []
        for s in self.stacks.values():
            s.relations = []

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

        msg.relations = self.relations
        return msg

    def from_ros(self, msg):
        self.clear_state()

        for obj in msg.boxes:
            self.boxes[obj.unique_name] = obj

        for obj in msg.containers:
            self.containers[obj.unique_name] = obj

        for obj in msg.drawers:
            self.drawers[obj.unique_name] = obj

        for obj in msg.grippers:
            self.grippers[obj.unique_name] = obj

        for obj in msg.items:
            self.items[obj.unique_name] = obj

        for obj in msg.lids:
            self.lids[obj.unique_name] = obj

        for obj in msg.stacks:
            self.stacks[obj.unique_name] = obj

        return self
