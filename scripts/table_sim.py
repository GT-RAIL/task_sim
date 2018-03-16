#!/usr/bin/env python

# Python
import copy
from math import floor
from math import sqrt
from random import seed
from random import shuffle
from random import randint
from random import random

# ROS
import rospy
from numpy import sign
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

from task_sim import data_utils as DataUtils
from task_sim.srv import Execute, ExecuteResponse, QueryState, RequestIntervention, RequestInterventionResponse
from task_sim.msg import Action, State, Object, SmallContainer, Log
from task_sim.grasp_state import GraspState
from task_sim.plan_action import PlanAction

from task_sim.oomdp.oo_state import OOState

class TableSim:

    def __init__(self):
        self.error = ''

        self.action_service_ = rospy.Service('~execute_action', Execute, self.execute)
        self.state_service_ = rospy.Service('~query_state', QueryState, self.query_state)
        self.intervention_service_ = rospy.Service('~request_intervention', RequestIntervention, self.request_intervention)
        self.reset_service_ = rospy.Service('~reset_simulation', Empty, self.reset_sim)
        self.log_pub_ = rospy.Publisher('~task_log', Log, queue_size=1)

        self.quiet_mode = rospy.get_param('~quiet_mode', False)
        self.terminal_input = rospy.get_param('~terminal_input', True)
        self.history_buffer = rospy.get_param('~history_buffer', 10)

        self.sim_seed = rospy.get_param('~seed', None)
        if self.sim_seed == -1:
            self.sim_seed = None

        self.init_simulation(rand_seed = self.sim_seed, level = 1)
        #self.worldUpdate()

        # debug
        self.prev_state = None


    def query_state(self, req):
        return self.state_

    def reset_sim(self, req):
        # In case we want to reset to a different world
        self.sim_seed = rospy.get_param('~seed', None)
        if self.sim_seed == -1:
            self.sim_seed = None

        self.init_simulation(self.sim_seed, level=1)
        self.worldUpdate()
        return EmptyResponse()

    def request_intervention(self, req):
        res = RequestInterventionResponse()

        if self.terminal_input:
            print 'Interventions are not supported in terminal input mode.'
            return res

        loop_rate = rospy.Rate(30)

        while True:
            action = self.getInput()
            if action is not None:
                res.actions.append(action)
                loop_rate.sleep()
            else:
                break
        return res


    def init_simulation(self, rand_seed = None, level = 0):
        """Create the initial world configuration

        Keyword arguments:
        seed -- random seed for consistent starting situations
        level -- level of randomization, as follows:
            0 : randomize objects only
            1 : randomize objects and drawer/box positions, but not drawer theta
            2 : randomize objects, drawer and box dimensions and positions
        """
        seed(rand_seed)

        self.state_ = State()

        # Empty action history
        for i in range(self.history_buffer):
            self.state_.action_history.append(Action.NOOP)
            self.state_.result_history.append(True)

        # Table properties
        self.tableWidth = 40
        self.tableDepth = 15

        # Container properties
        if level >= 2:
            self.boxRadius = randint(1,4)
            self.boxHeight = randint(1,4)
            self.drawerWidth = randint(3,9)
            self.drawerDepth = randint(3,9)
            if self.drawerWidth % 2 == 0:
                self.drawerWidth += 1
            if self.drawerDepth % 2 == 0:
                self.drawerDepth += 1
            self.drawerHeight = randint(2, 3)
        else:
            self.boxRadius = 2
            self.boxHeight = 1
            self.drawerWidth = 5
            self.drawerDepth = 7
            self.drawerHeight = 2

        # Container positions
        if level >= 1:
            drawer_set = False
            while not drawer_set:
                self.state_.drawer_position.x = randint(self.drawerWidth/2 + 1,
                                                        self.tableWidth - (self.drawerWidth/2 + 1))
                self.state_.drawer_position.y = randint(self.drawerDepth/2 + 1,
                                                        self.tableDepth - (self.drawerDepth/2 + 1))
                self.state_.drawer_position.theta = randint(0, 3)*90 if level >= 2 else 0

                xmin, xmax, ymin, ymax, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer = self.getDrawerBounds()
                drawer_set = self.onTable(Point(xmin, ymin, self.drawerHeight)) \
                             and self.onTable(Point(xmin, ymax, self.drawerHeight)) \
                             and self.onTable(Point(xmax, ymin, self.drawerHeight)) \
                             and self.onTable(Point(xmax, ymax, self.drawerHeight)) \
                             and self.onTable(Point(xmaxDrawer, yminDrawer, self.drawerHeight - 1)) \
                             and self.onTable(Point(xmaxDrawer, ymaxDrawer, self.drawerHeight - 1))

                drawer_points = self.getDrawerValidPoints()
                for point in drawer_points:
                    drawer_set = drawer_set and self.reachable(point)

            self.state_.drawer_opening = randint(0, self.drawerDepth - 1)

            box_set = False
            while not box_set:
                self.state_.box_position.x = randint(self.boxRadius + 1,
                                                     self.tableWidth - (self.boxRadius + 1))
                self.state_.box_position.y = randint(self.boxRadius + 1,
                                                     self.tableDepth - (self.boxRadius + 1))
                self.state_.lid_position.x = self.state_.box_position.x
                self.state_.lid_position.y = self.state_.box_position.y
                self.state_.lid_position.z = self.boxHeight

                xmin = self.state_.box_position.x - self.boxRadius
                xmax = self.state_.box_position.x + self.boxRadius
                ymin = self.state_.box_position.y - self.boxRadius
                ymax = self.state_.box_position.y + self.boxRadius
                box_set = self.onTable(Point(xmin, ymin, 0)) and self.onTable(Point(xmin, ymax, 0)) \
                          and self.onTable(Point(xmax, ymin, 0)) and self.onTable(Point(xmax, ymax, 0)) \
                          and self.reachable(self.state_.lid_position) \
                          and self.euclidean2D(self.state_.box_position.x, self.state_.box_position.y,
                                               self.state_.drawer_position.x, self.state_.drawer_position.y) \
                              >= max(self.drawerWidth, self.drawerDepth)*3/2 + self.boxRadius + 2
        else:
            self.state_.drawer_position.x = 10
            self.state_.drawer_position.y = 12
            self.state_.drawer_position.theta = 0
            self.state_.drawer_opening = 3

            self.state_.box_position.x = 30
            self.state_.box_position.y = 6
            self.state_.lid_position.x = 30
            self.state_.lid_position.y = 6
            self.state_.lid_position.z = 1

        # Objects
        obj1 = Object()
        obj1.name = "apple"
        # handle name
        obj1.unique_name = obj1.name
        mod = 0
        for o in self.state_.objects:
            if o.name == obj1.name:
                mod += 1
        obj1.unique_name += str(mod)
        # place object
        object_set = False
        while not object_set:
            obj1.position.x = randint(1, self.tableWidth - 1)
            obj1.position.y = randint(1, self.tableDepth - 1)
            obj1.position.z = 0
            object_set = not self.inCollision(obj1.position) and self.reachable(obj1.position)
        self.state_.objects.append(obj1)

        obj2 = Object()
        obj2.name = "batteries"
        # handle name
        obj2.unique_name = obj2.name
        mod = 0
        for o in self.state_.objects:
            if o.name == obj2.name:
                mod += 1
        obj2.unique_name += str(mod)
        # place object
        object_set = False
        while not object_set:
            obj2.position.x = randint(1, self.tableWidth - 1)
            obj2.position.y = randint(1, self.tableDepth - 1)
            obj2.position.z = 0
            object_set = not self.inCollision(obj2.position) and self.reachable(obj2.position)
        self.state_.objects.append(obj2)

        obj3 = Object()
        obj3.name = "flashlight"
        # handle name
        obj3.unique_name = obj3.name
        mod = 0
        for o in self.state_.objects:
            if o.name == obj3.name:
                mod += 1
        obj3.unique_name += str(mod)
        # place object
        object_set = False
        while not object_set:
            obj3.position.x = randint(1, self.tableWidth - 1)
            obj3.position.y = randint(1, self.tableDepth - 1)
            obj3.position.z = 0
            object_set = not self.inCollision(obj3.position) and self.reachable(obj3.position)
        self.state_.objects.append(obj3)

        obj4 = Object()
        obj4.name = "granola"
        # handle name
        obj4.unique_name = obj4.name
        mod = 0
        for o in self.state_.objects:
            if o.name == obj4.name:
                mod += 1
        obj4.unique_name += str(mod)
        # place object
        object_set = False
        while not object_set:
            obj4.position.x = randint(1, self.tableWidth - 1)
            obj4.position.y = randint(1, self.tableDepth - 1)
            obj4.position.z = 0
            object_set = not self.inCollision(obj4.position) and self.reachable(obj4.position)
        self.state_.objects.append(obj4)

        obj5 = Object()
        obj5.name = "knife"
        # handle name
        obj5.unique_name = obj5.name
        mod = 0
        for o in self.state_.objects:
            if o.name == obj5.name:
                mod += 1
        obj5.unique_name += str(mod)
        # place object
        object_set = False
        while not object_set:
            obj5.position.x = randint(1, self.tableWidth - 1)
            obj5.position.y = randint(1, self.tableDepth - 1)
            obj5.position.z = 0
            object_set = not self.inCollision(obj5.position) and self.reachable(obj5.position)
        self.state_.objects.append(obj5)

        # Containers
        def place_container(c):
            # handle name
            c.unique_name = c.name
            mod = 0
            for cont in self.state_.containers:
                if cont.name == c.name:
                    mod += 1
            c.unique_name += str(mod)
            # place container
            container_set = False
            while not container_set:
                container_set = True
                c.position.x = randint(1, self.tableWidth - c.width)
                c.position.y = randint(1, self.tableDepth - c.height)
                c.position.z = 0
                for x in range(c.width):
                    for y in range(c.height):
                        container_set = container_set and \
                                        not self.inCollision(Point(c.position.x + x, c.position.y + y, c.position.z)) \
                                        and self.reachable(Point(c.position.x + x, c.position.y + y, c.position.z)) \
                                        and not self.inContainer(Point(c.position.x + x, c.position.y + y, c.position.z))
            self.state_.containers.append(c)

        c1 = SmallContainer()
        c1.name = "small"
        c1.width = 2
        c1.height = 2
        place_container(c1)

        c2 = SmallContainer()
        c2.name = "small"
        c2.width = 2
        c2.height = 2
        place_container(c2)

        c3 = SmallContainer()
        c3.name = "large"
        c3.width = 3
        c3.height = 3
        place_container(c3)

        # Initial robot configuration (home)
        self.state_.gripper_position.x = 8
        self.state_.gripper_position.y = 1
        self.state_.gripper_position.z = 2
        self.state_.gripper_open = True

        # Hidden state
        self.grasp_states = {}
        for object in self.state_.objects:
            self.grasp_states[object.unique_name] = GraspState(self.getNeighborCount(object.position),
                                                        self.copyPoint(object.position))


    # TODO: container version
    def getNeighborCount(self, position):
        return DataUtils.get_neighbor_count(
            self.state_, position,
            self.boxRadius, self.boxHeight,
            self.drawerWidth, self.drawerDepth, self.drawerHeight
        )


    def worldUpdate(self, action=None):
        # Perform action
        result = False
        if action:
            if action.action_type == Action.GRASP:
                result = self.grasp(action.object)
            elif action.action_type == Action.PLACE:
                result = self.place(action.position)
            elif action.action_type == Action.OPEN_GRIPPER:
                result = self.open()
            elif action.action_type == Action.CLOSE_GRIPPER:
                result = self.close()
            elif action.action_type == Action.MOVE_ARM:
                result = self.move(action.position)
            elif action.action_type == Action.RAISE_ARM:
                result = self.raiseArm()
            elif action.action_type == Action.LOWER_ARM:
                result = self.lowerArm()
            elif action.action_type == Action.RESET_ARM:
                result = self.resetArm()

        # Additional state updates
        self.gravity()
        # TODO: container grasp state
        self.updateContainerStates()
        for container in self.state_.containers:
            container.lost = True
            for x in range(container.position.x, container.position.x + container.width):
                for y in range(container.position.y, container.position.y + container.height):
                    container.lost = container.lost and (x <= 0 or x >= self.tableWidth 
                                                         or y <= 0 or y >= self.tableDepth) \
                                     and not self.state_.object_in_gripper == container.unique_name \
                                     and not container.on_lid
        self.updateObjectStates()
        for object in self.state_.objects:
            object.lost = (object.position.x <= 0 or object.position.x >= self.tableWidth
                           or object.position.y <= 0 or object.position.y >= self.tableDepth) \
                          and not self.state_.object_in_gripper == object.unique_name and not object.on_lid
            if object.lost:
                for c in self.state_.containers:
                    for o_name in c.contains:
                        if o_name == object.unique_name and not c.lost:
                            object.lost = False
            if not object.lost:
                self.grasp_states[object.unique_name].updateGraspRate(self.getNeighborCount(object.position),
                                                               self.copyPoint(object.position))

        # Update state history
        if action is not None and action.action_type != Action.NOOP and self.history_buffer > 0:
            self.state_.action_history.append(action.action_type)
            self.state_.result_history.append(result)
            self.state_.action_history = self.state_.action_history[-self.history_buffer:]
            self.state_.result_history = self.state_.result_history[-self.history_buffer:]


        # Create a log message and send it along
        log_msg = Log(
            action=(action or Action(action_type=Action.NOOP)),
            state=self.state_
        )
        self.log_pub_.publish(log_msg)

        self.show()

        # # debug
        # state = OOState(self.state_)
        # state.relations.sort()
        # for rel in state.relations:
        #     print rel
        # # print DataUtils.readable_state(DataUtils.semantic_state_vector(self.state_, return_dict=True)[0])
        # if self.prev_state is not None:
        #     pa = PlanAction(self.prev_state, (action or Action(action_type=Action.NOOP)), self.state_)
        #     print str(pa)
        # self.prev_state = copy.deepcopy(self.state_)

    def execute(self, req):
        """Handle execution of all robot actions as a ROS service routine"""
        req.action.position.x = int(req.action.position.x)
        req.action.position.y = int(req.action.position.y)
        req.action.position.z = int(req.action.position.z)
        self.worldUpdate(req.action)
        return ExecuteResponse(state=self.state_)


    def grasp(self, object):
        """Move the gripper to an object and grasp it

        Keyword arguments:
        object - name of the object to grasp

        Return: success/failure of action
        """
        if not self.state_.gripper_open:
            self.open()

        # special cases
        if object == 'lid':
            if not self.motionPlanChance(self.state_.lid_position):
                self.error = 'Motion planner failed.'
                return False
            if DataUtils.get_object_at(self.state_, self.state_.lid_position):
                self.error = 'Lid is occluded and cannot be grasped.'
                return False
            self.state_.gripper_position = self.copyPoint(self.state_.lid_position)
            self.state_.gripper_open = False
            self.state_.object_in_gripper = 'lid'
            return True
        elif object == 'drawer' or object == 'drawer':
            if not self.motionPlanChance(self.getDrawerHandle()):
                self.error = 'Motion planner failed.'
                return False
            self.state_.gripper_position = self.copyPoint(self.getDrawerHandle())
            self.state_.gripper_open = False
            self.state_.object_in_gripper = 'drawer'
            return True

        target = DataUtils.get_object_by_name(self.state_, object)
        if target:
            if target.lost:
                self.error = target.name + ' is lost.'
                return False

            if target.occluded:
                self.error = target.name + ' is occluded and cannot be grasped.'
                return False

            if not self.grasp_states[target.unique_name].graspable:
                self.error = 'Could not find a grasp for ' + target.name + '.'
                return False

            if not self.motionPlanChance(target.position):
                self.error = 'Motion planner failed.'
                return False

            self.state_.gripper_position = self.copyPoint(target.position)
            self.state_.gripper_open = False
            self.state_.object_in_gripper = object
            return True
        else:
            target = DataUtils.get_container_by_name(self.state_, object)
            if target:
                if target.lost:
                    self.error = target.name + ' is lost.'
                    return False

                if target.occluded:
                    self.error = target.name + ' is occluded and cannot be grasped.'
                    return False

                # TODO: grasp state for containers

                # pick a point to grasp
                points = []
                for i in range(target.width):
                    for j in range(target.height):
                        if i == 0 or i == target.width - 1 or j == 0 or j == target.height - 1:
                            if self.getOutput(self.output, target.position.x + i + 1, target.position.y + j) != '0':
                                continue
                            if target.position.x + i < 0 or target.position.x + i > self.tableWidth \
                                    or target.position.y + j < 0 or target.position.y + j > self.tableDepth:
                                continue
                            points.append(Point(target.position.x + i, target.position.y + j, target.position.z))
                if len(points) == 0:
                    self.error = 'could not find graspable point for ' + target.name
                    return False

                shuffle(points)
                point = points[0]
                if not self.motionPlanChance(point):
                    self.error = 'Motion planner failed.'
                    return False

                self.state_.gripper_position = point
                self.state_.gripper_open = False
                self.state_.object_in_gripper = object
            else:
                self.error = 'Object ' + object + ' does not exist.'
                return False


    def motionPlanChance(self, target):
        """Determine if a motion planner will succeed or fail

        Keyword arguments:
        target -- goal position for the planner
        """
        if not self.reachable(target):
            return False
        dst = self.euclidean3D(self.state_.gripper_position, target)
        chance = min(1 - (dst - 10)/50.0, 1)
        blocked = 0
        free = 0
        for z in range(0, 2):
            for x in range(-2, 3):
                for y in range(-2, 3):
                    if z == 0:  # Special case: ignore collision at gripper level if lid is in gripper
                        if self.state_.object_in_gripper == 'lid':
                            free += 1
                            continue
                    temp_pos = self.copyPoint(self.state_.gripper_position)
                    temp_pos.x += x
                    temp_pos.y += y
                    temp_pos.z += z
                    if self.inCollision(temp_pos):
                        blocked += 1
                    else:
                        free += 1
        chance = max(chance - 1.5*float(blocked)/(blocked + free), 0)
        # Special case: harder motion planning when holding objects of different sizes
        if self.state_.object_in_gripper == 'lid':
            chance *= 0.5
        elif self.state_.object_in_gripper in ['Apple', 'Batteries', 'Flashlight', 'Granola', 'Knife']:
            chance *= 0.8
        return random() < chance


    def euclidean3D(self, p1, p2):
        return sqrt(float(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)))


    def place(self, position):
        """Move the gripper to a pose just above the nearest object and release whatever is in the gripper

        Keyword arguments:
        position -- place position, uses only position.x and position.y since position.z is calculated
        """
        # Special case: drawer
        if self.state_.object_in_gripper == 'drawer':
            self.error = 'Cannot execute place while grasping drawer.'
            return False

        height = 4
        tempPos = Point(position.x, position.y, 0)
        for i in range(3,-1,-1):
            tempPos.z = i
            # lid case
            if self.state_.object_in_gripper == 'lid':
                collision = False
                for x in range(tempPos.x - self.boxRadius, tempPos.x + self.boxRadius + 1):
                    for y in range(tempPos.y - self.boxRadius, tempPos.y + self.boxRadius + 1):
                        if self.checkLidCollision(Point(x, y, tempPos.z)):
                            collision = True
                            break
                    if collision:
                        break
                if collision:
                    break
            else:
                # Container case
                if self.state_.object_in_gripper != '':
                    c = DataUtils.get_container_by_name(self.state_, self.state_.object_in_gripper)
                    if c is not None:
                        collision = False
                        for x in range(c.width):
                            for y in range(c.height):
                                collision = self.inCollision(Point(tempPos.x + x, tempPos.y + y, tempPos.z),
                                                             c.unique_name)
                                if collision:
                                    break
                            if collision:
                                break
                        if collision:
                            break
                    else:
                        # Regular place
                        if self.inCollision(tempPos):
                            break
                else:
                    # Regular place
                    if self.inCollision(tempPos):
                        break
            height -= 1
        if height == 4:  # place failed
            return False

        tempPos.z = height

        if not self.motionPlanChance(tempPos):
            self.error = 'Motion planner failed.'
            return False
        self.moveGripper(tempPos)
        self.open()
        return True


    def open(self):
        """Open the gripper, dropping anything currently in it"""
        self.state_.gripper_open = True
        if self.state_.object_in_gripper != '':
            if self.state_.object_in_gripper == 'lid':
                self.state_.object_in_gripper = ''
                self.gravity()
            elif self.state_.object_in_gripper == 'drawer':
                self.state_.object_in_gripper = ''
            else:
                object = DataUtils.get_object_by_name(self.state_, self.state_.object_in_gripper)
                if object is None:
                    object = DataUtils.get_container_by_name(self.state_, self.state_.object_in_gripper)
                self.state_.object_in_gripper = ''
                self.gravity()
        return True


    def close(self):
        """Close the gripper, grasping anything at its current location"""
        if self.state_.gripper_open:
            self.state_.gripper_open = False
            o = DataUtils.get_object_at(self.state_, self.state_.gripper_position)
            if o:
                self.state_.object_in_gripper = o.unique_name
            else:
                # Handle special cases
                if self.state_.gripper_position == self.state_.lid_position:
                    self.state_.object_in_gripper = 'lid'
                elif self.state_.gripper_position == self.getDrawerHandle():
                    self.state_.object_in_gripper = 'drawer'
                else:
                    for c in self.state_.containers:
                        if self.inSpecificContainer(self.state_.gripper_position, c):
                            if (self.state_.gripper_position.x == c.position.x or
                                        self.state_.gripper_position.x == c.position.x + c.width - 1 or
                                        self.state_.gripper_position.y == c.position.y or
                                        self.state_.gripper_position.y == c.position.y + c.height - 1):
                                self.state_.object_in_gripper = c.unique_name
                                return True
        return True


    def move(self, position):
        """Move the gripper in a straight line, allowing object collisions

        Keyword arguments:
        position -- point to move to, uses only position.x and position.y as this is a planar move (for ease of sim)
        """
        points = self.interpolate(self.state_.gripper_position.x, self.state_.gripper_position.y, position.x,
                                  position.y)
        start = self.copyPoint(self.state_.gripper_position)
        goal = self.copyPoint(self.state_.gripper_position)
        c = DataUtils.get_container_by_name(self.state_, self.state_.object_in_gripper)
        dx = 0
        dy = 0
        if c is not None:
            dx = c.position.x - self.state_.gripper_position.x
            dy = c.position.y - self.state_.gripper_position.y
        for point in points:
            testPos = Point(int(floor(point[0] + 0.5)), int(floor(point[1] + 0.5)), self.state_.gripper_position.z)
            # handle special cases
            if self.state_.object_in_gripper == 'lid':
                # Test each corner for collision as well
                corner1 = Point(testPos.x + self.boxRadius, testPos.y + self.boxRadius, testPos.z)
                corner2 = Point(testPos.x + self.boxRadius, testPos.y - self.boxRadius, testPos.z)
                corner3 = Point(testPos.x - self.boxRadius, testPos.y + self.boxRadius, testPos.z)
                corner4 = Point(testPos.x - self.boxRadius, testPos.y - self.boxRadius, testPos.z)
                if self.environmentWithoutLidCollision(testPos) \
                        or self.environmentWithoutLidCollision(corner1) \
                        or self.environmentWithoutLidCollision(corner2) \
                        or self.environmentWithoutLidCollision(corner3) \
                        or self.environmentWithoutLidCollision(corner4):
                    break
                if not self.reachable(testPos):
                    break
            elif self.state_.object_in_gripper == 'drawer':
                if not any(valid_point == testPos for valid_point in self.getDrawerValidPoints()):
                    break
                if self.environmentWithoutDrawerCollision(testPos):
                    break
                if not self.reachable(testPos):
                    break
            elif c is not None:  # container in gripper
                if not self.reachable(testPos):
                    break

                # drawer collision
                collision = False
                for i in range(c.width):
                    for j in range(c.height):
                        checkPos = Point(testPos.x + dx + i, testPos.y + dy + j, testPos.z)
                        drawer_collisions = self.drawerCollision(checkPos)
                        # special case: colliding with protruding drawer only
                        if (drawer_collisions[1] or drawer_collisions[2]) and not drawer_collisions[0]:
                            if self.openingDrawer(checkPos):
                                if not self.pushOpen():
                                    collision = True
                            elif self.closingDrawer(checkPos):
                                if not self.pushClosed():
                                    collision = True
                            else:
                                collision = True
                        if collision:
                            break
                    if collision:
                        break
                if collision:
                    break

                # check for environment collision for all drawer points
                for i in range(c.width):
                    for j in range(c.height):
                        collision = self.environmentCollision(Point(testPos.x + dx + i, testPos.y + dy + j, testPos.z))
                        if collision:
                            break
                    if collision:
                        break
                if collision:
                    break
            else:
                if not self.reachable(testPos):
                    break
                drawer_collisions = self.drawerCollision(testPos)
                # special case: colliding with protruding drawer only
                if (drawer_collisions[1] or drawer_collisions[2]) and not drawer_collisions[0]:
                    if self.openingDrawer(testPos):
                        if not self.pushOpen():
                            break
                    elif self.closingDrawer(testPos):
                        if not self.pushClosed():
                            break
                    else:
                        break
                if self.environmentCollision(testPos):
                    break

            goal = self.copyPoint(testPos)

        self.moveGripper(goal)

        for container in self.state_.containers:
            if c is not None and c.unique_name == container.unique_name:
                continue
            if container.position.z != start.z:
                continue
            dst = 999
            for i in range(container.width):
                for j in range(container.height):
                    if c is not None:
                        for x in range(c.width):
                            for y in range(c.height):
                                new_dst = self.distanceFromPath(container.position.x + i, container.position.y + j,
                                                                start.x + dx + x, start.y + dy + y,
                                                                goal.x + dx + x, goal.y + dy + y)
                                if new_dst < dst:
                                    dst = new_dst
                    else:
                        new_dst = self.distanceFromPath(container.position.x + i, container.position.y + j,
                                                        start.x, start.y, goal.x, goal.y)
                        if new_dst < dst:
                            dst = new_dst
            if dst < 1.2:
                center = self.copyPoint(goal)
                cont_x = int(floor(container.position.x + container.width/2.0))
                cont_y = int(floor(container.position.y + container.height/2.0))
                x_off = 0
                y_off = 0
                if cont_x - start.x > 0:
                    x_off = 2
                elif cont_x - start.x < 0:
                    x_off = -2 - container.width + 1
                else:
                    x_off = -int(floor(container.width/2.0))
                if cont_y - start.y > 0:
                    y_off = 2
                elif cont_y - start.y < 0:
                    y_off = -2 - container.height + 1
                else:
                    y_off = -int(floor(container.height/2.0))
                center.x += x_off
                center.y += y_off
                container_goal = self.randomFreePoint(center, 1, 1, container.width, container.height,
                                                      avoid_containers=True)
                if container_goal:
                    container_points = self.interpolate(container.position.x, container.position.y,
                                                        container_goal.x, container_goal.y)
                    for point in container_points:
                        testPos = Point(int(floor(point[0] + 0.5)), int(floor(point[1] + 0.5)), start.z)
                        if testPos.x == container.position.x and testPos.y == container.position.y \
                                and testPos.z == container.position.z:
                            continue
                        collision = False
                        for i in range(container.width):
                            for j in range(container.height):
                                checkPos = Point(testPos.x + i, testPos.y + j, testPos.z)
                                collision =  self.environmentCollision(checkPos)
                                if collision:
                                    break
                            if collision:
                                break
                        if collision:
                            break
                        self.moveContainer(container, testPos.x - container.position.x,
                                           testPos.y - container.position.y, testPos.z - container.position.z)
                        if self.gravity(container=container):
                            break

        for object in self.state_.objects:
            if object.lost:
                continue
            if object.unique_name == self.state_.object_in_gripper:
                continue
            if (self.state_.object_in_gripper == 'drawer' and object.in_drawer) or \
                    (self.state_.object_in_gripper == 'lid' and object.on_lid):
                continue
            if object.position.z == start.z:
                cont = False
                for c in self.state_.containers:
                    for o_name in c.contains:
                        cont = o_name == object.unique_name
                        if cont:
                            break
                    if cont:
                        break
                if cont:
                    continue
                dst = self.distanceFromPath(object.position.x, object.position.y, start.x, start.y, goal.x, goal.y)
                if c is not None:
                    for i in range(c.width):
                        for j in range(c.height):
                            new_dst = self.distanceFromPath(object.position.x, object.position.y,
                                                            start.x + dx + i, start.y + dy + j,
                                                            goal.x + dx + i, goal.y + dy + j)
                            if new_dst < dst:
                                dst = new_dst
                if dst < 1.2:
                    center = self.copyPoint(goal)
                    xRadius = 2
                    yRadius = 2
                    goaldx = goal.x - start.x
                    goaldy = goal.y - start.y
                    if c is not None:
                        minx = min([center.x, center.x + dx])
                        maxx = min([center.x, center.x + dx + c.width - 1])
                        miny = min([center.y, center.y + dy])
                        maxy = min([center.y, center.y + dy + c.height - 1])
                        if goaldx == 0:
                            center.x = int(floor((minx + maxx)/2.0 + 0.5))
                        elif goaldx > 0:
                            center.x = maxx
                        else:
                            center.x = minx
                        if goaldy == 0:
                            center.y = int(floor((miny + maxy)/2.0 + 0.5))
                        elif goaldy > 0:
                            center.y = maxy
                        else:
                            center.y = miny

                    object_goal = self.randomFreePoint(center, xRadius, yRadius, avoid_containers=True)
                    if object_goal:
                        object_points = self.interpolate(object.position.x, object.position.y, object_goal.x, object_goal.y)
                        for point in object_points:
                            testPos = Point(int(floor(point[0] + 0.5)), int(floor(point[1] + 0.5)), start.z)
                            if testPos.x == object.position.x and testPos.y == object.position.y \
                                    and testPos.z == object.position.z:
                                continue
                            if self.environmentCollision(testPos) \
                                    or (c is None and self.containerCollision(object.position, testPos)) \
                                    or (c is not None and self.containerCollision(object.position, testPos,
                                                                                  ignore=c.unique_name)):
                                break
                            object.position = self.copyPoint(testPos)
                            if self.gravity(object):
                                break
        return goal != start

    def openingDrawer(self, position):
        if self.state_.drawer_position.theta == 0:
            return position.y == self.state_.gripper_position.y and position.x > self.state_.gripper_position.x
        elif self.state_.drawer_position.theta == 90:
            return position.x == self.state_.gripper_position.x and position.y > self.state_.gripper_position.y
        elif self.state_.drawer_position.theta == 180:
            return position.y == self.state_.gripper_position.y and position.x < self.state_.gripper_position.x
        else:
           return position.x == self.state_.gripper_position.x and position.y < self.state_.gripper_position.y


    def pushOpen(self):
        test_point = self.copyPoint(self.getDrawerHandle())
        if self.state_.drawer_position.theta == 0:
            test_point.x += 1
        elif self.state_.drawer_position.theta == 90:
            test_point.y += 1
        elif self.state_.drawer_position.theta == 180:
            test_point.x -= 1
        else:
            test_point.y -= 1
        if any(valid_point == test_point for valid_point in self.getDrawerValidPoints()):
            self.updateDrawerOffset(test_point)
            return True
        return False


    def pushClosed(self):
        test_point = self.copyPoint(self.getDrawerHandle())
        if self.state_.drawer_position.theta == 0:
            test_point.x -= 1
        elif self.state_.drawer_position.theta == 90:
            test_point.y -= 1
        elif self.state_.drawer_position.theta == 180:
            test_point.x += 1
        else:
            test_point.y += 1
        if any(valid_point == test_point for valid_point in self.getDrawerValidPoints()):
            self.updateDrawerOffset(test_point)
            return True
        return False


    def closingDrawer(self, position):
        if self.state_.drawer_position.theta == 0:
            return position.y == self.state_.gripper_position.y and position.x < self.state_.gripper_position.x
        elif self.state_.drawer_position.theta == 90:
            return position.x == self.state_.gripper_position.x and position.y < self.state_.gripper_position.y
        elif self.state_.drawer_position.theta == 180:
            return position.y == self.state_.gripper_position.y and position.x > self.state_.gripper_position.x
        else:
           return position.x == self.state_.gripper_position.x and position.y > self.state_.gripper_position.y

    def interpolate(self, x0, y0, x1, y1, resolution = 30):
        """Calculate a set of points between two points

        Keyword arguments:
        (x0, y0) -- start point
        (x1, y1) -- end point
        resolution -- how many points to calculate, including end point (default 30)

        Returns:
        list of interpolated points (end point included, start point not included)
        """
        points = []
        if x0 == x1:
            # special case: vertical line
            step = float(y1 - y0) / resolution
            for n in range(1, resolution):
                points.append([x0, y0 + n*step])
        else:
            # general case
            step = float(x1 - x0) / resolution
            for n in range(1, resolution):
                x = x0 + n*step
                y = y0 + (x - x0)*float(y1 - y0)/(x1 - x0)
                points.append([x, y])
        points.append([x1, y1])
        return points


    def distanceFromLine(self, x, y, x1, y1, x2, y2):
        """Calculate the minimum distance from a point to a line (unused?)

        Keyword arguments:
        (x, y) -- the point
        (x1, y1) -- the first point defining the line
        (x2, y2) -- the second point defining the line

        Returns:
        distance
        """
        return abs(float((x2 - x1)*(y1 - y) - (x1 - x)*(y2 - y1))) \
               / sqrt(float(pow(x2 - x1, 2) + pow(y2 - y1, 2)))


    def distanceFromPath(self, x, y, x1, y1, x2, y2):
        """Calculate the minimum distance from a point to a line segment

        Keyword arguments:
        (x, y) -- the point
        (x1, y1) -- the first point defining the line
        (x2, y2) -- the second point defining the line

        Returns:
        distance
        """
        l2 = float(pow(x2 - x1, 2) + pow(y2 - y1, 2))
        if l2 == 0:
            return self.euclidean2D(x, y, x1, y1)
        t = max(0, min(1, ((x - x1)*(x2 - x1) + (y - y1)*(y2 - y1))/l2))
        xp = x1 + t*(x2 - x1)
        yp = y1 + t*(y2 - y1)
        return self.euclidean2D(x, y, xp, yp)


    def euclidean2D(self, x1, y1, x2, y2):
        """Calculate 2D euclidean distance"""
        return sqrt(float(pow(x2 - x1, 2) + pow(y2 - y1, 2)))


    def raiseArm(self):
        """Move the arm up one z-level"""
        if self.state_.object_in_gripper == 'drawer':
            self.error = 'Cannot raise arm while grasping drawer'
            return False
        if self.state_.gripper_position.z < 4:
            # special case: holding container
            if self.state_.object_in_gripper != '':
                c = DataUtils.get_container_by_name(self.state_, self.state_.object_in_gripper)
                if c is not None:
                    for i in range(c.width):
                        for j in range(c.height):
                            checkPos = Point(c.position.x + i,
                                             c.position.y + j,
                                             c.position.z + 1)
                            if any(self.drawerCollision(checkPos)) or \
                                    self.lidCollision(checkPos) or \
                                    self.containerCollision(self.state_.gripper_position, checkPos):
                                return False
            else:
                checkPos = Point(self.state_.gripper_position.x,
                                 self.state_.gripper_position.y,
                                 self.state_.gripper_position.z + 1)
                if any(self.drawerCollision(checkPos)) or \
                        self.lidCollision(checkPos) or \
                        self.containerCollision(self.state_.gripper_position, checkPos):
                    return False
            self.moveGripper(Point(self.state_.gripper_position.x,
                                   self.state_.gripper_position.y,
                                   self.state_.gripper_position.z + 1))
            return True
        return False


    def lowerArm(self):
        """Move the arm down one z-level"""
        if self.state_.object_in_gripper == 'drawer':
            self.error = 'Cannot lower arm while grasping drawer'
            return False
        if self.state_.gripper_position.z == 0:
            return False

        # special case: holding lid
        if self.state_.object_in_gripper == 'lid':
            for x in range(self.state_.lid_position.x - self.boxRadius,
                           self.state_.lid_position.x + self.boxRadius + 1):
                for y in range(self.state_.lid_position.y - self.boxRadius,
                               self.state_.lid_position.y + self.boxRadius + 1):
                    if self.lidCollision(Point(x, y, self.state_.lid_position.z - 1)):
                        return False
            self.moveGripper(Point(self.state_.gripper_position.x,
                                   self.state_.gripper_position.y,
                                   self.state_.gripper_position.z - 1))
            return True

        # special case: holding container
        c = DataUtils.get_container_by_name(self.state_, self.state_.object_in_gripper)
        if c is not None:
            for i in range(c.width):
                for j in range(c.height):
                    checkPos = Point(c.position.x + i,
                                     c.position.y + j,
                                     c.position.z - 1)
                    if self.boxCollision(checkPos) or any(self.drawerCollision(checkPos)) or \
                            self.containerCollision(self.state_.gripper_position, checkPos, ignore=c.unique_name) or \
                            self.objectCollision(checkPos) or \
                            self.lidCollision(self.state_.gripper_position):
                        return False
        else:
            o = DataUtils.get_object_by_name(self.state_, self.state_.object_in_gripper)
            checkPos = Point(self.state_.gripper_position.x, self.state_.gripper_position.y,
                                self.state_.gripper_position.z - 1)
            if self.boxCollision(checkPos) or any(self.drawerCollision(checkPos)) or \
                    self.containerCollision(self.state_.gripper_position, checkPos) or \
                    (o is not None and self.objectCollision(checkPos)) or \
                    self.lidCollision(self.state_.gripper_position):
                return False

        self.moveGripper(Point(self.state_.gripper_position.x,
                                   self.state_.gripper_position.y,
                                   self.state_.gripper_position.z - 1))
        return True


    def resetArm(self):
        """Move the arm to the initial position"""
        if self.state_.object_in_gripper == 'drawer':
            self.error = 'Cannot reset arm while grasping drawer'
            return False

        resetPosition = Point(8, 1, 2)
        if not self.motionPlanChance(resetPosition):
            self.error = 'Motion planner failed.'
            return False
        self.moveGripper(resetPosition)
        return True


    def gravity(self, object = None, container = None):
        """Apply gravity to the specified object

        Keyword arguments:
        object -- object to apply gravity to (msg/Object), if None gravity will apply to all objects
        """
        change = False
        if object is None and container is None:
            # Apply gravity to everything
            change = self.gravityLid()
            for container in self.state_.containers:
                if self.state_.object_in_gripper != container.unique_name:
                    change = change or self.gravity(container=container)
            for object in self.state_.objects:
                if self.state_.object_in_gripper != object.unique_name:
                    change = change or self.gravity(object=object)
        elif object is not None:
            # Object gravity
            tempPos = Point(object.position.x, object.position.y, object.position.z)
            fall_dst = 0
            while object.position.z > 0:
                tempPos.z -= 1
                if self.onEdge(tempPos) or self.objectCollision(tempPos):
                    # randomly select pose around the object that's out of collision
                    drop = self.randomFreePoint(tempPos, 2, 2)
                    if drop:
                        tempPos = drop
                        object.position = self.copyPoint(tempPos)
                        continue
                    else:
                        break
                elif self.environmentCollision(tempPos) or self.containerCollision(object.position, tempPos):
                    break
                object.position.z -= 1
                fall_dst += 1
                change = True
            if fall_dst > 0:
                # try some random roll positions
                roll = self.randomFreePoint(object.position, fall_dst, fall_dst, ignore=object.unique_name)
                if roll:
                    points = self.interpolate(object.position.x, object.position.y, roll.x, roll.y)
                    final_point = self.copyPoint(object.position)
                    prev_point = self.copyPoint(object.position)
                    for point in points:
                        test_point = Point(int(floor(point[0] + 0.5)), int(floor(point[1] + 0.5)), object.position.z)
                        if self.environmentCollision(test_point) or self.containerCollision(prev_point, test_point):
                            break
                        final_point = self.copyPoint(test_point)
                        prev_point = self.copyPoint(test_point)
                    object.position = self.copyPoint(final_point)
                    # roll into open air case
                    if object.position.z > 0:
                        self.gravity(object)
        else:
            #Container gravity
            tempPos = Point(container.position.x, container.position.y, container.position.z)
            fall_dst = 0
            while container.position.z > 0:
                tempPos.z -= 1
                collision = 0
                edge = False
                for i in range(container.width):
                    for j in range(container.height):
                        checkPos = Point(tempPos.x + i, tempPos.y + j, tempPos.z)
                        edge = edge or self.onEdge(checkPos)
                        if self.environmentCollision(checkPos) or self.inContainer(checkPos,
                                                                                   ignore=container.unique_name) \
                                or self.objectCollision(checkPos):
                            collision += 1
                if collision / float(container.width*container.height) > 0.35:
                    break
                elif collision > 0:
                    drop = self.randomFreePoint(tempPos, 2, 2, obj_width=container.width, obj_depth=container.height,
                                                ignore=container.unique_name)
                    if drop is not None:
                        tempPos = drop
                        # container.position = self.copyPoint(tempPos)
                        self.moveContainer(container, tempPos.x - container.position.x,
                                           tempPos.y - container.position.y, tempPos.z - container.position.z)
                        continue
                    else:
                        break
                if edge:
                    # randomly select pose around the object that's out of collision
                    drop = self.randomFreePoint(tempPos, 2, 2, obj_width=container.width, obj_depth=container.height,
                                                ignore=container.unique_name)
                    if drop is not None:
                        tempPos = drop
                        # container.position = self.copyPoint(tempPos)
                        self.moveContainer(container, tempPos.x - container.position.x,
                                           tempPos.y - container.position.y, tempPos.z - container.position.z)
                        continue
                    else:
                        break
                self.moveContainer(container, 0, 0, -1)
                # container.position.z -= 1
                fall_dst += 1
                change = True
            if fall_dst > 0:
                # try some random roll positions
                roll = self.randomFreePoint(container.position, int(floor(fall_dst/2.0)), int(floor(fall_dst/2.0)),
                                            obj_width=container.width, obj_depth=container.height,
                                            ignore=container.unique_name)
                if roll:
                    points = self.interpolate(container.position.x, container.position.y, roll.x, roll.y)
                    final_point = self.copyPoint(container.position)
                    for point in points:
                        test_point = Point(int(floor(point[0] + 0.5)), int(floor(point[1] + 0.5)), container.position.z)
                        if self.environmentCollision(test_point):
                            break
                        final_point = self.copyPoint(test_point)
                    # container.position = self.copyPoint(final_point)
                    self.moveContainer(container, final_point.x - container.position.x,
                                       final_point.y - container.position.y, final_point.z - container.position.z)
                    # roll into open air case
                    if container.position.z > 0:
                        self.gravity(container=container)

        return change


    def gravityLid(self):
        """Apply gravity to the box lid"""
        if self.state_.object_in_gripper == 'lid':
            return
        change = False
        tempPos = self.copyPoint(self.state_.lid_position)
        while self.state_.lid_position.z > 0:
            tempPos.z -= 1
            collision = False
            for x in range(tempPos.x - self.boxRadius, tempPos.x + self.boxRadius + 1):
                for y in range(tempPos.y - self.boxRadius, tempPos.y + self.boxRadius + 1):
                    if self.inCollision(Point(x, y, tempPos.z)) or self.inContainer(Point(x, y, tempPos.z)):
                        collision = True
                        break
                if collision:
                    break
            if collision:
                break
            self.state_.lid_position.z -= 1
            change = True
        # TODO: handle case of lid on edge of box/drawers falling off

        return change


    def randomFreePoint(self, center, width, depth, obj_width = 1, obj_depth = 1, ignore=None, avoid_containers=False):
        """Calculate a random point not in collision with anything, within a bounding box

        Keyword arguments:
        center -- center point
        width -- x radius for the bounding box
        depth -- y radius for the bounding box
        obj_width -- width of object to fit at free point
        obj_depth -- depth of object to fit at free point

        Returns:
        a point if a free point is found
        None if no free points are found
        """
        poseCandidates = []
        for i in range(center.x - width, center.x + width + 1):
            for j in range(center.y - depth, center.y + depth + 1):
                candidate = Point(i, j, center.z)
                collision = False
                for k in range(obj_width):
                    for l in range(obj_depth):
                        collision = collision or \
                                    self.inCollision(Point(candidate.x + k, candidate.y + l, candidate.z), ignore) or \
                                    self.gripperCollision(Point(candidate.x + k, candidate.y + l, candidate.z))
                        # special case: containers not stackable
                        if obj_width > 1 or obj_depth > 1:
                            collision = collision or \
                                        self.inContainer(Point(candidate.x + k, candidate.y + l, candidate.z), ignore)
                        if avoid_containers:
                            collision = collision or \
                                       self.inContainer(Point(candidate.x + k, candidate.y + l, candidate.z), ignore)
                if not collision:
                    poseCandidates.append(candidate)
        shuffle(poseCandidates)
        if len(poseCandidates) > 0:
            return poseCandidates[0]
        return None


    def moveGripper(self, position):
        """Move the gripper and any attached objects to a given position

        Keyword arguments:
        position -- where to move the gripper to
        """
        if self.state_.object_in_gripper != '':
            if self.state_.object_in_gripper == 'lid':
                # Lid case
                dx = position.x - self.state_.lid_position.x
                dy = position.y - self.state_.lid_position.y
                dz = position.z - self.state_.lid_position.z
                self.state_.lid_position = self.copyPoint(position)
                for c in self.state_.containers:
                    if c.on_lid:
                        self.moveContainer(c, dx, dy, dz)
                for object in self.state_.objects:
                    if object.on_lid:
                        in_container = False
                        for c in self.state_.containers:
                            if object.unique_name in c.contains:
                                in_container = True
                        if in_container:
                            continue
                        object.position.x += dx
                        object.position.y += dy
                        object.position.z += dz
            elif self.state_.object_in_gripper == 'drawer':
                self.updateDrawerOffset(position)
            else:
                o = DataUtils.get_object_by_name(self.state_, self.state_.object_in_gripper)
                if o is not None:
                    # object case
                    o.position = self.copyPoint(position)
                else:
                    # container case
                    c = DataUtils.get_container_by_name(self.state_, self.state_.object_in_gripper)
                    dx = position.x - self.state_.gripper_position.x
                    dy = position.y - self.state_.gripper_position.y
                    dz = position.z - self.state_.gripper_position.z
                    self.moveContainer(c, dx, dy, dz)

        self.state_.gripper_position = position


    def moveContainer(self, container, dx, dy, dz):
        container.position.x += dx
        container.position.y += dy
        container.position.z += dz
        for o_name in container.contains:
            o = DataUtils.get_object_by_name(self.state_, o_name)
            o.position.x += dx
            o.position.y += dy
            o.position.z += dz
        # move chance
        for o_name in container.contains:
            o = DataUtils.get_object_by_name(self.state_, o_name)
            if abs(dx) > 0:
                shake_x = randint(-1,1)
                check_pos_x = Point(o.position.x + shake_x, o.position.y, o.position.z)
                if not (self.environmentCollision(check_pos_x) or self.objectCollision(check_pos_x) or
                            self.containerCollision(o.position, check_pos_x)):
                    o.position.x += shake_x
            if abs(dy) > 0:
                shake_y = randint(-1,1)
                check_pos_y = Point(o.position.x, o.position.y + shake_y, o.position.z)
                if not (self.environmentCollision(check_pos_y) or self.objectCollision(check_pos_y) or
                            self.containerCollision(o.position, check_pos_y)):
                    o.position.y += shake_y


    def copyPoint(self, point):
        """Make a copy of a point"""
        copy = Point()
        copy.x = point.x
        copy.y = point.y
        copy.z = point.z
        return copy


    def updateObjectStates(self):
        """Update the semantic portions of each object state"""
        for object in self.state_.objects:
            object.in_drawer = self.inDrawer(object)
            object.in_box = self.inBox(object)
            object.on_lid = self.onLid(object)
            object.on_stack = self.onStack(object)

    def updateContainerStates(self):
        """Update the semantic portions of each container state"""
        for c in self.state_.containers:
            c.contains = []
            for object in self.state_.objects:
                if self.inVolume(object.position, c.position.x, c.position.x + c.width - 1, c.position.y,
                                 c.position.y + c.height - 1, c.position.z, c.position.z):
                    c.contains.append(object.unique_name)
            c.in_drawer = self.inDrawer(c)
            c.in_box = self.inBox(c)
            c.on_lid = self.onLid(c)
            c.on_stack = self.onStack(c)

    def inDrawer(self, object):
        xmin, xmax, ymin, ymax, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer = self.getDrawerBounds()
        if object.__class__ == Object:
            return self.inVolume(
                object.position,
                xminDrawer + 1, xmaxDrawer - 1,
                yminDrawer + 1, ymaxDrawer - 1,
                self.drawerHeight, self.drawerHeight
            )
        elif object.__class__ == SmallContainer:
            result = False
            for i in range(object.width):
                for j in range(object.height):
                    result = result or self.inVolume(
                        Point(object.position.x + i, object.position.y + j, object.position.z),
                        xminDrawer + 1, xmaxDrawer - 1,
                        yminDrawer + 1, ymaxDrawer - 1,
                        self.drawerHeight, self.drawerHeight
                    )
            return result
        else:
            return False


    def onLid(self, object):
        if object.__class__ == Object:
            return self.inVolume(
                object.position,
                self.state_.lid_position.x - self.boxRadius,
                self.state_.lid_position.x + self.boxRadius,
                self.state_.lid_position.y - self.boxRadius,
                self.state_.lid_position.y + self.boxRadius,
                self.state_.lid_position.z + 1,
                self.state_.lid_position.z + 1
            )
        elif object.__class__ == SmallContainer:
            result = False
            for i in range(object.width):
                for j in range(object.height):
                    result = result or self.inVolume(
                        Point(object.position.x + i, object.position.y + j, object.position.z),
                        self.state_.lid_position.x - self.boxRadius,
                        self.state_.lid_position.x + self.boxRadius,
                        self.state_.lid_position.y - self.boxRadius,
                        self.state_.lid_position.y + self.boxRadius,
                        self.state_.lid_position.z + 1,
                        self.state_.lid_position.z + 1
                    )
            return result
        else:
            return False

    def onStack(self, object):
        xmin, xmax, ymin, ymax, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer = self.getDrawerBounds()
        if object.__class__ == Object:
            return self.inVolume(
                object.position,
                xmin, xmax,
                ymin, ymax,
                self.drawerHeight + 1, self.drawerHeight + 1
            )
        elif object.__class__ == SmallContainer:
            result = False
            for i in range(object.width):
                for j in range(object.height):
                    result = result or self.inVolume(
                        Point(object.position.x + i, object.position.y + j, object.position.z),
                        xmin, xmax,
                        ymin, ymax,
                        self.drawerHeight + 1, self.drawerHeight + 1
                    )
            return result
        else:
            return False

    def inBox(self, object):
        if object.__class__ == Object:
            return self.inVolume(object.position,
                                 self.state_.box_position.x - self.boxRadius,
                                 self.state_.box_position.x + self.boxRadius,
                                 self.state_.box_position.y - self.boxRadius,
                                 self.state_.box_position.y + self.boxRadius,
                                 self.state_.box_position.z,
                                 self.boxHeight)
        elif object.__class__ == SmallContainer:
            result = False
            for i in range(object.width):
                for j in range(object.height):
                    result = result or self.inVolume(
                        Point(object.position.x + i, object.position.y + j, object.position.z),
                        self.state_.box_position.x - self.boxRadius,
                        self.state_.box_position.x + self.boxRadius,
                        self.state_.box_position.y - self.boxRadius,
                        self.state_.box_position.y + self.boxRadius,
                        self.state_.box_position.z,
                        self.boxHeight
                    )
            return result
        else:
            return False


    def reachable(self, position):
        dst = self.euclidean2D(self.tableWidth/2, 1, position.x, position.y)
        return dst > 3 and dst < 20 and self.onTable(position)


    def onTable(self, position):
        return position.x >= 0 and position.x <= self.tableWidth and position.y >= 0 and position.y <= self.tableDepth


    def inCollision(self, position, ignore=None):
        """Detect collision with anything in the environment (gripper not included)"""
        return DataUtils.in_collision(
            self.state_, position,
            self.boxRadius, self.boxHeight,
            self.drawerWidth, self.drawerDepth, self.drawerHeight,
            ignore
        )


    def inContainer(self, pos, ignore=None):
        return DataUtils.in_container(self.state_, pos, ignore)


    def inSpecificContainer(self, pos, c):
        return self.inVolume(pos, c.position.x, c.position.x + c.width - 1, c.position.y, c.position.y + c.height - 1,
                             c.position.z, c.position.z)


    def containerCollision(self, prev_pos, pos, obj_width = 1, obj_depth = 1, ignore=None):
        """Detect if collision with a container edge occurs (happens on state change from in to out or out to in)"""
        if prev_pos.x == pos.x and prev_pos.y == pos.y and prev_pos.z == pos.z:
            return False

        # Falling
        if prev_pos.z > pos.z:
            if obj_width == 1 and obj_depth == 1:
            # special case: falling through container
                for c in self.state_.containers:
                    if ignore is not None and c.unique_name == ignore:
                        continue
                    was_in = self.inVolume(prev_pos, c.position.x, c.position.x + c.width - 1,
                                           c.position.y, c.position.y + c.height - 1, c.position.z, c.position.z)
                    is_in = self.inVolume(pos, c.position.x, c.position.x + c.width - 1,
                                          c.position.y, c.position.y + c.height - 1, c.position.z, c.position.z)
                    if was_in and not is_in:
                        return True
                return False
            # special case: large object falling into container or falling through container
            for c in self.state_.containers:
                if ignore is not None and c.unique_name == ignore:
                        continue
                was_in = False
                is_in = False
                for i in range(obj_width):
                    for j in range(obj_depth):
                        was_in = was_in or self.inVolume(Point(prev_pos.x + i, prev_pos.y + j, prev_pos.z),
                                                          c.position.x, c.position.x + c.width - 1,
                                                          c.position.y, c.position.y + c.height - 1,
                                                          c.position.z, c.position.z)
                        is_in = is_in or self.inVolume(Point(pos.x + i, pos.y + j, pos.z),
                                                          c.position.x, c.position.x + c.width - 1,
                                                          c.position.y, c.position.y + c.height - 1,
                                                          c.position.z, c.position.z)
                if was_in or is_in:
                    return True
            return False

        # Raising
        if prev_pos.z < pos.z:
            if obj_width == 1 and obj_depth == 1:
            # special case: raising into container
                for c in self.state_.containers:
                    if ignore is not None and c.unique_name == ignore:
                        continue
                    was_in = self.inVolume(prev_pos, c.position.x, c.position.x + c.width - 1,
                                           c.position.y, c.position.y + c.height - 1, c.position.z, c.position.z)
                    is_in = self.inVolume(pos, c.position.x, c.position.x + c.width - 1,
                                          c.position.y, c.position.y + c.height - 1, c.position.z, c.position.z)
                    if not was_in and is_in:
                        return True
                return False
            # special case: large object raising into container
            for c in self.state_.containers:
                if ignore is not None and c.unique_name == ignore:
                        continue
                was_in = False
                is_in = False
                for i in range(obj_width):
                    for j in range(obj_depth):
                        was_in = was_in or self.inVolume(Point(prev_pos.x + i, prev_pos.y + j, prev_pos.z),
                                                          c.position.x, c.position.x + c.width - 1,
                                                          c.position.y, c.position.y + c.height - 1,
                                                          c.position.z, c.position.z)
                        is_in = is_in or self.inVolume(Point(pos.x + i, pos.y + j, pos.z),
                                                          c.position.x, c.position.x + c.width - 1,
                                                          c.position.y, c.position.y + c.height - 1,
                                                          c.position.z, c.position.z)
                if was_in or is_in:
                    return True
            return False

        # Sliding
        for c in self.state_.containers:
            if ignore is not None and c.unique_name == ignore:
                        continue
            was_in = False
            is_in = False
            for i in range(obj_width):
                for j in range(obj_depth):
                    was_in = was_in or self.inVolume(Point(prev_pos.x + i, prev_pos.y + j, prev_pos.z),
                                                      c.position.x, c.position.x + c.width - 1,
                                                      c.position.y, c.position.y + c.height - 1,
                                                      c.position.z, c.position.z)
                    is_in = is_in or self.inVolume(Point(pos.x + i, pos.y + j, pos.z),
                                                      c.position.x, c.position.x + c.width - 1,
                                                      c.position.y, c.position.y + c.height - 1,
                                                      c.position.z, c.position.z)
            if was_in != is_in:
                return True
        return False


    def environmentCollision(self, position):
        """Detect collision with either the box, lid, or drawer"""
        return DataUtils.environment_collision(
            self.state_, position,
            self.boxRadius, self.boxHeight,
            self.drawerWidth, self.drawerDepth, self.drawerHeight
        )

    def environmentWithoutLidCollision(self, position):
        """Detect collision with either the box or drawer"""
        return self.boxCollision(position) or any(self.drawerCollision(position))


    def environmentWithoutDrawerCollision(self, position):
        """Detect collision with all environment objects except the drawer"""
        return self.boxCollision(position) or self.lidCollision(position) or self.drawerCollision(position)[0]


    def checkLidCollision(self, position):
        """Detect collision with anything the lid can collide with"""
        return self.boxCollision(position) or any(self.drawerCollision(position)) or self.objectCollision(position) \
               or self.inContainer(position)


    def onEdge(self, position):
        """Detect collision with either the box edge or the drawer edge"""
        drawer_collision = self.drawerCollision(position)
        return (self.boxCollision(position) and not self.lidCollision(position)) \
               or (drawer_collision[2] and not drawer_collision[0])


    def objectCollision(self, position):
        """Detect collision with any object"""
        return DataUtils.object_collision(self.state_, position)


    def gripperCollision(self, position):
        """Detect collision with only the gripper"""
        return DataUtils.gripper_collision(self.state_, position)


    def boxCollision(self, position):
        """Detect collision with only the box"""
        return DataUtils.box_collision(self.state_, position, self.boxRadius, self.boxHeight)


    def lidCollision(self, position):
        """Detect collision with only the lid"""
        return DataUtils.lid_collision(self.state_, position, self.boxRadius)


    def drawerCollision(self, position):
        """Detect collision with only the drawer

        Returns:
        List of collisions as follows:
        [drawer stack collision, drawer bottom collision, drawer edge collision]
        """
        return DataUtils.drawer_collision(self.state_, position, self.drawerWidth, self.drawerDepth, self.drawerHeight)

    def getDrawerBounds(self):
        """Determine the bounds of the drawer stack and drawer itself

        Returns:
        xmin, xmax, ymin, ymax, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer
            (xmin, xmax, ymin, ymax) -- bounding box of the drawer stack
            (xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer) -- bounding box of the drawer
        """
        return DataUtils.get_drawer_bounds(self.state_, self.drawerWidth, self.drawerDepth)

    def getDrawerHandle(self):
        """Calculate the point corresponding to the handle on the drawer"""
        return DataUtils.get_handle_pos(self.state_, self.drawerDepth, self.drawerHeight)


    def getDrawerValidPoints(self):
        """Calculate the set of valid points that make up the drawer path"""
        depthAdjustment = (self.drawerDepth - 1)/2
        min_point = Point(self.state_.drawer_position.x, self.state_.drawer_position.y, self.drawerHeight)
        max_point = self.copyPoint(min_point)
        offset = depthAdjustment + 1
        points = []
        if self.state_.drawer_position.theta == 0:
            min_point.x += offset
            points.append(min_point)
            for x in range(1, self.drawerDepth):
                point = self.copyPoint(min_point)
                point.x += x
                points.append(point)
        elif self.state_.drawer_position.theta == 90:
            min_point.y += offset
            points.append(min_point)
            for y in range(1, self.drawerDepth):
                point = self.copyPoint(min_point)
                point.y += y
                points.append(point)
        elif self.state_.drawer_position.theta == 180:
            min_point.x -= offset
            max_point.x = min_point.x - self.drawerDepth
            points.append(min_point)
            for x in range(1, self.drawerDepth):
                point = self.copyPoint(max_point)
                point.x += x
                points.append(point)
        else:
            min_point.y -= offset
            max_point.y = min_point.y - self.drawerDepth
            points.append(min_point)
            for y in range(1, self.drawerDepth):
                point = self.copyPoint(max_point)
                point.y += y
                points.append(point)
        return points


    def updateDrawerOffset(self, position):
        """Calculate the drawer offset given the drawer handle position"""
        depthAdjustment = ((self.drawerDepth - 1)/2)
        handle_closed_point = Point(self.state_.drawer_position.x, self.state_.drawer_position.y, self.drawerHeight)
        offset = depthAdjustment + 1
        prev_opening = self.state_.drawer_opening
        xchange = 0
        ychange = 0
        if self.state_.drawer_position.theta == 0:
            handle_closed_point.x += offset
            self.state_.drawer_opening = position.x - handle_closed_point.x
            xchange = self.state_.drawer_opening - prev_opening
        elif self.state_.drawer_position.theta == 90:
            handle_closed_point.y += offset
            self.state_.drawer_opening = position.y - handle_closed_point.y
            ychange = self.state_.drawer_opening - prev_opening
        elif self.state_.drawer_position.theta == 180:
            handle_closed_point.x -= offset
            self.state_.drawer_opening = handle_closed_point.x - position.x
            xchange = prev_opening - self.state_.drawer_opening
        else:
            handle_closed_point.y -= offset
            self.state_.drawer_opening = handle_closed_point.y - position.y
            ychange = prev_opening - self.state_.drawer_opening
        for container in self.state_.containers:
            if container.in_drawer:
                self.moveContainer(container, xchange, ychange, 0)
        for object in self.state_.objects:
            if object.in_drawer:
                in_container = False
                for container in self.state_.containers:
                    in_container = object.unique_name in container.contains
                if in_container:
                    continue
                object.position.x += xchange
                object.position.y += ychange


    def onBoxEdge(self, position, xmin, xmax, ymin, ymax, zmin, zmax):
        """Detect whether a point is on the perimeter of a given rectangular prism

        Keyword arguments:
        position -- point to check
        (xmin, xmax, ymin, ymax, zmin, zmax) -- bounds of the prism
        """
        return DataUtils.on_box_edge(position, xmin, xmax, ymin, ymax, zmin, zmax)


    def inVolume(self, position, xmin, xmax, ymin, ymax, zmin, zmax):
        """Detect whether a point is within a rectangular prism

        Keyword arguments:
        position -- point to check
        (xmin, xmax, ymin, ymax, zmin, zmax) -- bounds of the prism
        """
        return DataUtils.in_volume(position, xmin, xmax, ymin, ymax, zmin, zmax)


    def show(self):
        """Write everything to the screen

        Note: this also calculates occlusion
        TODO: we may need a version that writes to a buffer (to calculate occlusion) without writing to the screen
        """
        # reset occlusion
        for object in self.state_.objects:
                object.occluded = False

        self.output = []
        output_level = []
        for y in range(0, self.tableDepth + 1):
            line = []
            line_level = []
            for x in range(0, self.tableWidth + 1):
                line.append(' ')
                line_level.append(0)
            self.output.append(line)
            output_level.append(line_level)

        xmin, xmax, ymin, ymax, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer = self.getDrawerBounds()
        xminBox = self.state_.box_position.x - self.boxRadius
        xmaxBox = self.state_.box_position.x + self.boxRadius
        yminBox = self.state_.box_position.y - self.boxRadius
        ymaxBox = self.state_.box_position.y + self.boxRadius

        # table edges
        for x in range(0, self.tableWidth + 1):
            self.output[0][x] = ':'
            self.output[self.tableDepth][x] = ':'

        for y in range(0, self.tableDepth + 1):
            self.output[y][0] = ':'
            self.output[y][self.tableWidth] = ':'

        for z in range(0,5):
            # box
            if self.state_.box_position.z == z:
                for x in range(xminBox, xmaxBox + 1):
                    for y in range(yminBox, ymaxBox + 1):
                        if not (x == xminBox or x == xmaxBox or y == yminBox or y == ymaxBox):
                            self.setOutput(self.output, output_level, x, y, z, '+')
            if self.boxHeight == z:
                for x in range(xminBox, xmaxBox + 1):
                    for y in range(yminBox, ymaxBox + 1):
                        if x == xminBox or x == xmaxBox or y == yminBox or y == ymaxBox:
                            self.setOutput(self.output, output_level, x, y, z, '%')

            # containers
            for container in self.state_.containers:
                if not container.lost and container.position.z == z:
                    for i in range(container.width):
                        for j in range(container.height):
                            self.setOutput(self.output, output_level, container.position.x + i, container.position.y + j, z,
                                           '0')

            # objects
            for object in self.state_.objects:
                if not object.lost and object.position.z == z:
                    self.setOutput(self.output, output_level, object.position.x, object.position.y, z, object.name[0].upper())

            # drawer
            if self.drawerHeight - 1 == z:
                for x in range(xminDrawer, xmaxDrawer + 1):
                    for y in range(yminDrawer, ymaxDrawer + 1):
                        if x == xminDrawer or x == xmaxDrawer or y == yminDrawer or y == ymaxDrawer:
                            self.setOutput(self.output, output_level, x, y, z + 1, '%')
                        else:
                            self.setOutput(self.output, output_level, x, y, z, '*')
            if self.drawerHeight == z:
                for x in range(xmin, xmax + 1):
                    for y in range(ymin, ymax + 1):
                        self.setOutput(self.output, output_level, x, y, z, '#')

            # lid
            if self.state_.lid_position.z == z:
                for x in range(self.state_.lid_position.x - self.boxRadius,
                               self.state_.lid_position.x + self.boxRadius + 1):
                    for y in range(self.state_.lid_position.y - self.boxRadius,
                                   self.state_.lid_position.y + self.boxRadius + 1):
                        self.setOutput(self.output, output_level, x, y, z, '@')

            # gripper
            if self.state_.gripper_position.z == z:
                cx = self.state_.gripper_position.x
                cy = self.state_.gripper_position.y
                self.setOutput(self.output, output_level, cx-1, cy+1, z, '-')
                self.setOutput(self.output, output_level, cx+1, cy+1, z, '-')
                self.setOutput(self.output, output_level, cx-1, cy-1, z, '-')
                self.setOutput(self.output, output_level, cx+1, cy-1, z, '-')
                if self.state_.gripper_open:
                    self.setOutput(self.output, output_level, cx, cy-1, z, '-')
                    self.setOutput(self.output, output_level, cx, cy+1, z, '-')
                    self.setOutput(self.output, output_level, cx-1, cy, z, '[')
                    self.setOutput(self.output, output_level, cx+1, cy, z, ']')
                else:
                    self.setOutput(self.output, output_level, cx, cy+1, z, 'v')
                    self.setOutput(self.output, output_level, cx, cy-1, z, '^')
                    self.setOutput(self.output, output_level, cx-1, cy, z, '>')
                    self.setOutput(self.output, output_level, cx+1, cy, z, '<')

        # arm
        self.drawLine(self.output, output_level, self.tableWidth/2, 1, self.state_.gripper_position.x, self.state_.gripper_position.y, 4)
        self.drawLine(self.output, output_level, self.tableWidth/2 - 1, 1, self.state_.gripper_position.x, self.state_.gripper_position.y, 4)
        self.drawLine(self.output, output_level, self.tableWidth/2 + 1, 1, self.state_.gripper_position.x, self.state_.gripper_position.y, 4)
        self.drawLine(self.output, output_level, self.tableWidth/2, 0, self.state_.gripper_position.x, self.state_.gripper_position.y, 4)
        self.drawLine(self.output, output_level, self.tableWidth/2, 2, self.state_.gripper_position.x, self.state_.gripper_position.y, 4)

        # merge color levels into output buffer
        output_buffer = copy.copy(self.output)
        for i in range(len(output_buffer)):
            for j in range(len(output_buffer[i])):
                if output_buffer[i][j] == ' ':
                    continue

                if output_level[i][j] == 0:
                    pass
                elif output_level[i][j] == 1:
                    output_buffer[i][j] = '\033[1;33m' + output_buffer[i][j] + '\033[0m'
                elif output_level[i][j] == 2:
                    output_buffer[i][j] = '\033[33m' + output_buffer[i][j] + '\033[0m'
                elif output_level[i][j] == 3:
                    output_buffer[i][j] = '\033[31m' + output_buffer[i][j] + '\033[0m'
                else:
                    output_buffer[i][j] = '\033[35m' + output_buffer[i][j] + '\033[0m'

        # state
        output_buffer[0].append(' | Objects:')
        line_index = 1
        for object in self.state_.objects:
            output_buffer[line_index].append(' |  ' + object.unique_name + ': (' + str(object.position.x) + ', '
                                      + str(object.position.y) + ', ' + str(object.position.z) + ')')
            line_index += 1

        output_buffer[line_index].append(' |')
        line_index += 1
        output_buffer[line_index].append(' | Gripper: (' + str(self.state_.gripper_position.x) + ', '
                                  + str(self.state_.gripper_position.y) + ', ' + str(self.state_.gripper_position.z)
                                  + ') ')
        if self.state_.gripper_open:
            output_buffer[line_index].append('(open)')
        else:
            output_buffer[line_index].append('(closed)')
        line_index += 1

        output_buffer[line_index].append(' |')
        line_index += 1
        output_buffer[line_index].append(' | Drawers:')
        line_index += 1
        output_buffer[line_index].append(' |  Stack center, height: (' + str(self.state_.drawer_position.x) + ', '
                                  + str(self.state_.drawer_position.y) + '), ' + str(self.drawerHeight))
        line_index += 1
        output_buffer[line_index].append(' |  Drawer opening: ' + str(self.state_.drawer_opening))
        line_index += 1

        output_buffer[line_index].append(' |')
        line_index += 1
        output_buffer[line_index].append(' | Box:')
        line_index += 1
        output_buffer[line_index].append(' |  Center, height: (' + str(self.state_.box_position.x) + ', '
                                  + str(self.state_.box_position.y) + '), ' + str(self.boxHeight))
        line_index += 1
        output_buffer[line_index].append(' |  Lid: (' + str(self.state_.lid_position.x) + ', '
                                  + str(self.state_.lid_position.y) + ', ' + str(self.state_.lid_position.z) + ')')

        # tic marks
        for i in range(len(output_buffer)):
            if self.tableDepth - i < 10:
                output_buffer[i].insert(0, ' ' + str(self.tableDepth - i) + '| ')
            else:
                output_buffer[i].insert(0, str(self.tableDepth - i) + '| ')
        tics = '    '
        labels = '    '
        for i in range(0, self.tableWidth + 1, 5):
            tics += '|    '
            labels += str(i)
            if i < 10:
                labels += '    '
            else:
                labels += '   '
        output_buffer.insert(0, [tics])
        output_buffer.insert(0, [labels])
        output_buffer.append([tics])
        output_buffer.append([labels])

        #  print
        if not self.quiet_mode:
            print('')
            for line in output_buffer:
                print(''.join(line))
            if len(self.error) > 0:
                print(self.error)
                self.error = ''

    def drawLine(self, output, output_level, x1, y1, x2, y2, z):
        """Draw a line with the Bresenham algorithm

        Kewyord arguments:
        output -- the output buffer (a list of list of strings)
        (x1, y1) -- the start point
        (x2, y2) -- the end point
        """
        x = x1
        y = y1
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        s1 = sign(x2 - x1)
        s2 = sign(y2 - y1)
        swap = False
        if dy > dx:
            temp = dx
            dx = dy
            dy = temp
            swap = True
        d = 2*dy - dx
        for i in range(0, dx):
            check = self.getOutput(output, x, y)
            if not (check == '[' or check == ']' or check == '<' or check == '>'
                or check == '-' or check == 'v' or check == '^'):
                self.setOutput(output, output_level, x, y, z, '$')
            while d >= 0:
                d -= 2*dx
                if swap:
                    x += s1
                else:
                    y += s2
            d += 2*dy
            if swap:
                y += s2
            else:
                x += s1

    def setOutput(self, output, output_level, x, y, z, value):
        """Fill an output cell with a given value

        Keyword arguments:
        output -- the output buffer (a list of list of strings)
        output_level -- output buffer storing heights
        (x, y) -- the point to fill, in the table coordinate frame
        value -- the value to fill
        """
        if x >= 0 and x <= self.tableWidth and y >= 0 and y <= self.tableDepth:
            prev = self.getOutput(output, x, y)
            if prev != ' ':
                for object in self.state_.objects:
                    if object.name[0] == prev:
                        object.occluded = True
                        break
            output[self.tableDepth - y][x] = value
            output_level[self.tableDepth - y][x] = z

    def getOutput(self, output, x, y):
        """Get the value of an output cell

        Keyword arguments:
        output -- the output buffer (a list of list of strings)
        (x, y) -- the point to fill, in the table coordinate frame
        """
        if x >= 0 and x <= self.tableWidth and y >= 0 and y <= self.tableDepth:
            return output[self.tableDepth - y][x]
        return ' '

    def getInput(self):
        """Get a command from the user

        Returns:
        False on 'quit', True otherwise (used for breaking loops)
        action_msg, A message structure for the executed action
        """
        try:
            action = raw_input('Action?: ').split(' ')
            action_msg = Action()
            if len(action) == 0:
                self.error = 'Invalid command. Type ? for a command list.'
                action_msg.action_type = Action.NOOP
            elif action[0].lower() == 'grasp' or action[0].lower() == 'g':
                if len(action) >= 2:
                    action_msg.action_type = Action.GRASP
                    action_msg.object = action[1]
                else:
                    self.error = 'Grasp takes parameter: object'
                    action_msg.action_type = Action.NOOP
            elif action[0].lower() == 'place' or action[0].lower() == 'p':
                if len(action) >= 3:
                    action_msg.action_type = Action.PLACE
                    action_msg.position = Point(int(action[1]), int(action[2]), 0)
                else:
                    self.error = 'Place takes parameters: x y'
                    action_msg.action_type = Action.NOOP
            elif action[0].lower() == 'open' or action[0].lower() == 'o':
                action_msg.action_type = Action.OPEN_GRIPPER
            elif action[0].lower() == 'close' or action[0].lower() == 'c':
                action_msg.action_type = Action.CLOSE_GRIPPER
            elif action[0].lower() == 'home' or action[0].lower() == 'h':
                action_msg.action_type = Action.RESET_ARM
            elif action[0].lower() == 'raise' or action[0].lower() == 'r':
                action_msg.action_type = Action.RAISE_ARM
            elif action[0].lower() == 'lower' or action[0].lower() == 'l':
                action_msg.action_type = Action.LOWER_ARM
            elif action[0].lower() == 'move' or action[0].lower() == 'm':
                if len(action) >= 3:
                    action_msg.action_type = Action.MOVE_ARM
                    action_msg.position = Point(int(action[1]), int(action[2]), 0)
                else:
                    self.error = 'Move takes parameters: x y'
                    action_msg.action_type = Action.NOOP
            elif action[0].lower() == 'finish' or action[0].lower() == 'f':
                return None
            elif action[0].lower() == 'quit' or action[0].lower() == 'q':
                return None
            elif action[0] == '?' or action[0].lower() == 'help' or action[0].lower() == 'h':
                cmd_template = "{:1}\t{:10}\t{:69}"
                print("Valid commands:")
                print(cmd_template.format('o', 'open', 'Open the gripper'))
                print(cmd_template.format('c', 'close', 'Close the gripper'))
                print(cmd_template.format('g', 'grasp', 'Grasp object. param: object_name'))
                print(cmd_template.format('p', 'place', 'Place object. param: x y'))
                print(cmd_template.format('h', 'home', 'Arm to home'))
                print(cmd_template.format('r', 'raise', 'Raise arm'))
                print(cmd_template.format('l', 'lower', 'Lower arm'))
                print(cmd_template.format('m', 'move', 'Move to location. param: x y'))
                print(cmd_template.format('f', 'finish', 'Finish intervention (for autonomous execution mode)'))
                print(cmd_template.format('q', 'quit', 'Quit simulator'))
                print(cmd_template.format('?', 'help', 'Show list of actions'))
                action_msg.action_type = Action.NOOP
            else:
                self.error = 'Invalid command. Type ? for a command list.'
                action_msg.action_type = Action.NOOP

            # s0 = copy.deepcopy(self.state_)

            self.worldUpdate(action_msg)

            # s1 = copy.deepcopy(self.state_)
            # test = PlanAction(s0, action_msg, s1)
            # print '\n\nPlanAction Check:\n'
            # print str(test)
            # print '\n\n'

            return action_msg
        except (KeyboardInterrupt, EOFError) as e:
            return None


if __name__ == '__main__':
    rospy.init_node('table_sim')
    table_sim = TableSim()

    rospy.sleep(1.0)
    table_sim.worldUpdate()

    # Shutdown based on the ROS signal. Call the callbacks
    no_quit, user_action = None, None
    loop_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if table_sim.terminal_input:
            if table_sim.getInput() is None:
                break
        loop_rate.sleep()
