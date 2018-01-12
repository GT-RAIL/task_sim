#!/usr/bin/env python

# Python
from math import floor
from math import sqrt
from random import seed
from random import shuffle
from random import randint
from random import random

# ROS
import rospy
from geometry_msgs.msg import Point
from numpy import sign
from task_sim.srv import Execute, ExecuteResponse, QueryState, RequestIntervention, RequestInterventionResponse
from task_sim.msg import Action, State, Object, Log
from std_srvs.srv import Empty, EmptyResponse
from grasp_state import GraspState

from data_utils import DataUtils

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


    def query_state(self, req):
        return self.state_

    def reset_sim(self, req):
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
            1 : randomize objects and drawer/box positions
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
                self.state_.drawer_position.theta = randint(0, 3)*90

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
        obj1.name = "Apple"
        object_set = False
        while not object_set:
            obj1.position.x = randint(1, self.tableWidth - 1)
            obj1.position.y = randint(1, self.tableDepth - 1)
            obj1.position.z = 0
            object_set = not self.inCollision(obj1.position) and self.reachable(obj1.position)

        obj2 = Object()
        obj2.name = "Batteries"
        object_set = False
        while not object_set:
            obj2.position.x = randint(1, self.tableWidth - 1)
            obj2.position.y = randint(1, self.tableDepth - 1)
            obj2.position.z = 0
            object_set = not self.inCollision(obj2.position) and self.reachable(obj2.position)

        obj3 = Object()
        obj3.name = "Flashlight"
        object_set = False
        while not object_set:
            obj3.position.x = randint(1, self.tableWidth - 1)
            obj3.position.y = randint(1, self.tableDepth - 1)
            obj3.position.z = 0
            object_set = not self.inCollision(obj3.position) and self.reachable(obj3.position)

        obj4 = Object()
        obj4.name = "Granola"
        object_set = False
        while not object_set:
            obj4.position.x = randint(1, self.tableWidth - 1)
            obj4.position.y = randint(1, self.tableDepth - 1)
            obj4.position.z = 0
            object_set = not self.inCollision(obj4.position) and self.reachable(obj4.position)

        obj5 = Object()
        obj5.name = "Knife"
        object_set = False
        while not object_set:
            obj5.position.x = randint(1, self.tableWidth - 1)
            obj5.position.y = randint(1, self.tableDepth - 1)
            obj5.position.z = 0
            object_set = not self.inCollision(obj5.position) and self.reachable(obj5.position)

        self.state_.objects.append(obj1)
        self.state_.objects.append(obj2)
        self.state_.objects.append(obj3)
        self.state_.objects.append(obj4)
        self.state_.objects.append(obj5)

        # Initial robot configuration (home)
        self.state_.gripper_position.x = 8
        self.state_.gripper_position.y = 1
        self.state_.gripper_position.z = 2
        self.state_.gripper_open = True

        # Hidden state
        self.grasp_states = {}
        for object in self.state_.objects:
            self.grasp_states[object.name] = GraspState(self.getNeighborCount(object.position),
                                                        self.copyPoint(object.position))


    def init_simulation_fixed(self):
        """Create a hardcoded 40x15 table with a few objects, a closed drawer, and a closed box"""
        self.tableWidth = 40
        self.tableDepth = 15
        self.boxRadius = 2
        self.boxHeight = 1
        self.drawerWidth = 5
        self.drawerDepth = 7
        self.drawerHeight = 2

        self.state_ = State()

        # Hardcoded containers
        self.state_.drawer_position.x = 10
        self.state_.drawer_position.y = 12
        self.state_.drawer_position.theta = 0
        self.state_.drawer_opening = 3

        self.state_.box_position.x = 30
        self.state_.box_position.y = 6
        self.state_.lid_position.x = 30
        self.state_.lid_position.y = 6
        self.state_.lid_position.z = 1

        # Hardcoded objects
        obj1 = Object()
        obj1.name = "Apple"
        obj1.position.x = 3
        obj1.position.y = 10
        obj1.position.z = 0
        obj2 = Object()
        obj2.name = "Batteries"
        obj2.position.x = 20
        obj2.position.y = 6
        obj2.position.z = 0
        obj3 = Object()
        obj3.name = "Flashlight"
        obj3.position.x = 5
        obj3.position.y = 7
        obj3.position.z = 0
        obj4 = Object()
        obj4.name = "Granola"
        obj4.position.x = 2
        obj4.position.y = 9
        obj4.position.z = 0
        obj5 = Object()
        obj5.name = "Knife"
        obj5.position.x = 15
        obj5.position.y = 4
        obj5.position.z = 0
        self.state_.objects.append(obj1)
        self.state_.objects.append(obj2)
        self.state_.objects.append(obj3)
        self.state_.objects.append(obj4)
        self.state_.objects.append(obj5)

        # Initial robot configuration (home)
        self.state_.gripper_position.x = 8
        self.state_.gripper_position.y = 1
        self.state_.gripper_position.z = 2
        self.state_.gripper_open = True

        # Hidden state
        self.grasp_states = {}
        for object in self.state_.objects:
            self.grasp_states[object.name] = GraspState(self.getNeighborCount(object.position),
                                                        self.copyPoint(object.position))


    def getNeighborCount(self, position):
        count = 0
        for x in range(-1, 2):
            for y in range(-1, 2):
                if x == 0 and y == 0:
                    continue
                if self.inCollision(Point(position.x + x, position.y + y, position.z)):
                    count += 1
        return count


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
        self.updateObjectStates()
        for object in self.state_.objects:
            object.lost = (object.position.x <= 0 or object.position.x >= self.tableWidth
                           or object.position.y <= 0 or object.position.y >= self.tableDepth) \
                          and not self.state_.object_in_gripper == object.name and not object.on_lid
            if not object.lost:
                self.grasp_states[object.name].updateGraspRate(self.getNeighborCount(object.position),
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
        if object == 'lid' or object == 'Lid':
            if not self.motionPlanChance(self.state_.lid_position):
                self.error = 'Motion planner failed.'
                return False
            if self.getObjectAt(self.state_.lid_position):
                self.error = 'Lid is occluded and cannot be grasped.'
                return False
            self.state_.gripper_position = self.copyPoint(self.state_.lid_position)
            self.state_.gripper_open = False
            self.state_.object_in_gripper = 'Lid'
            return True
        elif object == 'drawer' or object == 'Drawer':
            if not self.motionPlanChance(self.getDrawerHandle()):
                self.error = 'Motion planner failed.'
                return False
            self.state_.gripper_position = self.copyPoint(self.getDrawerHandle())
            self.state_.gripper_open = False
            self.state_.object_in_gripper = 'Drawer'
            return True

        target = self.getObject(object)
        if target:
            if target.lost:
                self.error = target.name + ' is lost.'
                return False

            if target.occluded:
                self.error = target.name + ' is occluded and cannot be grasped.'
                return False

            if not self.grasp_states[target.name].graspable:
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
                        if self.state_.object_in_gripper == 'Lid':
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
        if self.state_.object_in_gripper == 'Lid':
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
        if self.state_.object_in_gripper == 'Drawer':
            self.error = 'Cannot execute place while grasping drawer.'
            return False
        height = 4
        tempPos = Point(position.x, position.y, 0)
        for i in range(3,-1,-1):
            tempPos.z = i
            if self.state_.object_in_gripper == 'Lid':
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
            if self.state_.object_in_gripper == 'Lid':
                self.state_.object_in_gripper = ''
                self.gravity()
            elif self.state_.object_in_gripper == 'Drawer':
                self.state_.object_in_gripper = ''
            else:
                object = self.getObject(self.state_.object_in_gripper)
                self.state_.object_in_gripper = ''
                self.gravity(object)
        return True


    def close(self):
        """Close the gripper, grasping anything at its current location"""
        if self.state_.gripper_open:
            self.state_.gripper_open = False
            o = self.getObjectAt(self.state_.gripper_position)
            if o:
                self.state_.object_in_gripper = o.name
            else:
                # Handle special cases
                if self.state_.gripper_position == self.state_.lid_position:
                    self.state_.object_in_gripper = 'Lid'
                elif self.state_.gripper_position == self.getDrawerHandle():
                    self.state_.object_in_gripper = 'Drawer'
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
        for point in points:
            testPos = Point(int(floor(point[0] + 0.5)), int(floor(point[1] + 0.5)), self.state_.gripper_position.z)
            # handle special cases
            if self.state_.object_in_gripper == 'Lid':
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
            elif self.state_.object_in_gripper == 'Drawer':
                if not any(valid_point == testPos for valid_point in self.getDrawerValidPoints()):
                    break
                if self.environmentWithoutDrawerCollision(testPos):
                    break
                if not self.reachable(testPos):
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

        for object in self.state_.objects:
            if object.lost:
                continue
            if object.name == self.state_.object_in_gripper:
                continue
            if (self.state_.object_in_gripper == 'Drawer' and object.in_drawer) or \
                    (self.state_.object_in_gripper == 'Lid' and object.on_lid):
                continue
            if object.position.z == start.z and \
                self.distanceFromPath(object.position.x, object.position.y, start.x, start.y, goal.x, goal.y) < 1.2:
                object_goal = self.randomFreePoint(Point(goal.x, goal.y, goal.z), 2, 2)
                if object_goal:
                    object_points = self.interpolate(object.position.x, object.position.y, object_goal.x, object_goal.y)
                    for point in object_points:
                        test_pos = Point(int(floor(point[0] + 0.5)), int(floor(point[1] + 0.5)), start.z)
                        if self.environmentCollision(testPos):
                            break
                        object.position = self.copyPoint(test_pos)
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
        if self.state_.object_in_gripper == 'Drawer':
            self.error = 'Cannot raise arm while grasping drawer'
            return False
        if self.state_.gripper_position.z < 4:
            checkPos = Point(self.state_.gripper_position.x,
                             self.state_.gripper_position.y,
                             self.state_.gripper_position.z + 1)
            if any(self.drawerCollision(checkPos)):
                return False
            self.moveGripper(checkPos)
            return True
        return False


    def lowerArm(self):
        """Move the arm down one z-level"""
        if self.state_.object_in_gripper == 'Drawer':
            self.error = 'Cannot lower arm while grasping drawer'
            return False
        if self.state_.gripper_position.z == 0:
            return False

        checkPos = Point(self.state_.gripper_position.x, self.state_.gripper_position.y,
                            self.state_.gripper_position.z - 1)
        if self.boxCollision(checkPos) or any(self.drawerCollision(checkPos)):
            return False

        self.moveGripper(Point(self.state_.gripper_position.x,
                                   self.state_.gripper_position.y,
                                   self.state_.gripper_position.z - 1))
        return True


    def resetArm(self):
        """Move the arm to the initial position"""
        if self.state_.object_in_gripper == 'Drawer':
            self.error = 'Cannot reset arm while grasping drawer'
            return False

        resetPosition = Point(8, 1, 2)
        if not self.motionPlanChance(resetPosition):
            self.error = 'Motion planner failed.'
            return False
        self.moveGripper(resetPosition)
        return True


    def gravity(self, object = None):
        """Apply gravity to the specified object

        Keyword arguments:
        object -- object to apply gravity to (msg/Object), if None gravity will apply to all objects
        """
        change = False
        if object is None:
            # Apply gravity to everything
            change = self.gravityLid()
            for object in self.state_.objects:
                if self.state_.object_in_gripper != object.name:
                    change = change or self.gravity(object)
        else:
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
                elif self.environmentCollision(tempPos):
                    break
                object.position.z -= 1
                fall_dst += 1
                change = True
            if fall_dst > 0:
                # try some random roll positions
                roll = self.randomFreePoint(object.position, fall_dst, fall_dst)
                if roll:
                    #TODO: Fix!
                    points = self.interpolate(object.position.x, object.position.y, roll.x, roll.y)
                    final_point = self.copyPoint(object.position)
                    for point in points:
                        test_point = Point(int(floor(point[0] + 0.5)), int(floor(point[1] + 0.5)), object.position.z)
                        if self.environmentCollision(test_point):
                            break
                        final_point = self.copyPoint(test_point)
                    object.position = self.copyPoint(final_point)

        return change


    def gravityLid(self):
        """Apply gravity to the box lid"""
        if self.state_.object_in_gripper == 'Lid':
            return
        change = False
        tempPos = self.copyPoint(self.state_.lid_position)
        while self.state_.lid_position.z > 0:
            tempPos.z -= 1
            collision = False
            for x in range(tempPos.x - self.boxRadius, tempPos.x + self.boxRadius + 1):
                for y in range(tempPos.y - self.boxRadius, tempPos.y + self.boxRadius + 1):
                    if self.inCollision(Point(x, y, tempPos.z)):
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


    def randomFreePoint(self, center, width, depth):
        """Calculate a random point not in collision with anything, within a bounding box

        Keyword arguments:
        center -- center point
        width -- x radius for the bounding box
        depth -- y radius for the bounding box

        Returns:
        a point if a free point is found
        None if no free points are found
        """
        poseCandidates = []
        for i in range(center.x - width, center.x + width + 1):
            for j in range(center.y - depth, center.y + depth + 1):
                candidate = Point(i, j, center.z)
                if not (self.inCollision(candidate) or self.gripperCollision(candidate)):
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
        self.state_.gripper_position = position
        if self.state_.object_in_gripper != '':
            if self.state_.object_in_gripper == 'Lid':
                ## Lid case
                dx = position.x - self.state_.lid_position.x
                dy = position.y - self.state_.lid_position.y
                dz = position.z - self.state_.lid_position.z
                self.state_.lid_position = self.copyPoint(position)
                for object in self.state_.objects:
                    if object.on_lid:
                        object.position.x += dx
                        object.position.y += dy
                        object.position.z += dz
            elif self.state_.object_in_gripper == 'Drawer':
                self.updateDrawerOffset(position)
            else:
                self.getObject(self.state_.object_in_gripper).position = self.copyPoint(position)


    def copyPoint(self, point):
        """Make a copy of a point"""
        copy = Point()
        copy.x = point.x
        copy.y = point.y
        copy.z = point.z
        return copy


    def getObject(self, name):
        """Find an object given an object name"""
        for object in self.state_.objects:
            if object.name == name:
                return object
        return None


    def getObjectAt(self, position):
        """Return the object at a given position"""
        for object in self.state_.objects:
            if object.position == position:
                return object
        return None


    def updateObjectStates(self):
        """Update the semantic portions of each object state"""
        for object in self.state_.objects:
            object.in_drawer = self.inDrawer(object)
            object.in_box = self.inBox(object)
            object.on_lid = self.onLid(object)


    def inDrawer(self, object):
        xmin, xmax, ymin, ymax, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer = self.getDrawerBounds()
        return self.inVolume(object.position, xminDrawer + 1, xmaxDrawer - 1, yminDrawer + 1, ymaxDrawer - 1,
                             self.drawerHeight, self.drawerHeight)


    def onLid(self, object):
        return self.inVolume(object.position, self.state_.lid_position.x - self.boxRadius,
                             self.state_.lid_position.x + self.boxRadius, self.state_.lid_position.y - self.boxRadius,
                             self.state_.lid_position.y + self.boxRadius, self.state_.lid_position.z + 1,
                             self.state_.lid_position.z + 1)


    def inBox(self, object):
        return self.inVolume(object.position, self.state_.box_position.x - self.boxRadius,
                             self.state_.box_position.x + self.boxRadius, self.state_.box_position.y - self.boxRadius,
                             self.state_.box_position.y + self.boxRadius, self.state_.box_position.z,
                             self.boxHeight)


    def reachable(self, position):
        dst = self.euclidean2D(self.tableWidth/2, 1, position.x, position.y)
        return dst > 3 and dst < 20 and self.onTable(position)


    def onTable(self, position):
        return position.x >= 0 and position.x <= self.tableWidth and position.y >= 0 and position.y <= self.tableDepth


    def inCollision(self, position):
        """Detect collision with anything in the environment (gripper not included)"""
        return self.objectCollision(position) or self.environmentCollision(position)


    def environmentCollision(self, position):
        """Detect collision with either the box, lid, or drawer"""
        return self.boxCollision(position) or self.lidCollision(position) or any(self.drawerCollision(position))


    def environmentWithoutLidCollision(self, position):
        """Detect collision with either the box or drawer"""
        return self.boxCollision(position) or any(self.drawerCollision(position))


    def environmentWithoutDrawerCollision(self, position):
        """Detect collision with all environment objects except the drawer"""
        return self.boxCollision(position) or self.lidCollision(position) or self.drawerCollision(position)[0]


    def checkLidCollision(self, position):
        """Detect collision with anything the lid can collide with"""
        return self.boxCollision(position) or any(self.drawerCollision(position)) or self.objectCollision(position)


    def onEdge(self, position):
        """Detect collision with either the box edge or the drawer edge"""
        return self.boxCollision(position) or self.drawerCollision(position)[2]


    def objectCollision(self, position):
        """Detect collision with any object"""
        for object in self.state_.objects:
            if object.position == position:
                return object
        return None


    def gripperCollision(self, position):
        """Detect collision with only the gripper"""
        return self.onBoxEdge(position,
                              self.state_.gripper_position.x - 1,
                              self.state_.gripper_position.x + 1,
                              self.state_.gripper_position.y - 1,
                              self.state_.gripper_position.y + 1,
                              self.state_.gripper_position.z,
                              self.state_.gripper_position.z)


    def boxCollision(self, position):
        """Detect collision with only the box"""
        return self.onBoxEdge(position,
                              self.state_.box_position.x - self.boxRadius,
                              self.state_.box_position.x + self.boxRadius,
                              self.state_.box_position.y - self.boxRadius,
                              self.state_.box_position.y + self.boxRadius,
                              self.state_.box_position.z,
                              self.state_.box_position.z + self.boxHeight - 1)


    def lidCollision(self, position):
        """Detect collision with only the lid"""
        return self.inVolume(position,
                             self.state_.lid_position.x - self.boxRadius,
                             self.state_.lid_position.x + self.boxRadius,
                             self.state_.lid_position.y - self.boxRadius,
                             self.state_.lid_position.y + self.boxRadius,
                             self.state_.lid_position.z,
                             self.state_.lid_position.z)


    def drawerCollision(self, position):
        """Detect collision with only the drawer

        Returns:
        List of collisions as follows:
        [drawer stack collision, drawer bottom collision, drawer edge collision]
        """
        xmin, xmax, ymin, ymax, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer = self.getDrawerBounds()

        return [self.inVolume(position, xmin, xmax, ymin, ymax, 0, self.drawerHeight),
                self.inVolume(position, xminDrawer + 1, xmaxDrawer - 1, yminDrawer + 1, ymaxDrawer - 1,
                              self.drawerHeight - 1, self.drawerHeight - 1),
                self.onBoxEdge(position, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer,
                               self.drawerHeight - 1, self.drawerHeight)]

    def getDrawerBounds(self):
        """Determine the bounds of the drawer stack and drawer itself

        Returns:
        xmin, xmax, ymin, ymax, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer
            (xmin, xmax, ymin, ymax) -- bounding box of the drawer stack
            (xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer) -- bounding box of the drawer
        """
        widthAdjustment = (self.drawerWidth - 1)/2
        depthAdjustment = (self.drawerDepth - 1)/2
        if self.state_.drawer_position.theta == 0:
            xmin = self.state_.drawer_position.x - depthAdjustment
            xmax = self.state_.drawer_position.x + depthAdjustment
            ymin = self.state_.drawer_position.y - widthAdjustment
            ymax = self.state_.drawer_position.y + widthAdjustment
            xminDrawer = xmin + self.state_.drawer_opening
            xmaxDrawer = xmax + self.state_.drawer_opening
            yminDrawer = ymin
            ymaxDrawer = ymax

        elif self.state_.drawer_position.theta == 90:
            xmin = self.state_.drawer_position.x - widthAdjustment
            xmax = self.state_.drawer_position.x + widthAdjustment
            ymin = self.state_.drawer_position.y - depthAdjustment
            ymax = self.state_.drawer_position.y + depthAdjustment
            xminDrawer = xmin
            xmaxDrawer = xmax
            yminDrawer = ymin + self.state_.drawer_opening
            ymaxDrawer = ymax + self.state_.drawer_opening

        elif self.state_.drawer_position.theta == 180:
            xmin = self.state_.drawer_position.x - depthAdjustment
            xmax = self.state_.drawer_position.x + depthAdjustment
            ymin = self.state_.drawer_position.y - widthAdjustment
            ymax = self.state_.drawer_position.y + widthAdjustment
            xminDrawer = xmin - self.state_.drawer_opening
            xmaxDrawer = xmax - self.state_.drawer_opening
            yminDrawer = ymin
            ymaxDrawer = ymax

        else:  # 270
            xmin = self.state_.drawer_position.x - widthAdjustment
            xmax = self.state_.drawer_position.x + widthAdjustment
            ymin = self.state_.drawer_position.y - depthAdjustment
            ymax = self.state_.drawer_position.y + depthAdjustment
            xminDrawer = xmin
            xmaxDrawer = xmax
            yminDrawer = ymin - self.state_.drawer_opening
            ymaxDrawer = ymax - self.state_.drawer_opening

        return xmin, xmax, ymin, ymax, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer


    def getDrawerHandle(self):
        """Calculate the point corresponding to the handle on the drawer"""
        depthAdjustment = ((self.drawerDepth - 1)/2)
        handle_point = Point(self.state_.drawer_position.x, self.state_.drawer_position.y, self.drawerHeight)
        offset = depthAdjustment + self.state_.drawer_opening + 1
        if self.state_.drawer_position.theta == 0:
            handle_point.x += offset
        elif self.state_.drawer_position.theta == 90:
            handle_point.y += offset
        elif self.state_.drawer_position.theta == 180:
            handle_point.x -= offset
        else:
            handle_point.y -= offset
        return handle_point


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
        for object in self.state_.objects:
            if object.in_drawer:
                object.position.x += xchange
                object.position.y += ychange


    def onBoxEdge(self, position, xmin, xmax, ymin, ymax, zmin, zmax):
        """Detect whether a point is on the perimeter of a given rectangular prism

        Keyword arguments:
        position -- point to check
        (xmin, xmax, ymin, ymax, zmin, zmax) -- bounds of the prism
        """
        return ((((position.x == xmin or position.x == xmax)
                  and position.y >= ymin
                  and position.y <= ymax)
                 or ((position.y == ymin or position.y == ymax)
                     and position.x >= xmin
                     and position.x <= xmax))
                and position.z >= zmin
                and position.z <= zmax)


    def inVolume(self, position, xmin, xmax, ymin, ymax, zmin, zmax):
        """Detect whether a point is within a rectangular prism

        Keyword arguments:
        position -- point to check
        (xmin, xmax, ymin, ymax, zmin, zmax) -- bounds of the prism
        """
        return (position.x >= xmin and position.x <= xmax
            and position.y >= ymin and position.y <= ymax
            and position.z >= zmin and position.z <= zmax)


    def show(self):
        """Write everything to the screen

        Note: this also calculates occlusion
        TODO: we may need a version that writes to a buffer (to calculate occlusion) without writing to the screen
        """
        # reset occlusion
        for object in self.state_.objects:
                object.occluded = False

        output = []
        output_level = []
        for y in range(0, self.tableDepth + 1):
            line = []
            line_level = []
            for x in range(0, self.tableWidth + 1):
                line.append(' ')
                line_level.append(0)
            output.append(line)
            output_level.append(line_level)

        xmin, xmax, ymin, ymax, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer = self.getDrawerBounds()
        xminBox = self.state_.box_position.x - self.boxRadius
        xmaxBox = self.state_.box_position.x + self.boxRadius
        yminBox = self.state_.box_position.y - self.boxRadius
        ymaxBox = self.state_.box_position.y + self.boxRadius

        # table edges
        for x in range(0, self.tableWidth + 1):
            output[0][x] = ':'
            output[self.tableDepth][x] = ':'

        for y in range(0, self.tableDepth + 1):
            output[y][0] = ':'
            output[y][self.tableWidth] = ':'

        for z in range(0,5):
            # box
            if self.state_.box_position.z == z:
                for x in range(xminBox, xmaxBox + 1):
                    for y in range(yminBox, ymaxBox + 1):
                        if not (x == xminBox or x == xmaxBox or y == yminBox or y == ymaxBox):
                            self.setOutput(output, output_level, x, y, z, '+')
            if self.boxHeight == z:
                for x in range(xminBox, xmaxBox + 1):
                    for y in range(yminBox, ymaxBox + 1):
                        if x == xminBox or x == xmaxBox or y == yminBox or y == ymaxBox:
                            self.setOutput(output, output_level, x, y, z, '%')

            # objects
            for object in self.state_.objects:
                if not object.lost and object.position.z == z:
                    self.setOutput(output, output_level, object.position.x, object.position.y, z, object.name[0])

            # drawer
            if self.drawerHeight - 1 == z:
                for x in range(xminDrawer, xmaxDrawer + 1):
                    for y in range(yminDrawer, ymaxDrawer + 1):
                        if x == xminDrawer or x == xmaxDrawer or y == yminDrawer or y == ymaxDrawer:
                            self.setOutput(output, output_level, x, y, z + 1, '%')
                        else:
                            self.setOutput(output, output_level, x, y, z, '*')
            if self.drawerHeight == z:
                for x in range(xmin, xmax + 1):
                    for y in range(ymin, ymax + 1):
                        self.setOutput(output, output_level, x, y, z, '#')

            # lid
            if self.state_.lid_position.z == z:
                for x in range(self.state_.lid_position.x - self.boxRadius,
                               self.state_.lid_position.x + self.boxRadius + 1):
                    for y in range(self.state_.lid_position.y - self.boxRadius,
                                   self.state_.lid_position.y + self.boxRadius + 1):
                        self.setOutput(output, output_level, x, y, z, '@')

            # gripper
            if self.state_.gripper_position.z == z:
                cx = self.state_.gripper_position.x
                cy = self.state_.gripper_position.y
                self.setOutput(output, output_level, cx-1, cy+1, z, '-')
                self.setOutput(output, output_level, cx+1, cy+1, z, '-')
                self.setOutput(output, output_level, cx-1, cy-1, z, '-')
                self.setOutput(output, output_level, cx+1, cy-1, z, '-')
                if self.state_.gripper_open:
                    self.setOutput(output, output_level, cx, cy-1, z, '-')
                    self.setOutput(output, output_level, cx, cy+1, z, '-')
                    self.setOutput(output, output_level, cx-1, cy, z, '[')
                    self.setOutput(output, output_level, cx+1, cy, z, ']')
                else:
                    self.setOutput(output, output_level, cx, cy+1, z, 'v')
                    self.setOutput(output, output_level, cx, cy-1, z, '^')
                    self.setOutput(output, output_level, cx-1, cy, z, '>')
                    self.setOutput(output, output_level, cx+1, cy, z, '<')

        # arm
        self.drawLine(output, output_level, self.tableWidth/2, 1, self.state_.gripper_position.x, self.state_.gripper_position.y, 4)
        self.drawLine(output, output_level, self.tableWidth/2 - 1, 1, self.state_.gripper_position.x, self.state_.gripper_position.y, 4)
        self.drawLine(output, output_level, self.tableWidth/2 + 1, 1, self.state_.gripper_position.x, self.state_.gripper_position.y, 4)
        self.drawLine(output, output_level, self.tableWidth/2, 0, self.state_.gripper_position.x, self.state_.gripper_position.y, 4)
        self.drawLine(output, output_level, self.tableWidth/2, 2, self.state_.gripper_position.x, self.state_.gripper_position.y, 4)

        # merge color levels into output buffer
        for i in range(len(output)):
            for j in range(len(output[i])):
                if output[i][j] == ' ':
                    continue

                if output_level[i][j] == 0:
                    pass
                elif output_level[i][j] == 1:
                    output[i][j] = '\033[1;33m' + output[i][j] + '\033[0m'
                elif output_level[i][j] == 2:
                    output[i][j] = '\033[33m' + output[i][j] + '\033[0m'
                elif output_level[i][j] == 3:
                    output[i][j] = '\033[31m' + output[i][j] + '\033[0m'
                else:
                    output[i][j] = '\033[35m' + output[i][j] + '\033[0m'

        # state
        output[0].append(' | Objects:')
        line_index = 1
        for object in self.state_.objects:
            output[line_index].append(' |  ' + object.name + ': (' + str(object.position.x) + ', '
                                      + str(object.position.y) + ', ' + str(object.position.z) + ')')
            line_index += 1

        output[line_index].append(' |')
        line_index += 1
        output[line_index].append(' | Gripper: (' + str(self.state_.gripper_position.x) + ', '
                                  + str(self.state_.gripper_position.y) + ', ' + str(self.state_.gripper_position.z)
                                  + ') ')
        if self.state_.gripper_open:
            output[line_index].append('(open)')
        else:
            output[line_index].append('(closed)')
        line_index += 1

        output[line_index].append(' |')
        line_index += 1
        output[line_index].append(' | Drawers:')
        line_index += 1
        output[line_index].append(' |  Stack center, height: (' + str(self.state_.drawer_position.x) + ', '
                                  + str(self.state_.drawer_position.y) + '), ' + str(self.drawerHeight))
        line_index += 1
        output[line_index].append(' |  Drawer opening: ' + str(self.state_.drawer_opening))
        line_index += 1

        output[line_index].append(' |')
        line_index += 1
        output[line_index].append(' | Box:')
        line_index += 1
        output[line_index].append(' |  Center, height: (' + str(self.state_.box_position.x) + ', '
                                  + str(self.state_.box_position.y) + '), ' + str(self.boxHeight))
        line_index += 1
        output[line_index].append(' |  Lid: (' + str(self.state_.lid_position.x) + ', '
                                  + str(self.state_.lid_position.y) + ', ' + str(self.state_.lid_position.z) + ')')

        # tic marks
        for i in range(len(output)):
            if self.tableDepth - i < 10:
                output[i].insert(0, ' ' + str(self.tableDepth - i) + '| ')
            else:
                output[i].insert(0, str(self.tableDepth - i) + '| ')
        tics = '    '
        labels = '    '
        for i in range(0, self.tableWidth + 1, 5):
            tics += '|    '
            labels += str(i)
            if i < 10:
                labels += '    '
            else:
                labels += '   '
        output.insert(0, [tics])
        output.insert(0, [labels])
        output.append([tics])
        output.append([labels])

        #  print
        if not self.quiet_mode:
            print('')
            for line in output:
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

            self.worldUpdate(action_msg)
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
