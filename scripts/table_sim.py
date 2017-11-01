#!/usr/bin/env python

# Python
from math import floor
from math import sqrt
from random import shuffle

# ROS
import rospy
from geometry_msgs.msg import Point
from numpy import sign
from task_sim.srv import Execute
from task_sim.msg import Action
from task_sim.msg import State
from task_sim.msg import Object

class TableSim:
    ## TODO: create a hidden state for whether an object can be picked up or not (dict with name as key and
    ## TODO: trinary state as value (yes, no, unknown)

    def __init__(self):
        self.init_simulation()

        self.action_service_ = rospy.Service('execute_action', Execute, self.execute)


    def init_simulation(self):
        '''Create a hardcoded 80x30 table with a few objects, a closed drawer, and a closed box'''
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


    def execute(self, req):
        if req.action == Action.GRASP:
            self.grasp(req.action.object)
        elif req.action == Action.PLACE:
            self.place(req.action.position)
        elif req.action == Action.OPEN_GRIPPER:
            self.open()
        elif req.action == Action.CLOSE_GRIPPER:
            self.close()
        elif req.action == Action.MOVE_ARM:
            self.move(req.position)
        elif req.action == Action.RAISE_ARM:
            self.raiseArm()
        elif req.action == Action.LOWER_ARM:
            self.lowerArm()
        elif req.action == Action.RESET_ARM:
            self.resetArm()


    def grasp(self, object):
        if not self.state_.gripper_open:
            self.open()

        # special cases
        if object == 'lid' or object == 'Lid':
            self.state_.gripper_position = self.copyPoint(self.state_.lid_position)
            self.state_.gripper_open = False
            self.state_.object_in_gripper = 'Lid'

        target = self.getObject(object)
        if target:
            # TODO: motion plan fail chance (proportional to distance)
            # TODO: object not in graspable position (hidden state)
            self.state_.gripper_position = self.copyPoint(target.position)
            self.state_.gripper_open = False
            self.state_.object_in_gripper = object
        else:
            self.error = 'Object ' + object + ' does not exist.'


    def place(self, position):
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
        print 'height for place: ' + str(height)
        if height == 4:  # place failed
            print 'place failed'
            return

        tempPos.z = height
        # TODO: motion plan fail chance
        self.moveGripper(tempPos)
        self.open()


    def open(self):
        self.state_.gripper_open = True
        if self.state_.object_in_gripper:
            if self.state_.object_in_gripper == 'Lid':
                self.state_.object_in_gripper = None
                self.gravity()
            else:
                object = self.getObject(self.state_.object_in_gripper)
                self.state_.object_in_gripper = None
                self.gravity(object)


    def close(self):
        if self.state_.gripper_open:
            self.state_.gripper_open = False
            o = self.getObjectAt(self.state_.gripper_position)
            if o:
                self.state_.object_in_gripper = o.name


    def move(self, position):
        points = self.interpolate(self.state_.gripper_position.x, self.state_.gripper_position.y, position.x, position.y)
        goal = self.copyPoint(self.state_.gripper_position)
        for point in points:
            testPos = Point(int(floor(point[0] + 0.5)), int(floor(point[1] + 0.5)), self.state_.gripper_position.z)
            if self.environmentCollision(testPos):
                break
            goal = self.copyPoint(testPos)

        # TODO: move objects towards goal (use interpolation and gravity calls)
        for object in self.state_.objects:
            if object.position.z == self.state_.gripper_position.z and self.distanceFromPath(object.position.x, object.position.y, self.state_.gripper_position.x, self.state_.gripper_position.y, goal.x, goal.y) < 1.2:
                object_goal = self.randomFreePoint(Point(goal.x, goal.y, goal.z), 2, 2)
                object_points = self.interpolate(object.position.x, object.position.y, object_goal.x, object_goal.y)
                for point in object_points:
                    test_pos = Point(int(floor(point[0] + 0.5)), int(floor(point[1] + 0.5)), self.state_.gripper_position.z)
                    if self.environmentCollision(testPos):
                        break
                    object.position = self.copyPoint(test_pos)
                    if self.gravity(object):
                        break

        # TODO: move gripper to goal
        self.moveGripper(goal)


    def interpolate(self, x0, y0, x1, y1):
        points = []
        if x0 == x1:
            # special case: vertical line
            step = float(y1 - y0) / 30
            for n in range(1, 30):
                points.append([x0, y0 + n*step])
        else:
            # general case
            step = float(x1 - x0) / 10
            for n in range(1, 10):
                x = x0 + n*step
                y = y0 + (x - x0)*float(y1 - y0)/(x1 - x0)
                points.append([x, y])
        points.append([x1, y1])
        return points


    def distanceFromPath(self, x, y, x1, y1, x2, y2):
        return abs(float((x2 - x1)*(y1 - y) - (x1 - x)*(y2 - y1))) \
               / sqrt(float(pow(x2 - x1, 2) + pow(y2 - y1, 2)))

    def raiseArm(self):
        if self.state_.gripper_position.z < 4:
            self.moveGripper(Point(self.state_.gripper_position.x,
                                   self.state_.gripper_position.y,
                                   self.state_.gripper_position.z + 1))


    def lowerArm(self):
        if self.state_.gripper_position.z == 0:
            return

        checkPos = Point(self.state_.gripper_position.x, self.state_.gripper_position.y,
                            self.state_.gripper_position.z - 1)
        if self.boxCollision(checkPos) or any(self.drawerCollision(checkPos)):
            return

        self.moveGripper(Point(self.state_.gripper_position.x,
                                   self.state_.gripper_position.y,
                                   self.state_.gripper_position.z - 1))


    def resetArm(self):
        resetPosition = Point()
        resetPosition.x = 8
        resetPosition.y = 1
        resetPosition.z = 2
        self.moveGripper(resetPosition)


    def gravity(self, object = None):
        change = False
        if object is None:
            # Apply gravity to everything
            change = self.gravityLid()
            for object in self.state_.objects:
                if self.state_.object_in_gripper != object.name:
                    change = change or self.gravity(object)
        else:
            tempPos = Point(object.position.x, object.position.y, object.position.z)
            while object.position.z > 0:
                tempPos.z -= 1
                if self.onEdge(tempPos) or self.objectCollision(tempPos):
                    # randomly select pose around the object that's out of collision
                    drop = self.randomFreePoint(tempPos, 2, 2)
                    if drop:
                        tempPos = drop
                        object.position.x = tempPos.x
                        object.position.y = tempPos.y
                        object.position.z = tempPos.z
                        continue
                    else:
                        break
                elif self.environmentCollision(tempPos):
                    break
                object.position.z -= 1
                change = True
        return change


    def gravityLid(self):
        change = False
        tempPos = self.copyPoint(self.state_.lid_position)
        while self.state_.lid_position.z > 0:
            tempPos.z -= 1
            collision = False
            for x in range(tempPos.x - self.boxRadius, tempPos.x + self.boxRadius + 1):
                for y in range(tempPos.y - self.boxRadius, tempPos.y + self.boxRadius + 1):
                    if self.inCollision(Point(x, y, tempPos.z)):
                        print 'Collision found for lid at height: ' + str(tempPos.z)
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
        self.state_.gripper_position = position
        if self.state_.object_in_gripper:
            if self.state_.object_in_gripper == 'Lid':
                ## Lid case
                objects = self.getObjectsOnLid()
                dx = position.x - self.state_.lid_position.x
                dy = position.y - self.state_.lid_position.y
                dz = position.z - self.state_.lid_position.z
                self.state_.lid_position = self.copyPoint(position)
                for object in objects:
                    object.position.x += dx
                    object.position.y += dy
                    object.position.z += dz
            else:
                self.getObject(self.state_.object_in_gripper).position = self.copyPoint(position)


    def copyPoint(self, point):
        copy = Point()
        copy.x = point.x
        copy.y = point.y
        copy.z = point.z
        return copy

    def getObject(self, name):
        for object in self.state_.objects:
            if object.name == name:
                return object
        return None


    def getObjectAt(self, position):
        for object in self.state_.objects:
            if object.position == position:
                return object
        return None


    def getObjectsOnLid(self):
        objects = []
        for x in range(self.state_.lid_position.x - self.boxRadius, self.state_.lid_position.x + self.boxRadius + 1):
            for y in range(self.state_.lid_position.y - self.boxRadius, self.state_.lid_position.y + self.boxRadius + 1):
                object = self.getObjectAt(Point(x, y, self.state_.lid_position.z + 1))
                if object:
                    objects.append(object)
        return objects


    def inCollision(self, position):
        return self.objectCollision(position) or self.environmentCollision(position)


    def environmentCollision(self, position):
        return self.boxCollision(position) or self.lidCollision(position) or any(self.drawerCollision(position))


    def checkLidCollision(self, position):
        return self.boxCollision(position) or any(self.drawerCollision(position)) or self.objectCollision(position)

    def onEdge(self, position):
        return self.boxCollision(position) or self.drawerCollision(position)[2]


    def objectCollision(self, position):
        for object in self.state_.objects:
            if object.position == position:
                return object
        return None


    def gripperCollision(self, position):
        return self.onBoxEdge(position,
                              self.state_.gripper_position.x - 1,
                              self.state_.gripper_position.x + 1,
                              self.state_.gripper_position.y - 1,
                              self.state_.gripper_position.y + 1,
                              self.state_.gripper_position.z,
                              self.state_.gripper_position.z)


    def boxCollision(self, position):
        return self.onBoxEdge(position,
                              self.state_.box_position.x - self.boxRadius,
                              self.state_.box_position.x + self.boxRadius,
                              self.state_.box_position.y - self.boxRadius,
                              self.state_.box_position.y + self.boxRadius,
                              self.state_.box_position.z,
                              self.state_.box_position.z + self.boxHeight - 1)


    def lidCollision(self, position):
        return self.inVolume(position,
                             self.state_.lid_position.x - self.boxRadius,
                             self.state_.lid_position.x + self.boxRadius,
                             self.state_.lid_position.y - self.boxRadius,
                             self.state_.lid_position.y + self.boxRadius,
                             self.state_.lid_position.z,
                             self.state_.lid_position.z)


    def drawerCollision(self, position):
        xmin, xmax, ymin, ymax, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer = self.getDrawerBounds()

        return [self.inVolume(position, xmin, xmax, ymin, ymax, 0, self.drawerHeight),
                self.inVolume(position, xminDrawer + 1, xmaxDrawer - 1, yminDrawer + 1, ymaxDrawer - 1, self.drawerHeight - 1,
                              self.drawerHeight - 1),
                self.onBoxEdge(position, xminDrawer, xmaxDrawer, yminDrawer, ymaxDrawer,
                               self.drawerHeight - 1, self.drawerHeight)]

    def getDrawerBounds(self):
        widthAdjustment = ((self.drawerWidth - 1)/2)
        depthAdjustment = ((self.drawerDepth - 1)/2)
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


    def onBoxEdge(self, position, xmin, xmax, ymin, ymax, zmin, zmax):
        return ((((position.x == xmin or position.x == xmax)
                  and position.y >= ymin
                  and position.y <= ymax)
                 or ((position.y == ymin or position.y == ymax)
                     and position.x >= xmin
                     and position.x <= xmax))
                and position.z >= zmin
                and position.z <= zmax)


    def inVolume(self, position, xmin, xmax, ymin, ymax, zmin, zmax):
        return (position.x >= xmin and position.x <= xmax
            and position.y >= ymin and position.y <= ymax
            and position.z >= zmin and position.z <= zmax)


    def show(self):
        output = []
        for y in range(0, self.tableDepth + 1):
            line = []
            for x in range(0, self.tableWidth + 1):
                line.append(' ')
            output.append(line)

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
            # drawer
            if self.drawerHeight - 1 == z:
                for x in range(xminDrawer, xmaxDrawer + 1):
                    for y in range(yminDrawer, ymaxDrawer + 1):
                        if x == xminDrawer or x == xmaxDrawer or y == yminDrawer or y == ymaxDrawer:
                            self.setOutput(output, x, y, '%')
                        else:
                            self.setOutput(output, x, y, '*')
            if self.drawerHeight == z:
                for x in range(xmin, xmax + 1):
                    for y in range(ymin, ymax + 1):
                        self.setOutput(output, x, y, '#')

            # box
            if self.state_.box_position.z == z:
                for x in range(xminBox, xmaxBox + 1):
                    for y in range(yminBox, ymaxBox + 1):
                        if not (x == xminBox or x == xmaxBox or y == yminBox or y == ymaxBox):
                            self.setOutput(output, x, y, '+')
            if self.boxHeight == z:
                for x in range(xminBox, xmaxBox + 1):
                    for y in range(yminBox, ymaxBox + 1):
                        if x == xminBox or x == xmaxBox or y == yminBox or y == ymaxBox:
                            self.setOutput(output, x, y, '%')
            if self.state_.lid_position.z == z:
                for x in range(self.state_.lid_position.x - self.boxRadius,
                               self.state_.lid_position.x + self.boxRadius + 1):
                    for y in range(self.state_.lid_position.y - self.boxRadius,
                                   self.state_.lid_position.y + self.boxRadius + 1):
                        self.setOutput(output, x, y, '@')

            # objects
            for object in self.state_.objects:
                if object.position.z == z:
                    self.setOutput(output, object.position.x, object.position.y, object.name[0])

            # gripper
            if self.state_.gripper_position.z == z:
                cx = self.state_.gripper_position.x
                cy = self.state_.gripper_position.y
                self.setOutput(output, cx-1, cy+1, '-')
                self.setOutput(output, cx+1, cy+1, '-')
                self.setOutput(output, cx-1, cy-1, '-')
                self.setOutput(output, cx+1, cy-1, '-')
                if self.state_.gripper_open:
                    self.setOutput(output, cx, cy-1, '-')
                    self.setOutput(output, cx, cy+1, '-')
                    self.setOutput(output, cx-1, cy, '[')
                    self.setOutput(output, cx+1, cy, ']')
                else:
                    self.setOutput(output, cx, cy+1, 'v')
                    self.setOutput(output, cx, cy-1, '^')
                    self.setOutput(output, cx-1, cy, '>')
                    self.setOutput(output, cx+1, cy, '<')

        # arm
        self.drawLine(output, self.tableWidth/2, 1, self.state_.gripper_position.x, self.state_.gripper_position.y)
        self.drawLine(output, self.tableWidth/2 - 1, 1, self.state_.gripper_position.x, self.state_.gripper_position.y)
        self.drawLine(output, self.tableWidth/2 + 1, 1, self.state_.gripper_position.x, self.state_.gripper_position.y)
        self.drawLine(output, self.tableWidth/2, 0, self.state_.gripper_position.x, self.state_.gripper_position.y)
        self.drawLine(output, self.tableWidth/2, 2, self.state_.gripper_position.x, self.state_.gripper_position.y)

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

        #TODO

        #  print
        print('')
        for line in output:
            print(''.join(line))

    def drawLine(self, output, x1, y1, x2, y2):
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
                self.setOutput(output, x, y, '$')
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

    def setOutput(self, output, x, y, value):
        if x >= 0 and x <= self.tableWidth and y >= 0 and y <= self.tableDepth:
            output[self.tableDepth - y][x] = value
            # TODO: can handle occlusions in here - if cell already has value, search object list and update occluded

    def getOutput(self, output, x, y):
        if x >= 0 and x <= self.tableWidth and y >= 0 and y <= self.tableDepth:
            return output[self.tableDepth - y][x]
        return ' '

    def getInput(self):
        action = raw_input('Action?: ').split(' ')
        if len(action) > 0:
            if action[0] == 'grasp' or action[0] == 'g':
                if len(action) >= 2:
                    self.grasp(action[1])
                else:
                    print('Grasp takes parameter: object')
            elif action[0] == 'place' or action[0] == 'p':
                if len(action) >= 3:
                    self.place(Point(int(action[1]), int(action[2]), 0))
                else:
                    print('Place takes parameters: x y')
            elif action[0] == 'open' or action[0] == 'o':
                self.open()
            elif action[0] == 'close' or action[0] == 'c':
                self.close()
            elif action[0] == 'home' or action[0] == 'h':
                self.resetArm()
            elif action[0] == 'raise' or action[0] == 'r':
                self.raiseArm()
            elif action[0] == 'lower' or action[0] == 'l':
                self.lowerArm()
            elif action[0] == 'move' or action[0] == 'm':
                if len(action) >= 3:
                    self.move(Point(int(action[1]), int(action[2]), 0))
                else:
                    print('Move takes parameters: x y')
            elif action[0] == 'quit' or action[0] == 'q':
                return False
            else:
                print('Invalid command.')
        else:
            print('Invalid command.')

        return True


if __name__ == '__main__':
    rospy.init_node('table_sim')
    table_sim = TableSim()


    while True:
        table_sim.show()
        if not table_sim.getInput():
            break
    #rospy.spin()
