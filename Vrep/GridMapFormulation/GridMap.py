# Implement a class that contains the information of the now detected map
import vrep
import numpy as np
import math

class GridCell:
    """
    This class represents a single cell inside the total grid map
    Properties:
    Obstacle - a bool variable denotes whether there is an obstacle
    Height - height of the obstacle if exist otherwise -1
    location - tuple representing the absolute place in the map

    """
    def __init__(self, obstacle=False, height=-1, location):
        self.obstacle = obstacle
        self.height = height
        self.location = location

class GridMap:
    """
    This is a class denoting the observed graph distinguishing obstacles.
    Main Properties:
    Graph - a dictionary storing all the gridcells inside the graph
    Robot - the vrep robot handle
    Current position - current position of the robot

    """
    def __init__(self, size_x, size_y, initial_pos, robotHandle, origin, clientID):
        self.shape = (size_x, size_y)
        self.robot = robotHandle
        self.clientID = clientID
        current_pos = vrep.simxGetObjectPosition(clientID, self.robot, origin, vrep.simx_opmode_oneshot_wait)
        self.current_pos = (current_pos[0] / 0.1, current_pos[1] / 0.1)
        self.graph = {}
        self.robot_angle = 0
        self.floorPlace = 480

        self.graph[(0, 0)] = GridCell((0, 0))

    def refreshGraph(self, startX, startY, endX, endY, height):
        # refresh all the points between startX and startY with height
        if not height:
            # No obstacles
            angle = math.atan((endY - startY) / (endX - startX))
            self.graph[(startX, startY)] = GridCell((startX, startY))
            self.graph[(endX, endY)] = GridCell((endX, endY))
        else:
            # There are obstacles
            angle = math.atan((endY - startY) / (endX - startX))
            self.graph[(endX, endY)] = GridCell(True, height, (endX, endY))

    def observe(self, KinectDepth):
        KinectDepth = KinectDepth[:, :floorPlace]
        camXResolution = 640
        camYResolution = floorPlace
        camAngle = 57
        nearClippingPlane = 0.01
        fartherClippingPlane = 3.5
        camXHalfAngle = camXAngleInDegrees*0.5*math.pi/180
        camYHalfAngle = (camXAngleInDegrees*0.5*math.pi/180)*camYResolution/camXResolution

        for i in range(camXResolution):
            XAngle = (camXResolution / 2 - i) / camXResolution * camXHalfAngle * 2
            for j in range(camYResolution):
                YAngle = (camYResolution / 2 - j) / camYResolution * camYHalfAngle * 2
                if KinectDepth[i][j] == 255:
                    # There is no obstacles
                    armAngle = self.robot_angle + XAngle
                    halfArm = fartherClippingPlane * math.tan(camXHalfAngle)
                    endX = self.current_pos[0] + math.tan(armAngle) * fartherClippingPlane / math.cos(XAngle)
                    endX /= 0.1
                    endY = self.current_pos[1] + math.tan(armAngle) * fartherClippingPlane / math.cos(XAngle)
                    endY /= 0.1
                    startX = self.current_pos[0] + math.cos(math.pi / 2 - self.robot_angle) * halfArm
                    startX /= 0.1
                    startY = self.current_pos[1] + math.sin(math.pi / 2 - self.robot_angle) * halfArm
                    startY /= 0.1
                    self.refreshGraph(startX, startY, endX, endY, -1)
                else:
                    # There are some obstacles
                    depthValue = KinectDepth[i][j] / 255 * fartherClippingPlane
                    armAngle = self.robot_angle + XAngle
                    halfArm = depthValue * math.tan(camXHalfAngle)
                    endX = self.current_pos[0] + math.tan(armAngle) * depthValue / math.cos(XAngle)
                    endX /= 0.1
                    endY = self.current_pos[1] + math.tan(armAngle) * depthValue / math.cos(XAngle)
                    endY /= 0.1
                    startX = self.current_pos[0] + math.cos(math.pi / 2 - self.robot_angle) * halfArm
                    startX /= 0.1
                    startY = self.current_pos[1] + math.sin(math.pi / 2 - self.robot_angle) * halfArm
                    startY /= 0.1
                    self.refreshGraph(startX, startY, endX, endY, 1)

    def move(self, angle):
        current_pos = vrep.simxGetObjectPosition(clientID, self.robot, origin, vrep.simx_opmode_oneshot_wait)
        self.current_pos = (current_pos[0] / 0.1, current_pos[1] / 0.1)
        self.robot_angle += angle

    def isObstacle(self, x, y):
        return self.graph[(x, y)].obstacle
