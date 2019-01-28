# Robot class to denote a robot is needed
from utils import utils
import math
import random

class Robot:
    """
    The class is denoting a robot
    properties:
    next_move - the decided next move of the robot
    graph - the self-constructed map of the robot by kinect
    angle -  the total turned angle of the robot
    TO BE ADDED
    functions:
    refreshGraph - to construct the map
    decideNextMove - based on algorithm to move next

    """
    def __init__(self):
        self.next_move = None
        self.graph = {}     # Elements with feature: (x, y):0/1; 1 represents for obstacles, 0 for free space
        self.currentPosition = utils.CurrentPosition
        self.angleRange = utils.KinectAngleRange
        self.robotAngle = utils.RobotAngle
        self.cameraAngle = utils.CameraAngle
        self.robotRadius = utils.RobotRadius
        self.gridSize = utils.GridRadius
        self.partition = utils.PartNum
        self.cameraDepth = utils.cameraDepth
        self.cameraLength = utils.cameraLength
        self.sampleRange = utils.sampleRange
        self.sampleNum = utils.sampleNum
        self.sampleIter = utils.sampleIter

    def refreshGraph(self, observed):
        """
        observed: A list of length self.partition, containing the distances from the current position to obstacles
                  -1 represents none, we assume the elements are all integers
        This function would update self.graph
        """
        angleUnit = self.angleRange / (self.partition - 1)
        minAngle = self.cameraAngle - self.angleRange / 2
        maxAngle = self.cameraAngle + self.angleRange / 2
        for i in range(self.partition):
            distance = observed[i]
            if distance < 0:
                newX = self.currentPosition[0] + self.cameraDepth * math.cos(minAngle + angleUnit * i)
                newY = self.currentPosition[1] + self.cameraDepth * math.sin(minAngle + angleUnit * i)
                freeX, freeY = self.renderMap(self.currentPosition[0], self.currentPosition[1], newX, newY)
                for x, y in freeX, freeY:
                    self.graph[(x, y)] = 0
            else:
                newX = self.currentPosition[0] + distance * math.cos(minAngle + angleUnit * i)
                newY = self.currentPosition[1] + distance * math.sin(minAngle + angleUnit * i)
                self.graph[(newX, newY)] = 1
                freeX, freeY = self.renderMap(self.currentPosition[0], self.currentPosition[1], newX, newY)
                freeX = freeX[:-1]
                freeY = freeY[:-1]
                for x, y in freeX, freeY:
                    self.graph[(x, y)] = 0


    def samplePositions(self):
        """
        Sample a point from nearby spaces
        """
        samples = []
        for i in range(self.sampleIter):
            x = random.randint(-self.sampleRange, self.sampleRange)
            y = random.randint(-self.sampleRange, self.sampleRange)
            x += self.currentPosition[0]
            y += self.currentPosition[1]
            if (x, y) in self.graph.keys():
                if self.graph[(x, y)] == 0:
                    samples.append((x, y))
        return samples

    def decideNextMove(self):
        samples = self.samplePositions()
        minEntropy = 999 # Need to be changed
        nextPosition = None
        for position in samples:
            totalEntropy = self.exploreEntropy(position) + self.exploitEntropy(position)
            if totalEntropy < minEntropy:
                minEntropy = totalEntropy
                nextPosition = position
        return nextPosition

    def getNextMove(self):
        # The platform class gets the next move from this function
        nextPosition = self.decideNextMove()
        moveDistance = math.sqrt((self.currentPosition[0] - nextPosition[0])**2 + (self.currentPosition[1] - nextPosition[1])**2)
        angleChange = float(nextPosition[1]-self.currentPosition[1]) / (nextPosition[0]-self.currentPosition[0]) - self.robotAngle
        return (angleChange, moveDistance)

    def coverMapWithRectangles(self):
        pass

    def exploreEntropy(self, position):
        """
        This entropy should be based on the nearby unexplored spaces and obstacles
        position: a tuple, (x, y)
        """
        pass


    def exploitEntropy(self, position):
        """
        Maybe more important after we form a loop? Not really sure what to do here
        """
        pass

    def renderMap(self, x1, y1, x2, y2):
    """
    Renders a line between (x1, y1) and (x2, y2), containing the starting point and end point
    Bresenham algorithm is used
    Copywrite from CSDN
    """
        pointList = []
        if x1 == x2: # Special Case: Horizenal Line
            if y1 <= y2:
                return [[x1, y] for y in range(y1, y2 + 1)]
            else:
                pointList = [[x1, y] for y in range(y2, y1 + 1)]
                pointList.reverse()
                return pointList
        elif abs(y2 - y1) <= abs(x2 - x1): # abs(slope) <= 1
            return self._Bresenham(x1, y1, x2, y2)
        else: # abs(slope) >= 1: Axis Reverse
            pointList = self._Bresenham(y1, x1, y2, x2)
            return [[p[1], p[0]] for p in pointList]

    def _Bresenham(self, x1, y1, x2, y2): # abs(slope) <= 1
        # Parameter of Drawing
        slope = float(y2 - y1) / (x2 - x1)
        # Initialize
        p = 2 * slope - 1
        [x, y] = [x1, y1]
        pointList = []
        if x1 < x2:
            if slope >= 0:
                # Real Bresenham Algorithm
                while True:
                    pointList.append([x, y])
                    if x == x2:
                        return pointList
                    if p <= 0:
                        [x, y, p] = [x + 1, y, p + 2 * slope]
                    else:
                        [x, y, p] = [x + 1, y + 1, p + 2 * slope - 2]
            else: # Up-Down Symmetry
                pointList = self._Bresenham(x1, -y1, x2, -y2)
                return [[p[0], -p[1]] for p in pointList]
        else:# Left-Right Symmetry
            pointList = self._Bresenham(x2, y2, x1, y1)
            pointList.reverse()
            return pointList
