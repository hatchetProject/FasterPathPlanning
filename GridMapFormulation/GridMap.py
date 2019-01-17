# Implement a class that contains the information of the now detected map
import vrep
import numpy as np

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
        self.current_pos = vrep.simxGetObjectPosition(clientID, self.robot, origin, vrep.simx_opmode_oneshot_wait)
        self.graph = {}

        self.graph[(0, 0)] = GridCell((0, 0))

    def observe(self, direction, distance):
        pass

    def isObstacle(self, x, y):
        return self.graph[(x, y)].obstacle
