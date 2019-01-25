# The simulation platform class is needed
from robot import Robot
import numpy as np

class SimulationPlatform:
    """
    The class is used to do simulation without robotic in vrep
    properties:
    width - the width of the puzzle
    height - the height of the puzzle
    robot - the simulation robot in the platform
    functions:
    constructPuzzle - to construct the puzzle
    observationArea - return the observed puzzle to the robot 

    """
    def __init__(self, width, height):
        self.robot = Robot()
        self.puzzle = np.zeros((width, height))
        self.observation = None

    def constructPuzzle(self):
        pass

    def observationArea(self):
        pass
