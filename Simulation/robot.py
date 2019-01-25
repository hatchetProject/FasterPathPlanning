# Robot class to denote a robot is needed

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
        self.graph = {}
        self.angle = 0

    def refreshGraph(self, observed):
        pass

    def decideNextMove(self):
        pass
