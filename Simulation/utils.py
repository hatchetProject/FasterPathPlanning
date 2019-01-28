import math

class utils(object):
    """Record all the parameters  for the robot and the environment"""
    def __init__():
        self.KinectAngleRange = math.pi   # The angle range of the camera
        self.PartNum = 11                 # The number of directions for evaluating distance
        self.RobotRadius = 0.1            # robot raidus
        self.GridRadius = 0.1             # grid size
        self.RobotAngle = 0               # The robot is initially facing the positive x axis
        self.CameraAngle = 0              # The initial direction the camera is facing, same with the robot direction
        self.CurrentPosition = (0, 0)     # Denote the current position of the robot
        self.cameraDepth =
        self.cameraLength =

        # Sampling parameters, need to be tuned
        self.sampleRange = 10             # The range for sampling from the free space
        self.sampleNum = 10               # The maximum number of points to be sampled from the free space; Maybe not used...
        self.sampleIter = 120             # The number of iterations for sampling
