import math
from pose_2d import Pose2D

class Observation:
    def __init__(self, range : float, angle : float):
        self.range = range
        self.angle = angle

    
    def compute_position(self, lidar_pose : Pose2D) -> tuple:
        """ From the pose of the LIDAR which made this observation, compute 
            the global position of the observation """
        
        theta = lidar_pose.angle + self.angle

        x = lidar_pose.x + self.range * math.cos(theta)
        y = lidar_pose.y + self.range * math.sin(theta)

        return (x, y)
