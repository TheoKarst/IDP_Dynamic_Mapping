import math

class Pose2D:
    def __init__(self, x : float, y : float, angle : float):
        self.x = x
        self.y = y
        self.angle = angle

    def get_global_pose(self, parent_pose : 'Pose2D') -> 'Pose2D':
        """ Compute the global pose of this pose """

        costheta = math.cos(parent_pose.angle)
        sintheta = math.sin(parent_pose.angle)
        
        return Pose2D(
            parent_pose.x + self.x * costheta - self.y * sintheta,
            parent_pose.y + self.x * sintheta + self.y * costheta,
            parent_pose.angle + self.angle)
    
    def __str__(self):
        return f"[x: {self.x}; y: {self.y}; angle: {math.degrees(self.angle)}°]"