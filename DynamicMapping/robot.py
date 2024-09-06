from pose_2d import Pose2D
from lidar import Lidar
from observation import Observation
# from scene import Scene

class Robot:
    def __init__(self, 
                 pose : Pose2D, 
                 lidars : list[Lidar], 
                 width : float = 0.76,
                 height : float = 0.98, 
                 color : tuple = (255, 0, 0)):
        
        """
        Instantiates a new robot

            :param pose: Initial pose of the robot in world space
            :param lidars: List of LIDARs that are on the robot
            :param width: Width of the robot in meters
            :param height: Height of the robot in meters
            :param color: Color of the robot (for dispay)
        """
        
        self.color = color

        self.pose = pose
        self.lidars = lidars

        self.width = width
        self.height = height

    def update(self, robot_pose : Pose2D, lidar_observations : list[list[Observation]]):
        self.pose = robot_pose
        
        for index, observations in enumerate(lidar_observations):
            self.lidars[index].update_observations(observations)

    def draw(self, scene):
        # When drawing the rectangle, width and height are inverted because when
        # the angle is zero, the robot should be oriented to the right:
        scene.draw_rectangle(self.pose.x, self.pose.y, 
                             self.height, self.width, self.pose.angle, self.color)

        for lidar in self.lidars:
            lidar.draw(scene, self.pose, True)