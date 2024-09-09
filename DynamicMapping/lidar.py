import numpy as np
from pose_2d import Pose2D
# from robot import Robot
# from scene import Scene

class Lidar:
    def __init__(self, 
                 id : int,
                 name : str,
                 local_pose : Pose2D, 
                 min_range : float,
                 max_range : float,
                 radius : float = 0.075,
                 color : tuple = (0, 0, 0)):
        
        """
        Instantiates a new LIDAR

            :param id: Unique id of the LIDAR
            :param name: Name of the LIDAR
            :param local_pose: Pose of the LIDAR on the robot
            :param min_range: Min range of the LIDAR (in meters)
            :param max_range: Max range of the LIDAR (in meters)
            :param radius: Radius of the LIDAR in meters (for drawing)
            :param color: Color of the LIDAR (for drawing)
        """
        
        self.id = id
        self.name = name
        self.local_pose = local_pose
        self.global_pose = None
        self.min_range = min_range
        self.max_range = max_range
        self.radius = radius
        self.color = color
        
        # LIDAR observations are represented using a dict of ndarrays
        # for performance reasons:
        self.observations = None

    def update(self, robot_pose : Pose2D, observations : dict):
        """ Updates the global pose and the observations of the LIDAR """

        self.global_pose = self.local_pose.get_global_pose(robot_pose)
        self.observations = observations

    def draw(self, scene : 'Scene', draw_rays : bool):
        """ Draws a circle representing the LIDAR and optionnaly the LIDAR rays """
        
        # Draw the LIDAR rays:
        if draw_rays and self.observations is not None:
            rays_color = (255, 0, 0)

            for range, angle in zip(self.observations['ranges'], self.observations['angles']):
                # Get the position of the observation:
                observation_x, observation_y = self.observation_position(range, angle)

                # DRaw a line between the LIDAR and the observation position:
                scene.draw_line(self.global_pose.x, self.global_pose.y, 
                                observation_x, observation_y, rays_color)

        # Draw the LIDAR itself:
        scene.draw_circle(self.global_pose.x, self.global_pose.y, self.radius, self.color)

    def observation_position(self, range, angle):
        """
        Computes the global position of the given observation made by the LIDAR
        
            :param range: Range of the observation in meters
            :param angle: Angle between the LIDAR forward and the observation (counterclockwise),
                in radians
            :returns: The position of the observation as a tuple (x, y)
        """

        # Compute the global angle of the observation:
        theta = self.global_pose.angle + angle

        x = self.global_pose.x + range * np.cos(theta)
        y = self.global_pose.y + range * np.sin(theta)

        return x, y

    