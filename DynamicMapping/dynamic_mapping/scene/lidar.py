import numpy as np

from scene.pose_2d import Pose2D
# from scene.robot import Robot
# from scene.scene import Scene

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
        self.observations = {}

    def update(self, robot_pose : Pose2D, observations : dict):
        """ 
        Updates the global pose and the observations of the LIDAR, sorts
        observations by angle and clean inconsistent observations 
        """

        self.global_pose = self.local_pose.get_global_pose(robot_pose)
        
        # Remove observations with a range less than self.min_range:
        keep_indices = observations['ranges'] >= self.min_range
        self.observations['ranges'] = observations['ranges'][keep_indices]
        self.observations['angles'] = observations['angles'][keep_indices]

        # Sort observations by angle:
        sorted_indices = np.argsort(self.observations['angles'])
        self.observations['ranges'] = self.observations['ranges'][sorted_indices]
        self.observations['angles'] = self.observations['angles'][sorted_indices]

        # Clamp observations out of range:
        out_of_range = self.observations['ranges'] >= self.max_range
        self.observations['ranges'][out_of_range] = self.max_range
        self.observations['out_of_range'] = out_of_range


    def draw(self, scene : 'Scene', draw_rays : bool):
        """ Draws a circle representing the LIDAR and optionnaly the LIDAR rays """
        
        # Draw the LIDAR rays:
        if draw_rays and self.observations is not None:
            rays_color = (255, 0, 0)

            # For each observation, draw a line between the LIDAR and that observation:
            for position in self.observations_position():
                scene.draw_line(self.global_pose.x, self.global_pose.y, 
                                position[0], position[1], rays_color)

        # Draw the LIDAR itself:
        scene.draw_circle(self.global_pose.x, self.global_pose.y, self.radius, self.color)

    def observations_position(self):
        """
        Computes the global position of the observation made by the LIDAR
            :returns: The world position of the observations a ndarray
        """

        observations_count = len(self.observations['ranges'])

        # Compute the global angle of the observation:
        theta = self.global_pose.angle + self.observations['angles']

        positions = np.zeros((observations_count, 2))
        positions[:,0] = self.global_pose.x + self.observations['ranges'] * np.cos(theta)
        positions[:,1] = self.global_pose.y + self.observations['ranges'] * np.sin(theta)

        return positions

    