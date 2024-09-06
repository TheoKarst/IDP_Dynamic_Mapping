from pose_2d import Pose2D
from observation import Observation
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
        self.min_range = min_range
        self.max_range = max_range
        self.radius = radius
        self.color = color
        self.observations = None

    def draw(self, scene : 'Scene', robot_pose : Pose2D, draw_rays : bool):
        # Knowing the pose of the robot, compute the global pose of the LIDAR:
        lidar_pose = self.local_pose.get_global_pose(robot_pose)
        
        # Draw the LIDAR rays:
        if draw_rays and self.observations is not None:
            for observation in self.observations:
                # Get the position of the observation:
                observation_x, observation_y = observation.compute_position(lidar_pose)

                color = (255, 0, 0)
                scene.draw_line(lidar_pose.x, lidar_pose.y, observation_x, observation_y, color)

        # Draw the LIDAR itself:
        scene.draw_circle(lidar_pose.x, lidar_pose.y, self.radius, self.color)

    def update_observations(self, observations : list[Observation]):
        self.observations = observations