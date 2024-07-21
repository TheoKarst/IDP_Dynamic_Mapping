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
                 radius : float = 10,
                 color : tuple = (0, 0, 0)):
        
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
        scene.draw_circle(lidar_pose.x, lidar_pose.y, self.radius, self.color)

        if draw_rays and self.observations is not None:
            for observation in self.observations:
                # Get the position of the observation:
                observation_x, observation_y = observation.compute_position(lidar_pose)

                color = (255, 0, 0)
                scene.draw_line(lidar_pose.x, lidar_pose.y, observation_x, observation_y, color)

    def update_observations(self, observations : list[Observation]):
        self.observations = observations