import json
import os
from robot import Robot
from lidar import Lidar
from pose_2d import Pose2D
from observation import Observation

class Dataloader:
    def __init__(self, folder):
        self.folder = folder
        self.current_frame_id = -2
        self.current_frame = None
        self.next_frame = None
        self.reading_complete = False

    def load_robot_setup(self):
        data = self.load_data(os.path.join(self.folder, "config.json"))

        # Extract the initial pose of the robot from the data:
        robot_pose = Pose2D(**data['initial_robot_pose'])

        # Extract the initial pose of the LIDARs on the robot:
        lidars = []
        for setup in data['lidars_setup']:
            # Convert the pose from dict to a Pose2D:
            local_pose = Pose2D(**setup['local_pose'])

            lidar = Lidar(
                setup['id'], setup['name'], local_pose, 
                setup['min_range'], setup['max_range'])
            
            lidars.append(lidar)
            
        return Robot(robot_pose, lidars, 10, 20)

    def update(self, robot : Robot, time : float):
        if self.reading_complete:
            return

        while self.next_frame is None or self.next_frame['timestamp'] <= time:
            self.current_frame_id += 1
            filename = os.path.join(self.folder, f"frame_{self.current_frame_id + 1}.json")

            if not os.path.exists(filename):
                self.reading_complete = True
                return

            self.current_frame = self.next_frame
            self.next_frame = self.load_data(filename)

        if self.current_frame is not None:
            # Get the current pose of the robot:
            robot_pose = Pose2D(**self.current_frame['robot_pose'])

            # Get the observations from the LIDARs:
            lidar_observations = []
            for lidar_data in self.current_frame['lidars_data']:
                observations = []
                for range, angle in zip(lidar_data['ranges'], lidar_data['angles']):
                    observations.append(Observation(range, angle))

                lidar_observations.append(observations)

            robot.update(robot_pose, lidar_observations)

    def load_data(self, path : str):
        with open(path, "r") as f:
            return json.load(f)

