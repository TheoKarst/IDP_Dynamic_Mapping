import os
import numpy as np

import data.utils as utils
from scene.pose_2d import Pose2D
from scene.lidar import Lidar

class Dataloader:
    def __init__(self, folder):
        """ Instantiates a dataloader to load recorded data 
        
            :param folder: The folder in which the data is
        """

        self.folder = folder            # Folder from which the data is loaded
        self.current_frame_id = -2      # Index of the current frame of data
        self.current_frame = None       # JSON dictionnary containing the current frame
        self.next_frame = None          # JSON dictionnary containing the next frame
        self.reading_complete = False   # False if there are still frames to read

    def load_robot_setup(self):
        """
        Loads the data in the config file and return the initial setup of the robot as a dictionnary
        """

        # Load data from the config file:
        data = utils.load_json(os.path.join(self.folder, "config.json"))

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

        # Create a dictionnary to put the setup data for the robot:
        robot_config = {}
        robot_config['pose'] = robot_pose
        robot_config['lidars'] = lidars
            
        return robot_config

    def load_current_frame(self, time : float):
        """
        Loads data in the data folder until the next frame has a timestamp greater than
        the current time and return the current frame as a dictionnary or None if the
        reading is complete
        
            :param time: Current time in the simulation
        """

        if self.reading_complete:
            return None

        # Continue reading frames until the next frame has a timestamp greater than the current time:
        while self.next_frame is None or self.next_frame['timestamp'] <= time:
            self.current_frame_id += 1
            filename = os.path.join(self.folder, f"frame_{self.current_frame_id + 1}.json")

            if not os.path.exists(filename):
                self.reading_complete = True
                return None

            self.current_frame = self.next_frame
            self.next_frame = utils.load_json(filename)

        # Get the current pose of the robot:
        robot_pose = Pose2D(**self.current_frame['robot_pose'])

        # Get the observations from the LIDARs:
        lidars_observations = []
        for lidar_data in self.current_frame['lidars_data']:

            # Observations are represented using dictionnaries of ndarrays for performance:
            observations = {}
            observations['ranges'] = np.array(lidar_data['ranges'])
            observations['angles'] = np.array(lidar_data['angles'])

            lidars_observations.append(observations)

        # Return the frame as a dictionnary:
        data_frame = {}
        data_frame['robot_pose'] = robot_pose
        data_frame['lidars_observations'] = lidars_observations

        return data_frame