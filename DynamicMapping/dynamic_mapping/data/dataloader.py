import os
import numpy as np

import data.utils as utils
from scene.pose_2d import Pose2D
from scene.lidar import Lidar

class Dataloader:
    def __init__(self, folder : str):
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

    def load_next_frame(self):
        """
        Loads the next available frame in the data folder. This is useful to read all the data,
        even if we are late compared to the frames timestamp

            :returns: The frame as a dictionnary or None if the reading is complete
        """

        # If the reading is complete, return None:
        if self.reading_complete or not self._load_next():
            return None
        
        # Else, return the next frame:
        return self.parse_frame(self.next_frame)
        
    def load_current_frame(self, time : float):
        """
        Loads data in the data folder until the next frame has a timestamp greater than
        the current time. This is useful to read data in realtime
        
            :param time: Current time in the simulation
            :returns: The current frame as a dictionnary or None if the reading is complete
        """

        if self.reading_complete:
            return None

        # Continue reading frames until the next frame has a timestamp greater than the current time:
        while self.next_frame is None or self.next_frame['timestamp'] <= time:
            if not self.load_next():
                return None
            
        # Return the current frame:
        return self.parse_frame(self.current_frame)
    
    def _load_next(self):
        """
        Loads the next available frame and updates self.current_frame and self.next_frame 
        
            :returns: If we managed to load a new frame
        """
        
        self.current_frame_id += 1
        filename = os.path.join(self.folder, f"frame_{self.current_frame_id + 1}.json")

        if not os.path.exists(filename):
            self.reading_complete = True
            return False
        
        self.current_frame = self.next_frame
        self.next_frame = utils.load_json(filename)

        return True
    
    def parse_frame(self, frame : dict):
        """
        Parses the given frame and gives the correct type for the objects in the dictionnary
        
            :returns: The parsed dictionnary
        """

        # Get the current pose of the robot:
        robot_pose = Pose2D(**frame['robot_pose'])

        # Get the observations from the LIDARs:
        lidars_observations = []
        for lidar_data in frame['lidars_data']:

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
