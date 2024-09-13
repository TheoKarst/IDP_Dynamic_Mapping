import numpy as np

import parameters.parameters as params

from scene.pose_2d import Pose2D
from scene.lidar import Lidar
# from scene.scene import Scene

class Robot:
    def __init__(self, 
                 pose : Pose2D, 
                 lidars : list[Lidar],
                 state_covariance : np.ndarray | None = None,
                 observation_covariance : np.ndarray | None = None,
                 width : float = 0.76,
                 height : float = 0.98, 
                 color : tuple = (255, 0, 0)):
        
        """
        Instantiates a new robot

            :param pose: Initial pose of the robot in world space
            :param lidars: List of LIDARs that are on the robot
            :param state_covariance: Covariance matrix of the state (x, y, angle)
                of the robot. This can be used if we want to assume a constant
                state covariance matrix or if this matrix is not given in the
                recorded data. If None, the matrix is fetched from the parameters file.
            :param observation_covariance: Covariance matrix of the range and angle
                of the observations. This can be used if we want to assume a
                constant observation covariance matrix or if this matrix is not given 
                in the recorded data. If None, the matrix is fetched from the 
                parameters file.
            :param width: Width of the robot in meters
            :param height: Height of the robot in meters
            :param color: Color of the robot (for dispay)
        """
        
        self.color = color

        self.pose = pose
        self.lidars = lidars

        self.state_covariance = state_covariance if state_covariance is not None \
            else params.state_covariance
        
        self.observation_covariance = observation_covariance if observation_covariance is not None \
            else params.observation_covariance
        
        self.width = width
        self.height = height

    def update(self, robot_pose : Pose2D, lidars_observations : list[dict], 
               state_covariance : np.ndarray | None = None,
               observation_covariance : np.ndarray | None = None):
        
        self.pose = robot_pose

        if state_covariance is not None:
            self.state_covariance = state_covariance

        if  observation_covariance is not None:
            self.observation_covariance = observation_covariance
        
        for index, observations in enumerate(lidars_observations):
            self.lidars[index].update(self.pose, observations)

    def draw(self, scene : 'Scene', draw_rays : bool):
        # When drawing the rectangle, width and height are inverted because when
        # the angle is zero, the robot should be oriented to the right:
        scene.draw_rectangle(self.pose.x, self.pose.y, 
                             self.height, self.width, self.pose.angle, self.color)

        for lidar in self.lidars:
            lidar.draw(scene, draw_rays)

    def get_sensor_pose(self, sensor_index : int):
        """ Returns the world pose of the sensor with the given index """

        return self.lidars[sensor_index].global_pose
    
    def get_state_covariance(self):
        """ Returns the estimated state covariance of the robot """
        
        return self.state_covariance
    
    def get_observations_positions(self, lidar_index : int):
        """
        Computes the position of the observations of the LIDAR with the given index
        """

        return self.lidars[lidar_index].observations_position()
    
    def get_observations_positions_covariances(self, lidar_index : int):
        """
        Computes for each observation of the LIDAR with the given index, the covariance 
        matrix of the position of the observations in world space
        """

        # Let Xr = (xr, yr,ar) denote the pose of the robot and Cxr denote the corresponding
        # state covariance matrix. For each observation S=(range=ro, angle=ao) from the LIDAR, 
        # the position of the observation Xo = (xo, yo) is a function of Xr and S: 
        # Xo = f(Xr, S).
        #
        # We can thus compute the covariance matrix Cp of an observation using:
        # Cp = F.Cxr.transpose(F) + G.Cs.transpose(G)
        # 
        # Where:
        #   F = Jacobian of f with respect to Xr
        #   G = Jacobian of f with respect to S
        #   Cs = diag([sigma_rr, sigma_aa]) is the covariance of S

        # Get the observations of the lidar:
        observations = self.lidars[lidar_index].observations
        observations_count = len(observations['ranges'])

        # Get the local pose of the LIDAR on the robot:
        lidar_local = self.lidars[lidar_index].local_pose
        xl, yl, al = lidar_local.x, lidar_local.y, lidar_local.angle

        # Get the range and angle of the observations:
        ro, ao = observations['ranges'], observations['angles']

        # The position of an observation is computed according to the following equation:
        # xo = xr + xl * cos(ar) - yl * sin(ar) + ro * cos(ar + al + ao)
        # yo = yr + xl * sin(ar) + yl * cos(ar) + ro * sin(ar + al + ao)

        # Intermediate computations:
        ar = self.pose.angle
        cos_ar, sin_ar = np.cos(ar), np.sin(ar)
        cos_ar_al_ao, sin_ar_al_ao = np.cos(ar + al + ao), np.sin(ar + al + ao)
        ro_cos_ar_al_ao, ro_sin_ar_al_ao = ro * cos_ar_al_ao, ro * sin_ar_al_ao

        # First compute the Jacobian of f with respect to Xr for all the observations:
        F = np.zeros((observations_count, 2, 3))
        F[:,:,:2] = np.identity(2)
        F[:,0,2] = -xl * sin_ar - yl * cos_ar - ro_sin_ar_al_ao     # Derivative of xo with respect to ar
        F[:,1,2] = xl * cos_ar - yl * sin_ar + ro_cos_ar_al_ao      # Derivative of yo with respect to ar

        # Then compute the Jacobian of f with respect to S for all the observations:
        G = np.zeros((observations_count, 2, 2))
        G[:,0,0] = cos_ar_al_ao         # Derivative of xo with respect to ro
        G[:,0,1] = -ro_sin_ar_al_ao     # Derivative of xo with respect to ao
        G[:,1,0] = sin_ar_al_ao         # Derivative of yo with respect to ro
        G[:,1,1] = ro_cos_ar_al_ao      # Derivative of yo with respect to ao

        # We can finally compute the covariance matrix of the position of each observation:
        # Cp = F.Cxr.transpose(F) + G.Cs.transpose(G)

        return F @ self.state_covariance @ np.transpose(F, axes=(0,2,1)) \
            + G @ self.observation_covariance @ np.transpose(G, axes=(0,2,1))

