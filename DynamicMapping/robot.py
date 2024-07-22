import pygame as py
import math
from pose_2d import Pose2D
from lidar import Lidar
from observation import Observation
# from scene import Scene

class Robot:
    def __init__(self, 
                 pose : Pose2D, 
                 lidars : list[Lidar], 
                 width : float, 
                 height : float, 
                 color : tuple = (255, 0, 0)):
        
        self.color = color

        self.pose = pose
        self.lidars = lidars

        self.width = width
        self.height = height

        # Define a surface used to draw the robot on the screen:
        self.robot = py.Surface((width, height))

        # White should be rendered as transparent:
        self.robot.set_colorkey((255, 255, 255))

        self.robot.fill(color)

    def update(self, robot_pose : Pose2D, lidar_observations : list[list[Observation]]):
        self.pose = robot_pose

        for index, observations in enumerate(lidar_observations):
            self.lidars[index].update_observations(observations)

        # print("Robot pose: " + str(robot_pose))

    def draw(self, scene):
        scene.draw_surface(self.robot, self.pose.x, self.pose.y, math.degrees(self.pose.angle))

        for lidar in self.lidars:
            lidar.draw(scene, self.pose, True)