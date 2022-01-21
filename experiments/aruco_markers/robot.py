"""This file contains the model of the RoboMaster robot."""
from typing import Tuple
from math import sin, cos
import numpy as np
import pygame

screen_factor = np.array((1, 1))


class Robot:
    """Internal model of the robomaster robot."""

    chassis_polygon = np.array([
        [-0.12, -0.16],
        [0.12, -0.16],
        [0.12, 0.16],
        [-0.12, 0.16],
    ]) * 100
    turret_polygon = np.array([
        [-0.03, -0.3],
        [0.03, -0.3],
        [0.03, 0.03],
        [-0.03, 0.03],
    ]) * 100

    def __init__(self,
                 location: Tuple[int, int] = (0, 0),
                 chassis_yaw: float = 0.,
                 turret_yaw: float = 0.,
                 color: Tuple[int, int, int] = (255, 0, 0)):
        self.location = location
        self.chassis_yaw = chassis_yaw
        self.turret_yaw = turret_yaw
        self.color = color

    def draw(self, surface: pygame.Surface) -> None:
        """
        Draw this tank on the given surface visualizing the state of the internal model.

        :param surface: Pygame surface to draw on.
        """
        chassis_rotation_matrix = np.array([
            [cos(self.chassis_yaw), -sin(self.chassis_yaw)],
            [sin(self.chassis_yaw), cos(self.chassis_yaw)],
        ])
        turret_rotation_matrix = np.array([
            [cos(self.turret_yaw), -sin(self.turret_yaw)],
            [sin(self.turret_yaw), cos(self.turret_yaw)],
        ])
        transformed_chassis_polygon = (self.chassis_polygon @ chassis_rotation_matrix + self.location) * screen_factor
        pygame.draw.lines(surface, self.color, True, transformed_chassis_polygon, width=2)
        transformed_turret_polygon = (self.turret_polygon @ turret_rotation_matrix + self.location) * screen_factor
        pygame.draw.lines(surface, self.color, True, transformed_turret_polygon, width=2)
