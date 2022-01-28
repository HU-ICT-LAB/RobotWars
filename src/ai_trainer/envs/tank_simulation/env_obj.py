"""Environment object to create obstacles from the environment."""
from typing import Tuple, Set, Optional
from math import atan2
from cv2 import cv2
import numpy as np
from shapely.geometry import Polygon
from shapely.strtree import STRtree

import ai_trainer.envs.tank_simulation.environment as environment
from ai_trainer.envs.tank_simulation.utils import create_2d_rotation_matrix


class EnvObj:
    """Environment object class."""

    def __init__(self, rect_shape: Tuple[float, float], rect_position: np.array):
        self.rect_shape = rect_shape  # width, height
        self.rect_position = rect_position  # x, y, angle

    def get_location(self) -> np.array:
        """
        Get location of the object.

        :return: array of the location.
        """
        x, y, z = self.rect_position
        return np.array([x, y])

    def render(self, canvas: np.array, env: 'environment.TankEnv', verbosity: int = 1) -> np.array:
        """
        Render the world of the environment.

        :param canvas: array of the canvas.
        :param env: the environment of the object.
        :param verbosity:
        :return: array of location and rotation of the object.
        """
        rect_width, rect_height = self.rect_shape
        rect_x, rect_y, rect_z = self.rect_position
        screen_factor = np.array(env.canvas_size) / np.array(env.arena_size)

        polygon = np.array([
            [-rect_width / 2, -rect_height / 2],
            [rect_width / 2, -rect_height / 2],
            [rect_width / 2, rect_height / 2],
            [-rect_width / 2, rect_height / 2],
        ])
        polygon = (polygon @ create_2d_rotation_matrix(rect_z) + np.array([rect_x, rect_y])) * screen_factor

        cv2.polylines(canvas, np.int32([polygon]), True, (255, 255, 255), 2)
        return canvas

    def angle_inside_ray(self, origin: np.array, direction: float) -> float:
        """
        Calculate angle of this object relative to the given ray.

        :param origin: origin point of the ray.
        :param direction: direction of angle.
        :return: radians angle relative to the ray.
        """
        adjacent, opposite = (self.get_location() - origin) @ create_2d_rotation_matrix(-direction)
        return atan2(opposite, adjacent)

    @property
    def collision_poly(self):
        """
        Set property of the environment object.

        :return: Polygon of the enviroment object.
        """
        rect_width, rect_height = self.rect_shape
        rect_x, rect_y, rect_z = self.rect_position
        base_polygon = np.array([
            [-rect_width / 2, -rect_height / 2],
            [rect_width / 2, -rect_height / 2],
            [rect_width / 2, rect_height / 2],
            [-rect_width / 2, rect_height / 2],
        ])
        return base_polygon @ create_2d_rotation_matrix(rect_z) + self.get_location()

    def colliding(self, tank_env: 'environment.TankEnv', ignore: Optional[Set['EnvObj']] = None) -> bool:
        """
        Check if objects are placed at the correct position.

        :param tank_env: Environment of the tanks.
        :param ignore: Objects that wil be ignored.
        :return: Return bool of checking.
        """
        if ignore is None:
            ignore = {self}
        else:
            ignore.add(self)
        tree = STRtree([Polygon(o.collision_poly) for o in tank_env.environment_objects if o not in ignore])
        return len(tree.query(Polygon(self.collision_poly))) > 0
