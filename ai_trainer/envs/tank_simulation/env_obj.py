from typing import Tuple, Set, Optional
from math import atan2
from cv2 import cv2
import numpy as np
from shapely.geometry import Polygon
from shapely.strtree import STRtree

import ai_trainer.envs.tank_simulation.environment as environment
from ai_trainer.envs.tank_simulation.utils import create_2d_rotation_matrix


class EnvObj:
    def __init__(self, rect_shape: Tuple[float, float], rect_position: np.array):
        self.rect_shape = rect_shape  # width, height
        self.rect_position = rect_position  # x, y, angle

    def get_location(self) -> np.array:
        x, y, z = self.rect_position
        return np.array([x, y])

    def render(self, canvas: np.array, env: 'environment.TankEnv') -> np.array:
        rect_width, rect_height = self.rect_shape
        rect_x, rect_y, rect_z = self.rect_position
        screen_factor = np.array(env.canvas_size) / np.array(env.arena_size)

        polygon = np.array([
            [-rect_width/2, -rect_height/2],
            [rect_width/2, -rect_height/2],
            [rect_width/2, rect_height/2],
            [-rect_width/2, rect_height/2],
        ])
        polygon = (polygon @ create_2d_rotation_matrix(rect_z) + np.array([rect_x, rect_y])) * screen_factor

        cv2.polylines(canvas, np.int32([polygon]), True, (255, 255, 255))
        return canvas

    def angle_inside_frustum(self, origin: np.array, direction: float) -> float:
        adjacent, opposite = (self.get_location() - origin) @ create_2d_rotation_matrix(-direction)
        return atan2(opposite, adjacent)

    @property
    def collision_poly(self):
        rect_width, rect_height = self.rect_shape
        rect_x, rect_y, rect_z = self.rect_position
        base_polygon = np.array([
            [-rect_width/2, -rect_height/2],
            [rect_width/2, -rect_height/2],
            [rect_width/2, rect_height/2],
            [-rect_width/2, rect_height/2],
        ])
        return base_polygon @ create_2d_rotation_matrix(rect_z) + self.get_location()

    def colliding(self, tank_env: 'environment.TankEnv', ignore: Optional[Set['EnvObj']] = None) -> bool:
        if ignore is None:
            ignore = {self}
        else:
            ignore.add(self)
        tree = STRtree([Polygon(o.collision_poly) for o in tank_env.environment_objects if o not in ignore])
        return len(tree.query(Polygon(self.collision_poly))) > 0
