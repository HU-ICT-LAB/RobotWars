from typing import Tuple, Set
from math import sin, cos
import numpy as np

import ai_trainer.envs.tank_simulation.env_obj as env_obj


def create_2d_rotation_matrix(angle: float) -> np.array:
    return np.array([
        [cos(angle), sin(angle)],
        [-sin(angle), cos(angle)],
    ])


def closest_collision(origin: np.array, collisions: Set[Tuple['env_obj.EnvObj', np.array]]) -> Tuple['EnvObj', np.array]:
    return min(collisions, key=lambda x: np.linalg.norm(x[1] - origin))
