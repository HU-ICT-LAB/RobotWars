"""Generic funtions used for the simulation."""
from typing import Tuple, Set
from math import sin, cos
import numpy as np

from ai_trainer.envs.tank_simulation.env_obj import EnvObj


def create_2d_rotation_matrix(angle: float) -> np.array:
    """
    Create 2d rotation of a matrix.

    :param angle: angle of rotation added.
    :return: returns new matrix
    """
    return np.array([
        [cos(angle), sin(angle)],
        [-sin(angle), cos(angle)],
    ])


def closest_collision(origin: np.array, collisions: Set[Tuple['EnvObj', np.array]]) -> Tuple['EnvObj', np.array]:
    """
    Calculate the closest collision from origin point.

    :param origin: Position of origen
    :param collisions: Collisions set
    :return: The closest collision
    """
    return min(collisions, key=lambda x: np.linalg.norm(x[1] - origin))
