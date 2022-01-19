from typing import List, Tuple, Set, Optional
from math import sin, cos, tan, radians, pi
from pettingzoo import AECEnv
from pettingzoo.utils import wrappers
from pettingzoo.utils.to_parallel import parallel_wrapper_fn
import numpy as np
import gym

import ai_trainer.envs.tank_simulation.env_obj as env_obj
import ai_trainer.envs.tank_simulation.tank as tank
from ai_trainer.envs.tank_simulation.utils import create_2d_rotation_matrix

EnvObj = env_obj.EnvObj
Tank = tank.Tank


def wrap_env(**kwargs):
    env = TankEnv(**kwargs)
    if env.continuous:
        env = wrappers.ClipOutOfBoundsWrapper(env)
    else:
        env = wrappers.AssertOutOfBoundsWrapper(env)
    env = wrappers.OrderEnforcingWrapper(env)
    return env


parallel_env = parallel_wrapper_fn(wrap_env)


class TankEnv(AECEnv):
    def state(self):
        pass

    def observe(self, agent):
        pass

    metadata = {
        'render_modes': ['rgb_array'],
        'name': "tanks_v1",
        'is_parallelizable': True
    }
    time = 0.

    def __init__(self, step_size: float = 1/20, game_session_length: float = 20, canvas_square_size: int = 700,
                 arena_square_size: float = 5., n_lidar_rays: int = 200, n_tanks: int = 3, max_drive_speeds: Tuple[float, float, float, float, float] = ()):
        self.step_size = step_size  # 20 environment steps represent 1 second
        self.game_session_length = game_session_length  # length of one game/episode in seconds
        self.canvas_size = canvas_square_size, canvas_square_size
        self.arena_size = arena_square_size, arena_square_size  # meters
        self.n_lidar_rays = n_lidar_rays  # number of rays to simulate lidar
        self.n_tanks = n_tanks
        self.max_drive_speeds = max_drive_speeds  # chassis-x,y,z, gimbal-pitch,yaw

        # x,y,z chassis. pitch,yaw gimbal. fire
        self.action_space = gym.spaces.Box(low=-1., high=1., shape=(6,))
        self.observation_space = gym.spaces.Box(low=-1., high=1., shape=(self.n_lidar_rays + 10,))
        # TODO: zo werd het gedaan in de piston
        self.tank_names = [f"tank_{i}" for i in range(self.n_tanks)]
        self.action_spaces = dict(zip(self.tank_names, [gym.spaces.Box(low=-1., high=1., shape=(6,)) * self.n_tanks]))
        self.observation_spaces = dict(zip(self.tank_names, [gym.spaces.Box(low=-1., high=1., shape=(self.n_lidar_rays + 10,)) * self.n_tanks]))

        self.environment_objects: List[EnvObj] = []

    @property
    def tanks(self) -> List[Tank]:
        return [x for x in self.environment_objects if isinstance(x, Tank)]  # noqa

    def reset(self):
        self.environment_objects = [
            EnvObj((1., 1.), np.array([2., 2., radians(40)])),
            EnvObj((.4, .6), np.array([3.8, 3.7, radians(10)])),
        ]
        while len(self.tanks) < self.n_tanks:
            tank = Tank(np.random.rand(3) * (*self.arena_size, 2*pi), np.random.rand(2) * (2*pi))
            if not tank.colliding(self):
                self.environment_objects.append(tank)

        self.time = 0.
        return self.tanks[0].observe(self)

    def render(self, mode="rgb_array", verbosity: int = 1):
        canvas_width, canvas_height = self.canvas_size
        canvas = np.zeros((canvas_width, canvas_height, 3))
        for environment_object in self.environment_objects:
            if environment_object is self.tanks[0]:
                environment_object: Tank
                canvas = environment_object.render(canvas, self, verbosity, color=(1, .2, .2))
            else:
                canvas = environment_object.render(canvas, self, verbosity)
        return canvas

    def step(self, action):
        tanks = self.tanks
        tanks_rewards = {tank: 0 for tank in tanks}
        for i, tank in enumerate(tanks):
            if i == 0:
                tanks_rewards = tank.step(self, action, tanks_rewards)
            else:
                tanks_rewards = tank.step(self, self.action_space.sample(), tanks_rewards)
                #tanks_rewards = tank.step(self, np.zeros(6), tanks_rewards)
        self.time += self.step_size

        return tanks[0].observe(self), tanks_rewards[tanks[0]], self.time >= self.game_session_length, {}

    def shoot_ray(self, origin: np.array, ray_direction: float, ignore_objects: Set[EnvObj]) -> Set[Tuple[Optional[EnvObj], np.array]]:
        ox, oy = origin
        arena_width, arena_height = self.arena_size
        # Create set of intersection points, starting with the borders of the arena
        intersection_points = {
            *([(None, (ox + oy * np.tan(ray_direction, dtype=np.float64), 0))] if cos(ray_direction) > 0 else []),  # top
            *([(None, (ox + (arena_height - oy) * np.tan(-ray_direction, dtype=np.float64), arena_height))] if cos(ray_direction) <= 0 else []),  # bottom
            *([(None, (0, oy + ox / np.tan(ray_direction, dtype=np.float64)))] if sin(ray_direction) <= 0 else []),  # left
            *([(None, (arena_width, oy + (arena_width - ox) / np.tan(-ray_direction, dtype=np.float64)))] if sin(ray_direction) > 0 else []),  # right
        }

        for obj in self.environment_objects:
            if obj not in ignore_objects:
                rect_x, rect_y, rect_angle = obj.rect_position
                rect_width, rect_height = obj.rect_shape
                rect_coord = np.array([rect_x, rect_y])

                rect_x, rect_y = -(origin - rect_coord) @ create_2d_rotation_matrix(-rect_angle)
                direction = ray_direction - rect_angle
                # We can now assume the origin is at 0,0 and rect is point up
                # rect_coord and ray_direction are compensated for this assumption

                border_x = min(rect_x - rect_width/2, rect_x + rect_width/2, key=abs)
                border_y = min(rect_y - rect_height/2, rect_y + rect_height/2, key=abs)
                intersection_y = -border_x / tan(direction)
                intersection_x = -border_y * tan(direction)
                if rect_y-rect_height/2 < intersection_y < rect_y+rect_height/2 and sin(direction) * border_x > 0:
                    intersect_point = np.array([border_x, intersection_y])
                elif rect_x-rect_width/2 < intersection_x < rect_x+rect_width/2 and -cos(direction) * border_y > 0:
                    intersect_point = np.array([intersection_x, border_y])
                else:
                    continue
                intersection_points.add((obj, tuple(intersect_point @ create_2d_rotation_matrix(rect_angle) + origin)))

        return intersection_points