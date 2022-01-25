from typing import List, Tuple, Set, Optional
from math import sin, cos, tan, radians, pi
from pettingzoo import AECEnv
from pettingzoo.utils import agent_selector
from pettingzoo.utils.conversions import parallel_wrapper_fn
import numpy as np
import gym

import ai_trainer.envs.tank_simulation.env_obj as env_obj
import ai_trainer.envs.tank_simulation.tank as tank
from ai_trainer.envs.tank_simulation.utils import create_2d_rotation_matrix

EnvObj = env_obj.EnvObj
Tank = tank.Tank


class TankEnv(AECEnv):
    metadata = {
        'render_modes': ['rgb_array'],
        'name': "tanks_v1",
        'is_parallelizable': True
    }

    def __init__(self, step_size: float = 1 / 20, game_session_length: float = 20, canvas_square_size: int = 700,
                 arena_square_size: float = 5., n_lidar_rays: int = 20, n_tanks: int = 3,
                 max_drive_speeds: Tuple[float, float, float, float, float] = (2., 2., radians(300), radians(20), radians(20))):
        super().__init__()
        self.step_size = step_size  # 20 environment steps represent 1 second
        self.game_session_length = game_session_length  # length of one game/episode in seconds
        self.canvas_size = canvas_square_size, canvas_square_size
        self.arena_size = arena_square_size, arena_square_size  # meters
        self.n_lidar_rays = n_lidar_rays  # number of rays to simulate lidar
        self.n_tanks = n_tanks
        self.max_drive_speeds = max_drive_speeds  # chassis-x,y,z, gimbal-pitch,yaw

        self.environment_objects: List[EnvObj] = []
        self.time = 0.

        # Pettingzoo variables
        self.possible_agents = [f"tanks_{i}" for i in range(self.n_tanks)]
        self.agent_name_mapping = dict(zip(self.possible_agents, list(range(len(self.possible_agents)))))
        self._agent_selector = agent_selector(self.possible_agents)

    def observation_space(self, agent):
        return gym.spaces.Box(low=-1., high=1., shape=(self.n_lidar_rays + 10,))

    def action_space(self, agent):
        return gym.spaces.Box(low=-1., high=1., shape=(6,))

    @property
    def tanks(self) -> List[Tank]:
        return [x for x in self.environment_objects if isinstance(x, Tank)]

    def reset(self):
        self.environment_objects = [
            EnvObj((1., 1.), np.array([2., 2., radians(40)])),
            EnvObj((.4, .6), np.array([3.8, 3.7, radians(10)])),
        ]
        # Create tanks
        for agent in self.possible_agents:
            while True:
                tank = Tank(agent, np.random.rand(3) * (*self.arena_size, 2 * pi), np.random.rand(2) * (2 * pi))
                if not tank.colliding(self):
                    self.environment_objects.append(tank)
                    break

        self.time = 0.
        self.agents = self.possible_agents[:]
        self.rewards = {agent: 0 for agent in self.agents}
        self._cumulative_rewards = {agent: 0 for agent in self.agents}
        self.done = False
        self.dones = {agent: False for agent in self.agents}
        self.infos = {agent: {} for agent in self.agents}
        self.state = {agent: None for agent in self.agents}
        self.observations = {agent: None for agent in self.agents}

        '''
        Our agent_selector utility allows easy cyclic stepping through the agents list.
        '''
        self._agent_selector.reinit(self.agents)
        self.agent_selection = self._agent_selector.next()

    def render(self, mode="rgb_array", verbosity: int = 1):
        canvas_width, canvas_height = self.canvas_size
        canvas = np.zeros((canvas_width, canvas_height, 3))
        for environment_object in self.environment_objects:
            canvas = environment_object.render(canvas, self, verbosity)
        return canvas

    def observe(self, agent: str):
        observation = self.tanks[self.agent_name_mapping[agent]].observe(self)
        return observation

    def step(self, action):
        if self.dones[self.agent_selection]:
            self._was_done_step(None)
            return None
        agent_id = self.agent_name_mapping[self.agent_selection]
        tank = self.tanks[agent_id]

        self._cumulative_rewards[self.agent_selection] = 0

        # stores action of current agent
        self.state[self.agent_selection] = action

        tank.step(self, action)

        self.time += self.step_size

        if self.time >= self.game_session_length:
            self.done = True
            self.dones = {agent: True for agent in self.agents}

        self.agent_selection = self._agent_selector.next()

        self._accumulate_rewards()

    def shoot_ray(self, origin: np.array, ray_direction: float, ignore_objects: Set[EnvObj]) -> Set[Tuple[Optional[EnvObj], np.array]]:
        ox, oy = origin
        arena_width, arena_height = self.arena_size
        # Create set of intersection points, starting with the borders of the arena
        intersection_points = {
            *([(None, (ox + oy * np.tan(ray_direction, dtype=np.float64), 0))] if cos(ray_direction) > 0 else []),
            # top
            *([(None, (ox + (arena_height - oy) * np.tan(-ray_direction, dtype=np.float64), arena_height))] if cos(
                ray_direction) <= 0 else []),  # bottom
            *([(None, (0, oy + ox / np.tan(ray_direction, dtype=np.float64)))] if sin(ray_direction) <= 0 else []),
            # left
            *([(None, (arena_width, oy + (arena_width - ox) / np.tan(-ray_direction, dtype=np.float64)))] if sin(
                ray_direction) > 0 else []),  # right
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

                border_x = min(rect_x - rect_width / 2, rect_x + rect_width / 2, key=abs)
                border_y = min(rect_y - rect_height / 2, rect_y + rect_height / 2, key=abs)
                intersection_y = -border_x / tan(direction)
                intersection_x = -border_y * tan(direction)
                if rect_y - rect_height / 2 < intersection_y < rect_y + rect_height / 2 and sin(
                        direction) * border_x > 0:
                    intersect_point = np.array([border_x, intersection_y])
                elif rect_x - rect_width / 2 < intersection_x < rect_x + rect_width / 2 and -cos(
                        direction) * border_y > 0:
                    intersect_point = np.array([intersection_x, border_y])
                else:
                    continue
                intersection_points.add((obj, tuple(intersect_point @ create_2d_rotation_matrix(rect_angle) + origin)))

        return intersection_points


parallel_env = parallel_wrapper_fn(TankEnv)
