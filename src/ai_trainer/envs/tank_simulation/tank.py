from typing import Dict
from math import sin, cos, radians, pi
from cv2 import cv2
import numpy as np

from ai_trainer.envs.tank_simulation.env_obj import EnvObj
import ai_trainer.envs.tank_simulation.environment as environment
from ai_trainer.envs.tank_simulation.utils import closest_collision, create_2d_rotation_matrix


class Tank(EnvObj):
    chassis_polygon = np.array([
        [-0.12, -0.16],
        [0.12, -0.16],
        [0.12, 0.16],
        [-0.12, 0.16],
    ])
    turret_polygon = np.array([
        [-0.03, -0.3],
        [0.03, -0.3],
        [0.03, 0.03],
        [-0.03, 0.03],
    ])
    fov = radians(70), radians(70)  # TODO: Find more realistic numbers

    def __init__(self, agent_name: str, chassis_position: np.array, gimbal_position: np.array):
        super().__init__(
            rect_shape=(.24, .32),
            rect_position=chassis_position
        )
        self.agent_name = agent_name
        self.gimbal_position = gimbal_position
        self.fire = False
        self.turret_temperature = 0
        self.max_temperature = 10

    def step(self, tank_env: "environment.TankEnv", action) -> None:
        x, y, z, pitch, yaw, fire = action
        max_x, max_y, max_z, max_yaw, max_pitch = tank_env.max_drive_speeds
        self.gimbal_position += np.array([pitch, yaw]) * np.array([max_pitch, max_yaw]) * tank_env.step_size
        self.gimbal_position[1] = min(max(self.gimbal_position[1], -1.5 * pi), 1.5 * pi)  # gimabl can't rotate more than 270 degrees
        # Collision
        new_pos = self.rect_position + np.array([x, y, z]) * np.array([max_x, max_y, max_z]) * tank_env.step_size
        new_pos[0] = min(max(new_pos[0], .1), tank_env.arena_size[0]-.1)
        new_pos[1] = min(max(new_pos[1], .1), tank_env.arena_size[1]-.1)
        new_pos[2] %= 2 * pi
        if not EnvObj(self.rect_shape, new_pos).colliding(tank_env, ignore={self}):
            self.rect_position = new_pos

        self.fire = False
        if fire > 0:
            if self.turret_temperature < self.max_temperature:
                self.fire = True
                self.turret_temperature += 1
        else:
            self.turret_temperature = max(self.turret_temperature-1, 0)
        if self.fire:
            t_x, t_y, t_z = self.rect_position
            gimbal_pitch, gimbal_yaw = self.gimbal_position
            origin = np.array([t_x, t_y])
            hits = tank_env.shoot_ray(origin, t_z + gimbal_yaw, {self})
            if len(hits) > 0:
                obj, collision_point = closest_collision(origin, hits)
                if isinstance(obj, Tank):
                    tank_env.rewards[self.agent_name] += 1
                    tank_env.rewards[obj.agent_name] -= 1

    def simulate_lidar(self, tank_env: "environment.TankEnv") -> np.array:
        t_x, t_y, t_z = self.rect_position
        origin = np.array([t_x, t_y])
        lidar_output = np.zeros(tank_env.n_lidar_rays)
        ray_slice = 2 * pi / tank_env.n_lidar_rays
        for i in range(tank_env.n_lidar_rays):
            direction = t_z + ray_slice * i
            hits = tank_env.shoot_ray(origin, direction, {self})
            if len(hits) > 0:
                obj, collision_point = closest_collision(origin, hits)
                lidar_output[i] = 1 / np.float64(np.linalg.norm(collision_point - origin))
        return lidar_output

    def observe(self, tank_env: "environment.TankEnv") -> np.array:
        t_x, t_y, t_z = self.rect_position
        origin = np.array([t_x, t_y])
        gimbal_pitch, gimbal_yaw = self.gimbal_position
        arena_width, arena_height = tank_env.arena_size

        # lidar
        lidar_output = self.simulate_lidar(tank_env)

        # object detection
        tanks = list(filter(lambda x: isinstance(x, Tank) and x is not self, tank_env.environment_objects))
        insight = [
            tank
            for tank in tanks
            if -self.fov[0] / 2 < tank.angle_inside_frustum(origin, t_z + gimbal_yaw - pi / 2) < self.fov[0] / 2
            # TODO: check if view is blocked
        ]
        if len(insight) > 0:
            closest = min(insight, key=lambda x: np.linalg.norm(x.get_location() - origin))
            distance = np.linalg.norm(closest.get_location() - origin)
            bbox_center = np.array([closest.angle_inside_frustum(origin, t_z + gimbal_yaw - pi / 2) / (self.fov[0]/2), 0.])
            bbox_size = np.array([min(distance/4, 1), min(distance/4, 1)])
        else:
            bbox_center = np.zeros(2)
            bbox_size = np.zeros(2)

        # location and angles
        location = np.array([t_x / arena_width * 2 - 1, t_y / arena_height * 2 - 1])
        angles = np.array([t_z, gimbal_pitch, gimbal_yaw]) / pi - 1  # TODO: normalize gimbal angles by their maximal rotation

        # turret temperature
        temperature = np.array([self.turret_temperature / self.max_temperature])

        # round time
        round_time = np.array([tank_env.time / tank_env.game_session_length])

        return np.concatenate([lidar_output, bbox_center, bbox_size, location, angles, temperature, round_time])

    def render(self, canvas: np.array, env: 'environment.TankEnv', verbosity: int = 1, color=(255, 255, 255)) -> np.array:
        rect_x, rect_y, rect_z = self.rect_position
        origin = np.array([rect_x, rect_y])
        pitch, yaw = self.gimbal_position
        screen_factor = np.array(env.canvas_size) / np.array(env.arena_size)

        # chassis
        cv2.polylines(canvas, np.int32([self.collision_poly * screen_factor]), True, color, 2)
        # turret
        polygon = (self.turret_polygon @ create_2d_rotation_matrix(rect_z + yaw) + self.get_location()) * screen_factor
        cv2.polylines(canvas, np.int32([polygon]), True, (0., 1 - (self.turret_temperature / self.max_temperature), 1.), 2)

        if verbosity >= 3:
            # lidar
            ray_slice = (2 * pi) / env.n_lidar_rays
            for i in range(env.n_lidar_rays):
                direction = (rect_z + ray_slice * i) % (2 * pi)
                hits = env.shoot_ray(origin, direction, {self})
                if len(hits) > 0:
                    obj, collision_point = closest_collision(origin, hits)
                    cv2.line(canvas, tuple((origin * screen_factor).astype(int)), tuple((collision_point * screen_factor).astype(int)), (.2, .2, .2), 2)

        # fire
        if self.fire:
            hits = env.shoot_ray(origin, rect_z + yaw, {self})
            if len(hits) > 0:
                obj, collision_point = closest_collision(origin, hits)
                cv2.line(canvas, tuple((origin * screen_factor).astype(int)), tuple((collision_point * screen_factor).astype(int)),
                         (0, 0, 1), 2)

        if verbosity >= 2:
            # object detection
            tanks = list(filter(lambda x: isinstance(x, Tank) and x is not self, env.environment_objects))
            insight = [
                tank
                for tank in tanks
                if -self.fov[0] / 2 < tank.angle_inside_frustum(origin, rect_z + yaw - pi/2) < self.fov[0] / 2
            ]
            if len(insight) > 0:
                closest = min(insight, key=lambda x: np.linalg.norm(x.get_location() - origin))
                distance = np.linalg.norm(closest.get_location() - origin)
                angle_inside = closest.angle_inside_frustum(origin, rect_z + yaw - pi/2)
                bbox = np.array([cos(angle_inside+rect_z+yaw - pi/2), sin(angle_inside+rect_z+yaw - pi/2)]) * (1/distance) + origin
                cv2.line(canvas, tuple((origin * screen_factor).astype(int)), tuple((bbox * screen_factor).astype(int)), (0., 1., 0.), 2)
            cv2.line(canvas, tuple((origin * screen_factor).astype(int)), tuple(((origin + np.array([cos(rect_z+yaw-pi/2-self.fov[0]/2), sin(rect_z+yaw-pi/2-self.fov[0]/2)]))*screen_factor).astype(int)), (.5, .5, .5), 2)
            cv2.line(canvas, tuple((origin * screen_factor).astype(int)), tuple(((origin + np.array([cos(rect_z+yaw-pi/2+self.fov[0]/2), sin(rect_z+yaw-pi/2+self.fov[0]/2)]))*screen_factor).astype(int)), (.5, .5, .5), 2)

        return canvas
