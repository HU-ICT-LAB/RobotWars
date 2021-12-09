from typing import List, Tuple, Dict, Set
from math import sin, cos, tan, atan, radians, pi
import gym
import cv2
import numpy as np


def create_2d_rotation_matrix(angle: float) -> np.array:
    return np.array([
        [cos(angle), sin(angle)],
        [-sin(angle), cos(angle)],
    ])


def closest_collision(origin: np.array, collisions: Set[Tuple['EnvironmentObject', np.array]]) -> Tuple['EnvironmentObject', np.array]:
    return min(collisions, key=lambda x: np.linalg.norm(x[1] - origin))


class EnvironmentObject:
    def __init__(self, rect_shape: Tuple[float, float], rect_position: np.array):
        self.rect_shape = rect_shape  # width, height
        self.rect_position = rect_position  # x, y, angle

    def get_location(self) -> np.array:
        x, y, z = self.rect_position
        return np.array([x, y])

    def render(self, canvas: np.array, env: 'TankEnv', mode="human") -> np.array:
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
        return atan(opposite / adjacent)

    def colliding(self, other: 'EnvironmentObject'):
        pass


class TankEnv(gym.Env):
    metadata = {'render_modes': ['human']}
    time = 0.

    def __init__(self):
        self.step_size = 1 / 20  # 20 environment steps represent 1 second
        self.game_session_length = 60.  # length of one game/episode in seconds
        self.canvas_size = 700, 700
        self.arena_size = 5., 5.  # meters
        self.n_lidar_rays = 10  # number of rays to simulate lidar
        self.max_drive_speeds = 4., 4., radians(300), radians(90), radians(90)  # chassis-x,y,z, gimbal-pitch,yaw
        # x,y,z chassis. pitch,yaw gimbal. fire
        self.action_space = gym.spaces.Box(low=-1., high=1., shape=(6,))
        self.observation_space = gym.spaces.Box(low=-1., high=1., shape=(self.n_lidar_rays + 4,))

        self.environment_objects: List[EnvironmentObject] = []

    def reset(self):
        self.environment_objects = [
            Tank(np.array([5., 5., 1]), np.array([1, 1.])),
            EnvironmentObject((3., 1.), np.array([2., 4., 0.3])),
            Tank(np.array([1., 1., 2]), np.array([1, 1.])),
            Tank(np.array([3., 1., 1]), np.array([1, 1.])),
            Tank(np.array([3., 4., 1]), np.array([1, 1.])),
        ]
        self.time = 0.
        tanks: List[Tank] = list(filter(lambda x: isinstance(x, Tank), self.environment_objects))  # noqa
        return tanks[0].observe(self)

    def render(self, mode="human"):
        canvas_width, canvas_height = self.canvas_size
        canvas = np.zeros((canvas_width, canvas_height, 3))
        for environment_object in self.environment_objects:
            if environment_object is self.environment_objects[0]:
                canvas = environment_object.render(canvas, self, color=(0, 0, 255))
            else:
                canvas = environment_object.render(canvas, self)
        return canvas

    def step(self, action):
        tanks: List[Tank] = list(filter(lambda x: isinstance(x, Tank), self.environment_objects))  # noqa
        tanks_rewards = {tank: 0 for tank in tanks}
        for i, tank in enumerate(tanks):
            if i == 0:
                tanks_rewards = tank.step(self, action, tanks_rewards)
            else:
                tanks_rewards = tank.step(self, self.action_space.sample(), tanks_rewards)
        self.time += self.step_size

        return tanks[0].observe(self), tanks_rewards[tanks[0]], self.time >= self.game_session_length, {}

    def shoot_ray(self, origin: np.array, ray_direction: float, ignore_objects: Set[EnvironmentObject]) -> Set[Tuple[EnvironmentObject, np.array]]:
        intersection_points = set()

        for obj in self.environment_objects:
            if obj not in ignore_objects:
                rect_x, rect_y, rect_angle = obj.rect_position
                rect_width, rect_height = obj.rect_shape
                rect_coord = np.array([rect_x, rect_y])

                rect_coord += (origin - rect_coord) @ create_2d_rotation_matrix(-rect_angle)
                direction = ray_direction - rect_angle
                # We can now assume the origin is at 0,0 and rect is point up
                # rect_coord and ray_direction are compensated for this assumption

                rect_border_x = min(rect_x - rect_width/2, rect_x + rect_width/2, key=abs)
                rect_border_y = min(rect_y - rect_height/2, rect_y + rect_height/2, key=abs)
                intersection_y = rect_border_x * tan(direction)
                intersection_x = rect_border_y / tan(direction)
                if rect_y-rect_height/2 < intersection_y < rect_y+rect_height/2:
                    intersection_points.add((obj, (rect_border_x, intersection_y)))
                elif rect_x-rect_width/2 < intersection_x < rect_x+rect_width/2:
                    intersection_points.add((obj, (rect_border_y, intersection_x)))

        return intersection_points


class Tank(EnvironmentObject):
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
    fov = 90., 90.  # TODO: Find more realistic numbers

    def __init__(self, chassis_position: np.array, gimbal_position: np.array):
        super().__init__(
            rect_shape=(.24, .32),
            rect_position=chassis_position
        )
        self.gimbal_position = gimbal_position
        self.fire = False

    def step(self, tank_env: TankEnv, action, tanks_rewards: Dict['Tank', float]) -> Dict['Tank', float]:
        x, y, z, pitch, yaw, fire = action
        max_x, max_y, max_z, max_yaw, max_pitch = tank_env.max_drive_speeds
        # TODO: collisions
        self.rect_position += np.array([x, y, z]) * np.array([max_x, max_y, max_z]) * tank_env.step_size
        self.gimbal_position += np.array([pitch, yaw])
        self.rect_position[0] = min(max(self.rect_position[0], 0.), tank_env.arena_size[0])
        self.rect_position[1] = min(max(self.rect_position[1], 0.), tank_env.arena_size[1])

        self.fire = fire > .0
        if self.fire:
            t_x, t_y, t_z = self.rect_position
            gimbal_pitch, gimbal_yaw = self.gimbal_position
            origin = np.array([t_x, t_y])
            hits = tank_env.shoot_ray(origin, t_z + gimbal_yaw, {self})
            if len(hits) > 0:
                obj, collision_point = closest_collision(origin, hits)
                if isinstance(obj, Tank):
                    tanks_rewards[self] += 1
                    tanks_rewards[obj] -= 1

        return tanks_rewards

    def observe(self, tank_env: TankEnv) -> np.array:
        t_x, t_y, t_z = self.rect_position
        origin = np.array([t_x, t_y])
        gimbal_pitch, gimbal_yaw = self.gimbal_position

        # lidar
        lidar_output = np.zeros(tank_env.n_lidar_rays)
        ray_slice = 2*pi / tank_env.n_lidar_rays
        for i in range(tank_env.n_lidar_rays):
            direction = t_z + ray_slice * i
            hits = tank_env.shoot_ray(origin, direction, {self})
            if len(hits) > 0:
                obj, collision_point = closest_collision(origin, hits)
                lidar_output[i] = 1 / np.linalg.norm(collision_point - origin)

        # object detection
        tanks = filter(lambda x: isinstance(x, Tank) and x is not self, tank_env.environment_objects)
        insight = [
            tank
            for tank in tanks
            if -self.fov[0]/2 < tank.angle_inside_frustum(origin, t_z + gimbal_yaw) < self.fov[0]/2
        ]
        if len(insight) > 0:
            closest = min(insight, key=lambda x: np.linalg.norm(x.get_location() - origin))
            distance = np.linalg.norm(closest.get_location() - origin)
            bbox_center = np.array([
                closest.angle_inside_frustum(origin, t_z + gimbal_yaw) / (self.fov[0]/2),
                0.])
            bbox_size = np.array([1 / distance, 1 / distance])
        else:
            bbox_center = np.zeros(2)
            bbox_size = np.zeros(2)

        return np.concatenate([lidar_output, bbox_center, bbox_size])

    def render(self, canvas: np.array, env: 'TankEnv', mode="human", color=(255, 255, 255)) -> np.array:
        rect_x, rect_y, rect_z = self.rect_position
        pitch, yaw = self.gimbal_position
        screen_factor = np.array(env.canvas_size) / np.array(env.arena_size)

        polygon = (self.chassis_polygon @ create_2d_rotation_matrix(rect_z) + self.get_location()) * screen_factor
        cv2.polylines(canvas, np.int32([polygon]), True, color)
        polygon = (self.turret_polygon @ create_2d_rotation_matrix(rect_z + yaw) + self.get_location()) * screen_factor
        if self.fire:
            color = (255, 0, 0)
        cv2.polylines(canvas, np.int32([polygon]), True, color)

        return canvas
