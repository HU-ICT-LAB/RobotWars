from typing import List, Tuple, Dict, Set, Optional
from math import sin, cos, tan, atan2, radians, pi, degrees
import gym
import cv2
import numpy as np
from shapely.geometry import Polygon
from shapely.strtree import STRtree


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


class TankEnv(gym.Env):
    metadata = {'render_modes': ['human']}
    time = 0.

    def __init__(self):
        self.step_size = 1 / 20  # 20 environment steps represent 1 second
        self.game_session_length = 60.  # length of one game/episode in seconds
        self.canvas_size = 700, 700
        self.arena_size = 5., 5.  # meters
        self.n_lidar_rays = 20  # number of rays to simulate lidar
        self.max_drive_speeds = 4., 4., radians(300), radians(90), radians(90)  # chassis-x,y,z, gimbal-pitch,yaw
        # x,y,z chassis. pitch,yaw gimbal. fire
        self.action_space = gym.spaces.Box(low=-1., high=1., shape=(6,))
        self.observation_space = gym.spaces.Box(low=-1., high=1., shape=(self.n_lidar_rays + 9,))

        self.environment_objects: List[EnvironmentObject] = []

    def reset(self):
        self.environment_objects = [
            Tank(np.array([1., 3., 0.]), np.array([0., pi/2])),
            Tank(np.array([2., 3., 0.]), np.array([1, 1.])),
            Tank(np.array([3., 3., 0.]), np.array([1, 1.])),
            Tank(np.array([4., 3., 0.]), np.array([1, 1.])),
            EnvironmentObject((1., 1.), np.array([2., 2., radians(40)])),
            EnvironmentObject((.4, .6), np.array([3.8, 3.7, radians(10)])),
        ]
        self.time = 0.
        tanks: List[Tank] = list(filter(lambda x: isinstance(x, Tank), self.environment_objects))  # noqa
        return tanks[0].observe(self)

    def render(self, mode="human"):
        canvas_width, canvas_height = self.canvas_size
        canvas = np.zeros((canvas_width, canvas_height, 3))
        for environment_object in self.environment_objects:
            if environment_object is self.environment_objects[0]:
                canvas = environment_object.render(canvas, self, color=(1, 0, 0))
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
                #tanks_rewards = tank.step(self, np.zeros(6), tanks_rewards)
        self.time += self.step_size

        return tanks[0].observe(self), tanks_rewards[tanks[0]], self.time >= self.game_session_length, {}

    def shoot_ray(self, origin: np.array, ray_direction: float, ignore_objects: Set[EnvironmentObject]) -> Set[Tuple[Optional[EnvironmentObject], np.array]]:
        ox, oy = origin
        arena_width, arena_height = self.arena_size
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
    fov = radians(70), radians(70)  # TODO: Find more realistic numbers

    def __init__(self, chassis_position: np.array, gimbal_position: np.array):
        super().__init__(
            rect_shape=(.24, .32),
            rect_position=chassis_position
        )
        self.gimbal_position = gimbal_position
        self.fire = False
        self.turret_temperature = 0
        self.max_temperature = 10

    def step(self, tank_env: TankEnv, action, tanks_rewards: Dict['Tank', float]) -> Dict['Tank', float]:
        x, y, z, pitch, yaw, fire = action
        max_x, max_y, max_z, max_yaw, max_pitch = tank_env.max_drive_speeds
        self.gimbal_position += np.array([pitch, yaw])
        # Collision
        tree = STRtree([Polygon(o.collision_poly) for o in tank_env.environment_objects if o is not self])
        new_pos = self.rect_position + np.array([x, y, z]) * np.array([max_x, max_y, max_z]) * tank_env.step_size
        new_pos[0] = min(max(new_pos[0], .1), tank_env.arena_size[0]-.1)
        new_pos[1] = min(max(new_pos[1], .1), tank_env.arena_size[1]-.1)
        if not tree.query(Polygon(EnvironmentObject(self.rect_shape, new_pos).collision_poly)):
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
                    tanks_rewards[self] += 1
                    tanks_rewards[obj] -= 1

        return tanks_rewards

    def simulate_lidar(self, tank_env: TankEnv) -> np.array:
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

    def observe(self, tank_env: TankEnv) -> np.array:
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
        angles = np.array([t_z, gimbal_pitch, gimbal_yaw]) / pi - 1

        return np.concatenate([lidar_output, bbox_center, bbox_size, location, angles])

    def render(self, canvas: np.array, env: 'TankEnv', mode="human", color=(255, 255, 255)) -> np.array:
        rect_x, rect_y, rect_z = self.rect_position
        origin = np.array([rect_x, rect_y])
        pitch, yaw = self.gimbal_position
        screen_factor = np.array(env.canvas_size) / np.array(env.arena_size)

        # chassis
        cv2.polylines(canvas, np.int32([self.collision_poly * screen_factor]), True, color)
        # turret
        polygon = (self.turret_polygon @ create_2d_rotation_matrix(rect_z + yaw) + self.get_location()) * screen_factor
        cv2.polylines(canvas, np.int32([polygon]), True, (0., 1 - (self.turret_temperature / self.max_temperature), 1.))

        # lidar
        ray_slice = (2 * pi) / env.n_lidar_rays
        for i in range(env.n_lidar_rays):
            direction = (rect_z + ray_slice * i) % (2 * pi)
            hits = env.shoot_ray(origin, direction, {self})
            if len(hits) > 0:
                obj, collision_point = closest_collision(origin, hits)
                cv2.line(canvas, tuple((origin * screen_factor).astype(int)), tuple((collision_point * screen_factor).astype(int)), (.2, .2, .2), 1)

        # fire
        if self.fire:
            hits = env.shoot_ray(origin, rect_z + yaw, {self})
            if len(hits) > 0:
                obj, collision_point = closest_collision(origin, hits)
                cv2.line(canvas, tuple((origin * screen_factor).astype(int)), tuple((collision_point * screen_factor).astype(int)),
                         (0, 0, 1), 1)

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
            cv2.line(canvas, tuple((origin * screen_factor).astype(int)), tuple((bbox * screen_factor).astype(int)), (0., 1., 0.), 1)
        cv2.line(canvas, tuple((origin * screen_factor).astype(int)), tuple(((origin + np.array([cos(rect_z+yaw-pi/2-self.fov[0]/2), sin(rect_z+yaw-pi/2-self.fov[0]/2)]))*screen_factor).astype(int)), (.5, .5, .5), 1)
        cv2.line(canvas, tuple((origin * screen_factor).astype(int)), tuple(((origin + np.array([cos(rect_z+yaw-pi/2+self.fov[0]/2), sin(rect_z+yaw-pi/2+self.fov[0]/2)]))*screen_factor).astype(int)), (.5, .5, .5), 1)

        return canvas
