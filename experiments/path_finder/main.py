"""
A simple path finding experiment using RTT on a given/drawn map.
Controls:
left-mouse-click: paint drivable space (white)
right-mouse-click: paint undrivable space (black)
g: set goal location to cursor position
r: set robot location to cursor position
"""

from typing import Tuple, Sequence, Union
from math import sin, cos
import threading
import pygame
import numpy as np

space_size = np.array((5, 5))   # space dimensions in meters
cell_size = 0.05  # pixel width and height in meters
screen = pygame.display.set_mode((700, 700))
max_step_size = 0.5     # maximum single step for pathfinding in meters
n_nodes = 5000


class Robot:
    chassis_polygon = np.array([
        [-0.12, -0.16],
        [0.12, -0.16],
        [0.12, 0.16],
        [-0.12, 0.16],
    ])
    turret_polygon = np.array([
        [-0.03, -0.15],
        [0.03, -0.15],
        [0.03, 0.03],
        [-0.03, 0.03],
    ])

    def __init__(self,
                 location: Tuple[int, int] = (0, 0),
                 chassis_yaw: float = 0.,
                 turret_yaw: float = 0.,
                 color: Tuple[int, int, int] = (255, 0, 0)):
        self.location = location
        self.chassis_yaw = chassis_yaw
        self.turret_yaw = turret_yaw
        self.color = color

    def draw(self, surface: pygame.Surface):
        chassis_rotation_matrix = np.array([
            [cos(self.chassis_yaw), -sin(self.chassis_yaw)],
            [sin(self.chassis_yaw), cos(self.chassis_yaw)],
        ])
        turret_rotation_matrix = np.array([
            [cos(self.chassis_yaw + self.turret_yaw), -sin(self.chassis_yaw + self.turret_yaw)],
            [sin(self.chassis_yaw + self.turret_yaw), cos(self.chassis_yaw + self.turret_yaw)],
        ])
        transformed_chassis_polygon = (self.chassis_polygon @ chassis_rotation_matrix + self.location) * screen_factor
        pygame.draw.lines(surface, self.color, True, transformed_chassis_polygon, width=2)
        transformed_turret_polygon = (self.turret_polygon @ turret_rotation_matrix + self.location) * screen_factor
        pygame.draw.lines(surface, self.color, True, transformed_turret_polygon, width=2)


class Node:
    def __init__(self, location: Tuple[float, float], parent: Union['Node', None], cost: float, children: Sequence['Node'] = None):
        self.location = location
        self.parent = parent
        self.cost = cost
        if children is None:
            self.children = []
        else:
            self.children = list(children)


def update_obstacle_surface():
    global drivable_surface
    drivable_surface = pygame.transform.scale(
        pygame.surfarray.make_surface(
            np.stack([drivable_mask] * 3, axis=-1) * 255
        ),
        screen.get_rect().size
    )


pygame.init()
clock = pygame.time.Clock()
pygame.mouse.set_visible(False)
drivable_mask = np.ones((space_size / cell_size).astype(int))
drivable_surface: pygame.Surface
update_obstacle_surface()
brush_size = sum(space_size) / 2 * cell_size
screen_factor = np.array(screen.get_rect().size) / space_size
robot0 = Robot(location=space_size / 2)
goal = None
all_nodes = []
tree_building_thread = None
tree_building_terminate = False


def closest_node(point: np.array) -> Node:
    return min(all_nodes, key=lambda x: np.linalg.norm(x.location - point))


def build_tree():
    global all_nodes, max_step_size, n_nodes, space_size, tree_building_terminate, drivable_mask
    tree_building_terminate = False
    while len(all_nodes) < n_nodes and not tree_building_terminate:
        point = np.random.rand(2) * space_size
        closest = closest_node(point)
        distance = np.linalg.norm(closest.location - point)
        if distance > max_step_size:
            point = closest.location + (closest.location - point) / distance * max_step_size
            distance = max_step_size

        # This part is modified from https://www.codegrepper.com/code-examples/python/python+bresenham+line+algorithm
        # TODO: clean a bit up
        collision = False
        (x1, y1), (x2, y2) = (point / cell_size).astype(int), (np.array(closest.location) / cell_size).astype(int)
        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)

        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            if not (0 <= coord[0] < drivable_mask.shape[0] and 0 <= coord[1] < drivable_mask.shape[1]):
                collision = True
                break
            elif not drivable_mask[coord[0], coord[1]]:
                collision = True
                break
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx

        if not collision:
            new_node = Node(point, closest, closest.cost + distance)
            closest.children.append(new_node)
            all_nodes.append(new_node)
    if not tree_building_terminate:
        print("Done building tree")


def reset_tree():
    global all_nodes, tree_building_thread, tree_building_terminate, goal
    tree_building_terminate = True
    if tree_building_thread is not None:
        tree_building_thread.join()
    all_nodes = [Node(tuple(goal), None, 0)]
    tree_building_thread = threading.Thread(target=build_tree)
    tree_building_thread.start()


def main():
    global clock, drivable_mask, drivable_surface, robot0, brush_size, goal

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEWHEEL:
                brush_size = max(brush_size + event.y * cell_size, cell_size)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_g:
                    goal = (np.array(pygame.mouse.get_pos()) + screen_brush_size/2) / screen_factor
                    reset_tree()
                elif event.key == pygame.K_r:
                    robot0.location = (np.array(pygame.mouse.get_pos()) + screen_brush_size/2) / screen_factor

        left_click, middle_click, right_click = pygame.mouse.get_pressed(3)
        if left_click or right_click:
            color = 1 if left_click else 0
            mouse_x, mouse_y = np.array(pygame.mouse.get_pos()) / screen.get_rect().size * drivable_mask.shape
            pixel_brush_size = brush_size / cell_size
            drivable_mask[int(mouse_x):int(mouse_x + pixel_brush_size), int(mouse_y):int(mouse_y + pixel_brush_size)] = color
            if goal is not None:
                reset_tree()
            update_obstacle_surface()

        # Draw
        # draw obstacle mask
        screen.blit(drivable_surface, (0, 0))
        # draw cursor brush
        screen_brush_size = np.array([brush_size / cell_size] * 2) / drivable_mask.shape * screen.get_rect().size
        screen.fill([128]*3, (*pygame.mouse.get_pos(), *screen_brush_size))
        # draw robot
        robot0.draw(screen)
        # draw goal
        if goal is not None:
            pygame.draw.circle(screen, (0, 255, 0), goal * screen_factor, 10)
            node = closest_node(np.array(robot0.location))
            points = [np.array(node.location) * screen_factor]
            while node.parent is not None:
                node = node.parent
                points.append(np.array(node.location) * screen_factor)
            if len(points) >= 2:
                pygame.draw.lines(screen, (0, 0, 255), False, points)

        # Update the window
        pygame.display.update()
        clock.tick(60)

    pygame.quit()


if __name__ == '__main__':
    main()
