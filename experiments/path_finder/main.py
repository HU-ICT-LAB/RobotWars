"""
A simple path finding experiment using RTT on a given/drawn map.

Controls:
left-mouse-click: paint drivable space (white)
right-mouse-click: paint undrivable space (black)
g: set goal location to cursor position
r: set robot location to cursor position
"""

from typing import Tuple, List, Union, Collection
from math import sin, cos, ceil, radians
import threading
import pygame
import numpy as np
import cv2
from robomaster import robot

space_size = np.array((10, 10))   # space dimensions in meters
cell_size = 0.05  # pixel width and height in meters
screen = pygame.display.set_mode((700, 700))
max_step_size = 0.5     # maximum single step for pathfinding in meters
n_nodes = 5000  # amount of nodes the tree should build
distance_keeping = 0.3  # amount of distance to keep from obstacles in meters
robot_speed = 0.6   # in meters per second


class Robot:
    """Internal model of the robomaster robot."""

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

    def __init__(self,
                 location: Tuple[int, int] = (0, 0),
                 chassis_yaw: float = 0.,
                 turret_yaw: float = 0.,
                 color: Tuple[int, int, int] = (255, 0, 0)):
        self.location = location
        self.chassis_yaw = chassis_yaw
        self.turret_yaw = turret_yaw
        self.color = color
        self.virtual_offset = space_size / 2

    def draw(self, surface: pygame.Surface) -> None:
        """
        Draw this tank on the given surface visualizing the state of the internal model.

        :param surface: Pygame surface to draw on.
        """
        chassis_rotation_matrix = np.array([
            [cos(self.chassis_yaw), -sin(self.chassis_yaw)],
            [sin(self.chassis_yaw), cos(self.chassis_yaw)],
        ])
        turret_rotation_matrix = np.array([
            [cos(self.turret_yaw), -sin(self.turret_yaw)],
            [sin(self.turret_yaw), cos(self.turret_yaw)],
        ])
        transformed_chassis_polygon = (self.chassis_polygon @ chassis_rotation_matrix + self.location) * screen_factor
        pygame.draw.lines(surface, self.color, True, transformed_chassis_polygon, width=2)
        transformed_turret_polygon = (self.turret_polygon @ turret_rotation_matrix + self.location) * screen_factor
        pygame.draw.lines(surface, self.color, True, transformed_turret_polygon, width=2)


class Node:
    """One node in the tree structure."""

    def __init__(self, location: Tuple[float, float], parent: Union['Node', None], children: List['Node'] = None):
        self.location = location
        self.parent = parent
        if children is None:
            self.children = []
        else:
            self.children = list(children)

    def cost(self) -> float:
        """
        Compute the cost of getting to this node from the root.

        Aka the travel distance to the root.

        :return: The cost aka distance.
        """
        if self.parent is None:
            return 0.
        else:
            return self.parent.cost() + np.linalg.norm(np.subtract(self.location, self.parent.location))


class Tree:
    """Tree structure."""

    def __init__(self):
        self.tiles = {}  # key: tile-coord, value: list of nodes in the tile
        self.all_nodes = []

    def add_node(self, node: Node) -> None:
        """
        Add the given node to the tree.

        The tree groups the nodes into tiles to make searching faster.

        :param node: The node to add to the tree.
        """
        global max_step_size
        tile_coord = tuple(np.array(node.location) // max_step_size)
        if tile_coord not in self.tiles:
            self.tiles[tile_coord] = []
        self.tiles[tile_coord].append(node)
        self.all_nodes.append(node)

    def get_surrounding_nodes(self, location: Tuple[float, float]) -> List[Node]:
        """
        Get a list of nodes surrounding the given location.

        The returned list is guaranteed to contain all the nodes within a radius of `max_step_size`.
        A few nodes outside this circle can show up in the result.

        :param location: The 2D location to search around.
        :return: List of nodes.
        """
        nodes = []
        center_tile_coord = np.array(location) // max_step_size
        for y_offset in range(-1, 2):
            for x_offset in range(-1, 2):
                nodes.extend(self.tiles.get(tuple(center_tile_coord + (x_offset, y_offset)), []))
        if len(nodes) > 0:
            return nodes
        else:
            return self.all_nodes

    @staticmethod
    def closest_node(nodes: Collection[Node], location: Tuple[float, float]) -> Node:
        """
        Return the closest node to the given location in the given collection of nodes.

        :param nodes: Collection of nodes to search.
        :param location: 2D location.
        :return: Closest node.
        """
        return min(nodes, key=lambda x: np.linalg.norm(np.subtract(x.location, location)))

    def clear(self) -> None:
        """Remove all nodes from the tree."""
        self.tiles = {}
        self.all_nodes = []


def update_obstacle_surface() -> None:
    """Update the global drivable_surface."""
    global drivable_surface
    drivable_surface = pygame.transform.scale(
        pygame.surfarray.make_surface(
            np.stack([drivable_mask] * 3, axis=-1) * 255
        ),
        screen.get_rect().size
    )


def handle_position(position: Tuple[float, float, float]) -> None:
    """
    Handle the physical robot's position update.

    Namely, give new drive instructions to the robot by following the build tree.

    :param position: New position gotten from the physical robot.
    """
    global robot0, tree, robot_speed
    x, y, z = position
    robot0.location = np.array([y, -x]) + robot0.virtual_offset

    drive_speed = (0, 0, 0)
    if len(tree.all_nodes) > 0 and follow:
        closest = tree.closest_node(tree.get_surrounding_nodes(robot0.location), robot0.location)
        if closest.parent is not None:
            if not path_blocked(robot0.location, closest.parent.location) or \
                    np.linalg.norm(np.subtract(robot0.location, closest.location)) < 0.5:
                delta = np.subtract(closest.parent.location, robot0.location)
                speed_normalized = delta / np.linalg.norm(delta) * robot_speed
                rotation_matrix = np.array([
                    [cos(-robot0.chassis_yaw), -sin(-robot0.chassis_yaw)],
                    [sin(-robot0.chassis_yaw), cos(-robot0.chassis_yaw)],
                ])
                rotated = speed_normalized @ rotation_matrix
                drive_speed = -rotated[1], rotated[0], 0
    physical_robot.chassis.drive_speed(*drive_speed)


def handle_angle(angle) -> None:
    """
    Handle the physical robot's angle update.

    Namely, update the internal model of the robot.

    :param angle: New angles gotten from the physical robot.
    """
    global robot0
    pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle = angle
    robot0.turret_yaw = - radians(yaw_ground_angle)
    robot0.chassis_yaw = - radians(yaw_ground_angle - yaw_angle)


drivable_mask = np.ones((space_size / cell_size).astype(int))
drivable_surface: pygame.Surface
erode_kernel = np.ones([ceil(distance_keeping / cell_size)] * 2)
drivable_eroded = cv2.erode(drivable_mask, erode_kernel, iterations=1)

update_obstacle_surface()
brush_size = sum(space_size) / 2 * cell_size
screen_factor = np.array(screen.get_rect().size) / space_size
robot0 = Robot(location=space_size / 2)
goal = None
tree = Tree()
tree_building_thread = None
tree_building_terminate = False
follow = False


physical_robot = robot.Robot()
physical_robot.initialize(conn_type="sta")
print("connected")
physical_robot.set_robot_mode(robot.CHASSIS_LEAD)
physical_robot.gimbal.recenter()
physical_robot.chassis.sub_position(freq=20, callback=handle_position, cs=0)
physical_robot.gimbal.sub_angle(freq=20, callback=handle_angle)


pygame.init()
clock = pygame.time.Clock()
font = pygame.font.Font(pygame.font.get_default_font(), 15)
pygame.mouse.set_visible(False)


def path_blocked(point1: Tuple[float, float], point2: Tuple[float, float]) -> bool:
    """
    Compute weather or not the path between the two given points is blocked by an undrivable section in the map.

    :param point1: First point, either origin or destination, doesn't matter.
    :param point2: Second point, either origin or destination, doesn't matter.
    :return: Boolean representing weather the path is blocked or not.
    """
    # This part is modified from https://www.codegrepper.com/code-examples/python/python+bresenham+line+algorithm
    global drivable_eroded, cell_size
    blocked = False
    (x1, y1), (x2, y2) = (np.array(point1) / cell_size).astype(int), (np.array(point2) / cell_size).astype(int)
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        if not (0 <= coord[0] < drivable_eroded.shape[0] and 0 <= coord[1] < drivable_mask.shape[1]):
            blocked = True
            break
        elif not drivable_eroded[coord[0], coord[1]]:
            blocked = True
            break
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    return blocked


def build_tree() -> None:
    """Asynchronous process that builds the global tree structure for the RRT* algorithm."""
    global tree, max_step_size, n_nodes, space_size, tree_building_terminate
    tree_building_terminate = False
    while len(tree.all_nodes) < n_nodes and not tree_building_terminate:
        point = np.random.rand(2) * space_size
        neighbors = tree.get_surrounding_nodes(tuple(point))
        closest = tree.closest_node(neighbors, tuple(point))
        distance = np.linalg.norm(closest.location - point)
        if distance > max_step_size:
            point = closest.location + (point - closest.location) / distance * max_step_size
            neighbors = tree.get_surrounding_nodes(tuple(point))

        cheapest_parent = min(neighbors, key=lambda x: x.cost() + np.linalg.norm(np.subtract(x.location, point)))

        if not path_blocked(point, cheapest_parent.location):
            new_node = Node(point, cheapest_parent)
            cheapest_parent.children.append(new_node)
            tree.add_node(new_node)

            # RRT* rewire tree
            new_node_cost = new_node.cost()
            for neighbor in neighbors:
                # if it would lower cost to neighbor
                if neighbor.cost() > new_node_cost + np.linalg.norm(np.subtract(neighbor.location, point)):
                    # if not blocked
                    if not path_blocked(point, neighbor.location):
                        neighbor.parent = new_node

    if not tree_building_terminate:
        print("Done building tree")


def reset_tree() -> None:
    """Clear global tree, terminate tree building process and start a new one."""
    global tree, tree_building_thread, tree_building_terminate, goal
    tree_building_terminate = True
    if tree_building_thread is not None:
        tree_building_thread.join()
    tree.clear()
    tree.add_node(Node(tuple(goal), None))
    tree_building_thread = threading.Thread(target=build_tree)
    tree_building_thread.start()


def main() -> None:
    """Main/game loop."""
    global clock, drivable_mask, drivable_surface, robot0, brush_size, goal, tree_building_thread, \
        tree_building_terminate, drivable_eroded, follow

    screen_brush_size = None
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEWHEEL:
                brush_size = max(brush_size + event.y * cell_size, cell_size)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_g:
                    goal = (np.array(pygame.mouse.get_pos()) + screen_brush_size / 2) / screen_factor
                    reset_tree()
                if event.key == pygame.K_f:
                    follow = not follow
        keys = pygame.key.get_pressed()
        if keys[pygame.K_r]:
            robot0.location = (np.array(pygame.mouse.get_pos()) + screen_brush_size / 2) / screen_factor

        left_click, middle_click, right_click = pygame.mouse.get_pressed(3)
        if left_click or right_click:
            color = 1 if left_click else 0
            mouse_x, mouse_y = np.array(pygame.mouse.get_pos()) / screen.get_rect().size * drivable_mask.shape
            pixel_brush_size = brush_size / cell_size
            drivable_mask[int(mouse_x):int(mouse_x + pixel_brush_size), int(mouse_y):int(mouse_y + pixel_brush_size)] = color
            if goal is not None:
                reset_tree()
            update_obstacle_surface()
            drivable_eroded = cv2.erode(drivable_mask, erode_kernel, iterations=1)

        # Draw
        # draw obstacle mask
        screen.blit(drivable_surface, (0, 0))
        # draw cursor brush
        screen_brush_size = np.array([brush_size / cell_size] * 2) / drivable_mask.shape * screen.get_rect().size
        screen.fill([128] * 3, (*pygame.mouse.get_pos(), *screen_brush_size))
        # draw robot
        robot0.draw(screen)
        # draw goal and closest tree path
        if goal is not None:
            pygame.draw.circle(screen, (0, 255, 0), goal * screen_factor, 10)
            node = tree.closest_node(tree.get_surrounding_nodes(robot0.location), robot0.location)
            points = [np.array(node.location) * screen_factor]
            while node.parent is not None:
                node = node.parent
                points.append(np.array(node.location) * screen_factor)
            if len(points) >= 2:
                pygame.draw.lines(screen, (0, 0, 255), False, points, width=2)
            # for node in tree.all_nodes:
            #     pygame.draw.circle(screen, (255, 100, 0), node.location * screen_factor, 2)
        # draw n_nodes
        screen.blit(font.render(f"N_nodes: {len(tree.all_nodes)}", False, [128] * 3), (10, 10))
        # draw cursor location
        mouse_x, mouse_y = pygame.mouse.get_pos()
        screen.blit(font.render(f"({mouse_x / screen_factor[0]:.2f}, {mouse_y / screen_factor[1]:.2f})",
                                False, [128] * 3), (mouse_x, mouse_y - 17))

        # Update the window
        pygame.display.update()
        clock.tick(60)

    tree_building_terminate = True
    if tree_building_thread is not None:
        tree_building_thread.join()
    pygame.quit()
    physical_robot.chassis.unsub_position()
    physical_robot.close()


if __name__ == '__main__':
    main()
