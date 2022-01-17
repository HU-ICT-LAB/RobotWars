import pygame
import time
import yaml
import math

aruco_len = 14
half_aruco_len = aruco_len / 2
border = 10
scale = 1  # scale of 1 draws a map where pixels are equal to cm
drivable_surface: pygame.Surface
empty = pygame.Color(0, 0, 0, 0)


def create_map(file_name) -> pygame.Surface:
    screen = pygame.display.set_mode((800, 600))
    screen.fill([255, 255, 255])
    with open(file_name, "r") as file:
        data = yaml.load(file, Loader=yaml.FullLoader)["aruco_codes"]

        print(data)
        # id = data[0]["id"].get(0)
        # print(id)
        for point in data:
            coord = [i / 10 for i in data[point]["coordinates"][0:2]]
            if data[point]["rotation"][1] == 90 or data[point]["rotation"][1] == 270:
                angle = data[point]["rotation"][0]
                # TODO maybe revert x and y. Is at the moment better in this order
                y1 = coord[0] + math.cos(math.radians(angle)) * half_aruco_len
                x1 = coord[1] + math.sin(math.radians(angle)) * half_aruco_len
                y2 = coord[0] - math.cos(math.radians(angle)) * half_aruco_len
                x2 = coord[1] - math.sin(math.radians(angle)) * half_aruco_len
                pygame.draw.line(surface=screen, color=[0, 0, 0], start_pos=(x1 + border, y1 + border),
                                 end_pos=(x2 + border, y2 + border), width=3)
            if data[point]["rotation"][1] == 0:
                rect = pygame.Rect((coord[0] - half_aruco_len, coord[1] - half_aruco_len), (aruco_len, aruco_len))
                pygame.draw.rect(screen, color=[0, 0, 0], rect=rect, width=2)
    pygame.display.update()
    return screen
