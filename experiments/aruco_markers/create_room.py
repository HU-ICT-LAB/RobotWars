import yaml


def create_room(name: str) -> None:
    print("The aruco code is the number the picture represents, for example 3.\n"
          "The x, y, z coordinates in meters where the picture is, for example [1.2, 3.4, 0.2].\n"
          "The pitch is the horizontal rotation in degrees, for example: 90 which is a rotation to the right.\n"
          "The yaw is the vertical rotation in degrees, for example: 90 which is a flip on the back from north to "
          "south.\n "
          "The yaw is the vertical rotation in degrees, for example: 180 which is a roll on the back from west to "
          "east.\n "
          "-------------------------------------")
    data = {"aruco_codes": []}
    try:
        while True:
            aruco_id = int(input("Input aruco id number:"))
            x_coord = float(input("  Input the x-coordinate:"))
            y_coord = float(input("  Input the y-coordinate:"))
            z_coord = float(input("  Input the z-coordinate:"))
            pitch = float(input("  Input the pitch in degrees:"))
            yaw = float(input("  Input the yaw in degrees:"))
            roll = float(input("  Input the roll in degrees:"))
            point = {"id": aruco_id, "coordinates": [x_coord, y_coord, z_coord], "rotation": [pitch, yaw, roll]}
            data["aruco_codes"].append(point)

    except KeyboardInterrupt:
        print("keyboard")
        pass
    with open(f"{name}.yaml", "w") as file:
        yaml.dump(data, file)


def create_room_direct(name: str, data: dict) -> None:
    with open(f"{name}.yaml", "w") as file:
        yaml.dump(data, file)


# example = {'aruco_codes': [{'id': int, 'coordinates': [x, y, z], 'rotation': [pitch, yaw, roll]}]}
data = {'aruco_codes': [{'id': 0, 'coordinates': [1.5, 2.5, 0], 'rotation': [0, 90.0, 0.0]},
                        # TODO the yaw is maybe not right
                        {'id': 1, 'coordinates': [0, 4.5, 0.3], 'rotation': [90.0, 0.0, 0.0]},
                        {'id': 2, 'coordinates': [0, 2.5, 0.3], 'rotation': [90.0, 0.0, 0.0]},
                        {'id': 3, 'coordinates': [0, 0.5, 0.3], 'rotation': [90.0, 0.0, 0.0]},
                        {'id': 4, 'coordinates': [0.5, 0, 0.3], 'rotation': [90.0, 0.0, 0.0]},
                        {'id': 5, 'coordinates': [2.5, 0, 0.3], 'rotation': [90.0, 0.0, 0.0]},
                        {'id': 6, 'coordinates': [3, 0.5, 0.3], 'rotation': [270.0, 0.0, 0.0]},
                        {'id': 7, 'coordinates': [3, 2.5, 0.3], 'rotation': [270.0, 0.0, 0.0]}]}
new_data = {'aruco_codes': {0: {'coordinates': [240, 0, 200], 'rotation': [0.0, 90.0, 0.0]},  # Distance in mm
                            1: {'coordinates': [0, 620, 200], 'rotation': [90.0, 90.0, 0.0]}},
            }
test2 = {'aruco_codes': {0: {'coordinates': [0, 210, 200], 'rotation': [90.0, 90.0, 0.0]},  # Distance in mm
                         1: {'coordinates': [500, 0, 200], 'rotation': [0.0, 90.0, 0.0]}},
         }

test3 = {'aruco_codes': {0: {'coordinates': [0, 4100, 200], 'rotation': [90.0, 90.0, 0.0]},  # Distance in mm
                         1: {'coordinates': [500, 2000, 200], 'rotation': [0.0, 90.0, 0.0]}},
         }
old_format = {'aruco_codes': {0: {'coordinates': [240, 0, 200], 'rotation': [0.0, 90.0, 0.0]},  # Distance in mm
                              1: {'coordinates': [0, 620, 200], 'rotation': [90.0, 90.0, 0.0]}},
              }

create_room_direct("test_corner3", test3)
# create_room("room3")
print("done")
