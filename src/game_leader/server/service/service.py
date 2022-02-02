"""All service calls."""
import os
import time
import requests
import threading

import mysql.connector.errors as db_errors

from src.common.data.database import connect, perform_write_query

db = ""
robot1Ip = os.environ.get("ROBOT1IP")  # Set these variables in your configuration.
robot2Ip = os.environ.get("ROBOT2IP")
robot3Ip = os.environ.get("ROBOT3IP")


def make_database_connection():
    """Make the database connection for the service layer."""
    global db
    while True:
        try:
            db = connect()
            break
        except db_errors.Error as e:
            print(str(e) + "\nretry in 5 seconds...")
            time.sleep(5)


def new_game(game_information):
    """
    Create a new game and writes it to the database.

    Send "start" post requests to all of the robots.

    :param: information of the new game.
    :return: all robot responses.
    """
    response1 = requests.post(robot1Ip + "/api/v1/start")

    response2 = requests.post(robot2Ip + "/api/v1/start")

    response3 = requests.post(robot3Ip + "/api/v1/start")

    game_time = game_information["gameTime"]
    perform_write_query("INSERT INTO game_session ("
                        "game_name, description, game_mode, "
                        "robot_1_team, robot_2_team, robot_3_team, "
                        "max_robot_hp, game_time, is_active) "
                        "VALUES(%s,%s,%s,%s,%s,%s,%s,%s, %s)",
                        (game_information["name"],
                         game_information["description"],
                         game_information["gameMode"],
                         game_information["robot1Team"],
                         game_information["robot2Team"],
                         game_information["robot3Team"],
                         game_information["gameHP"],
                         game_time,
                         True), db)

    threading.Thread(target=game_timer, args=(game_time, ))

    return {"robot_1_response": response1,
            "robot_2_response": response2,
            "robot_3_response": response3}


def game_timer(game_time):
    """Start a timer for the game."""
    start = time.time()
    while time.time() - start < game_time:
        time.sleep(1)
    stop_all_robots()


def stop_all_robots():
    """Send requests to all robots to stop."""
    response1 = requests.post(robot1Ip + "/api/v1/stop")
    response2 = requests.post(robot2Ip + "/api/v1/stop")
    response3 = requests.post(robot3Ip + "/api/v1/stop")

    return {"robot_1_response": response1,
            "robot_2_response": response2,
            "robot_3_response": response3}


def get_active_games():
    """
    Get all active games from database.

    :return: All active games from database.
    """
    cursor = db.cursor()
    cursor.execute("SELECT * FROM game_session WHERE is_active = TRUE")
    games = []
    results = cursor.fetchall()
    for result in results:
        games.append({"name": result[3],
                      "description": result[1],
                      "robot1Team": result[4],
                      "robot2Team": result[5],
                      "robot3Team": result[6],
                      "gameTime": result[8],
                      "maxHp": result[7],
                      "gameMode": result[9]})
    return games


def get_all_robots():
    """
    Get all robots and their information from the database.

    :return: All robots in database.
    """
    cursor = db.cursor()
    cursor.execute("SELECT * FROM robot")
    robots = []
    results = cursor.fetchall()
    for result in results:
        robots.append({"name": result[1],
                       "description": result[2],
                       "hp": result[3],
                       "location": result[4]})
    return robots
