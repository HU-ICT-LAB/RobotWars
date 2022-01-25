"""All service calls."""
import time
from src.common.database import connect, perform_write_query
import mysql.connector.errors as db_errors

db = ""


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
    Create a new game.

    :param: information of the new game.
    :return: true if game is created.
    """
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
                         game_information["gameTime"],
                         True), db)
    return True


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


def all_robots():
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
