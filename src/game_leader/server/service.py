"""All service calls."""

from src.common.database import connect, perform_write_query

db = connect()


def new_game(game_information):
    """
    Create a new game.

    :param: information of the new game.
    :return: true if game is created.
    """
    perform_write_query("INSERT INTO game_session ("
                        "game_name, description, game_mode, "
                        "robot_1_team, robot_2_team, robot_3_team, "
                        "max_robot_hp, game_time, is_active) VALUES(%s,%s,%s,%s,%s,%s,%s,%s, %s)",
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


def active_games():
    """
    Get all active games from database.

    :return: All active games from database.
    """
    cursor = db.cursor()
    cursor.execute("SELECT game_name, description, robot_1_team, robot_2_team, robot_3_team"
                   " FROM game_session WHERE is_active = TRUE")
    games = []
    results = cursor.fetchall()
    for result in results:
        games.append({"name": result[0],
                      "description": result[1],
                      "robot1Team": result[2],
                      "robot2Team": result[3],
                      "robot3Team": result[4]})
    return games

