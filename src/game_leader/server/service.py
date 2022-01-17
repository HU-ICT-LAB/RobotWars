"""All service calls."""

from src.common.database import connect, perform_query


def new_game(game_information):
    """
    Create a new game.

    :param: information of the new game.
    :return: true if game is created.
    """
    db = connect()
    perform_query("INSERT INTO game_session ("
                  "game_name, description, game_mode, "
                  "robot_1_team, robot_2_team, robot_3_team, "
                  "max_robot_hp, game_time) VALUES(%s,%s,%s,%s,%s,%s,%s,%s)",
                  (game_information["name"],
                   game_information["description"],
                   game_information["gameMode"],
                   game_information["robot1Team"],
                   game_information["robot2Team"],
                   game_information["robot3Team"],
                   game_information["gameHP"],
                   game_information["gameTime"]), db)
    return True
