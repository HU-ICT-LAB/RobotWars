"""All api calls."""
import mysql.connector.errors as db_errors
from flask import Flask, jsonify, request
from flask_cors import cross_origin
from src.game_leader.server.service import new_game, \
    get_active_games, all_robots, make_database_connection


app = Flask(__name__)


@app.route("/api/v1/new_game", methods=["POST"])
@cross_origin()
def create_new_game():
    """
    Create a new game.

    :return: response corresponding the state of the request.
    """
    response = {"success": True}, 200
    if not new_game(request.json):
        print("create_new_game failed because the connection with the database cannot be made")
        response = {"success": False, "message": "Connection with database cannot be made"}, 500
    return jsonify(response)


@app.route("/api/v1/active_games", methods=["GET"])
@cross_origin()
def get_all_ongoing_games():
    """
    Get all ongoing games.

    :return: All active games.
    """
    try:
        response = get_active_games()
    except db_errors.Error as e:
        print("get_all_ongoing_games failed: " + str(e))
        response = {"success": False, "message": str(e)}, 404
    return jsonify(response, 200)


@app.route("/api/v1/robots", methods=["GET"])
@cross_origin()
def get_robots():
    """
    Get all robots and it's information.

    :return: All robots.
    """
    try:
        response = all_robots()
    except db_errors.Error as e:
        print("get_robots failed: " + str(e))
        response = {"success": False, "message": str(e)}, 404
    return jsonify(response, 200)


@app.route("/api/v1/start_robots", methods=["POST"])
def start_robots():
    """
    Start all robots.

    :return: robot started if successfully started
    """
    return jsonify("Robot started")


if __name__ == "__main__":
    app.run(debug=True)
    make_database_connection()
