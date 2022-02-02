"""All api calls."""
import mysql.connector as connector
from flask import Flask, jsonify, request
from flask_cors import cross_origin
from src.game_leader.server.service import new_game, \
    get_active_games, all_robots


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
        response = {"success": False, "message": "request can't be made"}, 500
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
    except connector.errors.Error as e:
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
    except connector.errors.Error as e:
        response = {"success": False, "message": str(e)}, 404
    return jsonify(response, 200)


@app.route("/api/v1/start_robots")
def start_robots():
    """
    Start all robots.

    :return: robot started if successfully started
    """
    return jsonify("Robot started")


if __name__ == "__main__":
    app.run(debug=True)