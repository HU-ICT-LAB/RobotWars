"""All api calls."""
from flask import Flask, jsonify, request
from flask_cors import cross_origin
from src.game_leader.server.service import new_game


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
        response = {"success": False}, 500
    return jsonify(response)


@app.route("/api/v1/start_robots")
def start_robots():
    """
    Start all robots.

    :return: robot started if successfully started
    """
    return jsonify("Robot started")


if __name__ == "__main__":
    app.run(debug=True)
