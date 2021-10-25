"""All api calls."""
from flask import Flask, jsonify


app = Flask(__name__)


@app.route("/api/v1/robots", methods=["GET"])
def get_robot_ips():
    """
    Get all robot ip's.

    :return: robot ip's
    """
    return jsonify(["192.168.0.1", "192.168.0.2"])


@app.route("/api/v1/start")
def start_robots():
    """
    Start all robots.

    :return: robot started if successfully started
    """
    return jsonify("Robot started")


if __name__ == "__main__":
    app.run(debug=True)
