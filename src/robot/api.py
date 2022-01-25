"""All api calls. This api will be running on every robomaster."""
import os
import mysql.connector as connector

from flask_cors import cross_origin
from flask import Flask, jsonify

app = Flask(__name__)


@app.route("/api/v1/information")
@cross_origin()
def get_robot_information():
    """
    Get name, description, hp of the robot.

    :return: robot's hp
    """
    return jsonify({"name": "test", "description": "test", "hp": 50})


@app.route("/api/v1/start")
@cross_origin()
def start_robot():
    """
    Stop the robot.

    :return: robot stopped if successfully stopped
    """
    # ToDo Start robot.
    return jsonify("Robot stopped")


@app.route("/api/v1/stop")
@cross_origin()
def stop_robot():
    """
    Stop the robot.

    :return: robot stopped if successfully stopped
    """
    # ToDo Stop all activities of robot.
    return jsonify("Robot stopped")


@app.route("/api/v1/update-policy")
@cross_origin()
def update_policy():
    """
    Tells the robot to retrieve a new policy from the database.

    :return: policy updated if policy gets successfully updated
    """
    try:
        retrieve_latest_policy()
        status = "policy updated"
        # ToDo Save policy to robot here.
    except connector.errors.Error as e:
        print(e)
        status = "policy not updated"
    return jsonify(status)


def connect():
    """Connect to the database."""
    return connector.connect(host=os.environ.get("DB_URL"),
                             user=os.environ.get("DB_USER"),
                             password=os.environ.get("DB_PASSWORD"),
                             database=os.environ.get("DB_NAME"))


def retrieve_latest_policy():
    """Retrieve policy with given id."""
    cursor = db.cursor()
    cursor.execute("SELECT file FROM policy ORDER BY policy_id DESC LIMIT 0, 1")
    result = cursor.fetchone()[0]
    convert_binary_to_file(result, "py")


def convert_binary_to_file(binary_data, filetype):
    """
    Convert file to binary data and save the file.

    :param binary_data: The binary data of the file.
    :param filetype: The type of the file.
    """
    with open(f"policy.{filetype}", "wb") as file:
        file.write(binary_data)


def perform_query(query, values):
    """
    Perform any query.

    :param query: The query that needs to be executed.
    :param values: The values for the query.
    """
    # ToDo Use this to upload data such as findings when learning to the database.
    db.cursor().execute(query, values)
    db.commit()


if __name__ == "__main__":
    db = connect()
    app.run(debug=True)
