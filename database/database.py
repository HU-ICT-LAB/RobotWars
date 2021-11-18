"""Connection to amazon web services relational database and some example functions."""
import os

import mysql.connector


def connect():
    """Connect to the database."""
    return mysql.connector.connect(host=os.environ.get("DB_URL"),
                                   user=os.environ.get("DB_USER"),
                                   password=os.environ.get("DB_PASSWORD"),
                                   database=os.environ.get("DB_NAME"))
    # These actually need to be github secrets.


def add_robot(name, modifications):
    """
    Add robot to the database.

    :param name: Name of the robot.
    :param modifications: Short description of the robot it's modifications.
    """
    perform_query("INSERT INTO robot (name, description_of_modifications) VALUES (%s,%s)", (name, modifications))


def add_policy(policy_description, policy_path):
    """
    Add policy to the database.

    :param policy_description: Short description of the policy.
    :param policy_path: The path of where the policy file is stored.
    """
    file = convert_file_to_binary(policy_path)
    perform_query("INSERT INTO policy (description, file) VALUES(%s,%s)", (policy_description, file))


def perform_query(query, values):
    """
    Perform any query.

    :param query: The query that needs to be executed.
    :param values: The values for the query.
    """
    db.cursor().execute(query, values)
    db.commit()


def convert_file_to_binary(path):
    """
    Convert file to binary data.

    :param path: Location of the file.
    :return: The file in binary.
    """
    with open(path, "rb") as file:
        binary_data = file.read()
    return binary_data


if __name__ == "__main__":
    db = connect()
