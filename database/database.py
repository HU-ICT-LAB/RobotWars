import mysql.connector


def connect():
    return mysql.connector.connect(host="robotwars.cviyowqzhluv.us-east-1.rds.amazonaws.com",
                                   user="RobotWars",
                                   password="ai_sd_ti_bim",
                                   database="robotwars")


def add_robot(name, modifications):
    perform_query("INSERT INTO robot (name, description_of_modifications) VALUES (%s,%s)", (name, modifications))


def add_policy(policy_description, policy_path):
    file = convert_file_to_binary(policy_path)
    perform_query("INSERT INTO policy (description, file) VALUES(%s,%s)", (policy_description, file))


def perform_query(query, values):
    try:
        db.cursor().execute(query, values)
        db.commit()
    except Exception as e:
        print("query error: " + str(e))


def convert_file_to_binary(path):
    with open(path, "rb") as file:
        binary_data = file.read()
    return binary_data


if __name__ == "__main__":
    db = connect()
    add_robot("Henk", "9mm pistol upgrade")
