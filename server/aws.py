import mysql.connector


def connect():
    return mysql.connector.connect(host="robotwars.cviyowqzhluv.us-east-1.rds.amazonaws.com",
                                   user="RobotWars",
                                   passwd="ai_sd_ti_bim",
                                   database="robotwars")


def add_findings(findings):
    for finding in findings:
        db.cursor().execute("INSERT INTO findings (action, result) VALUES ('bla', 'bla')")
        db.commit()


def add_policy(description, policy_path):
    db.cursor().execute("INSERT INTO Policy (description, file) VALUES(%s,%s)", (description, file_to_binary(policy_path)))
    db.commit()


def file_to_binary(path):
    with open(path, "rb") as file:
        binary_data = file.read()
    return binary_data


if __name__ == "__main__":
    db = connect()

