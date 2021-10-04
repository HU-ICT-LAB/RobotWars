from flask import Flask

app = Flask(__name__)

"""
Gets all robot ip's and returns them
"""
@app.route("/robots", methods=["GET"])
def get_robot_ips():
    ips = []
    ##TODO implement functionality
    return str(ips)


"""
Starts all robots when /start is called
"""
@app.route("/start", methods=["POST"])
def start_robots():
    ##ToDo implement functionality
    return "Robot started"


if __name__ == "__main__":
    app.run(debug=True)
