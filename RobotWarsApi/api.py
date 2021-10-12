from flask import Flask, jsonify

app = Flask(__name__)

"""
Gets all robot ip's and returns them
"""
@app.route("/api/v1/robots", methods=["GET"])
def get_robot_ips():
    ##TODO implement functionality
    return jsonify(["192.168.0.1", "192.168.0.2"])


"""
Starts all robots when /start is called
"""
@app.route("/api/v1/start")
def start_robots():
    ##TODO implement functionality
    return jsonify("Robot started")


if __name__ == "__main__":
    app.run(debug=True)
