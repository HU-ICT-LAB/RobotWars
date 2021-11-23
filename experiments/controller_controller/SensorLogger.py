"""This program consists of the class that logs and saves the sensor data for experience data."""
import datetime
from database import database

class SensorLogger:
    """This class creates the logs the sensor data and either saves the data."""

    def __init__(self, title: str):  # todo is this title necessary
        self.title = title
        self.filename = f"{title}.csv"
        self.relative_yaw = 0
        self.sensor_log = []

        self.image_log = []
        self.chassis_log = []
        self.chassis_speed_log = []
        self.gimbal_log = []
        self.gimbal_speed_log = []
        self.imu_log = []
        self.blaster_log = []

    def log_chassis(self, sensor_input: tuple):
        """Log the chassis x, y position. The z position data get added later, since this one is not working."""
        self.chassis_log = [datetime.datetime.now(), sensor_input[0], sensor_input[1]]

    def log_chassis_speed(self, sensor_input: tuple):
        """Log the x, y, z speed of the chassis."""
        self.chassis_speed_log = list(sensor_input)

    def log_gimbal(self, sensor_input: tuple):
        """Log the gimbal pitch and yaw."""
        self.relative_yaw = sensor_input[1]
        self.gimbal_log = [sensor_input[0], sensor_input[1]]

    def log_gimbal_speed(self, sensor_input: tuple):
        """Log the gimbal pitch and yaw speed."""
        self.gimbal_speed_log = list(sensor_input)

    def log_imu(self, sensor_input: tuple):
        """Log the x, y, z acc and the pitch, yaw, roll gyro."""
        self.imu_log = list(sensor_input)

    def log_blaster_fire(self, blaster_fire: bool):
        """Log if the blaster has fired in a moment."""
        self.blaster_log = [blaster_fire]

    def add_chassis_rotation(self):
        """Add the rotation to the log_chassis list."""
        self.chassis_log.append(self.gimbal_log[3] - self.gimbal_log[1])

    def create_row(self):
        """Combine all sensor data to create a row for the data."""
        self.add_chassis_rotation()
        self.sensor_log.append([self.chassis_log + self.chassis_speed_log + self.gimbal_log
                                + self.gimbal_speed_log + self.imu_log + self.blaster_log, self.image_log])

    def log_img(self, img):
        """Log the img in bit-format."""
        self.image_log = [img]

    def save(self):
        """Save the log to a csv file."""
        head = ["x_pos", "y_pos", "z_pos", "x_speed", "y_speed", "z_speed",  # Chassis
                "pitch_pos", "yaw_pos", "pitch_speed", "yaw_speed",  # Gimbal
                "x_acc", "y_acc", "z_acc", "pitch_gyro", "yaw_gyro", "roll_gyro",  # Imu
                "blaster_fire", "image"]  # Blaster
        with open(self.filename, 'a') as file:
            # Add header
            file.write(f"{';'.join(map(str, head))}\n")
            for row in self.sensor_log:
                file.write(f"{';'.join(map(str, row))}\n")

    # TODO Add connection between ACTIONPOINTS

    def send_to_db(self):  # TODO
        """Send the extracted experience to the database."""
        db = database.connect()
        query = "INSERT into "
        values = self.sensor_log
        db.cursor().execute(query, values)
        db.commit()
