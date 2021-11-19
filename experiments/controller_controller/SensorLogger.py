import datetime


class SensorLogger:
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
        """Logs the chassis x, y position. The z position data get added later, since this one is not working."""
        self.chassis_log = [datetime.datetime.now(), sensor_input[0], sensor_input[1]]

    def log_chassis_speed(self, sensor_input: tuple):
        """Logs the x, y, z speed of the chassis."""
        self.chassis_speed_log = list(sensor_input)

    def log_gimbal(self, sensor_input: tuple):
        """Logs the gimbal pitch and yaw."""
        self.relative_yaw = sensor_input[1]
        self.gimbal_log = [sensor_input[0], sensor_input[1]]

    def log_gimbal_speed(self, sensor_input: tuple):
        """Logs the gimbal pitch and yaw speed."""
        self.gimbal_speed_log = list(sensor_input)

    def log_imu(self, sensor_input: tuple):
        """Logs the x, y, z acc and the pitch, yaw, roll gyro."""
        self.imu_log = list(sensor_input)

    def log_blaster_fire(self, blaster_fire: bool):
        """Logs if the blaster has fired in a moment."""
        self.blaster_log = [blaster_fire]

    def add_chassis_rotation(self):
        """Adds the rotation to the log_chassis list of lists."""
        self.chassis_log.append(self.gimbal_log[3] - self.gimbal_log[1])

    def create_row(self):
        self.add_chassis_rotation()
        self.sensor_log.append([self.chassis_log + self.chassis_speed_log + self.gimbal_log +
                                self.gimbal_speed_log + self.imu_log + self.blaster_log])

    def log_img(self, img_name): # TODO
        pass

    def save(self):
        """Save the log to a csv file."""
        head = ["x_pos", "y_pos", "z_pos", "x_speed", "y_speed", "z_speed",  # Chassis
                "pitch_pos", "yaw_pos", "pitch_speed", "yaw_speed",  # Gimbal
                "x_acc", "y_acc", "z_acc", "pitch_gyro", "yaw_gyro", "roll_gyro",  # Imu
                "blaster_fire"]  # Blaster
        with open(self.filename, 'a') as file:
            # Add header
            file.write(f"{';'.join(map(str, head))}\n")
            for row in self.sensor_log:
                file.write(f"{';'.join(map(str, row))}\n")

    def sent_to_db(self): # TODO
        """Sends the extrected experience to the database."""
        pass
