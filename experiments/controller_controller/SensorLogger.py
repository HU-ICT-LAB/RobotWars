import datetime


class SensorLogger:
    def __init__(self, title:str): # todo is this title necessary
        self.title = title
        self.filename = f"{title}.csv"
        self.relative_yaw = 0
        self.start = False
        self.sensor_log = []

        self.image_log = []
        self.chassis_log = []
        self.chassis_speed_log = []
        self.gimbal_log = []
        self.gimbal_speed_log = []
        self.imu_log = []
        self.blaster_log = []

    def log_chassis(self, sensor_input:tuple):
        """Logs the chassis x, y position. The z position data get added later, since this one is not working."""
        if self.start:
            self.chassis_log.append([datetime.datetime.now(), sensor_input[0], sensor_input[1]])

    def log_chassis_speed(self, sensor_input:tuple):
        """Logs the x, y, z speed of the chassis."""
        self.chassis_speed_log.append([*sensor_input]) # Todo miss remove datetime here and in the next functions

    def log_gimbal(self, sensor_input:tuple):
        """Logs the gimbal pitch and yaw."""
        self.relative_yaw = sensor_input[1]
        self.gimbal_log.append([sensor_input[0], sensor_input[1]])

    def log_gimbal_speed(self, sensor_input:tuple):
        """Logs the gimbal pitch and yaw speed."""
        self.gimbal_speed_log.append([*sensor_input]) # todo unpacking maybe unnecessary

    def log_imu(self, sensor_input:tuple):
        """Logs the x, y, z acc and the pitch, yaw, roll gyro."""
        self.imu_log.append([*sensor_input])

    def log_blaster_fire(self, blaster_fire:bool):
        """Logs if the blaster has fired in a moment."""
        self.blaster_log.append([blaster_fire])

    def add_chassis_rotation(self):
        """Adds the rotation to the log_chassis list of lists."""
        l = min([len(self.chassis_log), len(self.gimbal_log)])
        [self.chassis_log[i].append(self.gimbal_log[i][3] - self.gimbal_log[i][1]) for i in range(l)]

    def combine_data(self):
        """Combines all the sensor data to create 1 list of lists."""
        self.add_chassis_rotation()
        l = min([len(self.chassis_log), len(self.chassis_speed_log), len(self.gimbal_log),
                   len(self.gimbal_speed_log), len(self.imu_log), len(self.blaster_log)])
        for i in range(l):
            lst = [self.chassis_log[i] + self.chassis_speed_log[i] + self.gimbal_log[i] +
                   self.gimbal_speed_log[i] + self.imu_log[i] + self.blaster_log[i]]
            self.sensor_log.append(lst)

    def save(self):
        """Save the log to a csv file."""
        self.combine_data()
        head = ["x_pos", "y_pos", "z_pos", "x_speed", "y_speed", "z_speed",         # Chassis
                "pitch_pos", "yaw_pos", "pitch_speed", "yaw_speed",                 # Gimbal
                "x_acc", "y_acc", "z_acc", "pitch_gyro", "yaw_gyro", "roll_gyro",   # Imu
                "blaster_fire"]                                                     # Blaster
        with open(self.filename, 'a') as file:
            # Add header
            file.write(f"{';'.join(map(str, head))}\n")
            for row in self.sensor_log:
                file.write(f"{';'.join(map(str, row))}\n")

    def sent_to_db(self):
        """Sends the extrected experience to the database."""
        pass
