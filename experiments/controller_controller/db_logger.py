from typing import Tuple
import numpy as np
from database import database
import datetime

class DBLogger:
    def __init__(self, sn):
        self.id = int # Id
        self.chassis_position = (0., 0., 0.)
        self.gimbal_angle = (0., 0., 0., 0.)
        self.chassis_imu = (0., 0., 0., 0., 0., 0.)
        self.sn = sn        #  TODO maybe toher thing for robot id
        self.db = database.connect()
        self.prev_action_point = None # previous id

    def handle_position(self, position_data: Tuple[float, float, float]) -> None:
        self.chassis_position = position_data

    def handle_angle(self, angle_data: Tuple[float, float, float]) -> None:
        self.gimbal_angle = angle_data

    def handle_imu(self, imu_data: Tuple[float, float, float, float, float, float]) -> None:
        self.chassis_imu = imu_data

    def calc_z_pos(self):
        """Calculate the z position through the gimbal logger."""
        return self.gimbal_angle[3] - self.gimbal_angle[1]

    def log_action_point(self, camera_frame: np.array, action_chassis_speed: Tuple[float, float, float],
                         action_gimbal_speed: Tuple[float,float], action_blaster_fire: bool):
        self.id = datetime.datetime.now().strftime("%H:%M:%S,%f3")
        current_time = datetime.datetime.now('%m-%d-%Y %H:%M:%S,%f3')
        z_speed = self.calc_z_pos()
        #TODO check if tabel name ▼ is correct
        query = f"INSERT INTO ActionPoint (delta_x_position, delta_y_position, delta_z," \ 
                f"action_speed_x, action_speed_y, action_speed_z, gimbal_yaw, gimbal_picth," \
                f"action_gimbal_speed_yaw, action_gimbal_speed_pitch, acc_x, acc_y, acc_z," \
                f"gyro_pitch, gyro_yaw, gyro_roll, camera_frame, action_fire, robot_id, time," \
                f"action_point)" \
                "(%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s);" \
                "SELECT SCOPE_IDENTITY() AS [SCOPE_IDENTITY];"
        values = self.chassis_position[:2]+ z_speed + action_chassis_speed + self.gimbal_angle[:2], action_gimbal_speed \
                + self.chassis_imu + camera_frame + action_blaster_fire + (self.sn,) + current_time, self.prev_action_point
        self.prev_action_point = database.perform_query(self.db, query, values)



    subsscribe everything
    while loop:
        camera_frame = camera.get
        actions = controller.getactions
    
        dblogger.log_action_point()
