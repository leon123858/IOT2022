import sys
import time
import re
import numpy as np
from carla import Actor, Image, VehicleControl, VehicleLightState, Vector3D, Location
import math

DATA = {
    "waypoints": [
        #  start
        [12, -95],
        #  第一路口
        [12.6, -120.6],
        [13.1, -130.0],
        #  第一右彎	end
        [24.5, -130.5],
        #  路口
        [70.5, -129.3],
        [78.0, -129.3],
        [83.8, -135.2],
        [89.1, -141.0],
        #  第二左彎 end
        [89.1, -145.2],
        #  路口
        [90, -175.5],
        [89.7, -188.3],
        [88.8, -199.4],
        [83.5, -204.0],
        [77.5, -209.2],
        #  第三左彎 end
        [71.0, -209.5],
        #  C1
        [63.2446, -209.5],
        #  路口
        [24.6, -210.6],
        [11.9, -210.3],
        [0.5, -201.3],
        [-2.8, -192.0],
        #  第三左彎
        [-3.2, -183.3],
        #  路口
        [-3.9, -151.1],
        [-5.6, -76.9],
        [-6.5, -45.7],
        #  circle前
        [-7.4, -33.1],
        [-8.2, -27.4],
        [-10.7, -21.4],
        [-15.5, -16.8],
        [-21.2, -10.0],
        [-22.7, -5.6],
        [-22.1, 9.0],
        [-17.5, 15.8],
        [-13.0, 21.0],
        [-11.8, 26.9],
        #  cycle end
        [-10.3, 39.4],
        #  路口
        [-10.0, 110.3],
        [-10.0, 128.0],
        #  4 right end
        [-21.2, 130.0],
        #  路口
        [-61.4, 131.0],
        [-69.9, 131.5],
        [-79.4, 130.3],
        [-92.8, 128.9],
        #  路口 end
        [-99.4, 129.0],
        #  C2
        [-107.19, 129.4],
    ],
}


class Controller_C():
    def __init__(self) -> None:
        self.points = DATA["waypoints"]
        self.target_index = [15, 44]
        self.traffic_index = [1, 4, 9, 16, 21, 35, 39]
        self.index = 0
        self.length = len(self.points)
        self.break_timer = 0
        self.state = "RUN"

    def set_traffic_run(self):
        if self.state == "TRAFFIC":
            self.state = "RUN"

    def check_STOP_state(self) -> bool:
        if self.state == "TRAFFIC":
            return True
        if self.state == "STOP":
            self.break_timer += 1
            if self.break_timer > 20:
                self.break_timer = 0
                self.state = "RUN"
            return True
        if self.index == self.length:
            return True
        return False

    def move_point(self, distance) -> None:
        if distance <= 0.5 and self.index < self.length:
            # check if is Target index
            if self.target_index.count(self.index) > 0:
                self.state = "STOP"
            elif self.traffic_index.count(self.index) > 0:
                self.state = "STOP"  # just test, real is "TRAFFIC"
            self.index = self.index + 1

    @staticmethod
    def relative_location(car_location, car_rotation, point_location) -> Vector3D:
        origin = car_location
        forward = car_rotation.get_forward_vector()
        right = car_rotation.get_right_vector()
        up = car_rotation.get_up_vector()
        disp = point_location - origin
        x = np.dot([disp.x, disp.y, disp.z], [forward.x, forward.y, forward.z])
        y = np.dot([disp.x, disp.y, disp.z], [right.x, right.y, right.z])
        z = np.dot([disp.x, disp.y, disp.z], [up.x, up.y, up.z])
        return Vector3D(x, y, z)

    def get_steer(self, location, rotation):
        next_point = self.points[self.index]
        wheelbase = 1
        # get the vector from car to point
        vector_car2point = self.relative_location(location, rotation, Location(
            x=next_point[0], y=next_point[1], z=location.z))
        # the offset for wheel
        wheel_vector = Vector3D(wheelbase, 0, 0)
        # real distance
        distance = vector_car2point.length()
        # 轉彎半徑的向量
        relate_vector = vector_car2point+wheel_vector
        # 半徑長度
        d2 = relate_vector.x**2 + relate_vector.y**2
        # 向量套公式得旋轉徑度
        steer_rad = math.atan(2 * wheelbase * relate_vector.y / d2)
        # 轉角度
        steer_deg = math.degrees(steer_rad)
        # 上下裁切
        steer_deg = np.clip(steer_deg, -1, 1)
        steer = steer_deg / 1
        return steer, distance

    @staticmethod
    def get_PI_control_result(distance, velocity):
        p = 0.02
        i = 0.005
        throttle = distance*p + (20-velocity)*i
        brake = 0.5 if velocity > 7 else 0
        return throttle, brake

    def get_control(self, location, rotation, velocity):
        if self.check_STOP_state():
            return 0, 0, 1
        steer, distance = self.get_steer(location, rotation)
        self.move_point(distance)
        throttle, brake = self.get_PI_control_result(
            distance, velocity)
        return throttle, steer, brake


class Agent():

    def __init__(self):
        self.n_ticks = 0
        self.c_controller = Controller_C()

    def tick(self, actor: Actor):
        control = actor.get_control()
        # get_parameter
        velocity = actor.get_velocity().length()
        location = actor.get_location()
        rotation = actor.get_transform().rotation
        # controller
        throttle, steer, brake = self.c_controller.get_control(
            location, rotation, velocity)
        # control
        control.throttle = throttle
        control.steer = steer
        control.brake = brake
        actor.apply_control(control)
        self.n_ticks += 1
        return
