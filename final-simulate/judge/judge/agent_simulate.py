import sys
import time
import re
import numpy as np
from carla import Actor, Image, VehicleControl, VehicleLightState, Vector3D, Location
import math

DATA = [
    [12, -95],
    [10.4, -110.0],
    [9.5, -123.3],
    [9.7, -126],
    [11.1, -131.1],
    [14.4, -133.7],
    [15.8, -134],
    [20.3, -134.3],
    [70.5, -133.2],
    [78.0, -133.4],
    [83.8, -135.2],
    [89.1, -141.0],
    [89.1, -145.2],
    [90, -180],
    [89.0, -191.8],
    [86.2, -198.1],
    [80.0, -205.4],
    [73.8, -206.2],
    [63.2446, -209.55],
    [26.7, -207.6],
    [10.5, -205.0],
    [1.2, -198.9],
    [0.6, -194.7],
    [-2.6, -186.6],
    [-4.1, -149.4],
    [-5, -113.4],
    [-5, -50.4],
    [-7.4, -33.1],
    [-8.5, -27.4],
    [-11.1, -21.4],
    [-16.0, -16.8],
    [-21.2, -10.0],
    [-22.7, -5.6],
    [-23.6, 2.4],
    [-22.1, 9.0],
    [-17.5, 15.8],
    [-13.9, 21.7],
    [-11.8, 26.9],
    [-10.3, 39.4],
    [-9.6, 118.3],
    [-10.3, 124.2],
    [-13.4, 128.1],
    [-19.1, 130.6],
    [-61.4, 131.2],
    [-85.4, 130.6],
    [-98.3, 129.0],
    [-107.19, 129.895],
]


class Controller_C():
    def __init__(self) -> None:
        self.points = DATA
        self.index = 0
        self.length = len(self.points)
        self.break_timer = 0
        self.state = "RUN"

    def check_STOP_state(self) -> bool:
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
            if self.index == 18 or self.index == 46:
                self.state = "STOP"
            self.index = self.index + 1

    def relative_location(self, car_location, car_rotation, point_location) -> Vector3D:
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
        brake = 0.5 if velocity > 5 else 0
        return throttle, brake


class Agent():

    def __init__(self):
        self.n_ticks = 0
        self.c_controller = Controller_C()

    def tick(self, actor: Actor):
        control = actor.get_control()
        velocity = actor.get_velocity().length()
        location = actor.get_location()
        rotation = actor.get_transform().rotation
        if self.c_controller.check_STOP_state():
            control.throttle = 0
            control.steer = 0
            control.brake = 1
            actor.apply_control(control)
            return
        steer, distance = self.c_controller.get_steer(location, rotation)
        self.c_controller.move_point(distance)
        throttle, brake = self.c_controller.get_PI_control_result(
            distance, velocity)
        control.throttle = throttle
        control.steer = steer
        control.brake = brake
        actor.apply_control(control)
        self.n_ticks += 1
        return
