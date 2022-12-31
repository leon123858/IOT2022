import sys
import time
import re
import numpy as np
from carla import Actor, Image, VehicleControl, VehicleLightState, Vector3D, Location
import math


class Controller_C():
    def __init__(self) -> None:
        pass

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

    def get_steer(self, next_point, location, rotation):
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


class Agent():

    def __init__(self):
        self.n_ticks = 0

    def tick(self, actor: Actor) -> VehicleControl:
        control = actor.get_control()
        velocity = actor.get_velocity().length()
        location = actor.get_location()
        rotation = actor.get_transform().rotation
        print(location)
        control.throttle = 0
        control.steer = 0
        control.brake = 0
        self.n_ticks += 1
        actor.apply_control(control)
        return control
