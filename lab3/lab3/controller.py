from enum import Enum
from carla import Vector3D, Location
import numpy as np
import csv
import math


WAYPOINTS = []


def relative_location(car_location, car_rotation, point_location):
    origin = car_location
    forward = car_rotation.get_forward_vector()
    right = car_rotation.get_right_vector()
    up = car_rotation.get_up_vector()
    disp = point_location - origin
    x = np.dot([disp.x, disp.y, disp.z], [forward.x, forward.y, forward.z])
    y = np.dot([disp.x, disp.y, disp.z], [right.x, right.y, right.z])
    z = np.dot([disp.x, disp.y, disp.z], [up.x, up.y, up.z])
    return Vector3D(x, y, z)


def load_waypoints(path):
    global WAYPOINTS
    with open(path) as f:
        rows = csv.reader(f)
        WAYPOINTS = [[float(x) for x in row] for row in rows]


def step(my_car):

    global WAYPOINTS   # The waypoints are stored here

    wheelbase = 1
    now_velocity = my_car.get_velocity().length()
    # all point have arrive
    if len(WAYPOINTS) == 0:
        control = my_car.get_control()
        control.brake = 1  # 0~1
        my_car.apply_control(control)
        return
    next_point = WAYPOINTS.pop(0)
    # get the vector from car to point
    vector_car2point = relative_location(my_car.get_location(), my_car.get_transform(
    ).rotation, Location(x=next_point[0], y=next_point[1], z=my_car.get_location().z))
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
    if distance >= 0.5:
        WAYPOINTS.insert(0, next_point)
    # TODO
    # Apply control to the car. See the API here.
    # https://carla.readthedocs.io/en/latest/python_api/#carla.VehicleControl
    control = my_car.get_control()
    p = 0.01
    i = 0.005
    control.throttle = distance*p + (20-now_velocity)*i
    control.steer = steer
    control.brake = 0  # 0~1
    my_car.apply_control(control)
