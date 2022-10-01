from carla import Vector3D, CityObjectLabel
import numpy as np
import math


def step(my_car):
    # TODO
    # Apply control to the car. See the API here.
    # https://carla.readthedocs.io/en/latest/python_api/#carla.VehicleControl
    try:
        distance = my_car.distance_front
        print("distance:", distance)
        control = my_car.get_control()
        control.throttle = 1.0
        control.brake = 1.0
        my_car.apply_control(control)
    except:
        return


def on_sensor_data(event, my_car):
    # Find points casted on vehicles
    vehicle_tag = int(CityObjectLabel.Vehicles)
    vehicle_points = filter(lambda det: det.object_tag == vehicle_tag, event)

    # TODO
    # Find the distance to the coach car using LiDAR points. You may check this.
    # https://carla.readthedocs.io/en/latest/python_api/#carla.SemanticLidarDetection
    for _, point in enumerate(vehicle_points):
        # point(x,y,z),cos_inc_angle,object_idx,object_tag
        if point.object_idx == my_car.id:
            continue
        location = point.point
        cosine = point.cos_inc_angle
        origin_dis = np.sqrt(location.x**2 + location.y**2 + location.z**2)
        real_dis = cosine * origin_dis
        my_car.distance_front = real_dis
