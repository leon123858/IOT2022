from carla import Vector3D, CityObjectLabel


def step(my_car):
    # TODO
    # Apply control to the car. See the API here.
    # https://carla.readthedocs.io/en/latest/python_api/#carla.VehicleControl
    try:
        distance = my_car.distance_front
        my_car.before_distance = distance
        print("distance:", distance)
        control = my_car.get_control()
        p = 0.5
        i = 0
        control.throttle = distance*p + (20 - my_car.get_velocity().length())*i
        control.brake = 0
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
        tea_vec = Vector3D(x=location.x, y=location.y, z=location.z)
        my_car.distance_front = tea_vec.length()
