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
        s_err = distance - 5
        if s_err > 10:
            p = 0.05
            i = 0.05
            v_err = 20 - my_car.get_velocity().length()
            control.throttle = s_err*p+v_err*i
            control.throttle = min(control.throttle, 0.8)
            control.brake = 0
        elif s_err > 5:
            p = 0.05
            i = 0.05
            v_err = 10 - my_car.get_velocity().length()
            control.throttle = s_err*p+v_err*i
            control.throttle = min(control.throttle, 0.4)
            control.brake = 0
        else:
            control.brake = 1

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
