from carla import Vector3D, CityObjectLabel


def step(my_car):
    ## TODO
    ## Apply control to the car. See the API here.
    ## https://carla.readthedocs.io/en/latest/python_api/#carla.VehicleControl
    control = my_car.get_control()
    control.throttle = 1.0
    control.brake = 1.0
    my_car.apply_control(control)


def on_sensor_data(event, my_car):
    ## Find points casted on vehicles
    vehicle_tag = int(CityObjectLabel.Vehicles)
    vehicle_points = filter(lambda det: det.object_tag == vehicle_tag, event)

    ## TODO
    ## Find the distance to the coach car using LiDAR points. You may check this.
    ## https://carla.readthedocs.io/en/latest/python_api/#carla.SemanticLidarDetection
    for i,point in enumerate(vehicle_points):
        # x,y,z,cos_inc_angle,object_idx,object_tag
        print(point)