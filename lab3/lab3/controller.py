from enum import Enum
import csv

WAYPOINTS = []


def load_waypoints(path):
    global WAYPOINTS
    with open(path) as f:
        rows = csv.reader(f)
        WAYPOINTS = [[float(x) for x in row] for row in rows]


def step(my_car):

    global WAYPOINTS   # The waypoints are stored here

    car_orientation = my_car.get_transform().rotation.yaw   # Car orientation
    location = (my_car.get_location().x,
                my_car.get_location().y)   # Car location

    # TODO
    # Apply control to the car. See the API here.
    # https://carla.readthedocs.io/en/latest/python_api/#carla.VehicleControl
