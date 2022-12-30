import sys
import time
import re
import numpy as np
from carla import Actor, Image, VehicleControl, VehicleLightState, Vector3D, Location


class Agent():

    def __init__(self):
        self.n_ticks = 0

    def tick(self, actor: Actor) -> VehicleControl:
        control = actor.get_control()
        control.throttle = 0.1
        control.steer = 0
        control.brake = 0
        self.n_ticks += 1
        actor.apply_control(control)
        return control
