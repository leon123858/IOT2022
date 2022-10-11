from carla import Client, Location, Rotation, Transform
from argparse import ArgumentParser
from enum import Enum
import controller
import csv

WORLD = "Town06"
TIMEOUT_F = 6000
VELOCIDY_THRESH = 1e-3
DIS_THRESH = 0.5

START_X = -180.0
START_Y = -18.77
SPEC_HEIGHT = 3

SPECTATOR_TRANS = Transform(
    Location(START_X, START_Y, SPEC_HEIGHT), Rotation(0.0, 180.0, 0.0)
)
STUDENT_TRANS = Transform(
    Location(START_X, START_Y, 0.1), Rotation(0.0, 180.0, 0.0)
)


class State(Enum):
    FORWARDING = 1
    FAIL = 2
    FINISH = 3
    TERMINATING = 4


WAYPOINTS = []


def load_waypoints(path):
    global WAYPOINTS
    with open(path) as f:
        rows = csv.reader(f)
        WAYPOINTS = [[float(x) for x in row] for row in rows]


def stop_car(car):
    control = car.get_control()
    control.brake = 0.1
    car.apply_control(control)


def main():
    parser = ArgumentParser()
    parser.add_argument("--addr", default="localhost",
                        help="CARLA server address")
    parser.add_argument("--port", default=2000, help="CARLA server port")
    parser.add_argument("--waypoints_path", default='./waypoints.csv',
                        help="The path of the waypoints file")
    parser.add_argument("--no-follow-car", action="store_true")
    args = parser.parse_args()

    # Connect to the client and retrieve the world object
    client = Client(args.addr, int(args.port))
    client.load_world(WORLD)
    world = client.get_world()

    # Enable synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True  # Enables synchronous mode
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # Configure spectator
    spec = world.get_spectator()
    spec.set_transform(SPECTATOR_TRANS)

    # Create state variable
    state = State.FORWARDING

    # Spawn vehicles
    vblu = world.get_blueprint_library().find("vehicle.tesla.model3")
    stu_car = world.spawn_actor(vblu, STUDENT_TRANS)

    def on_collision(event):
        nonlocal state
        nonlocal waypoint_idx
        if state == State.FINISH:
            pass
        elif state != State.FAIL:
            state = State.FAIL
            waypoint_idx = 0
            print("Collision detected!!!!!!!!!!")

    # Add a collision sensor on the student car
    cblu = world.get_blueprint_library().find("sensor.other.collision")
    col_sensor = world.spawn_actor(cblu, Transform(), attach_to=stu_car)
    col_sensor.listen(on_collision)

    controller.load_waypoints(args.waypoints_path)
    load_waypoints(args.waypoints_path)

    def get_score():
        nonlocal waypoint_idx
        print("Your score is %2.0f." % ((waypoint_idx / len(WAYPOINTS)) * 100))

    def check_timeout():
        nonlocal state
        nonlocal counter
        if counter > TIMEOUT_F:
            print("FAIL: Exceed timeout {} steps.".format(TIMEOUT_F))
            state = State.FAIL

    try:
        # Skip 10 frames (~1s for 10fps)
        for _ in range(10):
            world.tick()

        waypoint_idx = 0
        counter = 0

        # start loopin
        while True:

            controller.step(stu_car)
            world.tick()

            # Stick the spectator to the student car
            if not args.no_follow_car:
                stu_trans = stu_car.get_transform()
                spec_loc = Location(
                    stu_trans.location.x, stu_trans.location.y, SPEC_HEIGHT
                )
                spec_rot = stu_trans.rotation
                spec_trans = Transform(spec_loc, spec_rot)
                spec.set_transform(spec_trans)

            # Check timeout
            if state == State.FORWARDING:
                check_timeout()

            # Check distance to target waypoint
            if state == State.FORWARDING:
                dis = ((stu_car.get_location().x - WAYPOINTS[waypoint_idx][0]) ** 2 +
                       (stu_car.get_location().y - WAYPOINTS[waypoint_idx][1]) ** 2) ** (1 / 2)
                if dis < DIS_THRESH:
                    print("You have reached the point %s (location: %s)." %
                          (waypoint_idx, WAYPOINTS[waypoint_idx]))
                    waypoint_idx += 1
                    if waypoint_idx == len(WAYPOINTS):
                        state = State.TERMINATING

            elif state == State.TERMINATING:
                stu_vel = stu_car.get_velocity().length()
                if stu_vel <= VELOCIDY_THRESH:
                    state = State.FINISH

            elif state == State.FAIL or state == State.FINISH:
                get_score()
                break

            counter += 1

    except KeyboardInterrupt:
        print("INTERRUPTED")
        return

    try:
        # loop forever
        while True:
            world.tick()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
