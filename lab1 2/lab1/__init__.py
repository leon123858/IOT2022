import carla
import random


def main():
    # Connect to the client and retrieve the world object
    client = carla.Client("localhost", 2000)
    client.load_world("Town03")


if __name__ == "__main__":
    main()
