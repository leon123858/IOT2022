from carla import Actor, Image, VehicleControl, VehicleLightState
import numpy as np
import cv2 as cv
import colorsys
import math


class Imager:
    def __init__(self, image):
        self.image = image
        return


class Camera_Imager(Imager):
    # https://www.ncnynl.com/archives/201904/2960.html
    # image transform
    def __init__(self, image):
        super().__init__(image)
        # 圖片轉為俯視圖
        self.image = self.__get_perspective_img()

    def __get_perspective_img(self):
        src_image = np.float32(
            [[300,  600],  # Bottom left
             [350, 400],  # Top left
             [450,  400],  # Top right
             [500, 600]])  # Bottom right
        dst_image = np.float32(
            [[300,  600],  # Bottom left
             [300,    0],  # Top left
             [500,   0],  # Top right
             [500, 600]])  # Bottom right
        img_size = (self.image.shape[1], self.image.shape[0])
        M = cv.getPerspectiveTransform(src_image, dst_image)
        warped = cv.warpPerspective(self.image, M, img_size)
        return warped

    def get_position_histogram(self):
        image = np.clip(self.image, a_min=0, a_max=1)
        return np.sum(image[math.floor(image.shape[0]/2):, :], axis=0)

    def get_histogram_left_peak(self):
        histogram = self.get_position_histogram(
        )[0:math.floor(len(histogram)/2)]
        max = np.max(histogram)
        ret = []
        for i in range(len(histogram)):
            ispeak = True
            if i-1 > 0:
                ispeak &= (histogram[i] > 1.8 * histogram[i-1])
            if i+1 < len(histogram):
                ispeak &= (histogram[i] > 1.8 * histogram[i+1])

            ispeak &= (histogram[i] > 0.05 * max)
            if ispeak:
                ret.append(i)
        return ret


class StudentAgent:
    camera_image = None
    lidar_image = None

    def __init__(self):
        # TODO
        return

    def step(self, actor: Actor) -> VehicleControl:
        # TODO
        actor.set_light_state(VehicleLightState.HighBeam)
        control = actor.get_control()

        # To draw an image using OpenCV, please call imshow() in step().
        # Do not imshow() in on_xxx_data(). It freezes the program!
        if self.camera_image is not None:
            imager = Camera_Imager(self.camera_image)
            print(imager.get_histogram_left_peak())
            cv.imshow("camera", self.camera_image)

        if self.lidar_image is not None:
            cv.imshow("lidar", self.lidar_image)

        cv.waitKey(1)

        return control

    def on_lidar_data(self, points: np.ndarray):
        # TODO
        # 'points' is an Nx4 array with x, y, z, intensity columns

        lidar_range = 50.0
        ih = 600
        iw = 800

        points = points[:, :2].copy()
        points *= min(ih, iw) / (2.0 * lidar_range)
        points += (0.5 * ih, 0.5 * iw)
        points = np.fabs(points)  # pylint: disable=E1111
        points = points.astype(np.int32)
        points = np.reshape(points, (-1, 2))
        image = np.zeros((ih, iw, 3), dtype=np.uint8)
        image[tuple(points.T)] = (255, 255, 255)

        self.lidar_image = image

    def on_camera_data(self, image: np.ndarray):
        # TODO
        # 'image' is an MxNx3 array with r, g, b columns

        # HSV thresholding
        orange_min = np.array([28, 10, 10], np.uint8)
        orange_max = np.array([30, 80, 100], np.uint8)
        image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        image = cv.inRange(image, orange_min, orange_max)

        # erosion
        erosion_size = 1
        kernel = cv.getStructuringElement(
            cv.MORPH_ELLIPSE,
            (2 * erosion_size + 1, 2 * erosion_size + 1),
            (erosion_size, erosion_size),
        )
        image = cv.dilate(image, kernel, iterations=2)

        self.camera_image = image
