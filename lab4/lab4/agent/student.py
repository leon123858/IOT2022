from carla import Actor, Image, VehicleControl, VehicleLightState, Vector3D, Location
import numpy as np
import cv2 as cv
import colorsys
import math
import random
from scipy.signal import find_peaks_cwt


class Controller:
    def __init__(self):
        return

    def relative_location(self, car_location, car_rotation, point_location):
        origin = car_location
        forward = car_rotation.get_forward_vector()
        right = car_rotation.get_right_vector()
        up = car_rotation.get_up_vector()
        disp = point_location - origin
        x = np.dot([disp.x, disp.y, disp.z], [forward.x, forward.y, forward.z])
        y = np.dot([disp.x, disp.y, disp.z], [right.x, right.y, right.z])
        z = np.dot([disp.x, disp.y, disp.z], [up.x, up.y, up.z])
        return Vector3D(x, y, z)

    def get_control_parameters(self, my_car, next_point):
        wheelbase = 1
        now_velocity = my_car.get_velocity().length()
        # get the vector from car to point
        vector_car2point = next_point
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
        # 油門
        p = 0.01
        i = 0.005
        throttle = 0.25
        brake = 0 if now_velocity < 2 else 0.5
        return throttle, steer, brake


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
            [[250,  600],  # Bottom left
             [300, 400],  # Top left
             [600,  400],  # Top right
             [650, 600]])  # Bottom right
        dst_image = np.float32(
            [[200,  600],  # Bottom left
             [200,    0],  # Top left
             [600,   0],  # Top right
             [600, 600]])  # Bottom right
        img_size = (self.image.shape[1], self.image.shape[0])
        M = cv.getPerspectiveTransform(src_image, dst_image)
        warped = cv.warpPerspective(self.image, M, img_size)
        return warped

    def get_position_histogram(self,mode=0):
        image = np.clip(self.image, a_min=0, a_max=1)
        if mode == 0:
            return np.sum(image[:, :], axis=0)
        elif mode == 1: #取下半
            return np.sum(image[math.floor(image.shape[0]/2):, :], axis=0)
        elif mode == 2: #up half
            return np.sum(image[:math.floor(image.shape[0]/2), :], axis=0)
        

    def get_histogram_peaks(self,mode=0):
        histogram = self.get_position_histogram(mode)
        indices = find_peaks_cwt(histogram, 30, wavelet=None, max_distances=None,
                                 gap_thresh=None, min_length=None, min_snr=1, noise_perc=20, window_size=None)
        return indices


class StudentAgent:
    camera_image = None
    lidar_image = None

    def __init__(self):
        # TODO
        self.tmp = 0
        return

    def choice_peak(self,peaks,point):
        left_line_place = None
        if len(peaks) == 1:
            left_line_place = peaks[0]
        else:
            minv = 10000.0
            for peak in peaks:
                if abs(peak - point) < minv:
                    minv = abs(peak - point)
                    left_line_place = peak
                    break
        return left_line_place

    def step(self, actor: Actor) -> VehicleControl:
        # TODO
        actor.set_light_state(VehicleLightState.HighBeam)
        control = actor.get_control()
        controller = Controller()
        # To draw an image using OpenCV, please call imshow() in step().
        # Do not imshow() in on_xxx_data(). It freezes the program!
        if self.camera_image is not None:
            imager = Camera_Imager(self.camera_image)
            peaks_far = imager.get_histogram_peaks(1)
            peaks_close = imager.get_histogram_peaks(2)
            point = 28.0
            left_line_place_far = self.choice_peak(peaks_far,point) 
            left_line_place_close = self.choice_peak(peaks_close,point)
            if left_line_place_close - left_line_place_far > 30:
                point = 95.0
            print(left_line_place_far,left_line_place_close)
            if left_line_place_far is None:
                left_line_place_far = left_line_place_close
            if left_line_place_close is None:
                left_line_place_close = left_line_place_far
            if left_line_place_far is None and left_line_place_close is None:
                left_line_place_close = point
                left_line_place_far = point
            # if left_line_place_far  <  left_line_place_close:
            #     print(True)
            #     left_line_place = left_line_place_close
            # else:
            #     print(False) 
            left_line_place = left_line_place_far
            left_offset = point - (left_line_place + 0)
            left_offset = max(left_offset, -1)
            left_offset = min(left_offset, 1)
            # if left_line_place_close > 100:
            #     if random.choice([True,True,False,False,False]):
            #         print("adjust")
            #         left_offset = 1
            if self.tmp > 0:
                self.tmp = self.tmp -1 
                left_offset = -1
            forward_offset = 5.0
            throttle, steer, brake = controller.get_control_parameters(
                actor, Vector3D(x=forward_offset, y=-left_offset*0.05, z=0.0))
            control.throttle = throttle
            control.steer = steer
            control.brake = brake
            actor.apply_control(control)
            cv.imshow("camera", imager.image)

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
