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

    def find_line_fit(self, nwindows=9, margin=100, minpix=50):
        image = self.image
        histogram = self.get_position_histogram()
        # Create an output image to draw on and  visualize the result
        out_img = np.dstack((image, image, image)) * 255
        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Set height of windows
        window_height = np.int(image.shape[0]/nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = image.shape[0] - (window+1)*window_height
            win_y_high = image.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            cv.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
                         (0, 255, 0), 2)
            cv.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high),
                         (0, 255, 0), 2)
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # to plot
        out_img[nonzeroy[left_lane_inds],
                nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds],
                nonzerox[right_lane_inds]] = [0, 0, 255]

        # Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        return left_fit, right_fit, out_img


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
            print(imager.find_line_fit())
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
