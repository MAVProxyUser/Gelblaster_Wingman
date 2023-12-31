# File: coordinate_system.py

import numpy as np

class CoordinateSystem:
    # Define class constants
    CAMERA_FOCAL_LENGTH_MM = 4.74
    CAMERA_PIXEL_SIZE_MM = 0.0008
    PAN_SERVO_MAX_ANGLE_DEG = 180 
    TILT_SERVO_MAX_ANGLE_DEG = 90 
    IMAGE_WIDTH_PIXELS = 416
    IMAGE_HEIGHT_PIXELS = 416
    IMAGE_CENTER_X = IMAGE_WIDTH_PIXELS // 2
    IMAGE_CENTER_Y = IMAGE_HEIGHT_PIXELS // 2

    @staticmethod
    def pixels_to_angle(x_pixels, y_pixels):
        """
        Convert pixel coordinates to servo angles using the camera's intrinsic parameters.
        Angles are relative to the image center.
        """
        # Correct for image center
        x_pixels -= CoordinateSystem.IMAGE_CENTER_X
        y_pixels -= CoordinateSystem.IMAGE_CENTER_Y

        x_mm = x_pixels * CoordinateSystem.CAMERA_PIXEL_SIZE_MM
        y_mm = y_pixels * CoordinateSystem.CAMERA_PIXEL_SIZE_MM

        theta_x_rad = np.arctan2(x_mm, CoordinateSystem.CAMERA_FOCAL_LENGTH_MM)
        theta_y_rad = np.arctan2(y_mm, CoordinateSystem.CAMERA_FOCAL_LENGTH_MM)

        theta_x_deg = np.degrees(theta_x_rad)
        theta_y_deg = np.degrees(theta_y_rad)

        return theta_x_deg, theta_y_deg

    @staticmethod
    def calculate_interception_point(x_pixels, y_pixels, vx_pixels, vy_pixels, lead_time):
        """
        Calculate interception point given velocity and lead time.
        """
        interception_x = x_pixels + vx_pixels * lead_time
        interception_y = y_pixels + vy_pixels * lead_time

        return interception_x, interception_y

    @staticmethod
    def calculate_velocity(x_pixels, y_pixels, prev_x_pixels, prev_y_pixels, dt):
        """
        Calculate velocity given current and previous positions, and the time difference.
        """
        vx_pixels = (x_pixels - prev_x_pixels) / dt
        vy_pixels = (y_pixels - prev_y_pixels) / dt

        return vx_pixels, vy_pixels

    @staticmethod
    def pixels_to_servo_command(x_pixels, y_pixels):
        """
        Convert pixel coordinates to servo commands.
        """
        theta_x_deg, theta_y_deg = CoordinateSystem.pixels_to_angle(x_pixels, y_pixels)

        # Normalize to servo command range [0, 4095]
        pan_command = int((theta_x_deg / CoordinateSystem.PAN_SERVO_MAX_ANGLE_DEG) * 4095)
        tilt_command = int((theta_y_deg / CoordinateSystem.TILT_SERVO_MAX_ANGLE_DEG) * 4095)

        return pan_command, tilt_command

    @staticmethod
    def image_position_to_servo_goal(img_pos, img_size, servo_min, servo_max):
        """
        Translate a position in the image (e.g., the center of a bounding box) to a goal position for a servo.
        img_pos is the position in the image (either x or y coordinate).
        img_size is the size of the image (width for x coordinate, height for y coordinate).
        servo_min and servo_max are the minimum and maximum positions for the servo.
        """
        # Normalize image position to range [0, 1]
        norm_img_pos = img_pos / img_size
        # Translate to servo position
        servo_goal = servo_min + norm_img_pos * (servo_max - servo_min)
        return int(servo_goal)

