#!/usr/bin/env python3

import sys
import cv2
import math
import rospy
import numpy as np
from cv_bridge import CvBridge

from piarm_pnp.srv import SetTarget, SetTargetResponse
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

from std_srvs.srv import *

from armpi_sensor.msg import Led
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

# from armpi_kinematics import ik_transform

from armpi_fpv import PID
from armpi_fpv import Misc
from armpi_fpv import bus_servo_control

from piarm_pnp.msg import ImgTarget

# color tracking
class ObjectTracker:
    def __init__(self):


        self.size = (320, 240)
        self.target_color = 'red'
        self.pub_debug_img = True

        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }

        self.bridge = CvBridge()

        rospy.init_node('target_locater', log_level=rospy.DEBUG)

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        self.image_pub = rospy.Publisher('/target_locater/image_result', Image,
                                         queue_size=1)  # register result image publisher

        self.set_target_srv = rospy.Service('/target_locater/set_target', SetTarget, self.set_target)

        if self.pub_debug_img:
            self.image_debug_pub = rospy.Publisher('/target_locater/image_debug', Image, queue_size=1)

        self.target_pub = rospy.Publisher('/target_locater/img_target', ImgTarget, queue_size=1)

        # get hsv range from ros param server
        self.color_range = rospy.get_param('/pnp_colors/color_range_list', {})

    # Find the contour with the largest area
    # The argument is a list of contours to compare
    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # iterate over all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the area of the contour
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 10:  # Only when the area is greater than 10, the contour of the largest area is valid to filter interference
                    area_max_contour = c

        return area_max_contour, contour_area_max  # returns the largest contour

    def publish_debug_image(self, img, encoding='bgr8', hsv=False):
        # Other encodings: mono8 for black and white, bgr8, rgb8

        if hsv:
            img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        ros_msg = self.bridge.cv2_to_imgmsg(img, encoding=encoding)

        self.image_debug_pub.publish(ros_msg)


    def find_target_location(self, img):

        # Make copy of input image
        img_copy = img.copy()
        # Get height and width of image
        img_h, img_w = img.shape[:2]

        cv2.line(img, (int(img_w / 2 - 10), int(img_h / 2)), (int(img_w / 2 + 10), int(img_h / 2)), (0, 255, 255), 2)
        cv2.line(img, (int(img_w / 2), int(img_h / 2 - 10)), (int(img_w / 2), int(img_h / 2 + 10)), (0, 255, 255), 2)

        # Resize the image using interpolation nearest method, this method selects the nearest pixel in the original image to the pixel in the resized image
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)



        # Convert image to HSV space
        frame_hsv = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2HSV)

        # Initialize area tracking variables
        area_max = 0
        area_max_contour = 0



        # If the target color is one that you have parameters for
        if self.target_color in self.color_range:
            # Record the LAB color parameters
            target_color_range = self.color_range[self.target_color]

            print(target_color_range)
            # Create a binary mask of the image based on the color range defined in the config file, all in the LAB color space, values between the min and max will be white, those outside will be black
            frame_mask = cv2.inRange(frame_hsv, tuple(target_color_range['min']), tuple(target_color_range['max']))

            # erode operation
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            # dilate operation
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

            frame_resize = cv2.bitwise_and(frame_hsv, frame_hsv, mask=dilated)
            self.publish_debug_image(frame_resize, hsv=True)

            # Find the outline
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            # Find the largest contour
            area_max_contour, area_max = self.getAreaMaxContour(contours)

        # If an area was found that is larger than 100
        if area_max > 100:
            # Get the smallest circumscribed circle
            (center_x, center_y), radius = cv2.minEnclosingCircle(area_max_contour)
            center_x = int(Misc.map(center_x, 0, self.size[0], 0, img_w))
            center_y = int(Misc.map(center_y, 0, self.size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, self.size[0], 0, img_w))
            # If a large circle is found, then do nothing? Maybe becuase it's too close?
            if radius > 100:
                return img, False
            # Add a circle to the image at the color location
            cv2.circle(img, (int(center_x), int(center_y)), int(radius), self.range_rgb[self.target_color], 2)

            return img, (center_x, center_y, area_max)
        return img, False
        #     if self.start_move:
        #         # Set setpoint as center of the image, not sure why this is continually reset, it should always be the same
        #         self.x_pid.SetPoint = img_w / 2.0
        #         # Set goal position for controller to move toward
        #         self.x_pid.update(center_x)
        #         # Get move amount in x direction from PID controller
        #         dx = self.x_pid.output
        #         # print(dx)
        #         # print(self.x_dis)
        #         self.x_dis += int(dx)  # output
        #         # print(self.x_dis)
        #         # print("\n")

        #         # Set mins and maxes for x_dis
        #         self.x_dis = 200 if self.x_dis < 200 else self.x_dis
        #         self.x_dis = 800 if self.x_dis > 800 else self.x_dis

        #         # Set the default position
        #         self.y_pid.SetPoint = 900  # set up
        #         # Y sets the in/out movement, so if the object is bigger, it moves backwards, if it's larger it move forward
        #         if abs(area_max - 900) < 50:
        #             area_max = 900
        #         self.y_pid.update(area_max)  # current
        #         dy = self.y_pid.output
        #         self.y_dis += dy  # output

        #         # Set mins and maxes for y
        #         self.y_dis = 0.12 if self.y_dis < 0.12 else self.y_dis
        #         self.y_dis = 0.25 if self.y_dis > 0.25 else self.y_dis

        #         # Set setpoint as the center of the image (vertically)
        #         self.z_pid.SetPoint = img_h / 2.0
        #         self.z_pid.update(center_y)
        #         dy = self.z_pid.output
        #         self.z_dis += dy
        #         # self.z_dis = dy
        #         # Set the mins and maxes for the z dis
        #         self.z_dis = 0.22 if self.z_dis > 0.22 else self.z_dis
        #         self.z_dis = 0.17 if self.z_dis < 0.17 else self.z_dis

        #         self.target = self.ik.setPitchRanges((0, round(self.y_dis, 4), round(self.z_dis, 4)), -90, -85, -95)
        #         if self.target:
        #             self.servo_data = self.target[1]
        #             bus_servo_control.set_servos(self.joints_pub, 20, (
        #                 (3, self.servo_data['servo3']), (4, self.servo_data['servo4']), (5, self.servo_data['servo5']), (6, self.x_dis)))
        # return img

    def image_callback(self, ros_image):

        bgr_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        frame_result, target_location = self.find_target_location(bgr_image)
        if target_location:
            target_msg = ImgTarget()
            ImgTarget.header.stamp = rospy.Time.now()
            target_msg.target_x = target_location[0]
            target_msg.target_y = target_location[1]
            target_msg.target_size = target_location[2]
            target_msg.height = ros_image.height
            target_msg.width = ros_image.width
            self.target_pub.publish(target_msg)

        ros_msg = self.bridge.cv2_to_imgmsg(frame_result, encoding='bgr8')

        self.image_pub.publish(ros_msg)


    def set_target(self, msg):
        self.target_color = msg.data
        response = SetTargetResponse()
        response.success = True
        response.message = "Color successfully changed to: " + msg.data
        return response





if __name__ == '__main__':

    tracker = ObjectTracker()
    rospy.spin()


    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     rospy.loginfo("Shutting down")
    # finally:
    #     cv2.destroyAllWindows()
