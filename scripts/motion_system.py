#!/usr/bin/env python3

import sys
import rospy

from std_srvs.srv import *
from object_tracking.srv import *
from piarm_pnp.srv import GetIK, GetIKRequest

from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from piarm_pnp.msg import ImgTarget, MoveTarget

from armpi_fpv import PID
from armpi_fpv import bus_servo_control

class MotionSystem:
    def __init__(self):

        # Coordinate Initial Values
        self.x_dis = 500
        self.y_dis = 0.167
        self.z_dis_default = 0.2
        self.z_dis = self.z_dis_default

        # Setup PID for each axis
        # self.x_pid = PID.PID(P=0.06, I=0.005, D=0)
        # self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        # self.z_pid = PID.PID(P=0.00003, I=0, D=0)
        self.x_pid = PID.PID(P=0.03, I=0.005, D=0)
        self.y_pid = PID.PID(P=0.000005, I=0, D=0)
        self.z_pid = PID.PID(P=0.00001, I=0, D=0)

        rospy.init_node('motion_system', log_level=rospy.DEBUG)

        # Setup joint position publisher
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur,
                                          queue_size=1)

        # See if /get_ik service is running, exit if not
        try:
            rospy.wait_for_service('/get_ik', timeout=2)
        except rospy.ROSException:
            rospy.logwarn('Service /get_ik not available, exiting...')
            sys.exit(0)

        # Connect to /get_ik server
        self.ik_srv_connection = rospy.ServiceProxy('/get_ik', GetIK)

        # Wait a second then move to the initial position
        rospy.sleep(1)

        self.initMove()

        self.reset_service = rospy.Service('/reset_arm_position', Empty, self.reset)

        # Subscribe to img target topic
        self.img_target_sub = rospy.Subscriber('target_locater/img_target', ImgTarget, self.target_callback)

    # variable reset
    def reset(self, msg):
        self.img_target_sub.unregister()
        self.x_dis = 500
        self.y_dis = 0.167
        self.z_dis = self.z_dis_default
        self.initMove()
        self.img_target_sub = self.img_target_sub = rospy.Subscriber('target_locater/img_target', ImgTarget, self.target_callback)

    # Move the arm to the initial position
    def initMove(self, delay=True):

        # Setup inverse kinematics request
        ik_request = GetIKRequest()
        ik_request.target.target_x = 0
        ik_request.target.target_y = self.y_dis
        ik_request.target.target_z = self.z_dis_default
        ik_request.target.target_pitch = -90
        ik_request.target.pitch_upper_limit = -88
        ik_request.target.pitch_lower_limit = -92

        # Get servo positions from inverse kinematic service
        servo_msg = self.ik_srv_connection(ik_request)

        # If the position is reachable, write the servo values to the servos
        if servo_msg.success:
            bus_servo_control.set_servos(self.joints_pub, 1500, (
            (1, 200), (2, 500), (3, servo_msg.servo_angles.servo3), (4, servo_msg.servo_angles.servo4),
            (5, servo_msg.servo_angles.servo5), (6, servo_msg.servo_angles.servo6)))

        if delay:
            rospy.sleep(2)

    def target_callback(self, img_target_msg):

        # Extract the target data from the message
        img_w = img_target_msg.width
        img_h = img_target_msg.height
        center_x = img_target_msg.target_x
        center_y = img_target_msg.target_x
        area_max = img_target_msg.target_size


        # Set setpoint as center of the image
        self.x_pid.SetPoint = img_w / 2.0
        # Set goal position for controller to move toward
        self.x_pid.update(center_x)
        # Get move amount in x direction from PID controller
        dx = self.x_pid.output
        self.x_dis += int(dx)  # output

        # Set mins and maxes for x_dis
        self.x_dis = 100 if self.x_dis < 100 else self.x_dis
        self.x_dis = 900 if self.x_dis > 900 else self.x_dis

        # Set the default position
        self.y_pid.SetPoint = 900  # set up
        # Y sets the in/out movement, so if the object is bigger, it moves backwards, if it's larger it move forward
        if abs(area_max - 900) < 50:
            area_max = 900
        self.y_pid.update(area_max)  # current
        dy = self.y_pid.output
        self.y_dis += dy  # output

        # Set mins and maxes for y
        self.y_dis = 0.12 if self.y_dis < 0.12 else self.y_dis
        self.y_dis = 0.25 if self.y_dis > 0.25 else self.y_dis

        # Set setpoint as the center of the image (vertically)
        self.z_pid.SetPoint = img_h / 2.0
        self.z_pid.update(center_y)
        dy = self.z_pid.output
        self.z_dis += dy

        # Set the mins and maxes for the z dis
        self.z_dis = 0.22 if self.z_dis > 0.22 else self.z_dis
        self.z_dis = 0.17 if self.z_dis < 0.17 else self.z_dis

        ik_request = GetIKRequest()
        ik_request.target.target_x = 0
        ik_request.target.target_y = self.y_dis
        ik_request.target.target_z = self.z_dis
        ik_request.target.target_pitch = -90
        ik_request.target.pitch_upper_limit = -85
        ik_request.target.pitch_lower_limit = -95


        servo_msg = self.ik_srv_connection(ik_request)
        if servo_msg.success:
            bus_servo_control.set_servos(self.joints_pub, 20, ((3, servo_msg.servo_angles.servo3),
                                                               (4, servo_msg.servo_angles.servo4),
                                                               (5, servo_msg.servo_angles.servo5),
                                                               (6, self.x_dis)))


if __name__ == '__main__':

    tracker = MotionSystem()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
