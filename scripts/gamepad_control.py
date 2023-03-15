#!/usr/bin/env python3

import sys
import rospy

from std_srvs.srv import Empty
from object_tracking.srv import *
from piarm_pnp.srv import GetIK, GetIKRequest

from hiwonder_servo_msgs.msg import MultiRawIdPosDur, ServoStateList
from sensor_msgs.msg import Joy

from armpi_fpv import PID
from armpi_fpv import bus_servo_control



class MotionSystem:
    def __init__(self):

        # Coordinate Initial Values
        self.base_pos = 500
        self.x_pos = 0.167
        self.y_pos = 0.2
        self.gripper_roll = 500
        self.gripper_pitch = -90
        self.jaw_pos = 200
        # self.jaw_pos_actual = 200

        self.prev_jaw_pos = self.jaw_pos
        self.prev_gripper_roll = self.gripper_roll
        self.prev_x_pos = self.x_pos
        self.prev_y_pos = self.y_pos
        self.prev_gripper_pitch = self.gripper_pitch
        self.prev_base_pos = self.base_pos

        # Setup PID for each axis
        # self.x_pid = PID.PID(P=0.06, I=0.005, D=0)
        # self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        # self.z_pid = PID.PID(P=0.00003, I=0, D=0)
        self.base_pid = PID.PID(P=0.03, I=0, D=0)
        self.x_pid = PID.PID(P=0.000005, I=0, D=0)
        self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        self.jaw_pid = PID.PID(P=0.03, I=0, D=0)
        self.pitch_pid = PID.PID(P=0.03, I=0, D=0)
        self.roll_pid = PID.PID(P=0.03, I=0, D=0)


        rospy.init_node('joy_control', log_level=rospy.DEBUG)

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

        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.servo_state_sub = rospy.Subscriber('/servo_controllers/port_id_1/servo_states', ServoStateList,
                                                self.update_servo_pos_actual)

    # variable reset
    def reset(self, msg):
        self.base_pos = 500
        self.x_pos = 0.167
        self.y_pos = 0.2
        self.gripper_roll = 500
        self.gripper_pitch = -90
        self.jaw_pos = 200
        self.initMove()

    # Move the arm to the initial position
    def initMove(self, delay=True):

        # Setup inverse kinematics request
        ik_request = GetIKRequest()
        ik_request.target.target_x = 0
        ik_request.target.target_y = self.x_pos
        ik_request.target.target_z = self.y_pos
        ik_request.target.target_pitch = self.gripper_pitch
        ik_request.target.pitch_upper_limit = self.gripper_pitch + 2
        ik_request.target.pitch_lower_limit = self.gripper_pitch - 2

        # Get servo positions from inverse kinematic service
        servo_msg = self.ik_srv_connection(ik_request)
        # If the position is reachable, write the servo values to the servos
        if servo_msg.success:
            bus_servo_control.set_servos(self.joints_pub, 1500, ((1, self.jaw_pos),
                                                                 (2, self.gripper_roll),
                                                                 (3, servo_msg.servo_angles.servo3),
                                                                 (4, servo_msg.servo_angles.servo4),
                                                                 (5, servo_msg.servo_angles.servo5),
                                                                 (6, servo_msg.servo_angles.servo6)))

        if delay:
            rospy.sleep(2)

    def update_servo_pos_actual(self, servo_state_msg):
        for servo_state in servo_state_msg.servo_states:
            if servo_state.id == 1:
                self.jaw_pid.update(servo_state.position)
                # if servo_state.id == 2:
                # self.

    def joy_callback(self, joy_msg):
        base_rotate = joy_msg.axes[6] # D pad right left
        gripper_roll = joy_msg.axes[3] # right thumbstick up-down
        gripper_pitch = joy_msg.axes[4] # Right thumbstick up-down
        in_out_move = joy_msg.axes[0] # Left thumbstick side-to-side
        up_down_move = joy_msg.axes[1] # Left thumbstick up-down
        jaw_position = joy_msg.axes[2] # Left trigger

        d_base = base_rotate * 5
        dx = in_out_move / 500
        dy = up_down_move / 500
        d_roll = gripper_roll * 5
        d_pitch = gripper_pitch / 2

        self.base_pos += d_base
        self.x_pos += dx
        self.y_pos += dy
        self.gripper_roll += d_roll
        self.gripper_pitch += d_pitch

        in_min = 1
        in_max = -1
        out_max = 550
        out_min = 0
        jaw_position_mapped = (jaw_position - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


        self.jaw_pid.SetPoint = jaw_position_mapped
        d_jaw_pos = self.jaw_pid.output
        self.jaw_pos += int(d_jaw_pos)


    def publish_move(self):
        # Setup inverse kinematics request
        ik_request = GetIKRequest()
        ik_request.target.target_x = 0
        ik_request.target.target_y = self.x_pos
        ik_request.target.target_z = self.y_pos
        ik_request.target.target_pitch = self.gripper_pitch
        ik_request.target.pitch_upper_limit = self.gripper_pitch + 1
        ik_request.target.pitch_lower_limit = self.gripper_pitch - 1

        # Get servo positions from inverse kinematic service
        servo_msg = self.ik_srv_connection(ik_request)

        # If the position is reachable, write the servo values to the servos
        if servo_msg.success:
            bus_servo_control.set_servos(self.joints_pub, 333, ((1, self.jaw_pos),
                                                                 (2, self.gripper_roll),
                                                                 (3, servo_msg.servo_angles.servo3),
                                                                 (4, servo_msg.servo_angles.servo4),
                                                                 (5, servo_msg.servo_angles.servo5),
                                                                 (6, int(self.base_pos))))

            # If the inverse kinematics was successful, then save the current values, if not, then reset them to the
            # previous values
            self.prev_jaw_pos = self.jaw_pos
            self.prev_gripper_roll = self.gripper_roll
            self.prev_x_pos = self.x_pos
            self.prev_y_pos = self.y_pos
            self.prev_gripper_pitch = self.gripper_pitch
            self.prev_base_pos = self.base_pos
        else:
            self.jaw_pos = self.prev_jaw_pos
            self.gripper_roll = self.prev_gripper_roll
            self.x_pos = self.prev_x_pos
            self.y_pos = self.prev_y_pos
            self.gripper_pitch = self.prev_gripper_pitch
            self.base_pos = self.prev_base_pos


if __name__ == '__main__':

    tracker = MotionSystem()
    rate = rospy.Rate(3)
    try:
        # rospy.spin()
        while not rospy.is_shutdown():
            tracker.publish_move()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")