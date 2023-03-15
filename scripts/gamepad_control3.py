#!/usr/bin/env python3

import sys
import rospy

from std_srvs.srv import Empty
from object_tracking.srv import *
from piarm_pnp.srv import GetIK, GetIKRequest

from hiwonder_servo_msgs.msg import MultiRawIdPosDur, ServoStateList, CommandDuration
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from armpi_fpv import PID
from armpi_fpv import bus_servo_control



class MotionSystem:
    def __init__(self):

        # Coordinate Initial Values
        self.base_pos = 0.0

        self.prev_base_pos = self.base_pos

        rospy.init_node('joy_control', log_level=rospy.DEBUG)

        # Setup joint position publisher
        self.base_joint_pub = rospy.Publisher('/joint1_controller/command_duration', CommandDuration, queue_size=1)

        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.d_base = 0

    def publish_move(self):
        move_msg = CommandDuration()
        move_msg.data = self.base_pos #+ self.d_base*10
        move_msg.duration = 20
        self.base_joint_pub.publish(move_msg)


    def joy_callback(self, joy_msg):
        base_rotate = joy_msg.axes[6] # D pad right left

        self.d_base = base_rotate / 40


        self.base_pos += self.d_base


        self.publish_move()


if __name__ == '__main__':

    tracker = MotionSystem()
    # rate = rospy.Rate(100)
    try:
        rospy.spin()
        # while not rospy.is_shutdown() and tracker.base_pos < 1:
        #     # tracker.base_pos += 0.01
        #     tracker.publish_move()
        #     rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")