#!/usr/bin/env python3

import rospy
from kinematics import ik_transform
from piarm_pnp.srv import GetIK, GetIKResponse

class IKServer:
    def __init__(self):
        rospy.init_node("ik_server")
        self.ik = ik_transform.ArmIK()
        self.ik_service = rospy.Service('/get_ik', GetIK, self.get_ik)

    def get_ik(self, target_msg):
        x = target_msg.target.target_x
        y = target_msg.target.target_y
        z = target_msg.target.target_z
        pitch = target_msg.target.target_pitch
        pitch_lower_limit = target_msg.target.pitch_lower_limit
        pitch_upper_limit = target_msg.target.pitch_upper_limit

        ik_result = self.ik.setPitchRanges((round(x, 4), round(y, 4), round(z, 4)), pitch, pitch_upper_limit,
                                         pitch_lower_limit)

        response = GetIKResponse()
        if ik_result:
            # response.servo_angles.servo1 =
            # response.servo_angles.servo2 =
            response.servo_angles.servo3 = ik_result[1]['servo3']
            response.servo_angles.servo4 = ik_result[1]['servo4']
            response.servo_angles.servo5 = ik_result[1]['servo5']
            response.servo_angles.servo6 = ik_result[1]['servo6']
            response.pitch = ik_result[2]
            response.success = True
        else:
            response.success = False

        return response

ik_server = IKServer()
rospy.spin()
