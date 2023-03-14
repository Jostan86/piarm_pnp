#!/usr/bin/env python3
from time import sleep
import rospy
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur

rospy.init_node('ik_tester')
pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

servo_ids = [6, 5, 4, 3, 2, 1]
pos_s = [641, 626, 855, 270, 500, 450]
duration = 2

raw_list = []
for ids, pos in zip(servo_ids, pos_s):
    servo_msg = RawIdPosDur()
    servo_msg.id = ids
    servo_msg.position = pos
    servo_msg.duration = 3000
    raw_list.append(servo_msg)


multi_list = MultiRawIdPosDur()
multi_list.id_pos_dur_list = raw_list
print(multi_list)
x=0
print(x)
try:
    while x<1000:
        connection = pub.get_num_connections()
        print(connection)
        if connection > 0:
            print('hi')
            pub.publish(multi_list)
            break
        else:
            sleep(.1)
            x +=1
            print(x)

except rospy.ROSInterruptException:
    pass
