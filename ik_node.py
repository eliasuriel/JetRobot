#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from inverse_kinematics import InverseKinematics

def motor_publisher():
    rospy.init_node('ik_node',anonymous=True)
    rate = rospy.Rate(10)# Hz
    ik_handler = InverseKinematics([0.1,0.1,0.1,0.1])
    #                                   topic_name!
    joint3_publisher = rospy.Publisher('/joint3_controller/command')

    while not rospy.is_shutdown():
        q_vals = ik_handler.solve()
        msg_joint3 = Float64()
        msg_joint3.date = 1.0 #!!!SUSTITUIR
        joint3_publisher.publish(msg_joint3)
        rate.sleep()

if __name__ == '__main__':
    try:
        motor_publisher()
    except rospy.ROSInterruptException:
        pass