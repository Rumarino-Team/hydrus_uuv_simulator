#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench

def publish_wrench():
    rospy.init_node('wrench_publisher', anonymous=True)

    wrench_pub = rospy.Publisher('rexrov2/thruster_manager/input', Wrench, queue_size=10)

    rate = rospy.Rate(10)

    wrench_msg = Wrench()
    wrench_msg.force.x = 2000.0
    wrench_msg.force.y = 0.0
    wrench_msg.force.z = 0.0
    wrench_msg.torque.x = 0.0
    wrench_msg.torque.y = 0.0
    wrench_msg.torque.z = 0.0

    while not rospy.is_shutdown():
        wrench_pub.publish(wrench_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_wrench()
    except rospy.ROSInterruptException:
        pass