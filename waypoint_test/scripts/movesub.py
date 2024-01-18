#!/usr/bin/env python

# ros melodic node class that moves a rexrov2 submarine to a waypoint

#roslaunch rexrov2_control start_pid_controller.launch

# rosservice call /rexrov2/go_to "waypoint:
#   header:
#     seq: 0
#     stamp: {secs: 0, nsecs: 0}
#     frame_id: 'world_ned'
#   point: {x: 300.0, y: 300.0, z: 40.0}
#   max_forward_speed: 2.0
#   heading_offset: 0.0
#   use_fixed_heading: false
#   radius_of_acceptance: 0.5
# max_forward_speed: 2.0
# interpolator: 'dubins'"

import sys
import rospy
from uuv_control_msgs.srv import GoTo
from uuv_control_msgs.msg import Waypoint
from std_msgs.msg import Header, Time
from geometry_msgs.msg import Point

def move_sub_client(x, y, z):
    try:
        rospy.wait_for_service('/rexrov2/go_to', timeout=30)
        move_sub = rospy.ServiceProxy('/rexrov2/go_to', GoTo)

        header = Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = 'world_ned'

        point = Point()
        point.x = x
        point.y = y
        point.z = z

        waypoint = Waypoint(header, point, 2.0, 0.0, False, 0.5)
        waypoint.header = header
        waypoint.point = point
        waypoint.max_forward_speed = 2.0
        waypoint.heading_offset = 0.0
        waypoint.use_fixed_heading = False
        waypoint.radius_of_acceptance = 0.5

        resp = move_sub(waypoint, 2.0, 'dubins')
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y z]"%sys.argv[0]

if __name__ == "__main__":
    rospy.init_node('movesub')
    
    if rospy.is_shutdown():
        rospy.logerr('ROS master not running!')
        sys.exit(-1)
   

    if len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
    else:
        print(usage())
        sys.exit(1)
    print("Moving sub to %s, %s, %s"%(x, y, z))
    print("Sub is moving: %s"%move_sub_client(x, y, z))