#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
from ur_msgs.srv import SetGravity


def main():
    global client
    try:
        rospy.init_node("test_set_gravity", anonymous=True, disable_signals=True)
        rospy.wait_for_service('ur_driver/set_gravity')

        try:
            set_gravity = rospy.ServiceProxy('ur_driver/set_gravity', SetGravity)
            resp1 = set_gravity([0.0, 0.0, 0.0])
            return resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
