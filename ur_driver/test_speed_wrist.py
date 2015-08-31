#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
from std_msgs.msg import Float64MultiArray

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        pub = rospy.Publisher("ur_driver/joint_group_velocity_controller/command", Float64MultiArray, queue_size=1)
        time.sleep(1)

        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.1, 0.0]
        print("Publishing message: " + str(msg))
        for i in range(0,30):
            pub.publish(msg)
            time.sleep(0.1)


        # time.sleep(2.0)
        # msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
        # pub.publish(msg)
        # print("Published message: " + str(msg))
        #
        # time.sleep(2.0)
        # msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # pub.publish(msg)
        # print("Published message: " + str(msg))

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
