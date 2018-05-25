"""
author: az
"""
import objdet
import sys
import rospy


def main(args):
    """Initializes and cleanup ros node"""
    rospy.init_node('Object_detection_node', anonymous=True)
    try:
        node = objdet.Zauron()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down ROS")


if __name__ == '__main__':
    main(sys.argv)
