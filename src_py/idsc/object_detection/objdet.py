"""
author: az
"""
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from dvs_msgs.msg import EventArray
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


# todo fix this include/import


# remark: cv_bridge it does not support CompressedImage in python


class ObjectDetector:
    """
    Class doc
    """

    def __init__(self, ):
        """Initialize ros publisher, ros subscriber"""
        # Private "buffers"
        self._pylon_last_image = np.zeros(shape=(2048, 2592, 3), dtype=np.uint8)  # temp hard coded
        # OpenCV + Utils
        self._bridge = CvBridge()
        # Subscribers: pylon
        self.pylon_info_sub = rospy.Subscriber(
                "/pylon_rgb/camera_info", CameraInfo,
                self._pylonrgb_info_callback, queue_size=0)
        self.pylon_image_sub = rospy.Subscriber(
                "/pylon_rgb/image_raw", Image,
                self._pylonrgb_image_callback, queue_size=0)

    def _pylonrgb_info_callback(self, msg):
        pass

    def _pylonrgb_image_callback(self, msg):
        """Callback function of subscribed topic.
                Here """
        try:
            self._pylon_last_image = self._bridge.imgmsg_to_cv2(
                    img_msg=msg, desired_encoding="rgb8")
        except CvBridgeError as e:
            print(e)

        # timestamp = msg.header.stamp
        # print("pylon timestamp", timestamp.to_sec())
        # if self._pylon_last_seq + 1 != msg.header.seq:
        #    self._pylon_drop_counter += 1
        # self._pylon_last_seq = msg.header.seq
        # print("dropped image count", self._pylon_drop_counter)
        # if self._davis_last_image.any() and self._pylon_last_image.any() and \
        #        self._fov_fitter.H is None:
        #    self._try_to_match_fov()

        # image = msg.data
