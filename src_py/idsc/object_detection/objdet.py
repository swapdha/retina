"""
author: az
"""
import rospy
from cv_bridge import CvBridge, CvBridgeError
from inference_graph import DetectionGraph
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
# todo fix this include/import

# remark: cv_bridge it does not support CompressedImage in python


class ObjectDetectorRgb:
    """
    Class doc
    """

    def __init__(self, camera_topic="/pylon_rgb/image_raw",
                 camera_info_topic="/pylon_rgb/camera_info"):
        """Initialize ros publisher, ros subscriber"""
        # Private "buffers"
        self._pylon_last_image = None
        self._cvbridge = CvBridge()
        self._detector = DetectionGraph(arch=2)
        self._boxes, self._scores, self._classes = None, None, None
        # Subscribers
        self.pylon_info_sub = rospy.Subscriber(
                camera_info_topic, CameraInfo,
                self._pylonrgb_info_callback, queue_size=0)
        self.pylon_image_sub = rospy.Subscriber(
                camera_topic, Image,
                self._pylonrgb_image_callback, queue_size=0)

    def _pylonrgb_info_callback(self, msg):
        pass

    def _pylonrgb_image_callback(self, msg):
        """Callback function of subscribed topic.
                Here """
        try:
            self._pylon_last_image = self._cvbridge.imgmsg_to_cv2(
                    img_msg=msg, desired_encoding="rgb8")
            self._boxes, self._scores, self._classes = self._detector.run_inference_on_img(
                    self._pylon_last_image)
            self.broadcast_to_lcm()
        except CvBridgeError as e:
            print(e)

    def broadcast_to_lcm(self):
        pass
