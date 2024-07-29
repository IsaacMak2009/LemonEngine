import rospy
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image as _Image
from enum import Enum
from LemonEngine.sensors.sensor import BaseSensor
from typing import *
from loguru import logger

ENCODINGS = Literal[
    "rgb8", "rgba8", "rgb16", "rgba16",
    "bgr8", "bgra8", "bgr16", "bgra16",
    "mono8", "mono16", "passthrough"
]


class Camera(BaseSensor):
    def __init__(self,
                 topic_name: str = "/camera/color/image_raw",
                 encoding: ENCODINGS = "bgr8",
                 *args, **kwargs) -> None:
        """
        A simple camera sensor, can be used with depth and rgb camera

        :param topic_name: the topic name of camera
        :param timeout: the timeout for waiting for the first message (None for no waiting)
        """
        self.width = None
        self.height = None
        self.frame = None
        self.bridge = cv_bridge.CvBridge()
        self.encoding = encoding
        self.frame_cnt = 0

        self.get_frame, self._set_frame = self.data_field("frame", np.ndarray)
        self.get_frame_cnt, self._set_frame_cnt = self.data_field("frame_cnt", int)

        self.shape = None

        super().__init__(topic_name=topic_name, message_type=_Image, *args, **kwargs)

    def on_ready(self, _) -> None:
        self.shape = self.frame.shape
        self.width = self.frame.shape[1]
        self.height = self.frame.shape[0]
        logger.debug(f"Camera at {self.topic_name}'s parameter:")
        logger.debug(f" * topic: {self.topic_name}")
        logger.debug(f" * width: {self.width}")
        logger.debug(f" * height: {self.height}")
        logger.debug(f" * shape: {self.shape}")
        logger.debug(f" * dtype: {self.frame.dtype}")
        logger.debug(f"")

    def on_message(self, message: _Image) -> None:
        """
        :param message: the original message from ros topic
        :return: the converted data
        """
        self.frame_cnt += 1
        self.frame: np.ndarray = self.bridge.imgmsg_to_cv2(message, self.encoding)

        self._set_frame(self.frame)
        self._set_frame_cnt(self.frame_cnt)