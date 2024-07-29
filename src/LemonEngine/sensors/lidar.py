import rospy
import ros_numpy as rnp
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan as _LaserScan
from tf.transformations import euler_from_quaternion

from LemonEngine.sensors.sensor import BaseSensor
from typing import *



class Lidar(BaseSensor):
    def __init__(self, topic_name: str = "/scan", *args, **kwargs) -> None:
        """
        A simple Lidar sensor

        :param topic_name: The topic name of the lidar sensor.
        """
        self.get_header, self._set_header = self.data_field("header", Header)
        self.get_ranges, self._set_ranges = self.data_field("ranges", np.ndarray)
        self.get_intensities, self._set_intensities = self.data_field("intensities", np.ndarray)

        # angle_min, angle_max, angle_step
        self.get_angles, self._set_angles = self.data_field("angles", Tuple[float, float, float])

        super().__init__(topic_name=topic_name, message_type=_LaserScan, *args, **kwargs)

    def on_message(self, message: _LaserScan) -> None:
        """
        Handles incoming messages and converts them to numpy arrays.

        :param message: The original message from the ROS topic.
        """
        ranges = rnp.numpify(message.ranges)
        intensities = rnp.numpify(message.intensities)

        # Set data fields
        self._set_header(message.header)
        self._set_ranges(ranges)
        self._set_intensities(intensities)

        self._set_angles((message.angle_min,
                          message.angle_max,
                          message.angle_increment))