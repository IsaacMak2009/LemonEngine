import rospy
import ros_numpy as rnp
import numpy as np

from std_msgs.msg import Header
from nav_msgs.msg import Odometry as _Odometry
from tf.transformations import euler_from_quaternion

from LemonEngine.sensors.sensor import BaseSensor
from typing import *


class Odometer(BaseSensor):
    def __init__(self, topic_name: str = "/odom", *args, **kwargs) -> None:
        """
        A simple odometer sensor that obtains position and orientation.

        :param topic_name: The topic name of the odometer.
        """
        self.get_header, self._set_header = self.data_field("header", Header)
        self.get_position, self._set_position = self.data_field("position", np.ndarray)
        self.get_orientation, self._set_orientation = self.data_field("orientation", np.ndarray)
        self.get_linear_accel, self._set_linear_accel = self.data_field("linear_accel", np.ndarray)
        self.get_angular_vel, self._set_angular_vel = self.data_field("angular_vel", np.ndarray)

        super().__init__(topic_name=topic_name, message_type=_Odometry, *args, **kwargs)

    def on_message(self, message: _Odometry) -> None:
        """
        Handles incoming messages and converts them to numpy arrays.

        :param message: The original message from the ROS topic.
        """
        # Convert to numpy data
        position = rnp.numpify(message.pose.pose.position)
        orientation = rnp.numpify(message.pose.pose.orientation)
        linear_accel = rnp.numpify(message.twist.twist.linear)
        angular_vel = rnp.numpify(message.twist.twist.angular)
        orientation = euler_from_quaternion(orientation)  # Convert to Euler angles

        # Set data fields
        self._set_header(message.header)
        self._set_position(position)
        self._set_orientation(orientation)
        self._set_linear_accel(linear_accel)
        self._set_angular_vel(angular_vel)
