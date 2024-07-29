import rospy
import ros_numpy as rnp
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import Imu as _Imu
from tf.transformations import euler_from_quaternion

from LemonEngine.sensors.sensor import BaseSensor
from typing import *

class Imu(BaseSensor):
    def __init__(self, topic_name: str = "/imu", *args, **kwargs) -> None:
        """
        A simple Imu sensor.

        :param topic_name: The topic name of the Imu sensor.
        """
        self.get_header, self._set_header = self.data_field("header", Header)
        self.get_linear_accel, self._set_linear_accel = self.data_field("linear_accel", np.ndarray)
        self.get_angular_vel, self._set_angular_vel = self.data_field("angular_vel", np.ndarray)
        self.get_orientation, self._set_orientation = self.data_field("orientation", np.ndarray)

        super().__init__(topic_name=topic_name, message_type=_Imu, *args, **kwargs)

    def on_message(self, message: _Imu) -> None:
        """
        Handles incoming messages and converts them to numpy arrays.

        :param message: The original message from the ROS topic.
        """
        # Convert to numpy data
        linear_accel = rnp.numpify(message.linear_acceleration)
        angular_vel = rnp.numpify(message.angular_velocity)
        orientation = rnp.numpify(message.orientation)
        orientation = euler_from_quaternion(orientation)  # Convert to Euler angles

        # Set data fields
        self._set_header(message.header)
        self._set_linear_accel(linear_accel)
        self._set_angular_vel(angular_vel)
        self._set_orientation(orientation)