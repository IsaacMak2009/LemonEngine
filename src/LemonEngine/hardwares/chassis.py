from typing import *

import numpy as np
from loguru import logger

import actionlib
import ros_numpy as rnp
import rospy
import tf2_geometry_msgs as tf2msg
import tf2_ros
from LemonEngine.hardwares.hardware import BaseHardware
from LemonEngine.sensors import Odometer, Sensor
from LemonEngine.utils.checker import Checker
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, PoseStamped, TransformStamped
from geometry_msgs.msg import Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class Chassis(BaseHardware):
    """
    API wrapper for common chassis operations.
    """

    def __init__(self, cmd_vel_topic: str = "/cmd_vel", odometry_topic: str = "/odom") -> None:
        """
        Constructor

        :param cmd_vel_topic: topic to control the chassis, usually /cmd_vel. see the chassis docs for details
        :param odometry_topic: odometry topic of the chassis, usually /odom
        """
        super().__init__()
        Checker().check_topic_status(cmd_vel_topic, required=True)

        # Check the status of the odometry topic
        if Checker().check_topic_status(odometry_topic, required=False):
            self.sensor = Odometer(odometry_topic)
        else:
            logger.warning("Cannot find Odometry topics, which may affect accuracy")
            self.sensor: Optional[Odometer] = None

        # The twist publisher use to control the robot
        self.twist_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

    def _set_twist(self, twist: Twist) -> None:
        self.twist_publisher.publish(twist)

    @overload
    def set_angular(self, x: float) -> None:
        """
        Only set the angular velocity along the z-axis, Usually mobile base robots only support this type of rotation.
        :param x: the angular velocity along the z-axis
        :return:
        """
        ...

    @overload
    def set_angular(self, x: Vector3) -> None:
        """
        Set the angular velocity as vector
        :param x: the vector
        :return:
        """
        ...

    @overload
    def set_angular(self, x: float, y: float, z: float) -> None:
        """
        Set the angular velocity along the x-axis, y-axis and z-axis.
        :param x: the angular velocity along the x-axis
        :param y: the angular velocity along the y-axis
        :param z: the angular velocity along the z-axis
        :return:
        """
        ...

    def set_angular(self, x: Union[float, Vector3], y: float = None, z: float = None) -> None:
        """
        Set the angular velocity
        """
        twist = Twist()
        if isinstance(x, Vector3):
            twist.angular = x
        elif isinstance(x, float):
            twist.angular.z = x
            if y is not None and z is not None:
                twist.angular.y = y
                twist.angular.z = z

        self._set_twist(twist)

    @overload
    def set_linear(self, x: float) -> None:
        """
        Set the linear speed in the x-axis direction to move forward or backward.
        :param x: the linear speed in x-axis direction
        :return:
        """
        ...

    @overload
    def set_linear(self, x: Vector3) -> None:
        """
        Set the angular velocity as vector
        :param x: the vector
        :return:
        """
        ...

    @overload
    def set_linear(self, x: float, y: float, z: float) -> None:
        """
        Set the linear velocity along the x-axis, y-axis and z-axis.
        :param x: the linear velocity along the x-axis
        :param y: the linear velocity along the y-axis
        :param z: the linear velocity along the z-axis
        """
        ...

    def set_linear(self, x: Union[float, Vector3], y: float = None, z: float = None) -> None:
        """
        Set the linear velocity
        """
        twist = Twist()
        if isinstance(x, Vector3):
            twist.linear = x
        elif isinstance(x, float) and y is None and z is None:
            twist.linear.x = x
            if y is not None and z is not None:
                twist.linear.y = y
                twist.linear.z = z

        self._set_twist(twist)

    def stop_moving(self):
        """
        Make the robot stop moving
        :return:
        """
        twist = Twist()
        self._set_twist(twist)

    def get_current_position(self) -> Optional[np.ndarray]:
        """
        Get the current position of the chassis.
        :return: the current position of the chassis
        """
        if self.sensor is None:
            return None
        return self.sensor.get_position()

    def get_current_euler(self) -> Optional[np.ndarray]:
        """
        Get the current orientation of the chassis.
        :return: the orientation of the chassis (euler)
        """
        if self.sensor is None:
            return None
        return self.sensor.get_orientation()

    def turn_for_degrees(self,
                         max_speed: float,
                         degrees: float,
                         direction: Literal["clockwise", "counterclockwise", "auto"] = "auto") -> None:
        """
        Let the robot rotate a specified number of degrees.
        :param max_speed: the maximum angular speed of the chassis during turning
        :param degrees: the desired number of degrees
        :param direction: the direction of the turning, for `auto` it will turn in the shorter direction
        :return:
        """
        raise NotImplementedError

    # TODO: Add Pid control for turning and moving


class Navigator(BaseHardware):
    """
    API wrapper for Navigation.
    """
    def __init__(self, frame_id: str = "map") -> None:
        """
        Constructor

        :param frame_id: usually "map"
        """
        super().__init__()
        self.goal = None
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self._amcl_pose: Sensor[PoseWithCovarianceStamped] = Sensor("/amcl_pose", PoseWithCovarianceStamped)
        self._move_base_goal: Sensor[PoseStamped] = Sensor("/move_base_simple/goal", PoseStamped)
        self._move_base_status: Sensor[GoalStatusArray] = Sensor("/move_base/status", GoalStatusArray)

        self._clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

        self.frame_id = frame_id
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.trans = self.lookup_transform()

        logger.success("Navigator initialized")

    def lookup_transform(self) -> Optional[TransformStamped]:
        """
        Lookup transform of 'base_link'

        :return: the transform
        """
        try:
            return self.tf_buffer.lookup_transform(
                self.frame_id,
                'base_link',
                rospy.Time(0),
                rospy.Duration(1)
            )
        except Exception as e:
            logger.error(e)
            logger.error("Cannot find transform from base link")
            return None

    def transform_point(self, point: Union[np.ndarray, list, Point]) -> Optional[Point]:
        """
        Transform a point on the chassis to world coordinates.

        :param point: the point to be transformed
        :return:
        """
        if self.trans is None:
            logger.error("Cannot find transform from base link, so cannot transform point")

        if isinstance(point, (np.ndarray, list)):
            point = Point(x=point[0], y=point[1], z=point[2])
        elif not isinstance(point, Point):
            logger.warning("Cannot transform point from type {}".format(type(point)))
            return None

        _pos = tf2msg.PointStamped()
        _pos.header.frame_id = "base_link"
        _pos.header.stamp = rospy.Time(0)
        _pos.point = point
        return self.tf_buffer.transform(_pos, 'map').point

    @staticmethod
    def flat_point_to_pose(x: float, y: float, theta: float) -> Pose:  # we define a point (x, y, yaw)
        """
        Transform a flat point (x, y, yaw) to a pose.

        :param x: x
        :param y: y
        :param theta: yaw
        :return: the transformed pose
        """
        q = quaternion_from_euler(0.0, 0.0, theta)
        pose = Pose(Point(x, y, 0.0), Quaternion(q[0], q[1], q[2], q[3]))
        return pose

    @staticmethod
    def pose_to_flat_point(pose: Pose) -> Tuple[float, float, float]:
        """
        Transform a pose to a flat point (x, y, yaw).

        :param pose: pose
        :return: the transformed point
        """
        x, y = pose.position.x, pose.position.y
        q = rnp.numpify(pose.orientation)
        roll, pitch, yaw = euler_from_quaternion(q)
        return x, y, yaw

    def move_to(self, x, y, theta) -> bool:
        """
        navigate chassis to a specific location

        :param x: x
        :param y: y
        :param theta: yaw
        :return: success or not
        """
        self.clear_costmaps()

        location = Navigator.flat_point_to_pose(x, y, theta)

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = self.frame_id
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = location
        self.move_base.send_goal(self.goal)

        rospy.sleep(1)
        success = self.move_base.wait_for_result(rospy.Duration(300))

        if success:
            logger.info("Reached point.")
        else:
            logger.warning("Failed to reach point.")

        return success

    def cancel_goal(self) -> None:
        """
        Cancel navigation goal

        :return:
        """
        logger.debug("Cancel goal")
        self.move_base.cancel_goal()

    def clear_costmaps(self) -> None:
        """
        Clear costmaps

        :return:
        """
        logger.debug("Costmap clearing...")
        self._clear_costmaps()

    def get_goal_current_status(self) -> GoalStatus:
        """
        Get the current status of the navigator
        status :

            uint8 PENDING         = 0   # The goal has yet to be processed by the action server
            uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
            uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                                        #   and has since completed its execution (Terminal State)
            uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
            uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                                        #    to some failure (Terminal State)
            uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                                        #    because the goal was unattainable or invalid (Terminal State)
            uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                                        #    and has not yet completed execution
            uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                                        #    but the action server has not yet confirmed that the goal is canceled
            uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                                        #    and was successfully cancelled (Terminal State)
            uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                                        #    sent over the wire by an action server
        :return: the goal status
        """
        status_array = self._move_base_status.get_data()
        return status_array.status_list[-1]

    def get_current_pose(self) -> Pose:
        """
        Get the current pose

        :return:
        """
        data: PoseWithCovarianceStamped = self._amcl_pose.get_data()
        return data.pose.pose

    def get_current_goal_pose(self) -> Pose:
        """
        Get the current goal pose

        :return:
        """
        data: PoseStamped = self._move_base_goal.get_data()
        return data.pose
