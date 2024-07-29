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
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, PoseStamped
from geometry_msgs.msg import Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class Chassis(BaseHardware):
    def __init__(self, cmd_vel_topic: str = "/cmd_vel", odometry_topic: str = "/odom") -> None:
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
    def set_angular(self, x: float) -> None: ...
    @overload
    def set_angular(self, x: Vector3) -> None: ...
    @overload
    def set_angular(self, x: float, y: float, z: float) -> None: ...

    def set_angular(self, x: float | Vector3, y: float = None, z: float = None) -> None:
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
    def set_linear(self, x: float) -> None: ...
    @overload
    def set_linear(self, x: Vector3) -> None: ...
    @overload
    def set_linear(self, x: float, y: float, z: float) -> None: ...

    def set_linear(self, x: float | Vector3, y: float = None, z: float = None) -> None:
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
        twist = Twist()
        self._set_twist(twist)

    def get_current_position(self) -> Optional[np.ndarray]:
        if self.sensor is None:
            return None
        return self.sensor.get_position()

    def get_current_euler(self) -> Optional[np.ndarray]:
        if self.sensor is None:
            return None
        return self.sensor.get_orientation()

    def turn_for_degrees(self,
                         max_speed: float,
                         degrees: float,
                         direction: Literal["clockwise", "counterclockwise", "auto"] = "auto") -> None:
        raise NotImplementedError



class Navigator(BaseHardware):

    def __init__(self, frame_id: str = "map"):
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

    def lookup_transform(self):
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

    def transform_point(self, point: np.ndarray | list | Point) -> Optional[Point]:
        if self.trans is None:
            logger.error("Cannot find transform from base link, so cannot transform point")

        if isinstance(point, np.ndarray) or isinstance(point, list):
            point = Point(x=point[0], y=point[1], z=point[2])
        else:
            logger.warning("Cannot transform point from type {}".format(type(point)))
            return None

        _pos = tf2msg.PointStamped()
        _pos.header.frame_id = "base_link"
        _pos.header.stamp = rospy.Time(0)
        _pos.point = point
        return self.tf_buffer.transform(_pos, 'map').point

    @staticmethod
    def flat_point_to_pose(x: float, y: float, theta: float):  # we define a point (x, y, yaw)
        q = quaternion_from_euler(0.0, 0.0, theta)
        pose = Pose(Point(x, y, 0.0), Quaternion(q[0], q[1], q[2], q[3]))
        return pose

    @staticmethod
    def pose_to_flat_point(pose: Pose):
        x, y = pose.position.x, pose.position.y
        q = rnp.numpify(pose.orientation)
        roll, pitch, yaw = euler_from_quaternion(q)
        return x, y, yaw

    def move_to(self, x, y, theta):
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

    def cancel_goal(self):
        logger.debug("Cancel goal")
        self.move_base.cancel_goal()

    def clear_costmaps(self) -> None:
        logger.debug("Costmap clearing...")
        self._clear_costmaps()

    def get_goal_current_status(self) -> GoalStatus:
        status_array = self._move_base_status.get_data()
        return status_array.status_list[-1]

    def get_current_pose(self) -> Pose:
        data: PoseWithCovarianceStamped = self._amcl_pose.get_data()
        return data.pose.pose

    def get_current_goal_pose(self) -> Pose:
        data: PoseStamped = self._move_base_goal.get_data()
        return data.pose
