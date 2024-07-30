import rospy
from loguru import logger
from LemonEngine.utils.checker import Checker
from LemonEngine.hardwares.hardware import BaseHardware
from open_manipulator_msgs.srv import SetKinematicsPoseRequest, \
    SetKinematicsPose, \
    SetJointPositionRequest, \
    SetJointPosition

from geometry_msgs.msg import Point
from typing import *


class Manipulator(BaseHardware):
    """
    API wrapper for moveit based manipulator
    """
    def __init__(self):
        """
        Constructor
        """
        super().__init__()

        # Check and wait for the tool control service to be available
        Checker().check_service_status("/goal_tool_control")
        rospy.wait_for_service('/goal_tool_control')
        self.tool_control_service = rospy.ServiceProxy('/goal_tool_control', SetKinematicsPose)

        # Check and wait for the task space path position service to be available
        Checker().check_service_status("/goal_task_space_path_position_only")
        rospy.wait_for_service('/goal_task_space_path_position_only')
        self.kinematics_service = rospy.ServiceProxy("/goal_task_space_path_position_only", SetKinematicsPose)

        # Log that the manipulator is ready for use
        logger.success("Manipulator is ready!")

    @logger.catch
    def move_to(self, point: Tuple[float, float, float] | Point, t: float = 1.0):
        """
        Move the manipulator to a specified point in space.

        :param point: The target position to move to.
        :param t: The time duration for the motion. Default is 1.0 second.
        :return:
        """
        request = SetKinematicsPoseRequest()
        request.end_effector_name = "gripper"

        # Set the position based on the type of 'point'
        if isinstance(point, Point):
            request.kinematics_pose.pose.position = point
        else:
            x, y, z = point
            request.kinematics_pose.pose.position.x = x
            request.kinematics_pose.pose.position.y = y
            request.kinematics_pose.pose.position.z = z
        request.path_time = t

        # Call the service
        self.kinematics_service(request)

    @logger.catch
    def set_gripper(self, angle: float, t: float = 1.0):
        """
        Set the gripper to a specified angle.

        :param angle: The angle to set the gripper.
        :param t: The time duration for the motion. Default is 1.0 second.
        :return:
        """
        request = SetJointPositionRequest()
        request.joint_position.joint_name = ["gripper"]
        request.joint_position.position = [angle]
        request.path_time = t
        self.tool_control_service(request)

    def open_gripper(self, t: float = 1.0):
        """
        Open the gripper.

        :param t: The time duration for the motion. Default is 1.0 second.
        :return:
        """
        self.set_gripper(0.1, t)