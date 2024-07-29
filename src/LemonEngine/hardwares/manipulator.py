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
    def __init__(self):
        super().__init__()
        Checker().check_service_status("/goal_tool_control")
        rospy.wait_for_service('/goal_tool_control')
        self.tool_control_service = rospy.ServiceProxy('/goal_tool_control', SetKinematicsPose)

        Checker().check_service_status("/goal_task_space_path_position_only")
        rospy.wait_for_service('/goal_task_space_path_position_only')
        self.kinematics_service = rospy.ServiceProxy("/goal_task_space_path_position_only", SetKinematicsPose)

        logger.success("Manipulator is ready!")

    @logger.catch
    def move_to(self, point: Tuple[float, float, float] | Point, t: float = 1.0):
        request = SetKinematicsPoseRequest()
        request.end_effector_name = "gripper"
        if isinstance(point, Point):
            request.kinematics_pose.pose.position = point
        else:
            x, y, z = point
            request.kinematics_pose.pose.position.x = x
            request.kinematics_pose.pose.position.y = y
            request.kinematics_pose.pose.position.z = z
        request.path_time = t
        self.kinematics_service(request)

    @logger.catch
    def set_gripper(self, angle: float, t: float = 1.0):
        request = SetJointPositionRequest()
        request.joint_position.joint_name = ["gripper"]
        request.joint_position.position = [angle]
        request.path_time = t
        self.tool_control_service(request)

    def open_gripper(self, t: float = 1.0):
        self.set_gripper(0.1, t)