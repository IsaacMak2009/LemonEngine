import os
import genpy
import rospy
import rosservice
import rostopic
from loguru import logger
from typing import *


class Checker:
    def __init__(self):
        pass

    @staticmethod
    def _throw(text: str, required: bool = False):
        logger.warning(text)
        if required:
            raise RuntimeError(text)

    def check_ros_status(self,
                         verbose: bool = False,
                         required: bool = True) -> Optional[bool]:
        """
        Check if ROS is available
        :param required: if required, it will raise a RuntimeError
        :param verbose: verbose mode
        :return: a boolean value indicating if ROS is available
        """
        if not rospy.is_shutdown() and rospy.core.is_initialized():
            if verbose: logger.success("ROS is available")
            return True

        Checker._throw("ROS is not initialized or shutdown", required)
        return True

    def check_topic_status(self,
                           topic_name: str,
                           topic_type: Type[genpy.Message] = None,
                           verbose: bool = False,
                           required: bool = False) -> Optional[bool]:
        """
        Check if a topic is available.
        :param topic_name: topic name to check
        :param topic_type: topic type to check (when given a type)
        :param required: if required, it will raise a RuntimeError
        :param verbose: verbose mode
        :return: a boolean value indicating if the topic is available
        """
        try:
            pubs_topics, subs_topics = rostopic.get_topic_list()
        except ConnectionRefusedError:  # cannot communicate to master
            Checker._throw("ROS is not initialized or shutdown", required)
            return False

        for topic in [*pubs_topics, *subs_topics]:
            if topic_name == topic[0] and (topic_type is None or topic_type == topic[1]):
                if verbose: logger.success(f"Topic {topic_name} matched with type {topic[1]}, from {topic[2]}")
                return True

        Checker._throw(f"Topic {topic_name} not found", required)
        return False

    def check_service_status(self,
                             service_name: str,
                             service_type: Type[genpy.Message] = None,
                             verbose: bool = False,
                             required: bool = False) -> Optional[bool]:
        """
        Check if a service is available.
        :param service_name: service name to check
        :param service_type: service type to check (when given a type)
        :param required: if required, it will raise a RuntimeError
        :param verbose: verbose mode
        :return: a boolean value indicating if the service is available
        """
        try:
            service_list = rosservice.get_service_list()
        except rosservice.ROSServiceIOException:  # cannot communicate to master
            Checker._throw("ROS is not initialized or shutdown", required)
            return False

        for service in service_list:
            if service_name == service[0] and (service_type is None or service_type == service[1]):
                if verbose: logger.success(f"Service {service_name} matched with type {service[1]}, from {service[1]}")
                return True

        Checker._throw(f"Service '{service_name}' not found", required)
        return False


if __name__ == '__main__':
    # example usage
    rospy.init_node('checker', anonymous=True)
    checker = Checker()  # create a checker
    checker.check_ros_status(required=False)  # check ros status
    checker.check_topic_status("topic_name", required=False)  # check topic
    checker.check_service_status("service_name", required=False)  # check service
