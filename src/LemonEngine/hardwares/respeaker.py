import genpy
import rospy, os
from typing import *
from LemonEngine.utils.checker import Checker
from LemonEngine.hardwares.hardware import BaseHardware
from LemonEngine.sensors.sensor import Sensor
from mr_voice.msg import Voice
from std_msgs.msg import String
from loguru import logger


class Respeaker(BaseHardware):
    """
    API wrapper for https://github.com/supercatex/mr_voice
    """
    def __init__(self, enable_espeak_fix: bool = False):
        """
        Constructor
        """
        super().__init__()
        if not enable_espeak_fix:
            Checker().check_topic_status("/speaker/say", required=True)
        self.enable_espeak_fix = enable_espeak_fix
        self.publisher = rospy.Publisher('/speaker/say', String, queue_size=5)
        self.mic = Sensor("/voice/text", Voice, callback_message=self.custom_callback)
        self.get_voice_text, _ = self.mic.data_field("data")

    @staticmethod
    def custom_callback(msg: Union[Voice, genpy.Message]) -> None:
        """
        custom callback function to log the message and direction
        :param msg: the message from the `/voice/text`
        :return:
        """
        logger.info(f"\"{msg.text}\" from direction {msg.direction}° ")

    def say(self, text: str) -> None:
        """
        Function to convert text to speech and play it.
        :param text: the message to be speech
        :return:
        """
        logger.info(f"speak: \"{text}\"")
        if not self.enable_espeak_fix:
            self.publisher.publish(String(data=text))
            return

        os.system(f'espeak -s 120 "{text}"')
