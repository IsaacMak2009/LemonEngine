import genpy
import rospy
from LemonEngine.utils.checker import Checker
from LemonEngine.hardwares.hardware import BaseHardware
from LemonEngine.sensors.sensor import Sensor
from mr_voice.msg import Voice
from std_msgs.msg import String
from loguru import logger


class Respeaker(BaseHardware):
    def __init__(self):
        super().__init__()
        Checker().check_topic_status("/speaker/say", required=True)
        self.publisher = rospy.Publisher('/speaker/say', String, queue_size=5)
        self.mic = Sensor("/voice/text", Voice, callback_message=self.custom_callback)
        self.get_voice_text, _ = self.mic.data_field("data")

    @staticmethod
    def custom_callback(msg: Voice | genpy.Message):
        logger.info(f"\"{msg.text}\" from direction {msg.direction}° ")

    def say(self, text: str):
        logger.info(f"speak: \"{text}\"")
        self.publisher.publish(String(data=text))