import rospy
import genpy
from loguru import logger
from typing import *
from LemonEngine.utils.checker import Checker

T = TypeVar('T')
R = TypeVar('R')


class BaseSensor:
    def __init__(self,
                 topic_name: str,
                 message_type: Type[genpy.Message],
                 timeout: Optional[Union[rospy.Duration, float]] = None) -> None:
        """
        A base class for sensor objects. Available events:
         - `ready`: when the sensor has received the first message
         - `message`: when a message is received
         - `start`: when the sensor is started
         - `stop`: when the sensor is stopped

        :param topic_name: the ROS topic name of the sensor
        :param message_type: the message type from the ROS topic
        :param timeout: the timeout for waiting for the first message (None for no waiting)
        """

        Checker().check_ros_status()
        Checker().check_topic_status(topic_name)

        self.topic_name = topic_name
        self.message_type = message_type
        self.data: Dict[str, Any] = {}
        self.subscriber = None

        self.start()

        try:
            # set the on stop callback
            rospy.on_shutdown(self.on_stop)

            # wait for the first message
            message = rospy.wait_for_message(self.topic_name, self.message_type, timeout)

            logger.info(f"Received first message from {topic_name}")
            logger.success(f"Sensor at {topic_name} is ready!!")

            # trigger the on_ready event
            self.on_ready(message)

        except rospy.ROSException as e:
            logger.error(f"Failed to receive first message from {topic_name}: {e}")

    def on_start(self) -> None:
        """
        The callback function called when the sensor is started.

        :return:
        """
        logger.info(f"Sensor at {self.topic_name} started.")

    def on_stop(self) -> None:
        """
        The callback function called when the sensor is stopped manually.

        :return:
        """
        logger.info(f"Sensor at {self.topic_name} stopped.")

    def on_message(self, message) -> None:
        """
        The callback function called when a message is received.

        :param message: the original message from the ROS topic
        :return:
        """
        pass

    def on_ready(self, message) -> None:
        """
        The callback function called when everything is ready and the first message is received.

        :param message: the first message received from the ROS topic
        :return:
        """
        pass

    @logger.catch
    def _callback(self, message: genpy.Message) -> None:
        """
        Callback function wrapper

        :param message: the received message
        """
        self.on_message(message)

    @logger.catch
    def stop(self) -> None:
        """
        Stops the sensor by unregistering the subscriber
        """
        if self.subscriber:
            logger.info(f"Stopping sensor {self.topic_name}")

            # unregister the subscriber
            self.subscriber.unregister()

            # trigger on stop event
            self.on_stop()

            self.subscriber = None
        else:
            logger.warning(f"Sensor at {self.topic_name} already stopped!")

    @logger.catch
    def start(self) -> None:
        """
        Starts the sensor by registering the subscriber again
        """
        if not self.subscriber:

            logger.info(f"Starting sensor {self.topic_name}")

            # create the subscriber
            self.subscriber = rospy.Subscriber(
                self.topic_name, self.message_type,
                self._callback, queue_size=5)

            # trigger on start event
            self.on_start()

        else:
            logger.warning(f"Sensor at {self.topic_name} already running!")

    def data_field(self,
                   data_id: str,
                   data_type: Type[T] = type(None)) -> Tuple[Callable[[], T], Callable[[T], None]]:
        """
        A helper function that returns the getter and setter for the data field

        :param data_id: Identifier for the data field
        :param data_type: the type of the data
        :return: the getter and setter for the data field
        """

        # the getter function for the data field
        def getter() -> Union[data_type, None]:
            return self.data.get(data_id, None)

        # the setter function for the data field
        def setter(data: data_type) -> None:
            self.data[data_id] = data

        return getter, setter


class Sensor(BaseSensor, Generic[R]):
    def __init__(self,
                 topic_name: str,
                 message_type: Type[R],
                 callback_message: Optional[Callable[[genpy.Message], None]] = None,
                 callback_on_ready: Optional[Callable[[genpy.Message], None]] = None,
                 callback_on_start: Optional[Callable[[], None]] = None,
                 callback_on_stop: Optional[Callable[[], None]] = None,
                 *args, **kwargs) -> None:
        """
        a simple sensor

        :param topic_name: ROS topic name of the sensor
        :param message_type: Message type from the ROS topic
        :param callback_message: The custom callback function when received a message
        :param callback_on_ready: The custom callback function when received the first message (only call once)
        :param callback_on_start: The custom callback function when the sensor is started
        :param callback_on_stop: The custom callback function when the sensor is stopped manually
        """
        self.callback_message = callback_message
        self.callback_on_ready = callback_on_ready
        self.callback_on_start = callback_on_start
        self.callback_on_stop = callback_on_stop

        _getter, _setter = self.data_field("data", message_type)
        self.get_data: Callable[[], R] = _getter
        self._set_data: Callable[[R], None] = _setter

        super().__init__(topic_name=topic_name, message_type=message_type, *args, **kwargs)

    def on_ready(self, message) -> None:
        if self.callback_on_ready:
            self.callback_on_ready(message)

    def on_start(self) -> None:
        if self.callback_on_start:
            self.callback_on_start()

    def on_stop(self) -> None:
        if self.callback_on_stop:
            self.callback_on_stop()

    def on_message(self, message: genpy.Message) -> None:
        """
        Handles incoming messages

        :param message: The original message from the ROS topic.
        """
        if self.callback_message:
            self.callback_message(message)
        self._set_data(message)
