from LemonEngine.utils.checker import Checker


class BaseHardware:
    """
    Base class for all hardware.
    """
    def __init__(self):
        """
        Constructor.
        """
        Checker().check_ros_status()
