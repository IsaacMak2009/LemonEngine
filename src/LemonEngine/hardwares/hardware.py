from LemonEngine.utils.checker import Checker


class BaseHardware:
    def __init__(self):
        Checker().check_ros_status()
