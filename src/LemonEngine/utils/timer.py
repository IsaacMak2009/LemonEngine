import time
import rospy


class Timer:
    """
    Example usage:
        with Timer() as timer:
            print(timer.elapsed_time)
            super_slow_function()
            print(timer.elapsed_time)
            super_slow_function()

        # when exited, the timer will show the elapsed time
    """

    def __init__(self, name: str = None):
        self.name = name
        self.start_time = None
        self.end_time = None

    def __enter__(self):
        self.start_time = time.time()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.end_time = time.time()
        self._elapsed_time = round((self.end_time - self.start_time) * 1000, 2)
        if self.name:
            print(f"[{self.name}] Timer exit, the task took {self.elapsed_time} milliseconds")
        else:
            print(f"Timer exit, the task took {self.elapsed_time} milliseconds")
        return False  # Propagate any exceptions

    def reset(self):
        t = self.elapsed_time
        self.start_time = time.time()
        return t

    @property
    def elapsed_time(self):
        return time.time() - self.start_time


class RosTimer:
    """
    Example usage:
        with RosTimer() as timer:
            print(timer.elapsed_time)
            super_slow_function()
            print(timer.elapsed_time)
            super_slow_function()

        # when exited, the timer will show the elapsed time
    """

    def __init__(self, name: str = None):
        self.name = name
        self.start_time = None

    def __enter__(self):
        self.start_time = rospy.get_time()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.name:
            print(f"[{self.name}]", end=' ')
        print(f"Timer exit, the task took {self.elapsed_time} ros time")
        return False  # Propagate any exceptions

    def reset(self):
        t = self.elapsed_time
        self.start_time = time.get_time()
        return t

    @property
    def elapsed_time(self):
        return rospy.get_time() - self.start_time

