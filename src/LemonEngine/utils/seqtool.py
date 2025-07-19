from abc import ABC, abstractmethod
from typing import *
from typing import Any

class BaseSeqTool:
    def __init__(self, *args: Any, **kwds: Any) -> None:
        pass
    
    def __rmatmul__(self, value):
        return self._update(value)

    @abstractmethod
    def _update(self, value) -> Any:
        raise NotImplemented

class Smoother(BaseSeqTool):
    def __init__(self, alpha=0.9, start_value=0):
        self.alpha = alpha
        self.value = start_value

    def _update(self, value) -> Any:
        self.value = (self.alpha) * self.value + (1-self.alpha) * value
        return self.value

class Streak(BaseSeqTool):
    def __init__(self, cond: Callable[[Any], bool]) -> None:
        self.cond = cond
        self.streak = 0

    def _update(self, value) -> Any:
        if self.cond(value):
            self.streak += 1
        else:
            self.streak = 0
            
        return self.streak
