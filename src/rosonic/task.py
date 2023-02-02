from abc import ABC, abstractmethod
from threading import Thread
from typing import (
    Callable,
    Generic,
    Tuple,
    TypeVar,
)


__all__ = [
    'Task',
]


R = TypeVar('R')
class Task(Generic[R], ABC):

    @abstractmethod
    def __enter__(self) -> Callable[[], R]:
        ...

    @abstractmethod
    def __exit__(self, *exc) -> bool:
        ...

    def start(self) -> R:
        with self as task:
            return task()

    def start_in_thread(self) -> Tuple['Task', Thread]:
        thread = Thread(target=self.start)
        try: return self, thread
        finally: thread.start()

