from abc import ABC, abstractmethod


class Input(ABC):
    @classmethod
    @abstractmethod
    def is_available(cls) -> bool: ...
