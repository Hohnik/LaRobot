from abc import ABC, abstractmethod


class Input(ABC):
    @classmethod
    @abstractmethod
    def available(cls) -> bool: ...
