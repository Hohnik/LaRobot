from abc import ABC, abstractmethod
from typing import Self


class Input(ABC):
    """Abstract base class for input devices

    To implement:
        is_available: class method to check if the input device is available
        __enter__: context manager entry method
        __exit__: context manager exit method
    """

    @classmethod
    @abstractmethod
    def is_available(cls) -> bool: ...

    @abstractmethod
    def __enter__(self) -> Self: ...

    @abstractmethod
    def __exit__(
        self, *exc: object
    ) -> None: ...  # *exc is for conventions in threading
