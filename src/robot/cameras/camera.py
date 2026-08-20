import logging
from typing import Self
from abc import ABC, abstractmethod

from robot.cameras.frame import Frame

logger = logging.getLogger(__name__)


class Camera(ABC):
    """Abstract interface for a camera that produces frames.

    Example: 
    """

    name: str

    @classmethod
    @abstractmethod
    def is_available(cls) -> bool:
        """True if a camera of this type is present"""

    @abstractmethod
    def connect(self) -> None:
        """"Connect to and initialize the camera."""

    @abstractmethod
    def read(self) -> Frame:
        """One Frame from the current state"""

    @abstractmethod
    def close(self) -> None:
        """"Release the device"""

    def __enter__(self) -> Self:
        try:
            self.connect()
        except BaseException:
            self.close()
            raise    
        logger.info("%s connected", self.name)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
        logger.info("%s disconnected", self.name)