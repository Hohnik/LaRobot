import time
from io import BytesIO
from pathlib import Path
from typing import Self

import h5py
import numpy as np
from PIL import Image

from robot.cameras.frame import Frame
from robot.recording.sample import Sample

ARM_STATE_SIZE = 14
JPEG_QUALITY = 90


class Recorder:
    """Synchronously writes one episode to an HDF5 file."""

    def __init__(self, write_dir: str | Path):
        self.write_dir = Path(write_dir)
        self.file_path = self.write_dir / f"episode_{time.time_ns() // 1_000_000}.h5"
        self.step = 0
        self._file: h5py.File | None = None

    def __enter__(self) -> Self:
        self.write_dir.mkdir(parents=True, exist_ok=True)
        self._file = h5py.File(self.file_path, "x")
        self._file.create_dataset(
            "timestamps", shape=(0,), maxshape=(None,), dtype=np.uint64, chunks=True
        )
        self._file.create_dataset(
            "states",
            shape=(0, ARM_STATE_SIZE),
            maxshape=(None, ARM_STATE_SIZE),
            dtype=np.float32,
            chunks=True,
        )
        self._file.create_dataset(
            "actions",
            shape=(0, ARM_STATE_SIZE),
            maxshape=(None, ARM_STATE_SIZE),
            dtype=np.float32,
            chunks=True,
        )
        self._file.create_group("cameras")
        return self

    def record(self, sample: Sample) -> None:
        """Append one sample and any new camera frames to the episode."""
        if self._file is None:
            raise RuntimeError("Recorder must be opened before recording")

        self._validate(sample)

        for frame in sample.frames:
            self._record_frame(frame)
        self._append(self._file["timestamps"], sample.timestamp_ns)
        self._append(self._file["states"], sample.state)
        self._append(self._file["actions"], sample.action)
        self.step += 1

    def _record_frame(self, frame: Frame) -> None:
        assert self._file is not None
        cameras = self._file["cameras"]
        assert isinstance(cameras, h5py.Group)

        if frame.camera_name not in cameras:
            camera = cameras.create_group(frame.camera_name)
            rgb = camera.create_dataset(
                "frames",
                shape=(0,),
                maxshape=(None,),
                dtype=h5py.vlen_dtype(np.dtype(np.uint8)),
                chunks=True,
            )
            camera.create_dataset(
                "timestamps",
                shape=(0,),
                maxshape=(None,),
                dtype=np.uint64,
                chunks=True,
            )
            rgb.attrs["encoding"] = "jpeg"
            rgb.attrs["quality"] = JPEG_QUALITY
            rgb.attrs["color_space"] = "RGB"

        camera = cameras[frame.camera_name]
        assert isinstance(camera, h5py.Group)
        timestamps = camera["timestamps"]
        if len(timestamps) > 0 and frame.timestamp_ns_capture <= timestamps[-1]:
            if frame.timestamp_ns_capture == timestamps[-1]:
                return
            raise ValueError(f"{frame.camera_name}: frame timestamps must not decrease")

        output = BytesIO()
        Image.fromarray(frame.rgb).save(output, format="JPEG", quality=JPEG_QUALITY)
        self._append(camera["frames"], np.frombuffer(output.getvalue(), dtype=np.uint8))
        self._append(timestamps, frame.timestamp_ns_capture)

    @staticmethod
    def _append(dataset: h5py.Dataset, value: object) -> None:
        dataset.resize(len(dataset) + 1, axis=0)
        dataset[-1] = value

    @staticmethod
    def _validate(sample: Sample) -> None:
        Recorder._validate_vector("state", sample.state)
        Recorder._validate_vector("action", sample.action)
        if sample.timestamp_ns < 0:
            raise ValueError("timestamp_ns must be non-negative")
        for frame in sample.frames:
            Recorder._validate_frame(frame)

    @staticmethod
    def _validate_vector(name: str, values: np.ndarray) -> None:
        if values.shape != (ARM_STATE_SIZE,):
            raise ValueError(f"{name} must have shape ({ARM_STATE_SIZE},)")
        if values.dtype != np.float32:
            raise ValueError(f"{name} must have dtype float32")
        if not np.isfinite(values).all():
            raise ValueError(f"{name} must contain only finite values")

    @staticmethod
    def _validate_frame(frame: Frame) -> None:
        if not frame.camera_name or "/" in frame.camera_name:
            raise ValueError("camera_name must be non-empty and cannot contain '/'")
        if frame.timestamp_ns_capture < 0:
            raise ValueError("frame timestamp_ns_capture must be non-negative")
        if frame.rgb.ndim != 3 or frame.rgb.shape[2] != 3 or not frame.rgb.size:
            raise ValueError("frame RGB must have shape (height, width, 3)")
        if frame.rgb.dtype != np.uint8:
            raise ValueError("frame RGB must have dtype uint8")

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        if self._file is not None:
            self._file.close()
            self._file = None
        return False
