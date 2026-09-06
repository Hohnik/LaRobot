from pathlib import Path
import json 
from robot.recording.sample import Sample
from mcap.writer import Writer
from io import BytesIO
import numpy as np



class Recorder:
    """Writes frames and states of every recorded tick into write_dir"""

    def __init__(self, write_dir: str | Path):
        self.write_dir= Path(write_dir)
        self.step = 0
        self._camera_channels: dict[str, int] = {}
        self._last_frame_timestamps: dict[str, float] = {}

    def __enter__(self):
        self.write_dir.mkdir(parents=True, exist_ok=True)
        self._file = (self.write_dir / "episode.mcap").open("xb") #"xb" creates a binary file and refuses to overwrite an existing recording

        try:
            self._writer = Writer(self._file) # handles MCAP formattting
            self._writer.start()

            self._state_action_channel = self._writer.register_channel(
                topic="/robot/state_action",
                message_encoding="json",
                schema_id=0,
            )

        except Exception: 
            self._file.close()
            raise 

        return self

    def record(self, sample: Sample) -> None:
        #both times use simulation time for now 
        # for hardware, source publish time and recorder recieve time can differ need to double check
        timestamp_ns = round(sample.timestamp_s * 1_000_000_000) #mcap expects nanoseconds so we convert

        payload = {
            "state": sample.state.tolist(), #json supports lists not numpy arrays
            "action": sample.action.tolist(),
        }
        data = json.dumps(payload, allow_nan=False).encode("utf8") #

        self._writer.add_message(
            channel_id=self._state_action_channel,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            sequence=self.step,
            data=data,
        )

        for frame in sample.frames:
            camera_name = frame.camera_name

            if (
                self._last_frame_timestamps.get(camera_name)
                == frame.timestamp_s
            ):
                continue
            if camera_name not in self._camera_channels:
                self._camera_channels[camera_name] = (
                    self._writer.register_channel(
                        topic=f"/cameras/{camera_name}",
                        message_encoding="npz", #numpy format for storing arrays
                        schema_id=0
                    )
                )
            arrays = {"rgb": frame.rgb}
            if frame.depth is not None:
                arrays["depth"] = frame.depth

            with BytesIO() as buffer: #temporarily holds bytes in memory
                np.savez(buffer, **arrays) #packages RGB array into said bytes 
                image_data = buffer.getvalue()

            capture_time_ns = round(frame.timestamp_s * 1_000_000_000)
            self._writer.add_message(
                channel_id = self._camera_channels[camera_name],
                log_time = timestamp_ns,
                publish_time = capture_time_ns,
                data = image_data,
            )
            self._last_frame_timestamps[camera_name] = frame.timestamp_s

        self.step += 1 

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        try:
            self._writer.finish()

        finally:
            self._file.close()

        return False

