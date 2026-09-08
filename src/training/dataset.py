from pathlib import Path
import struct
import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader
from pprint import pprint as pp
from mcap.reader import make_reader
from lerobot.datasets.lerobot_dataset import LeRobotDataset
import os

LEROBOT_TRAINING_DATA_PATH = os.getenv("LEROBOT_TRAINING_DATA_PATH", "src/training/data/yam_teleop")
MCAP_RECORDINGS_DATA_PATH = os.getenv("MCAP_RECORDINGS_DATA_PATH", "recordings/episodes")
root = Path(LEROBOT_TRAINING_DATA_PATH) # Output path for where the converted training path will be saved
mcap_paths = sorted(Path(MCAP_RECORDINGS_DATA_PATH).glob("*.mcap")) # Input path where mcaps are

target_size = (480, 640)
fps = 30
robot_type = "yam_bimanual"
features = {
    "observation.state": {
        "dtype": "float32",
        "shape": (14,),
        "names": None,
    },
    "action": {
        "dtype": "float32",
        "shape": (14,),
        "names": None,
    },
    "observation.images.top_camera": {
        "dtype": "video",
        "shape": (480, 640, 3),
        "names": ["height", "width", "channels"],
    },
    # "observation.images.left_camera": {
    #     "dtype": "video",
    #     "shape": (480, 640, 3),
    #     "names": ["height", "width", "channels"],
    # },
    # "observation.images.right_camera": {
    #     "dtype": "video",
    #     "shape": (480, 640, 3),
    #     "names": ["height", "width", "channels"],
    # },
}

def read_mcap_episode(mcap_path: Path, target_size=(480, 640)):
    """
    MCAP binary decoding and conversion to training format without ROS dependencies
    """

    frames = []

    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        current_frame = {}

        for schema, channel, msg in reader.iter_messages():
            topic = channel.topic

            if topic == "/top-camera":
                fmt_len = struct.unpack('<I', msg.data[4:8])[0]
                offset = (8 + fmt_len + 3) & ~3
                data_len = struct.unpack('<I', msg.data[offset:offset+4])[0]
                img_bytes = msg.data[offset+4 : offset+4+data_len]

                img_bgr = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)
                img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

                if target_size:
                    img_rgb = cv2.resize(img_rgb, (target_size[1], target_size[0])) # OpenCV uses (width, height)

                current_frame["top_camera"] = img_rgb

            elif topic in [
                "/left-arm-state", "/left-ee-state", "/right-arm-state", "/right-ee-state",
                "/left-arm-action", "/left-ee-action", "/right-arm-action", "/right-ee-action",
            ]:
                seq_len = struct.unpack('<I', msg.data[4:8])[0]
                vals = struct.unpack(f'<{seq_len}d', msg.data[12 : 12+seq_len*8])
                current_frame[topic] = np.array(vals, dtype=np.float32)

            required_topics = [
                "top_camera",
                "/left-arm-state", "/left-ee-state", "/right-arm-state", "/right-ee-state",
                "/left-arm-action", "/left-ee-action", "/right-arm-action", "/right-ee-action",
            ]

            if all(k in current_frame for k in required_topics):
                state_vec = np.concatenate([
                    current_frame["/left-arm-state"],
                    current_frame["/left-ee-state"],
                    current_frame["/right-arm-state"],
                    current_frame["/right-ee-state"],
                ])

                action_vec = np.concatenate([
                    current_frame["/left-arm-action"],
                    current_frame["/left-ee-action"],
                    current_frame["/right-arm-action"],
                    current_frame["/right-ee-action"],
                ])

                frames.append({
                    "observation.images.top_camera": current_frame["top_camera"],
                    "observation.state": state_vec,
                    "action": action_vec,
                    "task": "Bimanual manipulation teleop",
                })

                current_frame = {}

    return frames

def create_dataset():
    dataset = LeRobotDataset.create(
        repo_id="local/yam_teleop",
        fps=fps,
        features=features,
        root=root,
        robot_type=robot_type,
        use_videos=True,
    )
    return dataset

def convert_dataset(dataset:LeRobotDataset):
    print(f"Converting {[mcap_path.name for mcap_path in mcap_paths]} to LeRobot...")
    for mcap_path in mcap_paths:
        episode_frames = read_mcap_episode(mcap_path, target_size=target_size)

        if not episode_frames:
            print(f"Skipping {mcap_path.name}, camera stream missing")
            continue

        for frame in episode_frames:
            dataset.add_frame(frame)

        dataset.save_episode()
        print(f"Saved {mcap_path.name}, {len(episode_frames)} frames")

    dataset.finalize()
    print(f"Dataset Finalized with a total of {dataset.meta.total_episodes} episodes and {dataset.meta.total_frames} frames.")

    return dataset

def load_dataset(repo_id, root):
    dataset = LeRobotDataset(repo_id="local/yam_teleop", root=root)
    sample = dataset[0]
    print(f"Sample 0 task: {sample.get("task")}")
    return dataset

if __name__ == "__main__":

    dataset = create_dataset()
    dataset = convert_dataset(dataset)