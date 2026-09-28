from dataclasses import dataclass
import mujoco
import numpy as np 

BOTTLE_NAMES = (
    "bottle_1",
    "bottle_2",
    "bottle_3",
    "bottle_4",
    "bottle_5",
    "bottle_6",
)
@dataclass(frozen=True)
class BottleEvaluation:
    inside: tuple[bool,...]

    def __post_init__(self) -> None:
        if len(self.inside) != len(BOTTLE_NAMES):
            raise ValueError(
                f"Expected {len(BOTTLE_NAMES)} bottle results, "
                f"got {len(self.inside)}"
            )

    @property
    def num_inside(self) -> int:
        return sum(self.inside)

    @property
    def progress(self) -> float:
        return self.num_inside / len(BOTTLE_NAMES)

    @property
    def inside_names(self) -> tuple[str, ...]:
        return tuple(
            name
            for name, is_inside in zip(BOTTLE_NAMES, self.inside)
            if is_inside
        )

    @property
    def success(self) -> bool:
        return self.num_inside ==len(BOTTLE_NAMES)

class PlacementEvaluator:
    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model

        ids = []
        for name in BOTTLE_NAMES:
            body_id = self._find_body_id(name)
            ids.append(body_id)
        self._bottle_body_ids = tuple(ids) #contains the 6 bottle ids

        self._bin_body_id = self._find_body_id("bin_container") #delivers id of container

    def _find_body_id(self, body_name: str) -> int:
        body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            body_name,
        )

        if body_id < 0: #if not found we raise a valueerror (mujoco returns -1)
            raise ValueError(
                f"Body {body_name!r} was not found in the MuJoCo model"
            )

        return body_id 

    def _bottle_positions_in_bin_frame( #bottles relative to the bin
            self,
            data:mujoco.MjData,
    ) -> np.ndarray:
        bottle_positions = []

        for body_id in self._bottle_body_ids:
            position = data.subtree_com[body_id] #center of mass [x,y,z] for every body in world coordinates 
            bottle_positions.append(position)

        bottle_positions_world = np.asarray(
            bottle_positions,
            dtype=float,
        )

        bin_position_world = data.xpos[self._bin_body_id] #position of bin's origin frame also [x,y,z]
        rotation_world_from_bin = data.xmat[
            self._bin_body_id
        ].reshape(3,3)

        positions_relative_to_bin = (
            bottle_positions_world - bin_position_world #distance of bottle from the bin origin
        )

        return positions_relative_to_bin @ rotation_world_from_bin 