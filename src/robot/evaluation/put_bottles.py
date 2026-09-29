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

        (
            self._bin_bottom_height,
            self._bin_top_height,
            self._bin_bottom_radius,
            self._bin_top_radius,
        ) = self._measure_bin_geometry()

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

    def _bin_collision_vertices(self) -> np.ndarray:
        vertices = []

        for geom_id in range(self.model.ngeom):
            belongs_to_bin = (
                self.model.geom_bodyid[geom_id] == self._bin_body_id
            )
            is_collision_geometry = self.model.geom_group[geom_id] == 3 #group 3 is collision geometry. in our XML File 

            if not belongs_to_bin or not is_collision_geometry:
                continue

            mesh_id = self.model.geom_dataid[geom_id]

            if mesh_id < 0:
                continue

            first_vertex = self.model.mesh_vertadr[mesh_id] #where vertices begin
            num_vertices = self.model.mesh_vertnum[mesh_id] #and amount of vertices it has

            mesh_vertices = self.model.mesh_vert[
                first_vertex : first_vertex + num_vertices
            ]

            rotation = np.empty(9)
            mujoco.mju_quat2Mat(
                rotation,
                self.model.geom_quat[geom_id], #orientation stored as quaternion
            )
            rotation = rotation.reshape(3,3)

            vertices_in_bin_frame = ( 
                mesh_vertices @ rotation.T + self.model.geom_pos[geom_id]
            )
            vertices.append(vertices_in_bin_frame)

        if not vertices:
            raise ValueError("no collision mesh found for the bin")

        return np.concatenate(vertices)

    def _measure_bin_geometry(
            self,
    ) -> tuple[float, float, float, float]:
        vertices = self._bin_collision_vertices()

        heights = vertices[:,1] #y of every vertex [x,y,z]
        radius = np.hypot(
            vertices[:,0],
            vertices[:,2],
        )

        bottom_height = float(heights.min())
        top_height = float(heights.max())

        if top_height <= bottom_height:
            raise ValueError("Bin collision mesh has no height")

        height_span = top_height - bottom_height
        band_size = height_span * 0.25

        bottom_vertices = heights <= bottom_height + band_size
        top_vertices = heights >= top_height - band_size

        bottom_radius = float(np.median(radius[bottom_vertices]))
        top_radius = float(np.median(radius[top_vertices])) 

        return (
            bottom_height,
            top_height,
            bottom_radius,
            top_radius,
        )

    def _allowed_radius(self, heights: np.ndarray) -> np.ndarray:
        height_fraction = (
            (heights - self._bin_bottom_height) / (self._bin_top_height - self._bin_bottom_height)
        )
        height_fraction = np.clip(height_fraction, 0.0, 1.0)

        return (
            self._bin_bottom_radius + height_fraction * (self._bin_top_radius - self._bin_bottom_radius)
        )

    def evaluate(self, data: mujoco.MjData) -> BottleEvaluation:
        positions = self._bottle_positions_in_bin_frame(data)

        heights = positions[:,1]
        radial_distances = np.hypot(
            positions[:,0],
            positions[:,2],
        )
        allowed_radius = self._allowed_radius(heights)

        inside = (
            (heights >= self._bin_bottom_height)
            & (heights <= self._bin_top_height)
            & (radial_distances <= allowed_radius)
        )

        return BottleEvaluation(
            inside=tuple(bool(value) for value in inside)
        )
    
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
            bottle_positions_world - bin_position_world #distance of bottle position vectors from the bin origin
        )

        return positions_relative_to_bin @ rotation_world_from_bin 