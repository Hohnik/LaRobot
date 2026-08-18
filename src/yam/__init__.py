"""The YAM arm library — everything reusable, importable as `yam.*`.

Layout (ROADMAP §10.2, shaped by the team's LaRobot repo per §10.6):

- `yam.robot` / `yam.can` — SafeRobot, build_robot, the CAN layer
- `yam.session` — ArmSession, one arm's state and mode machine
- `yam.teleop` · `yam.motion` · `yam.mirror` · `yam.recording` · `yam.settings` · `yam.collision` · `yam.incident` · `yam.provenance`
- `yam.inputs` — everything that produces commands: spacemouse, keyboard, axis_map
- `yam.fake` — the simulator that lags like the measured hardware
- `yam.ui` — screen rendering

⛔ REPO_ROOT is defined ONCE, here. Five modules used to each compute `Path(__file__).parent.parent`, and moving a file one level deeper silently re-anchored every config path it resolved — the fails-by-lying shape (FINDINGS §0) applied to the filesystem. One anchor, imported everywhere, cannot drift per-file.
"""

from pathlib import Path

#: The repository root: src/yam/__init__.py → yam → src → the repo.
REPO_ROOT = Path(__file__).resolve().parent.parent.parent
