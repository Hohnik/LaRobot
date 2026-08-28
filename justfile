# Use `just <recipy>` to execute a task

@_default:
  -just --list --unsorted

# Generate mujoco typehints
stubs:
  uv run pybind11-stubgen mujoco -o typings

sim:
  uv run scripts/start_sim.py
