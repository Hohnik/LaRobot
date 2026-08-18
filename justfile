# Use `just <recipy>` to execute a task

@_default:
  -just --list --unsorted

test:
  .venv/bin/python -m pytest -q

# Generate mujoco typehints
stubs:
    uv run pybind11-stubgen mujoco -o typings
