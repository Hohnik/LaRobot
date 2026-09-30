# Use `just <recipy>` to execute a task

@_default:
    -just --list --unsorted

# run formatter and linter
alias lint := check
alias format := check
check:
    @ruff format
    @ruff check --fix

test:
    uv run pytest

# setup project for development
setup-project:
    uv run pybind11-stubgen mujoco -o typings

# start sim with input device: `spacemouse`|`keyboard` and optional args
start-sim device *args:
    uv run scripts/start_sim.py --device {{ device }} {{ args }}

# start physical control with configured CAN interfaces
start-phys device *args:
    uv run scripts/start_phys.py --device {{ device }} {{ args }}
