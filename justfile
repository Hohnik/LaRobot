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
    pytest

# setup project for development
setup-project:
    uv run pybind11-stubgen mujoco -o typings

# start sim with input device: `spacemouse`|`keyboard` and optional args
start-sim device *args:
    uv run scripts/start_sim.py --device {{ device }} {{ args }}
