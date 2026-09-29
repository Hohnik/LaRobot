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

# start sim for teleoperation
start-sim *args:
    uv run scripts/start_sim.py {{ args }}
