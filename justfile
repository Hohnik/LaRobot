# Use `just <recipy>` to execute a task

@_default:
  -just --list --unsorted

# Run tests
test:
  .venv/bin/python -m pytest -q
