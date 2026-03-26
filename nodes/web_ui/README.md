# web_ui — ROS2 Web Dashboard

FastAPI bridge + React frontend for ROS2 web dashboard.

## Overview

Provides a web-based interface to interact with ROS2 topics, services, and actions.

## Building

```bash
cd nodes/web_ui
poetry install
```

## Testing

```bash
poetry run pytest tests/ -v
```

## Linting

```bash
poetry run poe lint
poetry run poe lint-fix
```
