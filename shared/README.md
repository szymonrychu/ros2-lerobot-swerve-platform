# Shared libraries

Python packages shared by multiple nodes. Add packages here (e.g. `shared/ros2_common/`) and install them in node containers via Dockerfile or mount. No node logic lives under `client/` or `server/`; node code is in [nodes/](../nodes/).

See [AGENTS.md](../AGENTS.md): use Python 3 type hints and extend unit tests as the project grows.
