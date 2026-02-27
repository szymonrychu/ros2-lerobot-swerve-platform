# ROS2 master node

**Purpose:** Runs the ROS2 daemon (DDS master). Used by both Server and Client; same image, different compose image names (server-ros2-master, client-ros2-master).

**Location:** All node source lives under [nodes/](.) (no node code under `client/` or `server/`). See [README.md](../../README.md).

## Build

Server: `cd server && docker compose build ros2-master` (build context `../nodes/ros2_master`).  
Client: `cd client && docker compose build ros2-master` (build context `../nodes/ros2_master`).
