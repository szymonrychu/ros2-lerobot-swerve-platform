# D435 switchover: user decisions (2026-10-27)

Original request (verbatim): "start implementation for the switchover, assume similar position as current head unit. restore ground estimation and other 3d view functionality, replace existing gripper position estimation and MCP tools. it's a bit epic including doing necessary recalibration, so it might also touch the calibration screen."

Context: "It will permanently replace gripper camera (eventually), but will be mounted on separate head."

Answers:
1. Mount: D435 head is on the pan/tilt servos, like the current head. Extrinsics solved per head pose through FK.
2. Old head sensors: both removed. D435 replaces head ToF (arducam_tof) and head RGB (Camera Module 3, head_camera_cpp); retire them and ToF-only code paths; marker_detector moves to D435 colour+depth.
3. "Gripper position estimation" = the camera-based solve of arm joint offsets / gripper camera mount / tool point (done 2026-10-10 with the gripper camera), re-done from the head D435 watching a gripper-held ChArUco board. Absorbs todo epic arm-joint-offsets-from-the-g.
4. Gripper UVC camera: retire now, in this epic (gripper_uvc_camera node, MCP 'gripper' camera, web_ui Arm-tab view, master2master steamdeck relay).

Probed facts:
- Client Pi: D435 8086:0b07, device serial 935422070045 (935422070045 is the ASIC/firmware-update serial), USB3 5000M, uvcvideo, /dev/video8-11.
- Installed: ros-jazzy-realsense2-camera 4.58.4, ros-jazzy-librealsense2 2.58.4 (apt). Pi 5, 7.8 GiB RAM, kernel 6.8.0-1065-raspi.
- Running on client 2026-10-27: ros2-arducam_tof, ros2-gripper_uvc_camera, ros2-head_camera_cpp, ros2-head_rgbd_calibration, ros2-tof_obstacle_cloud.

# 2026-10-28 decisions (CSI epic)
User request: "keep CSI cameras, label them front and back and let us use camera module 3 on both CSI1 and CSI2".
1. Mount: fixed on body (front forward, back backward), static TF; not on the head.
2. Roles: MCP agent images, web_ui live streams, marker detection. No zoom.
3. Naming: head (D435) / front / back everywhere.
4. Sequencing: finish the D435 epic unchanged first; the CSI epic restores head_camera_cpp from git history.
Probed 2026-10-28: cam0 imx708 (head CM3), cam1 arducam-pivariety (ToF still attached); config.txt dtoverlay=imx708,cam0 and arducam-pivariety,cam1, camera_auto_detect=0.

# 2026-10-28 later
- Push unblocked (GitHub setting changed); main pushed d0bab034..0c61914b; drivable-ground deployed to client (11 nodes, failed=0).
- Mid-epic merge of the D435 epic to main approved (deploys come from main).
- PSU 3000 mA accepted for the D435 (watch for USB resets/undervoltage).
- ChArUco board ready; board-to-jaw-tip offset to come before arm-offsets-compute-apply.

- 2026-10-28: D435 head physically mounted; calibration board = same 3D-printed 6x5 ChArUco, 4 mm thick, placed in front of the robot.

- 2026-10-28: user: D435 stream stable; enable slope/obstacle (ground) estimation from D435 and include it in map + path planning now. Gripper camera set enabled:false early to make memory room.

- 2026-10-29: user: ship all code first; user calibrates via the web UI (agent does not run the live calibration).
