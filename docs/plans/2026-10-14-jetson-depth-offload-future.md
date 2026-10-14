# Jetson depth offload: future improvements

On 2026-10-14 the user cut the delivery to the minimum that gives a usable disparity map and depth computation on ROS:
- T15: the depthd pipeline.
- T19: the deploy role, without the TensorRT engine step.
- T20: the command-line calibration tool.
- T30: the deploy plus an end-to-end check.

Everything below is planned in `2026-10-14-jetson-depth-offload-plan.md`. It is not built yet, and it is kept here as
future work.

| Item | Plan tasks | What it adds |
|---|---|---|
| Matcher benchmark (D3) | T16 | Picks the default matcher (cuda StereoSGM, VPI, libSGM) from measured fps and quality on the Jetson. Until then the default is `cv::cuda::StereoSGM`. |
| AI monocular fill (A3, D5) | T18 (T17 export already merged) | Depth Anything V2 Small in TensorRT. It fills stereo holes on `/stereo/depth/fused`. |
| Delete retired Pi stereo code | T26 | Removes the nodes/stereo_depth*, nodes/overview_camera dirs and their CI/lint entries. They are already undeployed by T24/T25. |
| Docs and diagrams | T29 | Deployment and architecture diagrams, deploy skill, MEMORY.md. |
| Calibrate page, stereo backend | T31-T35, T42, T43 | depthd ChArUco preview, pair capture to disk, cleanup, solve job, HTTP endpoints, client config, hardware check. The web_ui side (T36, T40) is built against the contract. |
| Arm calibration on the Calibrate page | none yet | Deferred by the user earlier. |

## Queued after the user confirms depth (2026-10-14)

| Item | What it adds |
|---|---|
| Pi 5 Camera Module 3 pair (front/back overview) | Both CSI ports on the client detect an IMX708 (i2c 10-001a and 11-001a, `camera_auto_detect=1`). Needs a camera node publishing front/back overview topics, shown in one web UI tab. It starts only after the depth fixes land and the user confirms depth looks right. |
| Right-camera AWB gains follow the left | nvarguscamerasrc on R32.6.1 cannot set WB gains, so this needs a libargus capture port. Depth uses grey images; only the RGB image is affected. |
