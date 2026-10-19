"""Head RGB-D calibration core, shared by the calibration CLI (scripts/) and the head_rgbd_calibration node.

Modules:
    config: StrictModel and the BoardConfig, SelectionConfig, LimitsConfig settings.
    charuco: ChArUco board construction and detection (Detection, detect, make_board, MIN_CORNERS).
    frames: frame types and still-pose selection (no bag or rosbags dependency).
    intrinsics: camera intrinsics from ChArUco detections, ToF zero-distortion choice, SDK comparison, YAML output.
    extrinsics: ToF-to-RGB extrinsic from ChArUco pairs, board-plane depth check, YAML output.
    calibrate: PoseView, CalibrationRun and calibrate_detections, the fit from per-pose detections.
    poses: PoseCounter, the live count of usable still board poses as the frames arrive.
    urdf_handoff: read the head URDF chain, compute and rewrite the tof_joint origin.
    urdf_text: standard-library tof_joint origin rewrite (the head_state_publisher launch loads it by path).

The core never imports `scripts`, `rosbags`, `subprocess` or ssh code.
"""
