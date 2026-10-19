"""Head RGB-D calibration core, shared by the calibration CLI (scripts/) and the head_rgbd_calibration node.

Modules:
    config: StrictModel and the BoardConfig, SelectionConfig, LimitsConfig settings.
    charuco: ChArUco board construction and detection (Detection, detect, make_board, MIN_CORNERS).
    calibrate: PoseView, CalibrationRun and calibrate_detections, the fit from per-pose detections.

The core never imports `scripts`, `rosbags`, `subprocess` or ssh code.
"""
