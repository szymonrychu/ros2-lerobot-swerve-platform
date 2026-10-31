"""Head RGB-D calibration core, used by the head_rgbd_calibration node.

Modules:
    config: StrictModel and the board, selection, limits and fit settings.
    charuco: ChArUco board construction and detection (Detection, detect, make_board, MIN_CORNERS).
    frames: frame types, the still-frame difference and the message decoders.
    intrinsics: camera intrinsics from ChArUco detections and the comparison with a factory camera_info.
    calibrate: the D435 colour intrinsics check and the board-plane depth bias.
    hints: capture-time corner-count hints.
    chain: forward kinematics over URDF text.
    floor_fit, head_arm_fit, arm_offsets_fit: the head chain and arm offsets fits.
    arm_pose_check: the arm pose collision check of the head-from-arm sequence.
    views: the arm board view type.
    urdf_text: standard-library joint origin rewrite and head_chain.json override (the head_state_publisher launch
        loads it by path).

The core never imports `scripts`, `rosbags`, `subprocess` or ssh code.
"""
