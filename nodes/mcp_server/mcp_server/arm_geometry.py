"""Arm link geometry shared by the surface checks (grasp planner) and the body self-collision check (self_collision).

Link bodies are capsules between FK link origins (radius = half the STS3215 servo body each link carries, about 4 cm
across); the gripper body and fixed jaw are a hull of points in gripper_link; the jaw points come from the floor guard
(FloorGuard.checked_points).
"""

import numpy as np

# Capsule radius of the arm links (m): half the STS3215 servo body.
LINK_CAPSULE_RADIUS_M = 0.02
UPPER_ARM_CAPSULE = ("upper arm", "shoulder", "elbow", LINK_CAPSULE_RADIUS_M)
# Link bodies checked against surface regions as capsules between FK link origins (radius = half the STS3215 servo
# body that each link carries, about 4 cm across).
FOREARM_CAPSULE = ("forearm", "elbow", "wrist", LINK_CAPSULE_RADIUS_M)
WRIST_CAPSULE = ("wrist link", "wrist", "gripper", LINK_CAPSULE_RADIUS_M)
JAW_POINTS = ("jaw_tip", "tool_point", "moving_jaw_tip")
GRIPPER_BODY = "gripper body"
# Hull of the gripper body and fixed jaw in gripper_link (m): the corners of the housing collision boxes and the extreme
# points of the fixed finger mesh per 1 cm slice, taken from the grasp_sim MuJoCo model with the jaws calibrated to
# the 2026-10-10 tool point (its gripper body frame is gripper_link). Checked against surfaces with
# surface_jaw_clearance_m: at a diagonal step edge the jaw body, not its tip, is what touches first.
GRIPPER_BODY_POINTS = np.array(
    [
        (-0.038, -0.015, -0.038),
        (-0.037, -0.021, -0.040),
        (-0.037, -0.021, -0.030),
        (-0.037, 0.009, -0.040),
        (-0.037, 0.009, -0.030),
        (-0.035, -0.015, -0.037),
        (-0.035, -0.015, -0.007),
        (-0.035, 0.015, -0.037),
        (-0.035, 0.015, -0.007),
        (-0.034, -0.012, -0.049),
        (-0.034, 0.000, -0.049),
        (-0.023, -0.002, -0.081),
        (-0.020, -0.022, -0.033),
        (-0.020, 0.010, -0.033),
        (-0.019, -0.009, -0.091),
        (-0.018, 0.005, -0.039),
        (-0.017, -0.021, -0.040),
        (-0.017, -0.021, -0.030),
        (-0.017, 0.009, -0.040),
        (-0.017, 0.009, -0.030),
        (-0.016, -0.015, -0.056),
        (-0.016, -0.014, -0.054),
        (-0.016, 0.002, -0.058),
        (-0.016, 0.002, -0.054),
        (-0.015, -0.012, -0.091),
        (-0.014, -0.013, -0.087),
        (-0.014, 0.000, -0.090),
        (-0.014, 0.001, -0.083),
        (-0.013, -0.001, -0.090),
        (-0.012, -0.006, -0.099),
        (0.030, -0.015, -0.037),
        (0.030, -0.015, -0.007),
        (0.030, 0.015, -0.037),
        (0.030, 0.015, -0.007),
    ]
)
# FK link frame whose origin each capsule end names: the shoulder_lift axis, the elbow, wrist_flex axis, gripper_link.
LINK_ORIGINS = {
    "shoulder": "upper_arm_link",
    "elbow": "lower_arm_link",
    "wrist": "wrist_link",
    "gripper": "gripper_link",
}
