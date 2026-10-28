"""Retirement pins: the ToF-only code paths of head_rgbd_cpp are gone.

The D435 publishes its depth already aligned to the colour camera (realsense2_camera align_depth.enable), so
head_rgbd_cpp no longer registers a ToF depth into the RGB camera, compensates head motion between the two, upsamples
the low-resolution ToF depth to the RGB size or removes ToF flying pixels.
"""

import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
HEAD_RGBD_DIR = "nodes/head_rgbd_cpp"
REGISTRATION_AND_MOTION_PATHS = (
    f"{HEAD_RGBD_DIR}/src/core/registration.cpp",
    f"{HEAD_RGBD_DIR}/include/head_rgbd_cpp/registration.hpp",
    f"{HEAD_RGBD_DIR}/test/core/test_registration.cpp",
    f"{HEAD_RGBD_DIR}/src/core/motion.cpp",
    f"{HEAD_RGBD_DIR}/include/head_rgbd_cpp/motion.hpp",
    f"{HEAD_RGBD_DIR}/test/core/test_motion.cpp",
)
# An #include of registration.hpp or motion.hpp, by any path prefix and with either delimiter.
REGISTRATION_OR_MOTION_INCLUDE = r'#include[[:space:]]*[<"]([^<>"]*/)?(registration|motion)\.hpp[>"]'
UPSAMPLING_AND_TOF_FILTER_PATHS = (
    f"{HEAD_RGBD_DIR}/src/core/upsample.cpp",
    f"{HEAD_RGBD_DIR}/include/head_rgbd_cpp/upsample.hpp",
    f"{HEAD_RGBD_DIR}/test/core/test_upsample.cpp",
    f"{HEAD_RGBD_DIR}/src/core/tof_filter.cpp",
    f"{HEAD_RGBD_DIR}/include/head_rgbd_cpp/tof_filter.hpp",
    f"{HEAD_RGBD_DIR}/test/core/test_tof_filter.cpp",
)
# An #include of upsample.hpp or tof_filter.hpp, by any path prefix and with either delimiter.
UPSAMPLE_OR_TOF_FILTER_INCLUDE = r'#include[[:space:]]*[<"]([^<>"]*/)?(upsample|tof_filter)\.hpp[>"]'


def git(*args: str) -> subprocess.CompletedProcess[str]:
    """Run git in the repo root and return the finished process.

    Args:
        *args (str): Arguments after `git`.

    Returns:
        subprocess.CompletedProcess[str]: The finished process with captured text output.
    """
    return subprocess.run(["git", *args], cwd=REPO, capture_output=True, text=True, check=False)


def test_head_rgbd_registration_and_motion_are_gone() -> None:
    tracked = git("ls-files", "--", *REGISTRATION_AND_MOTION_PATHS).stdout.strip()
    assert tracked == "", f"still tracked:\n{tracked}"
    includes = git("grep", "-n", "-E", REGISTRATION_OR_MOTION_INCLUDE, "--", HEAD_RGBD_DIR)
    assert includes.returncode in (0, 1), includes.stderr
    assert includes.stdout == "", f"still included:\n{includes.stdout}"


def test_head_rgbd_upsampling_and_tof_filter_are_gone() -> None:
    tracked = git("ls-files", "--", *UPSAMPLING_AND_TOF_FILTER_PATHS).stdout.strip()
    assert tracked == "", f"still tracked:\n{tracked}"
    includes = git("grep", "-n", "-E", UPSAMPLE_OR_TOF_FILTER_INCLUDE, "--", HEAD_RGBD_DIR)
    assert includes.returncode in (0, 1), includes.stderr
    assert includes.stdout == "", f"still included:\n{includes.stdout}"
