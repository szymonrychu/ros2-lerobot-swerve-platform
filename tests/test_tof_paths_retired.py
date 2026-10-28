"""Retirement pins: the ToF-only code paths of head_rgbd_cpp are gone.

The D435 publishes its depth already aligned to the colour camera (realsense2_camera align_depth.enable), so
head_rgbd_cpp no longer registers a ToF depth into the RGB camera or compensates head motion between the two.
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
