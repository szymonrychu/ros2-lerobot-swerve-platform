"""The packaged robot body holds a tight head envelope: its head cylinder contains the head of head.urdf (pan base,
pan link, head link and the D435 box) at every pan/tilt inside the URDF limits, and is no bigger than that."""

import math
from pathlib import Path

import numpy as np
from arm_collision.settings import load_robot_body
from ros2_common.head_rgbd_calibration.arm_pose_check import (
    HEAD_MOUNT_JOINT,
    HEAD_PAN,
    HEAD_SAMPLE_STEP_RAD,
    HEAD_TILT,
    PoseCheckConfig,
    UrdfChain,
    head_envelope,
    limit_samples,
    visual_hull_points,
)

from mcp_server.config import McpServerConfig

# The head links the envelope holds: the pan base (fixed under the mount) and everything moving with pan or tilt.
HEAD_PAN_BASE = "head_pan_base"
CONTAIN_TOLERANCE_M = 1e-9
# The body's envelope is the URDF head rounded outwards to 1 mm: never more than this beyond it.
TIGHT_SLACK_M = 0.002


def head_urdf() -> str:
    """The head URDF mcp_server loads.

    Returns:
        str: URDF XML.
    """
    return Path(McpServerConfig().head.urdf_path).read_text()


def lowest_head_point(urdf: str) -> float:
    """Lowest z of the head visuals over the pan/tilt grid of head_envelope, lowered by its between-sample bound.

    Args:
        urdf (str): Head URDF XML.

    Returns:
        float: Lowest base_link z (m) the head can reach.
    """
    chain = UrdfChain.from_text(urdf)
    grids = [limit_samples(*chain.limits(joint), HEAD_SAMPLE_STEP_RAD) for joint in (HEAD_PAN, HEAD_TILT)]
    step = max(float(np.max(np.diff(g), initial=0.0)) for g in grids)
    mount = np.array(chain.origin(HEAD_MOUNT_JOINT)[0])
    pivot_reach = sum(float(np.linalg.norm(chain.origin(j)[0])) for j in (HEAD_PAN, HEAD_TILT))
    low, reach = math.inf, 0.0
    for pan in grids[0]:
        for tilt in grids[1]:
            for link, local in visual_hull_points(urdf).items():
                if link != HEAD_PAN_BASE and not {HEAD_PAN, HEAD_TILT} & set(chain.joint_names(link)):
                    continue
                pose = chain.transform(link, {HEAD_PAN: float(pan), HEAD_TILT: float(tilt)})
                placed = local @ pose[:3, :3].T + pose[:3, 3]
                low = min(low, float(placed[:, 2].min()))
                reach = max(reach, float(np.linalg.norm(placed - mount, axis=1).max()) + pivot_reach)
    return low - 2.0 * reach * (1.0 - math.cos(step / 2.0))


def test_body_head_cylinder_is_the_tight_head_envelope() -> None:
    """The body's head cylinder contains the URDF head (no configured growth) and exceeds it by at most 2 mm."""
    urdf = head_urdf()
    envelope = head_envelope(urdf, PoseCheckConfig(head_radius_m=0.0, head_height_m=0.0))
    low = lowest_head_point(urdf)
    head = next(c for c in load_robot_body().cylinders if c.name == "head")
    offset = math.hypot(head.x - envelope.x, head.y - envelope.y)
    assert offset + envelope.radius <= head.radius + CONTAIN_TOLERANCE_M
    assert head.z_min <= low + CONTAIN_TOLERANCE_M
    assert head.z_max >= envelope.z_max - CONTAIN_TOLERANCE_M
    assert head.radius <= envelope.radius + TIGHT_SLACK_M
    assert head.z_min >= low - TIGHT_SLACK_M
    assert head.z_max <= envelope.z_max + TIGHT_SLACK_M
