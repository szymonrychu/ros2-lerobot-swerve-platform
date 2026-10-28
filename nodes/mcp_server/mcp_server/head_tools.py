"""Head tools: set_head_angles, head_look_at and head_home (pan/tilt head, degrees in and out).

All three need either the arm or the base lease (tools.TOOL_SUBSYSTEM "any") and are refused under battery cut-off.
The head carries the Intel RealSense D435 RGB-D camera; head_look_at aims its colour camera axis once at a point, it
never tracks.
"""

import math
from typing import Annotated, Literal

import numpy as np
from mcp.server.mcpserver.exceptions import ToolError
from pydantic import BaseModel, Field

from .head import HeadError, HeadMoveResult
from .head_kinematics import BASE_LINK, HeadAimError
from .models import HeadAngles
from .perception_tools import tool_errors
from .tool_context import ToolContext

TOOL_NAMES = ("set_head_angles", "head_look_at", "head_home")
MAP_FRAME = "map"
LEASE_NOTE = "Needs the arm or the base lease (take_control). Releasing the controlling lease sends the head home."


class Point(BaseModel):
    """A point (m) in base_link."""

    x: float
    y: float
    z: float


class HeadMoveReport(BaseModel):
    """Result of a head tool: status, commanded and measured angles (degrees), look_at target in base_link."""

    status: Literal["converged", "timeout", "no_feedback"]
    commanded: HeadAngles
    measured: HeadAngles | None = Field(description="Measured angles at the end; absent without fresh joint states")
    target_base_link: Point | None = Field(default=None, description="head_look_at: the aimed point in base_link")


def to_angles(values: dict[str, float], names: tuple[str, str]) -> HeadAngles:
    """Head joint values (rad) as degrees.

    Args:
        values (dict[str, float]): Joint name -> rad.
        names (tuple[str, str]): (pan, tilt) joint names.

    Returns:
        HeadAngles: Pan/tilt in degrees.
    """
    return HeadAngles(pan_deg=math.degrees(values[names[0]]), tilt_deg=math.degrees(values[names[1]]))


def report(result: HeadMoveResult, names: tuple[str, str], target: Point | None = None) -> HeadMoveReport:
    """Build the tool result from a controller move.

    Args:
        result (HeadMoveResult): Controller outcome (rad).
        names (tuple[str, str]): (pan, tilt) joint names.
        target (Point | None): look_at target in base_link.

    Returns:
        HeadMoveReport: Degrees report.
    """
    measured = None if result.measured is None else to_angles(result.measured, names)
    return HeadMoveReport(
        status=result.status, commanded=to_angles(result.target, names), measured=measured, target_base_link=target
    )


def map_to_base(x: float, y: float, z: float, pose_x: float, pose_y: float, yaw: float) -> Point:
    """Convert a map point to base_link with the planar robot pose (z is unchanged: base_link origin is on the floor).

    Args:
        x (float): Map x (m).
        y (float): Map y (m).
        z (float): Height (m).
        pose_x (float): Robot x in map (m).
        pose_y (float): Robot y in map (m).
        yaw (float): Robot yaw in map (rad).

    Returns:
        Point: The point in base_link.
    """
    dx, dy = x - pose_x, y - pose_y
    cos, sin = math.cos(yaw), math.sin(yaw)
    return Point(x=cos * dx + sin * dy, y=-sin * dx + cos * dy, z=z)


def register(ctx: ToolContext) -> None:
    """Register set_head_angles, head_look_at and head_home.

    Args:
        ctx (ToolContext): Shared registration context.
    """
    robot = ctx.robot
    names = ctx.config.head.joint_names

    @ctx.server.tool(
        description=(
            "Move the pan/tilt head (carries the D435 RGB-D camera) to pan_deg / tilt_deg, in degrees: "
            "pan + is left, tilt + is down, 0/0 is straight ahead and level. Angles outside the head limits are "
            f"refused with the limits. Waits until the head arrives. {LEASE_NOTE}"
        ),
    )
    def set_head_angles(
        pan_deg: Annotated[float, Field(description="Pan in degrees, + left")],
        tilt_deg: Annotated[float, Field(description="Tilt in degrees, + down")],
    ) -> HeadMoveReport:
        """Move the head to the given angles; the description is passed to the decorator."""
        ctx.battery_gate("set_head_angles")
        with tool_errors():
            try:
                result = robot.head.move(math.radians(pan_deg), math.radians(tilt_deg))
            except HeadError as exc:
                raise ToolError(str(exc)) from exc
            return report(result, names)

    @ctx.server.tool(
        description=(
            "Aim the head D435 colour camera axis once at a point (x, y, z in metres) in base_link (x forward, y left, "
            "z up from the floor) or in the map frame (converted with the current robot pose; refused when the pose is "
            "unknown), then move there. No tracking: the head does not follow the point or the robot afterwards. "
            f"A point the head cannot reach is refused with the pan/tilt it would need. {LEASE_NOTE}"
        ),
    )
    def head_look_at(
        x: Annotated[float, Field(description="m")],
        y: Annotated[float, Field(description="m")],
        z: Annotated[float, Field(description="m above the floor")],
        frame: Annotated[Literal["base_link", "map"], Field(description="Frame of x, y, z")] = BASE_LINK,
    ) -> HeadMoveReport:
        """Aim the head once at a point; the description is passed to the decorator."""
        ctx.battery_gate("head_look_at")
        with tool_errors():
            if frame == MAP_FRAME:
                pose = robot.robot_pose()
                if pose is None:
                    raise ToolError("head_look_at in the map frame refused: the robot map pose is unknown or stale")
                target = map_to_base(x, y, z, pose.x, pose.y, pose.yaw)
            else:
                target = Point(x=x, y=y, z=z)
            try:
                pan, tilt = robot.head.kinematics.look_at(np.array([target.x, target.y, target.z]))
                result = robot.head.move(pan, tilt)
            except (HeadAimError, HeadError) as exc:
                raise ToolError(str(exc)) from exc
            return report(result, names, target)

    @ctx.server.tool(
        description=(
            "Send the head to its configured home pose (forward and down at the ground). "
            f"{LEASE_NOTE.replace(' Releasing the controlling lease sends the head home.', '')}"
        ),
    )
    def head_home() -> HeadMoveReport:
        """Move the head home; the description is passed to the decorator."""
        ctx.battery_gate("head_home")
        with tool_errors():
            return report(robot.head.home(), names)
