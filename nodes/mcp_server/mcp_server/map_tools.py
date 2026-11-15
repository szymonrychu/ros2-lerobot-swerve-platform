"""Named map tools: save_map, list_maps, load_map, delete_map, plus relocalise (talk to map_manager, design D11, D13).

None of them moves a servo or drives the base; load_map stops the robot first because the map under it is replaced.
relocalise re-finds the robot on the current map (SLAM only) and stops nothing.
"""

from typing import Annotated, Any

from pydantic import BaseModel, Field, ValidationError
from ros2_common.maps import MAP_NAME_RULE, Localisation, MapCommand, MapSummary

from .models import RobotError
from .perception_tools import tool_errors
from .tool_context import ToolContext

TOOL_NAMES = ("save_map", "list_maps", "load_map", "delete_map", "relocalise")
NAME_DESC = f"Map name ({MAP_NAME_RULE})"
# map_manager's relocalise searches +-1 m and the full turn around the last good pose (map_manager search settings).
RELOCALISE_NEXT_STEP = (
    "relocalise searches only +-1 m and a full turn around the last good pose: after a longer carry ask a person "
    "where the robot is (or to place it back), or call load_map with a pose hint (x, y, yaw). Do not navigate until "
    "get_robot_state localisation.reliable is true."
)
RELOCALISE_DESC = (
    "Re-find the robot on the CURRENT map (map_manager relocalise: SLAM only, it moves nothing and needs no lease; "
    "the map and POIs are kept). It pauses SLAM, searches +-1 m and a full turn around the last good pose for the "
    "scan's best fit, restarts SLAM there and checks the scan-to-map agreement. Use it when get_robot_state "
    "localisation.reliable is false (or null), or when the camera view disagrees with the map pose, before "
    "navigate_to_pose / move_relative. Returns the verdict (state 'localised', ratio, pose, hint, ambiguous). A "
    "failure is an error with map_manager's reason; then the robot is probably more than 1 m from the last good "
    "pose: ask a person where it is, or call load_map with a pose hint (x, y, yaw). Waits up to the load timeout."
)


class RelocaliseVerdict(BaseModel):
    """relocalise result: the robot was found on the current map."""

    ok: bool
    state: str
    ratio: float | None = None
    pose: dict[str, float] | None = None  # the pose slam_toolbox confirmed (map frame)
    hint: dict[str, float] | None = None  # the last good pose the search started from
    ambiguous: bool = False
    message: str = ""


class MapOpResult(BaseModel):
    """Answer of save_map / delete_map."""

    ok: bool
    name: str
    message: str
    data: dict[str, Any] | None = None


class MapList(BaseModel):
    """list_maps result."""

    maps: list[MapSummary]
    active_map: str | None
    localisation: Localisation


class LoadVerdict(BaseModel):
    """load_map result: whether the robot found itself in the loaded map."""

    name: str
    state: str
    ratio: float | None = None
    pose: dict[str, float] | None = None
    reason: str = ""
    message: str = ""


def build_command(**fields: Any) -> dict[str, Any]:
    """Validate a map_manager command with the shared contract.

    Args:
        **fields (Any): MapCommand fields.

    Returns:
        dict[str, Any]: The command as a plain dict (no request_id).

    Raises:
        ValueError: A field breaks the contract (message is the contract's, e.g. the map name rule).
    """
    try:
        return MapCommand(**fields).model_dump()
    except ValidationError as exc:
        raise ValueError("; ".join(str(e["msg"]).removeprefix("Value error, ") for e in exc.errors())) from exc


def register(ctx: ToolContext) -> None:
    """Register the named map tools on the context's server.

    Args:
        ctx (ToolContext): Shared registration context.
    """
    server, robot = ctx.server, ctx.robot

    @server.tool(
        description=(
            "Save the current map together with its POIs, the last robot pose and a note as the named map 'name' "
            "(map_manager). Refuses an existing name unless overwrite is true. Errors if map_manager is not running. "
            "No motion."
        )
    )
    def save_map(
        name: Annotated[str, Field(description=NAME_DESC)],
        overwrite: Annotated[bool, Field(description="Replace a saved map of the same name")] = False,
        note: Annotated[str, Field(description="Free text stored with the map (max 500 characters)")] = "",
    ) -> MapOpResult:
        """Save the named map."""
        with tool_errors():
            result = robot.map_request(build_command(op="save", name=name, overwrite=overwrite, note=note))
        return MapOpResult(ok=True, name=name, message=result.message, data=result.data)

    @server.tool(
        description=(
            "List the saved named maps (name, saved_at, note, poi_count, last_pose, valid) plus the active map and the "
            "localisation (state: unknown, loading, localised, not_localised, ambiguous; ratio; pose). Errors when "
            "map_manager has published no status yet. Read-only, no motion."
        )
    )
    def list_maps() -> MapList:
        """List named maps and the localisation."""
        with tool_errors():
            current = robot.map_status()
        return MapList(maps=current.maps, active_map=current.active_map, localisation=current.localisation)

    @server.tool(
        description=(
            "Load the named map: it replaces the current map and POIs (the current ones are first kept as the "
            "'autosave' map) and the robot relocalises in it, starting from the saved last pose or the pose hint. "
            "Calls stop first. Returns the verdict: state, ratio (scan-to-map agreement 0..1), pose and reason. Only "
            "state 'localised' is safe: do not navigate when the state is not 'localised'; if the robot stands "
            "elsewhere, retry load_map with a pose hint (x, y, yaw all three in map metres/radians). A partial pose "
            "is an error. Waits up to the load timeout. No servo moves."
        )
    )
    def load_map(
        name: Annotated[str, Field(description=NAME_DESC)],
        x: Annotated[float | None, Field(description="Pose hint x (m); give x, y and yaw together or none")] = None,
        y: Annotated[float | None, Field(description="Pose hint y (m)")] = None,
        yaw: Annotated[float | None, Field(description="Pose hint yaw (rad)")] = None,
    ) -> LoadVerdict:
        """Stop, load the named map and return the localisation verdict."""
        given = [v is not None for v in (x, y, yaw)]
        pose = {"x": x, "y": y, "yaw": yaw} if all(given) else None
        with tool_errors():
            if any(given) and not all(given):
                raise ValueError("a pose hint needs x, y and yaw together (or none of them)")
            command = build_command(op="load", name=name, pose=pose)
            if ctx.queue is not None:
                ctx.queue.halt_for_stop()
            robot.stop()
            result = robot.map_request(command)
        data = result.data or {}
        return LoadVerdict(
            name=name,
            state=str(data.get("state", "unknown")),
            ratio=data.get("ratio"),
            pose=data.get("pose"),
            reason=str(data.get("reason") or ""),
            message=result.message,
        )

    @server.tool(description=RELOCALISE_DESC)
    def relocalise() -> RelocaliseVerdict:
        """Relocalise on the current map and return the verdict."""
        with tool_errors():
            command = build_command(op="relocalise")
            try:
                result = robot.map_request(command)
            except RobotError as exc:
                raise RobotError(f"{exc}. {RELOCALISE_NEXT_STEP}") from exc
        data = result.data or {}
        return RelocaliseVerdict(
            ok=result.ok,
            state=str(data.get("state", "unknown")),
            ratio=data.get("ratio"),
            pose=data.get("slam_pose"),
            hint=data.get("hint"),
            ambiguous=bool(data.get("ambiguous", False)),
            message=result.message,
        )

    @server.tool(
        description=(
            "Delete the saved named map and its POIs; this also removes the map's POIs and cannot be undone. Always "
            "ask the person first, then call with confirm=true; without confirm nothing is sent. The loaded map and "
            "the live POIs are not touched. No motion."
        )
    )
    def delete_map(
        name: Annotated[str, Field(description=NAME_DESC)],
        confirm: Annotated[bool, Field(description="true only after the person agreed to the deletion")] = False,
    ) -> MapOpResult:
        """Delete the named map after confirmation."""
        with tool_errors():
            if not confirm:
                raise ValueError(
                    f"delete_map removes map '{name}' and its POIs for good: ask the person first, "
                    "then call again with confirm=true"
                )
            result = robot.map_request(build_command(op="delete", name=name))
        return MapOpResult(ok=True, name=name, message=result.message, data=result.data)
