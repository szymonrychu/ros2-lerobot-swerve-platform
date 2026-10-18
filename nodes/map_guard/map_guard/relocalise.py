"""map_manager relocalise messages: the command the guard publishes and the result it reads (ROS-free)."""

from typing import NamedTuple

from pydantic import ValidationError
from ros2_common.maps import MapCommand, MapResult


class RelocaliseResult(NamedTuple):
    """The parts of a map_manager result the guard acts on."""

    request_id: str
    ok: bool
    state: str
    discarded_pois: int


def relocalise_command(request_id: str, since: float) -> str:
    """Build the relocalise command JSON.

    Args:
        request_id (str): Id echoed in the result.
        since (float): Epoch seconds when the uncontrolled move started.

    Returns:
        str: JSON for the map_manager command topic (discard_on_fail: map_manager clears the live POIs on failure).
    """
    command = MapCommand(op="relocalise", request_id=request_id, since=since, discard_on_fail=True)
    return command.model_dump_json(include={"op", "request_id", "since", "discard_on_fail"})


def parse_result(text: str) -> RelocaliseResult | None:
    """Parse a map_manager result message.

    Args:
        text (str): JSON from the result topic.

    Returns:
        RelocaliseResult | None: The result, None when the text is not a valid result.
    """
    try:
        result = MapResult.model_validate_json(text)
    except ValidationError:
        return None
    data = result.data or {}
    return RelocaliseResult(
        result.request_id, result.ok, str(data.get("state", "")), int(data.get("discarded_pois", 0))
    )
