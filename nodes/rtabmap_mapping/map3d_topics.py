"""Single typed definition of the cross-host map3d streams (design D2, D3).

Read by the client uplink launch file, the server rtabmap launch file, map3d_manager and the tests.
Depends only on the stdlib, pyyaml and pydantic so it imports on the Pis with the system python. Works with pydantic
1.10 (Ubuntu 24.04's python3-pydantic, what the launch files get on the Pis) and pydantic 2 (the dev and test venvs):
config through class keywords, validation through the model constructor and after_validator().
"""

from collections.abc import Callable
from pathlib import Path
from typing import Any, Literal

import pydantic
import yaml
from pydantic import BaseModel, Field

DEFAULT_YAML = Path(__file__).resolve().parent / "config" / "map3d_topics.yaml"
UPLINK_PREFIX = "/uplink/"
DOWNLINK_PREFIX = "/map3d/"
TF_STATIC_TOPIC = "/tf_static"
SMALL_BYTES_PER_S = 1000.0
BITS_PER_MBIT = 1e6
PYDANTIC_V2 = pydantic.VERSION.startswith("2.")

Reader = Literal["rtabmap_mapping", "map3d_manager"]


def after_validator(check: Callable[[Any], None]) -> Any:
    """Model validator that runs check(model) after the fields validated, on pydantic 1.10 and 2.

    Args:
        check: Raises ValueError when the model is invalid; pydantic reports it as a ValidationError.

    Returns:
        Any: The validator to assign in the class body.
    """
    if PYDANTIC_V2:

        def run(model: Any) -> Any:
            check(model)
            return model

        return pydantic.model_validator(mode="after")(run)

    def run_v1(cls: Any, values: dict[str, Any]) -> dict[str, Any]:
        check(cls.construct(**values))
        return values

    return pydantic.root_validator(skip_on_failure=True, allow_reuse=True)(run_v1)


class Forbid(BaseModel, extra="forbid"):
    """Base model that rejects unknown keys."""


class Qos(Forbid):
    """QoS profile of a relayed stream."""

    reliability: Literal["reliable", "best_effort"]
    durability: Literal["volatile", "transient_local"]
    depth: int = Field(gt=0)


class Stream(Forbid):
    """One relayed topic.

    Attributes:
        source: topic the relay subscribes.
        dest: topic the relay publishes (a new name, never a client topic).
        type: ROS message type of the source.
        rate_hz: throttle rate, None passes every message.
        expected_rate_hz: message rate used for budgeting when rate_hz is None.
        qos: relay QoS.
        transport: raw, or compressedDepth for the image_transport republisher.
        lazy: relay only while the destination has subscribers.
        budget_bytes: expected bytes per message (source documented in the yaml).
        server_readers: server processes subscribing the uplink dest.
        small: budget_bytes x rate below 1 KB/s, allowing several readers.
    """

    source: str
    dest: str
    type: str
    rate_hz: float | None = Field(gt=0)
    expected_rate_hz: float | None = Field(default=None, gt=0)
    qos: Qos
    transport: Literal["raw", "compressedDepth"] = "raw"
    lazy: bool = False
    budget_bytes: int = Field(gt=0)
    server_readers: list[Reader] = []
    small: bool = False

    def check_rates(self) -> None:
        """Require expected_rate_hz for unthrottled streams and a truthful small flag."""
        if self.rate_hz is None and self.expected_rate_hz is None:
            raise ValueError(f"expected_rate_hz is required when rate_hz is null ({self.source})")
        if self.small != (self.budget_bytes * self.effective_rate() < SMALL_BYTES_PER_S):
            raise ValueError(f"small must be true exactly when budget_bytes x rate < 1 KB/s ({self.source})")

    rates_valid = after_validator(check_rates)

    def effective_rate(self, expected_rates: dict[str, float] | None = None) -> float:
        """Return the message rate used for budgeting.

        Args:
            expected_rates: optional per-source override for unthrottled streams.

        Returns:
            rate_hz when set, else the override or expected_rate_hz.
        """
        if self.rate_hz is not None:
            return self.rate_hz
        if expected_rates and self.source in expected_rates:
            return expected_rates[self.source]
        assert self.expected_rate_hz is not None
        return self.expected_rate_hz


class Frames(Forbid):
    """TF frame names shared by RTAB-Map, map3d_manager and the web UI."""

    map_frame: str
    map3d_frame: str
    base_frame: str
    rgb_frame: str
    depth_frame: str


class Budget(Forbid):
    """WiFi link ceilings in Mbit/s."""

    uplink_mbit_s_max: float = Field(gt=0)
    downlink_mbit_s_max: float = Field(gt=0)


class Map3dTopics(Forbid):
    """The whole contract: uplink, downlink, frames and budget."""

    uplink: list[Stream]
    downlink: list[Stream]
    frames: Frames
    budget: Budget

    def check_contract(self) -> None:
        """Check prefixes, relay loops, uniqueness, tf_static QoS and reader counts."""
        for s in self.uplink:
            if not s.dest.startswith(UPLINK_PREFIX):
                raise ValueError(f"uplink dest must start with {UPLINK_PREFIX}: {s.dest}")
            if not s.server_readers:
                raise ValueError(f"server_readers must name at least one reader: {s.source}")
            if len(s.server_readers) > 1 and not s.small:
                raise ValueError(f"server_readers allows more than one only for small streams: {s.source}")
            if s.source == TF_STATIC_TOPIC and s.qos.durability != "transient_local":
                raise ValueError(f"{TF_STATIC_TOPIC} must be transient_local")
        for s in self.downlink:
            if not s.dest.startswith(DOWNLINK_PREFIX):
                raise ValueError(f"downlink dest must start with {DOWNLINK_PREFIX}: {s.dest}")
        streams = self.uplink + self.downlink
        sources = {s.source for s in streams}
        dests = [s.dest for s in streams]
        for d in dests:
            if d in sources:
                raise ValueError(f"dest {d} equals a source (relay loop)")
        if len(set(dests)) != len(dests):
            raise ValueError("dest values must be unique")

    contract_valid = after_validator(check_contract)


def load_map3d_topics(path: Path | None = None) -> Map3dTopics:
    """Load and validate the contract yaml.

    Args:
        path: yaml file, default the map3d_topics.yaml next to this module.

    Returns:
        The validated contract.
    """
    return Map3dTopics(**yaml.safe_load((path or DEFAULT_YAML).read_text()))


def estimated_mbit_s(streams: list[Stream], expected_rates: dict[str, float]) -> float:
    """Estimate the link load of streams.

    Args:
        streams: streams to sum.
        expected_rates: per-source message rate overrides for unthrottled streams.

    Returns:
        Sum of budget_bytes x rate x 8 / 1e6.
    """
    return sum(s.budget_bytes * s.effective_rate(expected_rates) * 8 for s in streams) / BITS_PER_MBIT
