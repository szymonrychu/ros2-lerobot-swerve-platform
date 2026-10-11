"""Body awareness tools: get_body_state and get_arm_reach (the arm's reach around the robot body)."""

from typing import Annotated, Any

from mcp.server.mcpserver.exceptions import ToolError
from pydantic import BaseModel, ConfigDict, Field

from .arm_reach import DEFAULT_SEED, ReachEnvelope, point_reach
from .models import BodyState
from .tool_context import ToolContext


class ReachPoint(BaseModel):
    """A tool point to check (robot base_link: x forward, y left, z up from the floor)."""

    model_config = ConfigDict(extra="forbid")

    x: float = Field(description="m, forward of the robot centre (base_link)")
    y: float = Field(description="m, left of the robot centre")
    z: float = Field(description="m, above the floor")
    pitch: float | None = Field(
        default=None, description="Approach pitch (rad, 0 horizontal, +1.57 straight down); omitted = try a range"
    )


class ArmReach(BaseModel):
    """get_arm_reach result."""

    obstacles: list[dict[str, Any]] = Field(description="Robot body obstacles (base_link) with their dimensions")
    arm_mount: dict[str, Any] = Field(description="Arm base pose in base_link and the shoulder_pan axis position")
    margins: dict[str, Any] = Field(description="enabled, soft_margin_m (slowed below), hard_margin_m (rejected below)")
    envelope: dict[str, Any] = Field(
        description="Collision-free reach envelope: status ready|computing, grid, rows per pan heading with cells "
        "{z_m, min_r_m, max_r_m} = horizontal tool distance from the pan axis (null = nothing collision-free there)"
    )
    points: list[dict[str, Any]] = Field(default_factory=list, description="Per requested point, in order")


def register(ctx: ToolContext) -> None:
    """Register get_body_state.

    Args:
        ctx (ToolContext): Shared registration context.
    """
    monitor = ctx.monitor
    battery_note = (
        f" Battery warning below {monitor.battery_warn_v():.2f} V, cut-off below "
        f"{ctx.guard.cells * ctx.guard.cutoff_cell_v:.2f} V."
        if ctx.guard is not None
        else ""
    )
    cfg = ctx.config.monitor

    @ctx.server.tool(
        description=(
            "Body vitals of the robot, always available (no motion): per-servo latest temperature (C), load/current "
            "(raw register values), voltage (V), status error flags and data age (servo registers are dumped about "
            "every 10 s), the hottest servo, battery voltage, per-cell voltage and margin to the cut-off, IMU tilt "
            "(roll/pitch deg) and the last bump, the tilt guard (level ok|warning|error, roll/pitch relative to the "
            "level reference, warning/error limits per axis, whether navigation and manual drive are allowed), wheel "
            "slip residual (m/s, null while parked), commanded versus "
            "measured base speed, CPU temperature and firmware throttling flag, the active arm source and whether "
            "this server holds the arm lease, and the last 10 robot events. Missing data is null with a reason in "
            f"`notes` (nothing is invented). Warnings: servo >= {cfg.servo_temp_warn_c:g} C (critical >= "
            f"{cfg.servo_temp_critical_c:g} C), CPU >= {cfg.cpu_temp_warn_c:g} C (critical >= "
            f"{cfg.cpu_temp_critical_c:g} C), IMU tilt > {cfg.tilt_warn_deg:g} deg only while the tilt guard is "
            f"silent (no /tilt_guard/state for {cfg.tilt_guard_stale_s:g} s).{battery_note}"
        )
    )
    def get_body_state() -> BodyState:
        """Body vitals; the tool description is passed to the decorator so it can state the configured thresholds."""
        return monitor.body_state()

    arm = ctx.robot.arm
    checker = arm.body_check
    settings = ctx.config.body_check
    envelope = ReachEnvelope(checker, settings, settings.reach_cache_file)
    # With a disk cache (the robot sets one) start now or load it, so the agent's first get_arm_reach call does not wait
    # for the computation; without one the envelope stays lazy (first call), which keeps test servers cheap.
    if settings.reach_cache_file is not None:
        envelope.start()
    mount = checker.mount

    @ctx.server.tool(
        description=(
            "Where the arm can reach WITHOUT hitting the robot body (no motion). Seen from above the robot is an H: two "
            "side frames above the wheels, a crossbar between them carrying the arm base and the lidar, the four "
            "steering wheels (their whole steering circle counts) and the lidar dome front-left of the arm. Every arm "
            f"motion is checked against these: within {settings.soft_margin_m:g} m it runs at "
            f"{settings.slow_speed_scale:g} of its speed, within {settings.hard_margin_m:g} m (or into the body) it is "
            "refused before it starts. Returns the obstacles with dimensions (base_link: x forward, y left, z up from "
            "the floor), the arm mount, the margins and a collision-free reach envelope (per shoulder_pan heading every "
            f"{settings.reach_heading_step_deg:g} deg and tool height 0..{settings.reach_height_max_m:g} m every "
            f"{settings.reach_height_step_m:g} m: min/max horizontal tool distance from the pan axis; computed in the "
            "background at server start; while it is still computing the envelope reports status computing). With points (up to "
            f"{settings.reach_max_points}, base_link x/y/z and optional pitch) each point reports reachable (IK within "
            "joint limits), collision_free, the working pitch range, the limiting obstacle/link/clearance and a "
            "suggestion. Call it when unsure whether a target near the robot body is safe."
        )
    )
    def get_arm_reach(
        points: Annotated[
            list[ReachPoint] | None,
            Field(description=f"Tool points to check (base_link), at most {settings.reach_max_points}"),
        ] = None,
    ) -> ArmReach:
        """Arm reach around the body; the tool description is passed to the decorator so it can state the margins."""
        wanted = points or []
        if len(wanted) > settings.reach_max_points:
            raise ToolError(f"at most {settings.reach_max_points} points per call, got {len(wanted)}")
        result = envelope.get(settings.reach_wait_s)
        env: dict[str, Any] = (
            result
            if result is not None
            else {"status": "computing", "note": "still computing; call again in a few seconds"}
        )
        sample = arm.fresh_sample()
        seed = DEFAULT_SEED if sample is None else arm.measured(sample)
        return ArmReach(
            obstacles=checker.body.describe(),
            arm_mount={
                "base_in_base_link": mount.model_dump(),
                "pan_axis_base_link": result["pan_axis_base_link"] if result is not None else None,
                "note": "the arm base and shoulder link stand on the crossbar and are not checked against it",
            },
            margins={
                "enabled": settings.enabled,
                "soft_margin_m": settings.soft_margin_m,
                "hard_margin_m": settings.hard_margin_m,
                "slow_speed_scale": settings.slow_speed_scale,
            },
            envelope=env,
            points=[point_reach(checker, p.model_dump(), seed, envelope) for p in wanted],
        )
