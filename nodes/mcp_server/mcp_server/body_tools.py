"""Body awareness tools: get_body_state."""

from .models import BodyState
from .tool_context import ToolContext


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
            "(roll/pitch deg) and the last bump, wheel slip residual (m/s, null while parked), commanded versus "
            "measured base speed, CPU temperature and firmware throttling flag, the active arm source and whether "
            "this server holds the arm lease, and the last 10 robot events. Missing data is null with a reason in "
            f"`notes` (nothing is invented). Warnings: servo >= {cfg.servo_temp_warn_c:g} C (critical >= "
            f"{cfg.servo_temp_critical_c:g} C), CPU >= {cfg.cpu_temp_warn_c:g} C (critical >= "
            f"{cfg.cpu_temp_critical_c:g} C), tilt > {cfg.tilt_warn_deg:g} deg.{battery_note}"
        )
    )
    def get_body_state() -> BodyState:
        """Body vitals; the tool description is passed to the decorator so it can state the configured thresholds."""
        return monitor.body_state()
