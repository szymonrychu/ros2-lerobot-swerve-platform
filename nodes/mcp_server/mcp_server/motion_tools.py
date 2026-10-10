"""Motion queue tools: enqueue_motions, get_motion_status, cancel_motions, wait_for_event."""

from collections.abc import Callable
from typing import Annotated

from mcp.server.mcpserver.exceptions import ToolError
from pydantic import Field

from .config import McpServerConfig
from .grasp import grasp_params
from .grasp_tools import GraspExecutor
from .motion_queue import (
    CancelResult,
    EnqueueResult,
    GraspMotionStep,
    GraspRunner,
    MotionQueue,
    MotionStatus,
    MotionStep,
    QueueError,
    WaitResult,
    WaitUntil,
    state_digest,
    step_floor,
)
from .tool_context import RobotApi, ToolContext

TOOL_NAMES = ("enqueue_motions", "get_motion_status", "cancel_motions", "wait_for_event")


def make_grasp_runner(robot: RobotApi, config: McpServerConfig) -> GraspRunner:
    """Grasp step runner over GraspExecutor (plan + execute, same as grasp_object).

    Args:
        robot (RobotApi): Robot.
        config (McpServerConfig): Node configuration.

    Returns:
        GraspRunner: Callable returning (outcome, reasons).
    """
    executor = GraspExecutor(robot.arm, config)

    def run(step: GraspMotionStep, stop_requested: Callable[[], bool]) -> tuple[str, list[str]]:
        result = executor.grasp(
            step.object,
            step.strategy,
            grasp_params(config.grasp, step.params),
            step.approach_pitch_deg,
            step_floor(step),
            stop_requested,
        )
        return result.outcome, list(result.reasons)

    return run


def register(ctx: ToolContext) -> None:
    """Register the motion queue tools on the context's queue.

    Args:
        ctx (ToolContext): Shared registration context (ctx.queue must be set).
    """
    queue = ctx.queue
    if not isinstance(queue, MotionQueue):
        raise TypeError("the motion queue tools need ctx.queue (a MotionQueue)")
    cfg = ctx.config.motion_queue
    robot, guard = ctx.robot, ctx.guard

    @ctx.server.tool(
        description=(
            "Queue several motions and return AT ONCE (job ids, queue length): the robot keeps moving while you "
            "think. Steps run in order by one background executor with every safety guard of the blocking tools "
            "(lease, stop, stale/tracking/effort aborts, critical events, roll guard, floor slow zone). Kinds: "
            "arm_joints {targets}, arm_cartesian {x, y, z, pitch?, wrist_roll?, object_width_m?} (IK solved now: an "
            "unreachable target refuses the whole call, nothing queued), both with speed_scale?, settle?, "
            "surface_z_m?, tilt_override_deg?; gripper {open_fraction | close_until_effort, effort_threshold?, "
            "grip_profile? (gentle | normal | firm or inline overrides, as set_gripper)}; "
            "base_relative {dx, dy, dyaw, precise?, timeout_s?}; navigate_to_pose {x, y, yaw, frame?, precise?, "
            "timeout_s?}; wait_s {seconds}; grasp {object, strategy?, params? (params.grip_profile picks the grip), "
            "approach_pitch_deg?} (as grasp_object). "
            "BLENDING: consecutive arm steps run as ONE continuous trajectory through their targets (no stop between "
            "them); a step with settle='final', any non-arm step, a precondition, a different speed_scale or "
            "slow-zone override ends the blend (the arm comes to rest there). The result lists the blend groups. "
            "Every step may carry precondition {type: none | gripper_holding | gripper_open (min_fraction) | arm_near "
            "(joints, tol) | base_still | battery_ok}, checked on live state right before it starts, and on_fail "
            "('stop_queue' default: drop the rest; 'skip': continue). replace=true drops the pending steps first (a "
            "running step finishes). stop and cancel_motions clear the queue. While the queue runs, the blocking "
            "motion tools are refused (one motion owner); enqueue_motions is refused while a blocking motion tool "
            f"runs. At most {cfg.max_steps} pending steps. Follow with wait_for_event, not polling."
        )
    )
    def enqueue_motions(
        steps: Annotated[list[MotionStep], Field(min_length=1, description="Steps in execution order")],
        replace: Annotated[bool, Field(description="Drop the pending steps before adding these")] = False,
    ) -> EnqueueResult:
        """Queue motions; the tool description is passed to the decorator."""
        ctx.battery_gate("enqueue_motions")
        try:
            return queue.enqueue(list(steps), replace)
        except QueueError as exc:
            raise ToolError(str(exc)) from exc

    @ctx.server.tool()
    def get_motion_status() -> MotionStatus:
        """Motion queue snapshot (no motion): whether it runs, the current step (job id, kind, elapsed seconds,
        progress 0..1 when known, the job ids blended into the current motion), the pending steps and the last events.
        Prefer wait_for_event to polling this."""
        return queue.status()

    @ctx.server.tool()
    def cancel_motions() -> CancelResult:
        """Drop every pending queued step and abort the running one (the base is zeroed and the arm held at its
        measured pose when a motion step was running). Use it when the plan changed; stop is the emergency stop."""
        return queue.cancel()

    @ctx.server.tool(
        description=(
            "Block until something relevant happens in the motion queue, then return the new events, the queue "
            "status and a compact state digest (arm joints, gripper position/open fraction/effort, base pose, battery "
            "V, lease), so a separate get_robot_state is rarely needed. until: 'queue_empty' (default: returns when "
            "the queue drained or a step failed, a precondition failed, a contact happened or a stop/cancel ended a "
            "step; a failure skipped with on_fail=skip does not end it), 'step_done' (also after any single step finished), 'failure' (only failures), 'any' (any new "
            "event). Returns at once with reason 'idle' when the queue is empty. Each call returns only events not "
            f"returned before (since_seq overrides). Default timeout {cfg.wait_default_s:g} s, at most "
            f"{cfg.wait_max_s:g} s (then reason 'timeout' and the queue keeps running)."
        )
    )
    def wait_for_event(
        timeout_s: Annotated[float | None, Field(gt=0.0, description="Longest wait (s)")] = None,
        until: Annotated[WaitUntil, Field(description="What ends the wait")] = "queue_empty",
        since_seq: Annotated[int | None, Field(ge=0, description="Return events after this seq")] = None,
    ) -> WaitResult:
        """Wait for queue events; the tool description is passed to the decorator."""
        result = queue.wait(cfg.wait_default_s if timeout_s is None else timeout_s, until, since_seq)
        return result.model_copy(update={"state": state_digest(robot, guard)})
