"""grasp-sim command line: replay plans, export the example scoops, capture frames, open the viewer."""

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco
import yaml

from grasp_sim.adapter import grasp_plan_to_replay
from grasp_sim.config import SceneConfig, SimConfig
from grasp_sim.examples import example_case
from grasp_sim.matrix import DEFAULT_RESULTS_FILE, run_matrix, summary_table
from grasp_sim.render import DEFAULT_FPS, DEFAULT_SIZE, FrameSaver, ViewerObserver
from grasp_sim.replay import Observer, simulate
from grasp_sim.report import SimReport

EXAMPLE_NAMES = ("floor", "ledge", "stair", "tip")
MJPYTHON_HINT = "The viewer needs the mjpython launcher on macOS: uv run mjpython -m grasp_sim.cli run ... --render"


def load_mapping(path: Path) -> dict[str, Any]:
    """Load a JSON or YAML mapping from disk.

    Args:
        path (Path): .json, .yaml or .yml file.

    Returns:
        dict[str, Any]: Parsed mapping.
    """
    text = path.read_text()
    return json.loads(text) if path.suffix == ".json" else yaml.safe_load(text)


def parse_size(text: str) -> tuple[int, int]:
    """Parse WIDTHxHEIGHT.

    Args:
        text (str): e.g. "640x480".

    Returns:
        tuple[int, int]: (width, height) in px.
    """
    width, height = text.lower().split("x")
    return int(width), int(height)


def format_summary(report: SimReport) -> str:
    """Short human-readable verdict with the key numbers.

    Args:
        report (SimReport): Replay result.

    Returns:
        str: Multi-line summary.
    """
    lines = [("PASS" if report.passed else "FAIL") + f" ({report.duration_s:.1f}s simulated)"]
    lines += [f"  reason: {r}" for r in report.reasons]
    lines += [f"  warning: {w}" for w in report.warnings]
    if report.object is not None:
        o = report.object
        lines.append(
            f"  object: tilt {o.approach_max_tilt_deg:.1f} deg, pushed {o.approach_max_displacement_m * 1000:.1f} mm "
            f"in approach, lifted {o.lift_height_m * 1000:.0f} mm, grasp_success={report.grasp_success}"
        )
    for s in report.segments:
        c = s.min_clearance
        lines.append(
            f"  {s.label or 'unlabelled':<10} jaws-floor {fmt_mm(c.jaws_floor)} jaws-support {fmt_mm(c.jaws_support)} "
            f"wrist-floor {fmt_mm(c.wrist_floor)} max torque {s.max_actuator_force_nm:.2f} Nm"
        )
    return "\n".join(lines)


def fmt_mm(value: float | None) -> str:
    """Format a clearance in millimetres (or n/a)."""
    return "n/a" if value is None else f"{value * 1000:.1f}mm"


def fan_out(observers: list[Observer]) -> Observer:
    """Combine several simulate() observers into one.

    Args:
        observers (list[Observer]): Observers to call in order after every step.

    Returns:
        Observer: Observer that forwards to all of them.
    """

    def observer(model: mujoco.MjModel, data: mujoco.MjData, t: float, label: str | None) -> None:
        for each in observers:
            each(model, data, t, label)

    return observer


def run_command(args: argparse.Namespace) -> int:
    """Replay a plan file and print the verdict.

    Args:
        args (argparse.Namespace): Parsed arguments of the run subcommand.

    Returns:
        int: 0 if the plan passed, 1 otherwise.
    """
    scene = SceneConfig.model_validate(load_mapping(args.scene)) if args.scene else SceneConfig()
    sim_cfg = SimConfig.model_validate(load_mapping(args.sim)) if args.sim else SimConfig()
    plan: Any = grasp_plan_to_replay(args.plan) if args.grasp_plan else args.plan
    saver = FrameSaver(args.frames, args.gif, args.fps, parse_size(args.size)) if args.frames or args.gif else None
    viewer = ViewerObserver() if args.render else None
    observers = [o for o in (saver, viewer) if o is not None]
    observer = fan_out(observers) if observers else None
    try:
        report = simulate(plan, scene, sim_cfg, observer)
    except RuntimeError as exc:
        if viewer is not None:
            raise SystemExit(f"{exc}\n{MJPYTHON_HINT}") from exc
        raise
    finally:
        if saver is not None:
            saver.close()
        if viewer is not None:
            viewer.close()
    if args.report:
        args.report.write_text(report.model_dump_json(indent=2))
    print(report.model_dump_json(indent=2) if args.json else format_summary(report))
    return 0 if report.passed else 1


def example_command(args: argparse.Namespace) -> int:
    """Write an example scene and plan to disk.

    Args:
        args (argparse.Namespace): Parsed arguments of the example subcommand.

    Returns:
        int: Always 0.
    """
    scene, plan = example_case(args.name)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / f"{args.name}.plan.json").write_text(json.dumps(plan, indent=1))
    (args.out / f"{args.name}.scene.json").write_text(scene.model_dump_json(indent=1))
    print(f"wrote {args.out / (args.name + '.plan.json')} and {args.out / (args.name + '.scene.json')}")
    return 0


def matrix_command(args: argparse.Namespace) -> int:
    """Replay a planner scenario matrix (scripts/plan_matrix.py output) and print the summary table.

    Args:
        args (argparse.Namespace): Parsed arguments of the matrix subcommand.

    Returns:
        int: Always 0 (the table is the result; failing scenarios are data, not errors).
    """
    results = run_matrix(args.dir, args.workers, args.stock_jaws)
    out = args.out or args.dir / DEFAULT_RESULTS_FILE
    out.write_text(json.dumps(results, indent=1))
    print(summary_table(results))
    print(f"wrote {out}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    """Build the argument parser.

    Returns:
        argparse.ArgumentParser: Parser with the run, example and matrix subcommands.
    """
    parser = argparse.ArgumentParser(prog="grasp-sim", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="replay a plan")
    run.add_argument("plan", type=Path, help="replay plan JSON (or a GraspPlan JSON with --grasp-plan)")
    run.add_argument("--grasp-plan", action="store_true", help="convert a mcp_server GraspPlan JSON first")
    run.add_argument("--scene", type=Path, help="scene config (.json/.yaml), see SceneConfig")
    run.add_argument("--sim", type=Path, help="thresholds config (.json/.yaml), see SimConfig")
    run.add_argument("--report", type=Path, help="write the full SimReport JSON here")
    run.add_argument("--json", action="store_true", help="print the full report JSON instead of the summary")
    run.add_argument("--render", action="store_true", help="open the interactive viewer (macOS: use mjpython)")
    run.add_argument("--frames", type=Path, help="write PNG frames into this directory")
    run.add_argument("--gif", type=Path, help="write an animated GIF")
    run.add_argument("--fps", type=float, default=DEFAULT_FPS, help="frame capture rate")
    run.add_argument("--size", default=f"{DEFAULT_SIZE[0]}x{DEFAULT_SIZE[1]}", help="frame size WIDTHxHEIGHT")
    run.set_defaults(func=run_command)
    example = sub.add_parser("example", help="write an example scene and plan")
    example.add_argument("name", choices=EXAMPLE_NAMES)
    example.add_argument("--out", type=Path, default=Path("examples"))
    example.set_defaults(func=example_command)
    matrix = sub.add_parser("matrix", help="replay a planner scenario matrix made by scripts/plan_matrix.py")
    matrix.add_argument("dir", type=Path, help="directory with index.json and the GraspPlan files")
    matrix.add_argument("--out", type=Path, help=f"results JSON (default <dir>/{DEFAULT_RESULTS_FILE})")
    matrix.add_argument("--workers", type=int, default=6, help="parallel processes")
    matrix.add_argument("--stock-jaws", action="store_true", help="replay with the uncalibrated Menagerie jaws")
    matrix.set_defaults(func=matrix_command)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Entry point.

    Args:
        argv (list[str] | None): Arguments (defaults to sys.argv).

    Returns:
        int: Process exit code.
    """
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
