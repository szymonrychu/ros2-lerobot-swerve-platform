"""Golden fixtures of stereo_depth_cpp, computed by the Python stereo_depth node's own functions.

Run inside the Python node's uv environment (it imports stereo_depth and its test doubles):

    cd nodes/stereo_depth && uv run --frozen python ../stereo_depth_cpp/test/gen_fixtures.py          # write
    cd nodes/stereo_depth && uv run --frozen python ../stereo_depth_cpp/test/gen_fixtures.py --check  # verify

It writes test/fixtures/:
- left.png, right.png: the shifted synthetic grey pair of nodes/stereo_depth/tests (conftest.shifted_pair).
- <case>/calibration/{stereo_left,stereo_right,stereo}.yaml: written by calibration.write_stereo_calibration.
- <case>/depth_mm.png (uint16, mm, 0 = invalid) and <case>/disparity_fp.png (StereoSGBM fixed point + 32768): the
  Python node's path on the pair sent as bgr8 Images: image_to_grey, rectify, compute_disparity, disparity_to_depth_mm.
- cases.yaml: per case fx, baseline, the depth limits and the valid pixel count.
- conversion.yaml: disparity_to_depth_mm on hand-made rows (NaN, inf, ranges, rounding, uint16 clamp).
- config.yaml: DepthConfig defaults, every-key and accepted edge cases with their parsed values, and every text the
  pydantic model rejects (each one checked here to raise ConfigError).
- calibration_invalid.yaml: calibration file sets that load_stereo_calibration rejects (each one checked here).

--check regenerates into a temporary directory and exits 1 naming every stale, missing or extra fixture.
"""

import argparse
import filecmp
import math
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
STEREO_DEPTH_DIR = HERE.parent.parent / "stereo_depth"
sys.path.insert(0, str(STEREO_DEPTH_DIR))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import tests.conftest as conftest  # noqa: E402  (installs the ROS message doubles before stereo_depth.messages)
import yaml  # noqa: E402
from stereo_depth.calibration import (  # noqa: E402
    CALIBRATION_FILES,
    StereoCalibration,
    load_stereo_calibration,
    rectification_maps,
    rectify,
    write_stereo_calibration,
)
from stereo_depth.config import ConfigError, DepthConfig, load_config  # noqa: E402
from stereo_depth.depth import MIN_DISPARITY, compute_disparity, create_matcher, disparity_to_depth_mm  # noqa: E402
from stereo_depth.messages import image_to_grey  # noqa: E402

FIXTURES_DIR = HERE / "fixtures"
DISPARITY_PX = 20
FIXED_POINT_OFFSET = 32768
DISPARITY_SCALE = 16.0
INPUT_ENCODING = "bgr8"
DISTORTED_FX_SCALE = 1.02
DISTORTED_CX_SHIFT = 3.0
DISTORTED_D = [-0.08, 0.02, 0.0005, -0.0003, 0.0]
DISTORTED_ROT_DEG = 0.3
PNG_SUFFIX = ".png"

# disparity_to_depth_mm rows: (disparity row, f, t, min_disparity, min_depth_m, max_depth_m), as in tests/test_depth.py.
CONVERSION_CASES: list[tuple[list[float], float, float, float, float, float]] = [
    ([40.0, 20.0, 10.0], 400.0, 0.1, 0.0, 0.2, 4.0),
    ([30.0], 400.0, 0.1, 0.0, 0.2, 4.0),
    ([0.0, -1.0, 5.0, 40.0], 400.0, 0.1, 8.0, 0.2, 4.0),
    ([math.nan, math.inf, -math.inf, 40.0], 400.0, 0.1, 0.0, 0.2, 4.0),
    ([400.0, 200.0, 9.0, 10.0], 400.0, 0.1, 0.0, 0.2, 4.0),
    ([0.5], 400.0, 0.1, 0.0, 0.2, 1000.0),
    ([16.0, 16.0625, 17.5, 63.9375, -0.0625], 400.0, 0.08, 0.0, 0.3, 4.0),
]

EVERY_KEY_TEXT = (
    "left_image_topic: /l\nright_image_topic: /r\ndepth_topic: /d\ncamera_info_topic: /i\n"
    "publish_disparity: true\ndisparity_topic: /disp\ncalibration_dir: /tmp/cal\ncalibration_check_s: 2.0\n"
    "num_disparities: 64\nblock_size: 7\np1: 100\np2: 400\nuniqueness_ratio: 5\nspeckle_window_size: 50\n"
    "speckle_range: 1\ndisp12_max_diff: 2\npre_filter_cap: 63\nmin_depth_m: 0.5\nmax_depth_m: 3.0\n"
    "sync_slop_s: 0.005\nmax_rate_hz: 2.5\nmatcher_threads: 1\nframe_id: ' f '\nmetrics_port: 19999\n"
)

# Accepted at the bounds (each parsed value is recorded for the C++ test to compare).
ACCEPTED_TEXTS = [
    "",
    EVERY_KEY_TEXT,
    "metrics_port: null\n",
    "metrics_port: 1\n",
    "metrics_port: 65535\n",
    "block_size: 1\n",
    "block_size: 11\n",
    "pre_filter_cap: 1\n",
    "disp12_max_diff: -1\n",
    "uniqueness_ratio: 0\nspeckle_window_size: 0\nspeckle_range: 0\n",
    "sync_slop_s: 0.1\n",
    "max_rate_hz: 30\n",
    "matcher_threads: 4\n",
    "num_disparities: 16\n",
    "p1: 1\np2: 2\n",
    "min_depth_m: 0.1\nmax_depth_m: 0.2\n",
    "left_image_topic: '  /padded  '\n",
]

# Every rejection of tests/test_config.py, plus one per remaining field bound, type and relation.
REJECTED_TEXTS = [
    "unknown_key: 1\n",
    "num_disparities: 40\n",
    "num_disparities: 0\n",
    "block_size: 4\n",
    "block_size: 0\n",
    "block_size: 13\n",
    "p1: 800\np2: 200\n",
    "p1: 0\n",
    "uniqueness_ratio: -1\n",
    "speckle_window_size: -1\n",
    "speckle_range: -1\n",
    "disp12_max_diff: -2\n",
    "pre_filter_cap: 0\n",
    "pre_filter_cap: 64\n",
    "min_depth_m: 2.0\nmax_depth_m: 1.0\n",
    "min_depth_m: 0\n",
    "sync_slop_s: 0\n",
    "sync_slop_s: 0.5\n",
    "max_rate_hz: 0\n",
    "max_rate_hz: 31\n",
    "matcher_threads: 0\n",
    "calibration_check_s: 0\n",
    "frame_id: '  '\n",
    "left_image_topic: ''\n",
    "metrics_port: 0\n",
    "metrics_port: 70000\n",
    "right_image_topic: ''\n",
    "depth_topic: ' '\n",
    "camera_info_topic: ''\n",
    "disparity_topic: ''\n",
    "num_disparities: -16\n",
    "p2: 0\n",
    "p1: 400\np2: 400\n",
    "max_depth_m: 0\n",
    "min_depth_m: 1.0\nmax_depth_m: 1.0\n",
    "matcher_threads: 5\n",
    "calibration_check_s: -1\n",
    "num_disparities: abc\n",
    "block_size: 5.5\n",
    "min_depth_m: abc\n",
    "publish_disparity: maybe\n",
    "left_image_topic: null\n",
    "frame_id: null\n",
    "calibration_dir: null\n",
    "num_disparities: null\n",
    "metrics_port: abc\n",
    "left_image_topic: [a]\n",
    "max_rate_hz: [1]\n",
    "extra_one: 1\nextra_two: 2\n",
    "- 1\n",
    "a: [1\n",
]


def textured_pair() -> tuple[np.ndarray, np.ndarray]:
    """The shifted synthetic pair of the Python tests.

    Returns:
        tuple[np.ndarray, np.ndarray]: (left, right) uint8 grey images, conftest HEIGHT x WIDTH.
    """
    return conftest.shifted_pair(DISPARITY_PX)


def rot_y(deg: float) -> list[float]:
    """A row-major 3x3 rotation about y.

    Args:
        deg (float): Angle in degrees.

    Returns:
        list[float]: 9 values.
    """
    a = math.radians(deg)
    return [math.cos(a), 0.0, math.sin(a), 0.0, 1.0, 0.0, -math.sin(a), 0.0, math.cos(a)]


def distorted_calibration() -> StereoCalibration:
    """The ideal pair with lens distortion, a different raw K and small rectification rotations, so remap does work.

    Returns:
        StereoCalibration: A valid pair (P and the baseline unchanged).
    """
    ideal = conftest.ideal_calibration()
    k = [
        conftest.FX * DISTORTED_FX_SCALE,
        0.0,
        conftest.CX + DISTORTED_CX_SHIFT,
        0.0,
        conftest.FX * DISTORTED_FX_SCALE,
        conftest.CY - DISTORTED_CX_SHIFT,
        0.0,
        0.0,
        1.0,
    ]
    left = ideal.left.model_copy(update={"k": k, "d": DISTORTED_D, "r": rot_y(DISTORTED_ROT_DEG)})
    right = ideal.right.model_copy(update={"k": k, "d": DISTORTED_D, "r": rot_y(-DISTORTED_ROT_DEG)})
    return StereoCalibration(left=left, right=right, extrinsics=ideal.extrinsics)


def python_depth(calibration: StereoCalibration, config: DepthConfig) -> tuple[np.ndarray, np.ndarray]:
    """The Python node's per-pair path (StereoDepthNode.on_pair after the gates, and match).

    Args:
        calibration (StereoCalibration): The pair.
        config (DepthConfig): Matcher and depth limits.

    Returns:
        tuple[np.ndarray, np.ndarray]: uint16 depth in mm, int16 StereoSGBM fixed-point disparity.
    """
    left, right = textured_pair()
    left_grey = image_to_grey(conftest.image_msg(left, INPUT_ENCODING))
    right_grey = image_to_grey(conftest.image_msg(right, INPUT_ENCODING))
    assert left_grey is not None and right_grey is not None
    cv2.setNumThreads(config.matcher_threads)
    rect_left, rect_right = rectify(rectification_maps(calibration), left_grey, right_grey)
    disparity = compute_disparity(create_matcher(config), rect_left, rect_right)
    depth = disparity_to_depth_mm(
        disparity, calibration.fx, calibration.baseline_m, float(MIN_DISPARITY), config.min_depth_m, config.max_depth_m
    )
    fixed = np.rint(disparity.astype(np.float64) * DISPARITY_SCALE).astype(np.int32)
    return depth, fixed


def write_png(path: Path, image: np.ndarray) -> None:
    """Write a lossless PNG (8- or 16-bit).

    Args:
        path (Path): Destination.
        image (np.ndarray): uint8 or uint16 image.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise OSError(f"cannot write {path}")


def write_yaml(path: Path, data: Any) -> None:
    """Write YAML in a stable order.

    Args:
        path (Path): Destination.
        data (Any): Data.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False, width=120))


def config_value(config: DepthConfig) -> dict[str, Any]:
    """Every DepthConfig field as plain YAML values.

    Args:
        config (DepthConfig): Validated config.

    Returns:
        dict[str, Any]: Field name to value (calibration_dir as a string).
    """
    return config.model_dump(mode="json")


def parse_config(directory: Path, text: str) -> DepthConfig:
    """Load a config text through the Python load_config.

    Args:
        directory (Path): Scratch directory.
        text (str): YAML text.

    Returns:
        DepthConfig: The validated config.
    """
    path = directory / "config.yaml"
    path.write_text(text)
    return load_config(path)


def config_fixture(scratch: Path) -> dict[str, Any]:
    """Defaults, accepted texts with their values and rejected texts, each checked against the Python model.

    Args:
        scratch (Path): Scratch directory.

    Returns:
        dict[str, Any]: The config.yaml content.
    """
    accepted = [{"text": text, "value": config_value(parse_config(scratch, text))} for text in ACCEPTED_TEXTS]
    for text in REJECTED_TEXTS:
        try:
            parse_config(scratch, text)
        except ConfigError:
            continue
        raise AssertionError(f"the Python config accepts a text listed as rejected: {text!r}")
    return {"defaults": config_value(DepthConfig()), "accepted": accepted, "rejected": REJECTED_TEXTS}


def conversion_fixture() -> list[dict[str, Any]]:
    """disparity_to_depth_mm on the hand-made rows.

    Returns:
        list[dict[str, Any]]: One entry per row, with the uint16 output.
    """
    rows = []
    for disparity, f, t, min_disparity, min_depth_m, max_depth_m in CONVERSION_CASES:
        out = disparity_to_depth_mm(
            np.array([disparity], dtype=np.float32), f, t, min_disparity, min_depth_m, max_depth_m
        )
        rows.append(
            {
                "disparity": [float(v) for v in disparity],
                "f": f,
                "t": t,
                "min_disparity": min_disparity,
                "min_depth_m": min_depth_m,
                "max_depth_m": max_depth_m,
                "depth_mm": [int(v) for v in out[0]],
            }
        )
    return rows


def calibration_files(calibration: StereoCalibration, scratch: Path) -> dict[str, Any]:
    """The three calibration files of a pair as parsed YAML.

    Args:
        calibration (StereoCalibration): The pair.
        scratch (Path): Scratch directory.

    Returns:
        dict[str, Any]: File name to mapping.
    """
    directory = scratch / "calibration_template"
    write_stereo_calibration(directory, calibration)
    return {name: yaml.safe_load((directory / name).read_text()) for name in CALIBRATION_FILES}


def invalid_calibrations(scratch: Path) -> list[dict[str, Any]]:
    """Calibration file sets the Python loader rejects (each checked), as {name, files} with files to write.

    A file mapped to None is left out (missing); a string is written verbatim.

    Args:
        scratch (Path): Scratch directory.

    Returns:
        list[dict[str, Any]]: The cases.
    """
    good = calibration_files(conftest.ideal_calibration(), scratch)
    left, right, stereo = (good[name] for name in CALIBRATION_FILES)

    def edit(data: dict[str, Any], path: list[str], value: Any) -> dict[str, Any]:
        copy = yaml.safe_load(yaml.safe_dump(data))
        target = copy
        for key in path[:-1]:
            target = target[key]
        if value is None:
            del target[path[-1]]
        else:
            target[path[-1]] = value
        return copy

    def p_with(data: dict[str, Any], index: int, value: float) -> dict[str, Any]:
        p = list(data["projection_matrix"]["data"])
        p[index] = value
        return edit(data, ["projection_matrix", "data"], p)

    cases: list[tuple[str, dict[str, Any]]] = [
        ("missing_left", {"stereo_left.yaml": None}),
        ("missing_right", {"stereo_right.yaml": None}),
        ("missing_stereo", {"stereo.yaml": None}),
        ("left_not_yaml", {"stereo_left.yaml": "a: [1\n"}),
        ("left_not_mapping", {"stereo_left.yaml": "- 1\n"}),
        ("stereo_not_mapping", {"stereo.yaml": "just text\n"}),
        ("left_width_zero", {"stereo_left.yaml": edit(left, ["image_width"], 0)}),
        ("left_height_missing", {"stereo_left.yaml": edit(left, ["image_height"], None)}),
        ("left_bad_distortion_model", {"stereo_left.yaml": edit(left, ["distortion_model"], "fisheye")}),
        ("left_distortion_model_missing", {"stereo_left.yaml": edit(left, ["distortion_model"], None)}),
        (
            "left_wrong_coefficient_count",
            # A consistent 1x4 entry, so only the plumb_bob coefficient count rejects it (not the shape check).
            {"stereo_left.yaml": edit(left, ["distortion_coefficients"], {"rows": 1, "cols": 4, "data": [0.0] * 4})},
        ),
        ("left_k_missing", {"stereo_left.yaml": edit(left, ["camera_matrix"], None)}),
        ("left_k_wrong_shape", {"stereo_left.yaml": edit(left, ["camera_matrix", "rows"], 2)}),
        ("left_k_data_count", {"stereo_left.yaml": edit(left, ["camera_matrix", "data"], [1.0] * 8)}),
        ("left_k_focal_zero", {"stereo_left.yaml": edit(left, ["camera_matrix", "data"], [0.0] * 9)}),
        ("left_p_focal_negative", {"stereo_left.yaml": p_with(left, 5, -1.0)}),
        ("left_p_nan", {"stereo_left.yaml": p_with(left, 2, math.nan)}),
        ("left_r_not_list", {"stereo_left.yaml": edit(left, ["rectification_matrix", "data"], "abc")}),
        ("stereo_t_wrong_shape", {"stereo.yaml": edit(stereo, ["T", "rows"], 1)}),
        ("stereo_q_missing", {"stereo.yaml": edit(stereo, ["Q"], None)}),
        ("stereo_rms_negative", {"stereo.yaml": edit(stereo, ["rms"], -0.1)}),
        ("stereo_rms_inf", {"stereo.yaml": edit(stereo, ["rms"], math.inf)}),
        ("stereo_rms_missing", {"stereo.yaml": edit(stereo, ["rms"], None)}),
        ("stereo_width_differs", {"stereo.yaml": edit(stereo, ["image_width"], 640)}),
        ("right_height_differs", {"stereo_right.yaml": edit(right, ["image_height"], 480)}),
        ("right_fx_differs", {"stereo_right.yaml": p_with(right, 0, 401.0)}),
        ("right_cy_differs", {"stereo_right.yaml": p_with(right, 6, 181.0)}),
        ("left_tx_nonzero", {"stereo_left.yaml": p_with(left, 3, -1.0)}),
        ("right_tx_positive", {"stereo_right.yaml": p_with(right, 3, 32.0)}),
        ("baseline_mismatch", {"stereo.yaml": edit(stereo, ["T", "data"], [-0.1, 0.0, 0.0])}),
    ]
    out = []
    for name, overrides in cases:
        files = {file_name: good[file_name] for file_name in CALIBRATION_FILES}
        files.update(overrides)
        directory = scratch / "invalid" / name
        directory.mkdir(parents=True)
        for file_name, content in files.items():
            if content is None:
                continue
            text = content if isinstance(content, str) else yaml.safe_dump(content, sort_keys=False)
            (directory / file_name).write_text(text)
        loaded = load_stereo_calibration(directory)
        if loaded.calibration is not None:
            raise AssertionError(f"the Python loader accepts the invalid calibration case {name}")
        out.append({"name": name, "files": overrides, "python_reason": loaded.reason.replace(str(directory), "<dir>")})
    return out


def generate(out_dir: Path) -> None:
    """Write every fixture into out_dir.

    Args:
        out_dir (Path): Fixture directory (created).
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    config = DepthConfig()
    left, right = textured_pair()
    write_png(out_dir / "left.png", left)
    write_png(out_dir / "right.png", right)
    cases = {}
    for name, calibration in (("ideal", conftest.ideal_calibration()), ("distorted", distorted_calibration())):
        write_stereo_calibration(out_dir / name / "calibration", calibration)
        depth, fixed = python_depth(calibration, config)
        write_png(out_dir / name / "depth_mm.png", depth)
        write_png(out_dir / name / "disparity_fp.png", (fixed + FIXED_POINT_OFFSET).astype(np.uint16))
        cases[name] = {
            "fx": calibration.fx,
            "baseline_m": calibration.baseline_m,
            "min_depth_m": config.min_depth_m,
            "max_depth_m": config.max_depth_m,
            "valid_pixels": int(np.count_nonzero(depth)),
        }
    write_yaml(
        out_dir / "cases.yaml",
        {
            "encoding": INPUT_ENCODING,
            "disparity_px": DISPARITY_PX,
            "fixed_point_offset": FIXED_POINT_OFFSET,
            "opencv_version": cv2.__version__,
            "cases": cases,
        },
    )
    write_yaml(out_dir / "conversion.yaml", conversion_fixture())
    with tempfile.TemporaryDirectory() as scratch:
        write_yaml(out_dir / "config.yaml", config_fixture(Path(scratch)))
        write_yaml(out_dir / "calibration_invalid.yaml", invalid_calibrations(Path(scratch)))


def same_file(a: Path, b: Path) -> bool:
    """Compare two fixtures: PNGs by decoded pixels (the encoder may differ), everything else byte for byte.

    Args:
        a (Path): Committed file.
        b (Path): Regenerated file.

    Returns:
        bool: True when equal.
    """
    if a.suffix == PNG_SUFFIX:
        img_a = cv2.imread(str(a), cv2.IMREAD_UNCHANGED)
        img_b = cv2.imread(str(b), cv2.IMREAD_UNCHANGED)
        return img_a is not None and img_b is not None and img_a.dtype == img_b.dtype and np.array_equal(img_a, img_b)
    return filecmp.cmp(a, b, shallow=False)


def stale_fixtures(committed: Path, fresh: Path) -> list[str]:
    """List fixtures that differ between the committed and the regenerated tree.

    Args:
        committed (Path): test/fixtures.
        fresh (Path): Regenerated directory.

    Returns:
        list[str]: "stale: x", "missing: x" (not committed) and "extra: x" (no longer generated) entries.
    """
    old = (
        {p.relative_to(committed).as_posix() for p in committed.rglob("*") if p.is_file()}
        if committed.is_dir()
        else set()
    )
    new = {p.relative_to(fresh).as_posix() for p in fresh.rglob("*") if p.is_file()}
    problems = [f"missing: {name}" for name in sorted(new - old)]
    problems += [f"extra: {name}" for name in sorted(old - new)]
    problems += [f"stale: {name}" for name in sorted(old & new) if not same_file(committed / name, fresh / name)]
    return problems


def main() -> int:
    """Write the fixtures, or with --check verify the committed ones.

    Returns:
        int: 0 on success, 1 when --check finds stale fixtures.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="fail when the committed fixtures are stale")
    args = parser.parse_args()
    if not args.check:
        shutil.rmtree(FIXTURES_DIR, ignore_errors=True)
        generate(FIXTURES_DIR)
        print(f"gen_fixtures: wrote {FIXTURES_DIR}")
        return 0
    with tempfile.TemporaryDirectory() as tmp:
        fresh = Path(tmp) / "fixtures"
        generate(fresh)
        problems = stale_fixtures(FIXTURES_DIR, fresh)
    if problems:
        print("gen_fixtures --check: fixtures are stale; rerun gen_fixtures.py:", file=sys.stderr)
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        return 1
    print("gen_fixtures --check: fixtures are up to date")
    return 0


if __name__ == "__main__":
    sys.exit(main())
