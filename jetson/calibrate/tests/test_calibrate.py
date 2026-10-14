"""Tests of the stereo ChArUco calibration tool on synthetic boards (virtual stereo rig, f 400 px, 640x480, 60 mm)."""

import hashlib
import http.server
import threading
from pathlib import Path

import cv2
import numpy as np
import pytest

from stereo_calibrate import calibrate as cal
from stereo_calibrate.config import CalibrateConfig, load_config

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = ROOT / "jetson/depthd/test/fixtures/stereo_valid.yaml"
REFERENCE_DIR = ROOT / "docs/plans/2026-10-14-jetson-depth-offload-briefs/charuco"
CALIBRATE_YAML = Path(__file__).resolve().parents[1] / "config/calibrate.yaml"
# sha256 of the raw pixels the board command renders; the PNGs under REFERENCE_DIR are exactly this output.
PINNED_BOARDS = {
    (1072, 1448): "4b58307862002fae9f661ebaf2cf9a65f96fcab700216722af8d92b71d7c97a1",
    (1236, 1648): "5f8ecb5e31409bedc52bc354957b4429f172c7dab2455bc85576a4c186479fee",
    (1264, 1680): "e26dc1d350390023500cda6560de91c15757c3b11a2f3308232c518ba09815aa",
}
WIDTH, HEIGHT, FOCAL, BASELINE = 640, 480, 400.0, 0.06
RASTER_PX_PER_MM = 6


def make_config(tmp_path: Path, **overrides) -> CalibrateConfig:
    data = {"captures_dir": tmp_path / "captures", "output_path": tmp_path / "stereo.yaml"}
    data.update(overrides)
    return CalibrateConfig(**data)


def rotation(yaw: float, pitch: float, roll: float) -> np.ndarray:
    rvec = np.array([pitch, yaw, roll], dtype=np.float64)
    return cv2.Rodrigues(rvec)[0]


def board_raster(config: CalibrateConfig) -> np.ndarray:
    board = cal.charuco_board(config)
    size = (
        int(config.squares_x * config.square_mm * RASTER_PX_PER_MM),
        int(config.squares_y * config.square_mm * RASTER_PX_PER_MM),
    )
    return board.generateImage(size, marginSize=0, borderBits=1)


def render_pair(raster: np.ndarray, config: CalibrateConfig, rot: np.ndarray, centre: np.ndarray):
    """Render the board (plane z=0, origin at its centre, metres) seen by the left and the right virtual camera."""
    k = np.array([[FOCAL, 0, WIDTH / 2], [0, FOCAL, HEIGHT / 2], [0, 0, 1]])
    raster_to_board = np.array(
        [
            [1 / (RASTER_PX_PER_MM * 1000.0), 0, -config.squares_x * config.square_mm / 2000.0],
            [0, 1 / (RASTER_PX_PER_MM * 1000.0), -config.squares_y * config.square_mm / 2000.0],
            [0, 0, 1],
        ]
    )
    images = []
    for shift in (0.0, BASELINE):
        t = centre - np.array([shift, 0.0, 0.0])
        homography = k @ np.column_stack([rot[:, 0], rot[:, 1], t]) @ raster_to_board
        image = cv2.warpPerspective(raster, homography, (WIDTH, HEIGHT), flags=cv2.INTER_AREA, borderValue=255)
        images.append(cv2.GaussianBlur(image, (3, 3), 0.6))
    return images[0], images[1]


def poses(count: int, seed: int = 7):
    rng = np.random.RandomState(seed)
    for _ in range(count):
        rot = rotation(
            np.radians(rng.uniform(-28, 28)), np.radians(rng.uniform(-28, 28)), np.radians(rng.uniform(-18, 18))
        )
        centre = np.array([rng.uniform(-0.05, 0.03), rng.uniform(-0.03, 0.03), rng.uniform(0.27, 0.36)])
        yield rot, centre


@pytest.fixture(scope="module")
def rig_pairs():
    config = CalibrateConfig()
    raster = board_raster(config)
    return [render_pair(raster, config, rot, centre) for rot, centre in poses(25)]


def test_solve_synthetic_rig(rig_pairs, tmp_path):
    config = make_config(tmp_path)
    result = cal.solve(rig_pairs, config)
    assert abs(result.baseline_m - BASELINE) / BASELINE < 0.01
    assert result.rms < 0.5
    assert abs(result.k_left[0, 0] - FOCAL) / FOCAL < 0.02
    path = cal.write_calibration(result, config)
    assert path == config.output_path
    assert not list(tmp_path.glob("*.tmp"))


def read_matrices(path: Path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    keys = {}
    for key in fs.root().keys():
        node = fs.getNode(key)
        keys[key] = node.mat().shape if node.isMap() else ("int" if node.isInt() else "real")
    fs.release()
    return keys


def test_written_file_matches_the_depthd_fixture_contract(rig_pairs, tmp_path):
    config = make_config(tmp_path)
    path = cal.write_calibration(cal.solve(rig_pairs, config), config)
    written, fixture = read_matrices(path), read_matrices(FIXTURE)
    assert set(written) == set(fixture)
    for key, shape in fixture.items():
        assert written[key] == shape, key
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    assert fs.getNode("T").mat()[0, 0] < 0
    assert abs(fs.getNode("P2").mat()[0, 3] + fs.getNode("P1").mat()[0, 0] * fs.getNode("baseline_m").real()) < 1e-6
    assert int(fs.getNode("image_width").real()) == WIDTH
    fs.release()


def test_rms_gate_refuses_a_noisy_set(rig_pairs, tmp_path):
    config = make_config(tmp_path)
    rng = np.random.RandomState(3)
    noisy = [(left, right) for left, right in rig_pairs]
    shared = [cal.shared_points(left, right, config) for left, right in noisy]
    objects = [s[0] for s in shared]
    left_pts = [s[1] + rng.normal(0, 1.5, s[1].shape).astype(np.float32) for s in shared]
    right_pts = [s[2] + rng.normal(0, 1.5, s[2].shape).astype(np.float32) for s in shared]
    result = cal.solve_points(objects, left_pts, right_pts, [s[3] for s in shared], (WIDTH, HEIGHT))
    assert result.rms > 0.5
    with pytest.raises(cal.RmsGateError):
        cal.write_calibration(result, config)
    assert not config.output_path.exists()


def test_too_few_pairs_fail(rig_pairs, tmp_path):
    with pytest.raises(cal.CalibrationError, match="min_pairs"):
        cal.solve(rig_pairs[:3], make_config(tmp_path))


def test_shared_points_need_twelve_corners(rig_pairs, tmp_path):
    config = make_config(tmp_path)
    assert cal.shared_points(*rig_pairs[0], config) is not None
    blank = np.full((HEIGHT, WIDTH), 255, np.uint8)
    assert cal.shared_points(rig_pairs[0][0], blank, config) is None
    strict = make_config(tmp_path, min_shared_corners=25)
    assert cal.shared_points(*rig_pairs[0], strict) is None


def test_pose_filter_rejects_duplicates_and_keeps_distinct_poses(rig_pairs, tmp_path):
    config = make_config(tmp_path)
    shared = [cal.shared_points(*p, config) for p in rig_pairs[:8]]
    pose_filter = cal.PoseFilter(config)
    assert pose_filter.accept(shared[0][1], shared[0][3], (WIDTH, HEIGHT))
    assert not pose_filter.accept(shared[0][1], shared[0][3], (WIDTH, HEIGHT))
    assert not pose_filter.accept(shared[0][1] + 0.5, shared[0][3], (WIDTH, HEIGHT))
    accepted = sum(pose_filter.accept(s[1], s[3], (WIDTH, HEIGHT)) for s in shared[1:])
    assert 3 <= accepted < 7  # some random poses are close to a kept one
    assert len(pose_filter) == 1 + accepted


def multipart(left: np.ndarray, right: np.ndarray) -> (bytes, str):
    boundary = "depthd-capture-boundary-7f3a9c"
    body = b""
    for name, data, ctype, filename in (
        ("left", cv2.imencode(".png", left)[1].tobytes(), "image/png", "left.png"),
        ("right", cv2.imencode(".png", right)[1].tobytes(), "image/png", "right.png"),
        ("left_capture_realtime_ns", b"1", "text/plain", ""),
        ("right_capture_realtime_ns", b"2", "text/plain", ""),
    ):
        disposition = 'form-data; name="%s"' % name + ('; filename="%s"' % filename if filename else "")
        body += (
            "--%s\r\nContent-Disposition: %s\r\nContent-Type: %s\r\n\r\n" % (boundary, disposition, ctype)
        ).encode()
        body += data + b"\r\n"
    body += ("--%s--\r\n" % boundary).encode()
    return body, "multipart/form-data; boundary=" + boundary


class FakeDepthd:
    """depthd's calibration endpoints: GET /calibration/capture serves the queued pairs, POST /calibration/reload."""

    def __init__(self, pairs, reload_status=200):
        self.pairs = list(pairs)
        self.reloads = 0
        outer = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path != "/calibration/capture":
                    self.send_error(404)
                    return
                if not outer.pairs:
                    self.send_error(503)
                    return
                body, ctype = multipart(*outer.pairs.pop(0))
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                outer.reloads += 1
                self.send_response(reload_status)
                self.send_header("Content-Length", "2")
                self.end_headers()
                self.wfile.write(b"{}")

        self.server = http.server.HTTPServer(("127.0.0.1", 0), Handler)
        self.url = "http://127.0.0.1:%d" % self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()


def test_capture_then_solve_end_to_end(rig_pairs, tmp_path):
    # Duplicates and a pair without the board are served between the good ones; capture must skip them.
    blank = np.full((HEIGHT, WIDTH), 255, np.uint8)
    queue = []
    for pair in rig_pairs:
        queue += [pair, pair, (blank, blank)]
    depthd = FakeDepthd(queue)
    try:
        config = make_config(
            tmp_path, depthd_url=depthd.url, capture_count=12, capture_poll_s=0.0, capture_timeout_s=30.0
        )
        assert cal.run_capture(config) == 0
        assert len(list(config.captures_dir.glob("pair_*_left.png"))) == 12
        assert cal.run_solve(config) == 0
        assert depthd.reloads == 1
        assert read_matrices(config.output_path)
    finally:
        depthd.close()


def test_capture_times_out_with_exit_1(rig_pairs, tmp_path):
    depthd = FakeDepthd([])
    try:
        config = make_config(tmp_path, depthd_url=depthd.url, capture_poll_s=0.0, capture_timeout_s=0.3)
        assert cal.run_capture(config) == 1
    finally:
        depthd.close()


def test_solve_reports_a_failed_reload(rig_pairs, tmp_path):
    depthd = FakeDepthd([], reload_status=422)
    try:
        config = make_config(tmp_path, depthd_url=depthd.url)
        config.captures_dir.mkdir()
        for i, (left, right) in enumerate(rig_pairs[:12]):
            cal.save_pair(config.captures_dir, i, left, right)
        assert cal.run_solve(config) == 4
        assert config.output_path.exists()
    finally:
        depthd.close()


def test_solve_exit_3_and_untouched_output_on_noisy_pairs(rig_pairs, tmp_path):
    config = make_config(tmp_path, rms_limit_px=0.0001, depthd_url="http://127.0.0.1:9")
    config.captures_dir.mkdir()
    for i, (left, right) in enumerate(rig_pairs[:12]):
        cal.save_pair(config.captures_dir, i, left, right)
    assert cal.run_solve(config) == 3
    assert not config.output_path.exists()


def square_runs(image: np.ndarray, row: int):
    line = image[row] < 128
    edges = np.flatnonzero(np.diff(line.astype(np.int8))) + 1
    return np.diff(np.concatenate([[0], edges, [len(line)]])), line


@pytest.mark.parametrize("size", [(1072, 1448), (1236, 1648), (1264, 1680)])
def test_ereader_board_layout(size):
    config = CalibrateConfig()
    width, height = size
    image = cal.render_board(config, width, height)
    assert image.shape == (height, width) and image.dtype == np.uint8
    ys, xs = np.where(image < 128)
    square = int(round((xs.max() - xs.min() + 1) / config.squares_y))
    # Integer pixels per square, the largest that keeps a 4 % margin on every side.
    margin = 0.04 * min(width, height)

    def fits(s):
        return (width - config.squares_y * s) / 2 >= margin and (height - config.squares_x * s) / 2 >= margin

    assert fits(square) and not fits(square + 1)
    # Centred on the canvas.
    assert abs(xs.min() - (width - xs.max() - 1)) <= square and abs(ys.min() - (height - ys.max() - 1)) <= square
    detector = cv2.aruco.CharucoDetector(cal.charuco_board(config))
    corners, ids, _, _ = detector.detectBoard(image)
    assert ids is not None and len(ids) == (config.squares_x - 1) * (config.squares_y - 1) == 24
    # Every square the same integer size: the board's first row of squares alternates dark and white runs of one size.
    left = (width - config.squares_y * square) // 2
    top = (height - config.squares_x * square) // 2
    row = image[top + 1, left : left + config.squares_y * square]
    runs, _ = square_runs(row[None, :], 0)
    assert list(runs) == [square] * config.squares_y
    column = image[top : top + config.squares_x * square, left + 1]
    runs, _ = square_runs(column[None, :], 0)
    assert list(runs) == [square] * config.squares_x
    assert set(np.unique(image)) == {0, 255}


@pytest.mark.parametrize("size", ["1072x1448", "1236x1648", "1264x1680"])
def test_reference_pngs_are_the_tools_output(size):
    path = REFERENCE_DIR / ("charuco-7x5-dict5x5_100-%s.png" % size)
    assert path.is_file(), "reference PNG missing: %s" % path
    width, height = (int(v) for v in size.split("x"))
    reference = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    rendered = cal.render_board(CalibrateConfig(), width, height)
    assert np.array_equal(reference, rendered)


@pytest.mark.parametrize("size", sorted(PINNED_BOARDS))
def test_board_pixels_are_pinned(size):
    width, height = size
    assert (
        hashlib.sha256(cal.render_board(CalibrateConfig(), width, height).tobytes()).hexdigest() == PINNED_BOARDS[size]
    )


def test_board_cli_writes_a_png(tmp_path):
    out = tmp_path / "board.png"
    assert (
        cal.main(
            ["--config", str(CALIBRATE_YAML), "board", "--width-px", "1072", "--height-px", "1448", "--out", str(out)]
        )
        == 0
    )
    assert cv2.imread(str(out), cv2.IMREAD_UNCHANGED).shape == (1448, 1072)


def test_board_rejects_a_canvas_too_small(tmp_path):
    with pytest.raises(cal.CalibrationError):
        cal.render_board(CalibrateConfig(), 40, 50)


def test_shipped_calibrate_yaml_loads_with_the_board_depthd_reads():
    config = load_config(CALIBRATE_YAML)
    assert (config.squares_x, config.squares_y, config.square_mm, config.marker_mm, config.dictionary) == (
        7,
        5,
        30.0,
        22.0,
        "DICT_5X5_100",
    )
    assert config.rms_limit_px == 0.5 and config.capture_count == 30 and config.min_shared_corners == 12
    assert str(config.output_path) == "/var/lib/depthd/stereo.yaml"


def test_config_rejects_a_marker_not_below_the_square():
    with pytest.raises(ValueError):
        CalibrateConfig(marker_mm=30.0)
    with pytest.raises(ValueError):
        CalibrateConfig(dictionary="DICT_NOPE")
