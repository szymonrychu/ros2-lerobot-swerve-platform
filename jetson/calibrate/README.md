# stereo_calibrate - stereo ChArUco calibration for the Jetson

Calibrates the IMX219 stereo pair and writes `/var/lib/depthd/stereo.yaml` for depthd. The calibration lives on the
Jetson next to depthd, never on the Pi. Runs on the Jetson's system Python 3.8 with its OpenCV 4.8 (aruco), numpy and
pydantic 2; no other runtime dependency. Settings: `config/calibrate.yaml` (or `STEREO_CALIBRATE_CONFIG`, or `--config`).

```
python3 -m stereo_calibrate board   --width-px W --height-px H --out board.png
python3 -m stereo_calibrate capture
python3 -m stereo_calibrate solve [--no-reload]
```

Run them from `/opt/depthd/calibrate` (or with that directory on `PYTHONPATH`). depthd must be running (`capture` and the
reload talk to its HTTP API, `depthd_url` in `calibrate.yaml`).

## USER STEP - show the board on your e-reader

The board is 7x5 squares, `DICT_5X5_100`, marker = square x 22/30. Its real size is whatever your screen shows, so you
measure it once.

1. Generate the PNG at the e-reader's native portrait resolution (any `board.png` path):

   | Device resolution (portrait) | Command |
   |---|---|
   | 1072x1448 | `python3 -m stereo_calibrate board --width-px 1072 --height-px 1448 --out board.png` |
   | 1236x1648 | `python3 -m stereo_calibrate board --width-px 1236 --height-px 1648 --out board.png` |
   | 1264x1680 | `python3 -m stereo_calibrate board --width-px 1264 --height-px 1680 --out board.png` |

   Other resolutions work the same way. The board has a whole number of pixels per square, is centred on white and
   keeps a margin of at least 4 % of the short side. Ready-made copies of the three above are in
   `docs/plans/2026-10-14-jetson-depth-offload-briefs/charuco/`.
2. Copy `board.png` to the e-reader and open it **unscaled** (100 %, 1 image pixel = 1 screen pixel; turn off
   fit-to-screen and zoom). Turn the front light to a steady medium and disable any auto-sleep.
3. **Measure one square with a ruler** (edge to edge of a black square, in mm, as precisely as you can; measure a few
   squares across the board and average). Put it in `config/calibrate.yaml`:
   `square_mm: <measured>` and `marker_mm: <measured x 22 / 30>` (for example 11.4 mm gives marker 8.36 mm). A wrong
   square size does not hurt the RMS, it scales the baseline and so every depth value.
4. Keep the e-reader **flat and still** (the glass is flat, do not hold it bent or let the case flex). Move it, then
   hold still while a pair is taken.

## Capture (on the Jetson)

```
python3 -m stereo_calibrate capture
```

Move the e-reader through the view of both cameras: fill every corner of the image, tilt it left, right, up and down,
rotate it a little, bring it near (it must stay fully visible in both images) and far. The tool asks depthd for the
latest unrectified pair (`GET /calibration/capture`) and keeps it only when the board is detected in both images with at
least 12 shared corners and its pose differs from every kept pose (position, size, tilt, rotation). Duplicates are
skipped, so holding still is fine. It stops at 30 pairs (about two minutes), or after `capture_timeout_s` with exit 1.
Old pairs in `captures_dir` are removed when capture starts.

## Solve

```
python3 -m stereo_calibrate solve
```

Runs per-camera ChArUco calibration, `stereoCalibrate` with fixed intrinsics and `stereoRectify` (alpha 0), refuses to
write when the RMS is above 0.5 px (`rms_limit_px`), otherwise writes `/var/lib/depthd/stereo.yaml` atomically (temp file
and rename) and calls `POST /calibration/reload`.

Expected: **stereo RMS 0.1 - 0.3 px**, baseline about 60 mm (depthd rejects a file outside 30-120 mm), fx about 400-520
px at the capture resolution. Above 0.5 px nothing is written (exit 3): recapture with the e-reader flatter, steadier,
in better light, and with more tilt and corner coverage.

Exit codes: 0 done, 1 failure, 2 config error, 3 refused by the RMS gate, 4 written but depthd did not reload it.

## Check afterwards

```
curl -s http://127.0.0.1:8090/health        # state must not report a calibration problem
curl -s http://127.0.0.1:8090/calibration   # status valid, rms, baseline_m
```

`/health` leaves its no-calibration state and reports depth running once depthd has loaded the file. If the reload was
refused (exit 4) the reason is printed; fix it and run `curl -X POST http://127.0.0.1:8090/calibration/reload`.

## Development

```
uv run --frozen --python 3.8 pytest -q && uv run --frozen poe lint
```

The tests render the board, project it into two virtual cameras (f 400 px, 640x480, baseline 60 mm) and check the solve,
the depthd file contract (`jetson/depthd/test/fixtures/stereo_valid.yaml`), the RMS gate, the pose filter and the
e-reader board layout.
