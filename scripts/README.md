# Scripts

Helper scripts for deployment, telemetry, and GPS RTK.

## GPS RTK

Run these from your computer (or from a host with SSH and network access to server/client).

### 1. Calibration (base station survey-in)

**One-time** survey-in of the LC29H(BS) base on the server. Takes about 1–2 hours.

```bash
# From repo root: run calibration on server via SSH (stops gps_rtk_base, runs survey-in, then instructs next steps)
./scripts/rtk_calibrate.sh

# Optional: custom samples and accuracy (default: 3600 samples, 15 m)
./scripts/rtk_calibrate.sh --samples 3600 --accuracy 15

# Skip factory restore (if module already reset)
./scripts/rtk_calibrate.sh --no-restore

# Run locally (you must be on the server with /dev/ttyS0)
./scripts/rtk_calibrate.sh --local
```

**Env:** `RTK_SERVER_HOST` (default `192.168.1.33`), `RTK_SSH_USER` (from Ansible inventory or `$USER`).

After calibration, the script tells you to **power cycle** the base (unplug/plug HAT or reboot) and start the service again.

### 2. Verify fixes (topic scraper)

Requires **topic_scraper_api** running on server and client (port 18100) and **jq** installed.

```bash
# One-shot: print latest /server/gps/fix and /client/gps/fix (status, lat, lon, alt)
./scripts/rtk_verify.sh

# Poll every 5 s until Ctrl+C
./scripts/rtk_verify.sh --watch

# Capture for 60 s, then show client status distribution and last fixes
./scripts/rtk_verify.sh --capture 60
```

**Fix status:** `-1` = no fix, `0` = GPS fix, `1` = SBAS, `2` = **RTK fix** (goal for rover).

**Env:** `RTK_SERVER_HOST`, `RTK_CLIENT_HOST` (defaults `192.168.1.33`, `192.168.1.34`), `SCRAPER_PORT` (default `18100`).

### 3. Quick status (services + RTCM port)

```bash
# Service and RTCM port check only
./scripts/rtk_status.sh

# Include one-shot fix from topic scraper
./scripts/rtk_status.sh --fix
```

**Env:** `RTK_SERVER_HOST`, `RTK_CLIENT_HOST`, `RTK_SSH_USER`.

### 4. Post-calibration (base survey-in done + power cycle)

After the base is calibrated and the server RPi has been power-cycled:

1. **Start the base** (if it didn’t auto-start):  
   `ssh $USER@<server> 'sudo systemctl start ros2-gps_rtk_base'`
2. **Check stack**:  
   `./scripts/rtk_status.sh` → both services active, server TCP 5016 listening.
3. **Check fix quality**:  
   `./scripts/rtk_verify.sh` or `./scripts/rtk_verify.sh --fix` → client should show `status=2` (RTK fix) once corrections are flowing and the rover has converged.
4. **Optional capture**:  
   `./scripts/rtk_verify.sh --capture 60` → client status distribution and last fixes (confirm sustained RTK fix and position).

Run these from a machine that can reach both server and client (e.g. your laptop on the same network). Topic scraper must be running on both hosts (port 18100) for verify steps.

### Other

- **calibrate_rtk_base.py** — Low-level calibration script (run on the machine that has the base serial port). Used by `rtk_calibrate.sh` when not using `--local`.
- **topic_scraper_collect.py** — Generic NDJSON collector for topic_scraper_api; used by `rtk_verify.sh`.
