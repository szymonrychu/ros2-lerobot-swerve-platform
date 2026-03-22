---
name: rtk-diagnostics
description: Use when checking GPS RTK status, debugging RTK fix issues, verifying NTRIP connectivity, or after deploying gps_rtk nodes to server or client RPi
---

# RTK Diagnostics

Run `scripts/rtk_diag.sh` for a complete one-command RTK diagnostic. Never SSH manually to check RTK status — the script handles everything and requires no approvals.

## Quick Reference

```bash
./scripts/rtk_diag.sh              # Full one-shot diagnostic
./scripts/rtk_diag.sh --watch      # Repeat every 10s (Ctrl+C to stop)
./scripts/rtk_diag.sh --watch --interval 5   # Every 5s
./scripts/rtk_diag.sh --capture 60 # 60s capture + fix distribution summary
./scripts/rtk_diag.sh --logs-only  # Only journalctl [diag] lines
./scripts/rtk_diag.sh --lines 30   # More log lines per node (default: 10)
./scripts/rtk_diag.sh --no-scraper # Skip topic_scraper (if not running)
```

## Output Sections

| Section | What to look for |
|---------|-----------------|
| Services | Both should be `active`. If not, check logs first. |
| NTRIP port :5016 | Should be `listening`. If not, base node isn't running. |
| Base `[diag]` | `rtcm_tx=Nf/NB` with `types={1005,...}` = base is healthy. `types=[]` = no RTCM output (calibration needed). `clients=N` = rover(s) connected. |
| Rover `[diag]` | `ntrip=UP` + `ntrip_rx=NB` > 0 = receiving corrections. `fix=RTK_Fixed(q4)` or `fix=RTK_Float(q5)` = RTK achieved. |
| Topic scraper | `status=2 (RTK)` = success. `status=0 (GPS)` = no corrections reaching rover. |

## Interpreting Results — Common Patterns

| Symptom | Cause | Fix |
|---------|-------|-----|
| `types=[]`, `rtcm_tx=0f/0B` | Base hasn't done survey-in calibration | Run `./scripts/rtk_calibrate.sh` |
| `ntrip_rx=0B`, `ntrip=UP` | NTRIP connected but base not sending RTCM | Fix base first (see above) |
| `ntrip=DOWN` | Rover can't reach NTRIP caster | Check network, port 5016, base service |
| `fix=GPS(q1)` on rover after corrections flow | Corrections too new; wait a few minutes | Also check `age=` field in rover diag |
| `no_fix` on base (WARNING/ERROR) | No GPS sky view or calibration lost | Ensure sky view, re-run calibration if needed |
| `types={...}` but no 1005/1006 | Reference station position missing from RTCM stream | Re-run calibration |
| `RTK_Float(q5)` → stuck, never reaches `RTK_Fixed(q4)` | Baseline too long or multipath; normal in open sky | Wait longer; check HDOP and `drift2d` |

## Service Names

- Base: `ros2-gps_rtk_base` on `server.ros2.lan`
- Rover: `ros2-gps_rtk_rover` on `client.ros2.lan`

## Calibration

If base needs re-calibration (survey-in):

```bash
./scripts/rtk_calibrate.sh --accuracy 20   # ~1h with relaxed accuracy
```

After calibration completes, power-cycle the server RPi to commit to module flash.
