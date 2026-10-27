---
title: D435 live bring-up and topic pin
status: todo
repo: ros2-lerobot-sverve-platform
value: "The D435 is proven on the real Pi 5 (USB3, firmware, power, 30 min soak) and its real topic names are pinned."
files: [nodes/bridges/realsense_d435/config/topics.yaml, nodes/bridges/realsense_d435/config/realsense_d435.yaml, nodes/web_ui/urdf/head.urdf, tests/test_d435_topics.py, tests/test_d435_runbook.py, docs/runbooks/d435-switchover.md, tests/README.md]
verify: "uv run pytest tests/test_d435_topics.py tests/test_d435_runbook.py tests/test_realsense_d435_deploy.py tests/test_web_ui_agent_tab.py -q && uv run poe lint"
depends_on: [d435-node-type-and-profile, d435-udev-rule-via-node-depl, d435-diagnostics-script, d435-frames-on-the-head-urdf]
model: sonnet
review: ""
verified_head: ""
points: 5
expected_minutes: 57
expected_tokens: 3750000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# D435 live bring-up and topic pin

## Requirement

LIVE task. Goal G1: "... deployed by Ansible ... and the live topic list pinned in config." Design D2, D3, D4, D16, Q4. Prior-art fix-as-ac: "the D435 enumerates at USB 3 (5000M) on the Pi 5 and stays up for 30 min with no VIDIOC_QBUF/No such device"; "the Pi 5 USB power budget is confirmed"; docs fix-as-ac "verify firmware >= 5.17.3.10 at bring-up".

Steps on the robot (no manual SSH beyond scripts/d435_diag.sh; deploy only with scripts/deploy-nodes.sh, logs in .logs/; invoke the ansible-deploy skill first; no servo moves in this task):
1. `./scripts/deploy-nodes.sh client realsense_d435 -e ros2_deploy_ignore_agent=false 2>&1 | tee .logs/deploy-client-realsense.log` (udev + unit). Then stop only the realsense unit through a deploy with `enabled: false` is NOT needed: run `--device` in the window before the first enable by deploying once with the instance temporarily disabled in the working tree (not committed), or read the same facts from the journal; record which.
2. `./scripts/d435_diag.sh --device`: serial 935723024187, firmware, the factory Depth-to-Color extrinsic. If firmware < 5.17.3.10: back up with `rs-fw-update -b`, flash the signed 5.17.3.10 image with `rs-fw-update -s 935723024187 -f <image>` with the unit stopped (irreversible operation listed in design.md, Q4), re-read.
3. Write the factory extrinsic into `nodes/web_ui/urdf/head.urdf` `d435_color_joint` (translation in m, rotation as rpy) with a comment naming the capture date and serial.
4. Enable, then `./scripts/d435_diag.sh`: 8086:0b07 at 5000, USB type 3.x, PSU check passes (usb_max_current_enable=1 or the 5A supply line), zero VIDIOC_QBUF / No such device / Frame didn't arrive over 30 min (`--since 30min`).
5. Enable topic_scraper_api for the session (CLAUDE.md debug workflow), `./scripts/d435_diag.sh --topics` and `--rates`: pin the live topic list (name and type) in new `nodes/bridges/realsense_d435/config/topics.yaml`; colour and depth camera_info at >= 14 Hz idle. Disable topic_scraper_api again.
6. New `tests/test_d435_topics.py`: every `/head/d435/` topic named anywhere in `ansible/group_vars/client.yml` is in topics.yaml with the expected type; topics.yaml lists the colour compressed topic, aligned depth, depth image_rect_raw and their camera_info.
7. New `docs/runbooks/d435-switchover.md` section "Bring-up": each command above with its output summary line (firmware, speed, error counts, rates). `tests/test_d435_runbook.py::test_bring_up_section_is_recorded` pins that the section exists with firmware, USB speed, soak window and rates filled (no placeholders). tests/README.md documents both test files.
If the V4L2 apt build fails the soak, stop and return a `discovered` entry for an RSUSB source build (do-not-build unless it fails).

## Acceptance conditions

- AC1: the D435 enumerates as 8086:0b07 at 5000M with firmware >= 5.17.3.10 and the PSU check passing [evidence: command:./scripts/d435_diag.sh]
- AC2: 30 min with no VIDIOC_QBUF, No such device or Frame didn't arrive [evidence: command:./scripts/d435_diag.sh --since 30min]
- AC3: the live topic list and rates are captured [evidence: command:./scripts/d435_diag.sh --topics && ./scripts/d435_diag.sh --rates]
- AC4: every D435 topic used in client.yml is in the pinned topics.yaml [evidence: test:tests/test_d435_topics.py::test_every_configured_d435_topic_is_pinned]
- AC5: head.urdf carries the factory colour extrinsic of serial 935723024187 [evidence: test:tests/test_d435_topics.py::test_head_urdf_colour_joint_is_the_factory_extrinsic]
- AC6: the runbook records the bring-up evidence [evidence: test:tests/test_d435_runbook.py::test_bring_up_section_is_recorded]

## Files

- `nodes/bridges/realsense_d435/config/topics.yaml`
- `nodes/bridges/realsense_d435/config/realsense_d435.yaml`
- `nodes/web_ui/urdf/head.urdf`
- `tests/test_d435_topics.py`
- `tests/test_d435_runbook.py`
- `docs/runbooks/d435-switchover.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
