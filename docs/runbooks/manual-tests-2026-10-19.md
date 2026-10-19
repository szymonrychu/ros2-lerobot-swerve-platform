# Manual robot tests: 2026-10-19 deliveries

Pan/tilt head, driving-agent skills, POI fixes, head RGB-D fusion and ToF 3D obstacle detection are merged, pushed
and deployed to the client (`./scripts/deploy-nodes.sh client --all`, failed=0). Everything below needs a person at
the robot or a browser. The 2026-10-18 list (`manual-tests-2026-10-18.md`, T1-T14) still applies.

Run on or against the client (`client.ros2.lan`). ROS commands need
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. Web UI: `http://client.ros2.lan:8080`.
Servo safety (CLAUDE.md): smallest moves first, stop and ask before anything larger.

| ID | Area | Person at robot | Status |
|---|---|---|---|
| U1 | Head: home pose faces forward after startup | yes | PENDING |
| U2 | Head: web UI sliders and Home | yes (browser) | PENDING |
| U3 | Head: 3D map view follows the head | yes (browser) | PENDING |
| U4 | Head: agent head tools and lease release homes the head | yes | PENDING |
| U5 | Skills: agent loads skills, survey/sightseeing, workflows | yes | PENDING |
| U6 | Skills: Retros tab and retrospective after a job | yes (browser) | PENDING |
| U7 | POI list: category filter, mark, mark all visible, delete marked | yes (browser) | PENDING |
| U8 | POI fixes: save/load/delete a map with photos | yes | PENDING |
| U9 | POI fixes: Reset map clears POIs and photos | yes (browser) | PENDING |
| U10 | RGB-D: calibration and validation (runbook steps 2-8) | yes | PENDING |
| U11 | ToF obstacles: empty floor, objects above/below lidar plane | yes | PENDING |
| U12 | ToF obstacles: lidar marks survive outside the ToF view | yes | PENDING |
| U13 | ToF obstacles: cliff stop at low speed (spotter, physical stop) | yes | PENDING |
| U14 | ToF obstacles: outage toggle stops and resumes Nav2 | yes | PENDING |
| U15 | Markers: print the 4 mm two-colour 3MFs and check IR contrast | yes | PENDING |
| U16 | Metadata markers: printed QR markers become POIs (runbook) | yes | PENDING |
| U17 | RTAB-Map 3D mapping: map a room, check alignment, CPU and WiFi (runbook) | yes | PENDING |

## Head on pan/tilt (source: `docs/plans/done/pan-tilt-head-control-with-a`)

### U1 - Home pose
- Before the first deploy the pan servo read raw 2 (about half a turn from centre); startup drove it to pan 0 / tilt
  0.52 rad (measured -0.005 / 0.518).
- Check: the head faces forward and looks down at the ground. If it faces backwards, the pan horn is mounted 180 deg off;
  report it and the pan zero offset is corrected in config.

### U2 - Web UI sliders
- Head tab: drag pan a little, then tilt a little; the head follows within the speed cap. Home returns to the home pose.
- Move the head with the agent, then drag only one slider: the other joint must stay where the agent left it.

### U3 - 3D view
- Map tab, 3D view with the head layer on: the drawn head turns with the real head.

### U4 - Agent
- From an agent session: `head_look_at` a point ahead on the floor, `set_head_angles` small values, `head_home`.
- Release the lease: the head returns home.

## Driving-agent skills and POI (sources: `docs/plans/done/driving-agent-skills-and-wor`, `docs/plans/done/poi-fixes-photos-as-pois-and`)

### U5 - Skills and workflows
- Start a job from free text that needs a survey; the agent loads the matching skill and follows the workflow's
  mandatory steps (job end refuses while a mandatory step is open).

### U6 - Retros
- After the job ends, a retrospective appears in the Retros tab.

### U7 - POI list
- Filter by category, mark single POIs and "mark all visible", delete the marked ones (keep-outs ask for consent).

### U8 - Maps with photos
- Take a photo (agent `get_camera_image` with record), save the map, load another map, load it back: POIs and photos
  return. Delete the saved map: its POIs and photos are gone. Save, restart map_manager, delete: the live POIs go too.

### U9 - Reset map
- Reset map in the web UI: the dialog names the POI and photo counts; afterwards no POIs and no photos remain live;
  saved maps are untouched.

## Head RGB-D fusion (source: `docs/runbooks/head-rgbd-fusion.md`)

### U10 - Calibration and validation
- Follow runbook steps 2-8: ChArUco board (printed, calipered), 20-30 hand-held poses, `scripts/head_rgbd_calibrate.py`,
  URDF handoff, time offset from a pan sweep, depth accuracy at taped distances, alignment check, CPU.
- Until `/etc/ros2/head_rgbd_cpp/tof_calibration.yaml` exists the node logs "cannot read the ToF calibration" and
  publishes nothing (by design).

## ToF 3D obstacles (source: `docs/plans/in-progress/tof-3d-obstacle-detection-fo/tasks/final-suite-and-runbook.md`)

Already checked after deploy: all units active, the STVL `tof_voxel_layer` loads, no nav2 errors after restart,
`/tof/points_filtered` about 10.5 Hz, `/scan_filtered` about 7 Hz.

### U11 - Floor and objects (steps 6-7)
- Robot still, head home, 60 s: no ToF cells on the empty floor, no CliffStop/StopBox in `/collision_monitor_state`.
- A box edge at 0.3-0.5 m height 1 m ahead: cells within 1 s, gone about 15 s after removal. Same for an object
  5-15 cm high.

### U12 - Lidar survival (step 8)
- An obstacle behind the robot (lidar only) stays marked while the ToF looks at empty space for 30 s.

### U13 - Cliff stop (step 9)
- With a spotter and a physical stop, approach a 10-20 cm step down at <= 0.1 m/s, head home: the robot stops before
  the edge (CliffStop); a backward command still moves it.

### U14 - Outage (step 10)
- `ros2 param set /collision_monitor tof_obstacles.enabled false`, then true: works. Stopping arducam_tof stops Nav2
  motion ("invalid source"); restarting resumes.

## Markers (source: `.logs/markers`, generated 2026-10-18)

### U15 - Two-colour prints
- Print the 3MFs on the P2S (white PETG filament 1, black PETG filament 2, matte or textured plate). If Bambu Studio
  warns about the mesh, repair it.
- Check the ToF amplitude image of the ChArUco board: black cells dark, white bright.

## Metadata markers (source: `docs/runbooks/printable-markers.md`)

### U16 - Markers become POIs
- Needs U10 first (head camera and ToF calibration files deployed).
- Tape printed markers at measured poses; drive past them; each marker appears as a POI with its kind, name and pose;
  record the measured detection ranges in the runbook's Range section.

## RTAB-Map 3D mapping (source: `docs/runbooks/map3d-mapping.md`)

### U17 - Map a room
- Needs U10 first (calibration artifacts and `camera_info_file` set).
- Follow the runbook: deploy check, drive a room, alignment agreement >= 0.8, CPU/memory and WiFi measurements,
  the ToF-unplugged placeholder check; replace the design estimates with the measured numbers.
