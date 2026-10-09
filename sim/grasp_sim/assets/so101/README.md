# SO-101 MuJoCo model (vendored)

Vendored unmodified from MuJoCo Menagerie, `robotstudio_so101`:

- Upstream: https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotstudio_so101
- Upstream commit: `0059d4335f8156206f63a35662313385f7ad6d74` (main at vendoring time, 2026-10-09)
- License: Apache-2.0, see `LICENSE` in this directory
- Files kept: `so101.xml` (the robot), `assets/` (meshes), `LICENSE`, `CHANGELOG.md`, `UPSTREAM_README.md`
  (the upstream README). The upstream `scene*.xml` and `so101.png` are not vendored: scenes are generated in
  Python (`grasp_sim.scene`) from `so101.xml`.

Do not edit `so101.xml`. Everything the harness changes (arm mount height, widened shoulder_lift range, floor,
objects) is applied programmatically through `mujoco.MjSpec` in `grasp_sim/scene.py`.
