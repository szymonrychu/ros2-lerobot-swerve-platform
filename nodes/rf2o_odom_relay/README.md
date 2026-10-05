# rf2o_odom_relay

Turns the raw rf2o lidar odometry (`/odom_rf2o`) into a body-frame twist with a real covariance on
`/odom_rf2o_twist`, which the EKF fuses (vx, vy). Client only; started right after `rf2o_laser_odometry`.

## Why

rf2o publishes zero covariance, a laser-frame x speed and `vy = 0` (see `nodes/rf2o_laser_odometry/README.md`).
Fusing that twist as is would over-trust it, flip the sign of vx (the lidar is mounted rotated 180 deg) and force the
lateral speed of the holonomic base to zero. The pose is correct, so the relay differences consecutive poses:

- twist = pose difference rotated into the body frame at the mid heading, divided by the time between the two messages
  (`twist.py: body_twist`); nothing is published for the first message or when the time step is not in (0, `max_dt_s`];
- covariance = diagonal `var_vx_vy` for vx and vy, `var_vyaw` for the yaw rate (`twist_covariance`).

## Config

`/etc/ros2/rf2o_odom_relay/config.yaml` (env `RF2O_ODOM_RELAY_CONFIG`), written from the `config: |` block in
`ansible/group_vars/client.yml`:

| Key | Default | Meaning |
|---|---|---|
| `input_topic` | `/odom_rf2o` | rf2o odometry. |
| `output_topic` | `/odom_rf2o_twist` | Topic the EKF fuses. |
| `var_vx_vy` | `0.02` | (m/s)^2. rf2o gives no quality figure; pose differences over ~0.1 s scans carry about 0.05-0.1 m/s of noise. Larger than the wheel floor (0.01), so the wheels win while consistent and the lidar takes over as the wheel covariance grows with the slip residual. |
| `var_vyaw` | `0.05` | (rad/s)^2. Large against the gyro (0.0004); the EKF does not fuse the rf2o yaw rate anyway. |
| `max_dt_s` | `1.0` | Largest usable gap between two rf2o poses, s. |

## Develop

```bash
cd nodes/rf2o_odom_relay && ../../.venv/bin/python -m pytest tests -q
```
