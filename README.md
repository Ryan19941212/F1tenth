# F1Tenth Autonomous Racing Stack

A full 1:10 F1Tenth autonomous racing pipeline running on NVIDIA Jetson Orin Nano — from raw sensors to control commands.

```
LiDAR + VESC odom + IMU
  |- [1] SLAM           slam_toolbox (pose-graph + loop closure)
  |- [2] Localization   Particle Filter + range_libc (CUDA ray marching)
  |- [3] Planning       TUM minimum curvature raceline (offline) + Pure Pursuit (online)
  `- [4] Control        Pure Pursuit | Linear MPC
         -> /drive -> VESC
```

## Highlights

- **GPU ray casting**: `range_libc` built with CUDA for Orin Nano (sm_87). 4000 particles @ 833 Hz vs 186 Hz CPU (**4.47x speedup**).
- **Offline raceline**: TUM `global_racetrajectory_optimization` fork — minimum curvature IQP solver producing optimal line + velocity profile.
- **Clean 4-stage layout**: 6 active packages, legacy labs kept in `src/archive/` but not built.
- **One-shot pipeline**: record -> map -> extract centerline -> optimize raceline -> run.

## Hardware

| Component | Model |
|-----------|-------|
| Compute   | NVIDIA Jetson Orin Nano (Super) |
| LiDAR     | RPLIDAR A2M12 |
| Motor     | VESC 6+ |
| Teleop    | PS5 DualSense |

## Layout

```
src/
 |- f1tenth_system/       # bringup, joy, VESC, lidar driver
 |- f1tenth_utils/        # status_monitor, throttle_interpolator
 |- sllidar_ros2/         # RPLIDAR driver
 |- vesc/                 # motor driver (odom sign bug fixed)
 |- particle_filter/      # MIT PF, config set to rmgpu + variant 2
 |- pure_pursuit/         # adaptive lookahead + curvature speed cap
 `- mpc_controller/       # linear MPC, horizon=12, dt=0.1

tools/
 |- global_racetrajectory_optimization/   # TUM fork (main_f1tenth.py)
 `- range_libc/                           # CUDA build (sm_87)

scripts/
 |- extract_centerline.py  # medial-axis centerline -> TUM input
 |- optimize_raceline.sh   # one-shot TUM mincurv
 |- plot_raceline.py       # visualize recorded vs optimal
 |- bench_rangelibc.py     # CPU vs GPU benchmark
 `- run_pf_{pp,mpc}.sh     # launch stacks

maps/<track>/
 |- map.pgm, map.yaml      # slam_toolbox output
 |- waypoints.csv          # recorded path
 |- waypoints_optimal.csv  # TUM output (x, y, yaw, v)
 `- debug/*.png
```

## Pipeline Theory (quick)

**[1] SLAM** — Pose-graph SLAM. Scan matching gives frame-to-frame pose deltas; loop closure triggers a full non-linear least-squares (g2o / Ceres) to flatten drift. The map is a log-odds occupancy grid.

**[2] Localization** — Particle approximation of a recursive Bayes filter. Motion update propagates particles with odometry noise; sensor update casts rays per particle (244k rays/tick at 4000×61 — embarrassingly parallel on GPU) and scores them with a beam model (`z_hit` Gaussian + `z_short` + `z_rand` + `z_max`), then low-variance resamples.

**[3a] TUM Raceline (offline)** — Discretize the track into N points; each point has a normal offset `alpha` bounded by track width. Path = centerline + alpha * normal. Objective `min sum(kappa^2)` (minimum total curvature ≈ minimum lap time) is a QP in alpha, solved via quadprog / cvxopt with IQP iterations to handle curvature non-linearity. Velocity profile: friction-circle speed cap + forward/backward pass to respect ax_min/max.

**[3b] Pure Pursuit (online)** — Pick a lookahead point at distance L on the waypoints, compute body-frame (x, y), then curvature `kappa = 2y / L^2` (geometry of a circular arc tangent to heading hitting the target). Steering `delta = atan(wheelbase * kappa)`. Adaptive `L = clamp(k*v, L_min, L_max)`; corner speed cap `v <= sqrt(a_lat_max / |kappa|)`.

**[4] MPC** — Linearize the bicycle model around a reference, solve a receding-horizon QP of horizon H=12 minimizing tracking + control cost with actuator and rate constraints, apply only the first control, repeat next tick.

## Install

```bash
# ROS2 Humble (Jetson Orin Nano / Ubuntu 22.04)
sudo apt install ros-humble-desktop ros-humble-slam-toolbox \
                 ros-humble-ackermann-msgs python3-cvxopt python3-skimage

# TUM deps
pip3 install --user trajectory-planning-helpers quadprog networkx

# Build range_libc with CUDA (sm_87 for Orin)
cd tools/range_libc/pywrapper
export PATH=/usr/local/cuda/bin:$PATH WITH_CUDA=ON
python3 setup.py build_ext --inplace
cp range_libc.*.so ~/.local/lib/python3.10/site-packages/

# colcon workspace
cd ../../.. && colcon build --symlink-install
source install/setup.zsh
```

## Workflow

```bash
# 1. Mapping
ros2 launch f1tenth_system bringup_launch.py
ros2 launch slam_toolbox online_async_launch.py
# teleop a few laps -> save map -> maps/<track>/map.{pgm,yaml}

# 2. Record waypoints
python3 src/mpc_controller/scripts/record_waypoints.py \
    -o maps/<track>/waypoints.csv --dist 0.1

# 3. Generate optimal raceline
./scripts/optimize_raceline.sh <track> mincurv
./scripts/plot_raceline.py <track>   # visualize

# 4. Drive
./scripts/run_pf_pp.sh  <track>      # Pure Pursuit
./scripts/run_pf_mpc.sh <track>      # MPC
```

## Benchmarks

| Component | Metric | Value |
|-----------|--------|-------|
| range_libc CPU | PyRayMarching @ 4000p x 61 rays | 5.37 ms (186 Hz) |
| range_libc GPU | PyRayMarchingGPU @ same | **1.20 ms (833 Hz)** |
| TUM raceline   | 75 pts, 0.05 m step | laptime 2.58 s, v = 1.32 – 1.64 m/s |
| colcon build   | 6 packages | ~90 s |

Re-run: `python3 scripts/bench_rangelibc.py <track> 4000`

## Key Patches

- **VESC odom sign bug** — extra negation in `vesc_to_odom.cpp:102`. Fixed; older recorded waypoints may be mirrored.
- **range_libc CUDA** — py2 -> py3 port, `sm_20` -> `sm_87`, replaced `tf.transformations` with inline quaternion math, shim for missing `distutils.msvccompiler`.
- **TUM on aarch64** — broken quadprog wheel replaced by cvxopt shim (`quadprog_shim.py`); `spline_approximation.py` patched for scipy 1.15 compatibility.

## Status

- [x] Offline pipeline (SLAM -> centerline -> TUM raceline)
- [x] Online stack single-lap tested (PF + MPC, 2026-04-12)
- [ ] Larger map + optimal raceline field test, PP vs MPC benchmark (pending track time)

## License

MIT

## Credits

- [MIT Racecar PF](https://github.com/mit-racecar/particle_filter)
- [TUM global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)
- [range_libc](https://github.com/kctess5/range_libc) by Corey Walsh
- [F1Tenth Foundation](https://f1tenth.org/)
