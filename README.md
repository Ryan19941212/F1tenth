# F1Tenth Autonomous Racing Stack

一套在 NVIDIA Jetson Orin Nano 上跑的 F1Tenth (1:10) 自駕系統，從感測到控制的完整 pipeline。

```
LiDAR + VESC odom + IMU
  ├─ [1] SLAM           slam_toolbox (pose-graph + loop closure)
  ├─ [2] Localization   Particle Filter + range_libc (CUDA ray marching)
  ├─ [3] Planning       TUM minimum curvature raceline (offline) + Pure Pursuit (online)
  └─ [4] Control        Pure Pursuit | Linear MPC
         → /drive → VESC
```

## Highlights

- **GPU ray casting**：`range_libc` 在 Orin Nano (sm_87) 編 CUDA，4000 粒子 833 Hz (CPU 186 Hz，4.47x 加速)
- **離線 raceline**：TUM global_racetrajectory_optimization fork，mincurv + IQP 輸出最佳圈速路徑 + 速度剖面
- **乾淨 4 階段架構**：6 個 active packages，archive 保留但不 build
- **一鍵 pipeline**：錄 → 建圖 → 抽中線 → 產 raceline → 跑

## Hardware

| 組件 | 型號 |
|------|------|
| Compute | NVIDIA Jetson Orin Nano (Super) |
| LiDAR | RPLIDAR A2M12 |
| Motor | VESC 6+ |
| Teleop | PS5 DualSense |

## 目錄結構

```
src/
├── f1tenth_system/        # bringup, joy, VESC, lidar driver
├── f1tenth_utils/         # status_monitor, throttle_interpolator
├── sllidar_ros2/          # RPLIDAR driver
├── vesc/                  # 馬達驅動 (已修 odom sign bug)
├── particle_filter/       # MIT PF，config 切 rmgpu + variant 2
├── pure_pursuit/          # 自適應 lookahead + 彎道限速
└── mpc_controller/        # linear MPC，horizon=12，dt=0.1

tools/
├── global_racetrajectory_optimization/   # TUM fork (main_f1tenth.py)
└── range_libc/                           # CUDA build (sm_87)

scripts/
├── extract_centerline.py  # medial axis 抽中線 → TUM 輸入
├── optimize_raceline.sh   # 一鍵 TUM mincurv
├── plot_raceline.py       # 視覺化比對
├── bench_rangelibc.py     # CPU vs GPU benchmark
└── run_pf_{pp,mpc}.sh     # 啟動對應 stack

maps/<track>/
├── map.pgm, map.yaml      # slam_toolbox 輸出
├── waypoints.csv          # 錄製路徑
├── waypoints_optimal.csv  # TUM 輸出 (x, y, yaw, v)
└── debug/*.png
```

## Pipeline 原理速覽

**[1] SLAM** — Pose-graph SLAM。scan matching 估 pose，loop closure 觸發 g2o/Ceres 全圖最小二乘；地圖用 log-odds 佔據柵格。

**[2] Localization** — Bayes filter 的粒子近似。motion update 用 odom 推進粒子；sensor update 對每粒子做 ray casting（CUDA 平行化 4000×61 條），用 beam model（z_hit 高斯 + z_short + z_rand + z_max）算 likelihood → 加權重採樣。

**[3a] TUM Raceline (離線)** — 賽道離散成 N 點，每點法向偏移 α 為變數，路徑 = 中線 + α·normal。目標 `min Σ κ²`（最少總曲率 ≈ 最短圈時），化為 QP 用 quadprog/cvxopt 解；IQP 迭代處理 κ 非線性。速度剖面用摩擦圓限速 + forward-backward pass 產生可達曲線。

**[3b] Pure Pursuit (線上)** — 前方 lookahead L 點，車體坐標 (x, y) → 曲率 κ = 2y/L²（幾何弧切線求解），轉向角 δ = atan(L_wheelbase·κ)。L = clamp(k·v, L_min, L_max) 自適應，彎道 v ≤ √(a_lat_max/|κ|)。

**[4] MPC** — 自行車模型線性化後做 receding horizon QP：未來 H 步優化 (δ, a) 使 `||pos - ref||² + ||v - v_ref||² + ||u||² + ||Δu||²` 最小，硬約束滿足 |δ|≤δ_max、|Δδ|≤rate，只執行第一步，下 tick 重解。

## 安裝

```bash
# ROS2 Humble (Jetson Orin Nano / Ubuntu 22.04)
sudo apt install ros-humble-desktop ros-humble-slam-toolbox \
                 ros-humble-ackermann-msgs python3-cvxopt python3-skimage

# TUM deps
pip3 install --user trajectory-planning-helpers quadprog networkx

# build range_libc with CUDA (sm_87 for Orin)
cd tools/range_libc/pywrapper
export PATH=/usr/local/cuda/bin:$PATH WITH_CUDA=ON
python3 setup.py build_ext --inplace
cp range_libc.*.so ~/.local/lib/python3.10/site-packages/

# colcon
cd ../../.. && colcon build --symlink-install
source install/setup.zsh
```

## 使用流程

```bash
# 1. 建圖
ros2 launch f1tenth_system bringup_launch.py
ros2 launch slam_toolbox online_async_launch.py
# 遙控跑幾圈 → save map → maps/<track>/map.{pgm,yaml}

# 2. 錄 waypoints
python3 src/mpc_controller/scripts/record_waypoints.py \
    -o maps/<track>/waypoints.csv --dist 0.1

# 3. 產 optimal raceline
./scripts/optimize_raceline.sh <track> mincurv
./scripts/plot_raceline.py <track>   # 視覺化

# 4. 跑車
./scripts/run_pf_pp.sh <track>      # Pure Pursuit
./scripts/run_pf_mpc.sh <track>     # MPC
```

## Benchmark

| Component | Metric | Value |
|-----------|--------|-------|
| range_libc CPU | PyRayMarching @ 4000p × 61 rays | 5.37 ms (186 Hz) |
| range_libc GPU | PyRayMarchingGPU @ same | **1.20 ms (833 Hz)** |
| TUM raceline | 75 pts, 0.05 m step | laptime 2.58s, v=1.32–1.64 m/s |
| colcon build | 6 packages | ~90 s |

Re-run: `python3 scripts/bench_rangelibc.py <track> 4000`

## 關鍵修補記錄

- **VESC odom sign bug** — `vesc_to_odom.cpp:102` 多一個負號，修掉後舊 waypoints 會是鏡像
- **range_libc CUDA** — py2→py3 port、`sm_20` → `sm_87`、`tf.transformations` 換 inline 四元數、shim `distutils.msvccompiler`
- **TUM on aarch64** — quadprog wheel 壞掉改用 cvxopt shim (`quadprog_shim.py`)；修 `spline_approximation.py` scipy 1.15 兼容

## 狀態

- ✅ 離線 pipeline（SLAM → 中線 → TUM raceline）
- ✅ 線上 stack 成功跑過單圈（PF + MPC，2026-04-12）
- ⏳ 大地圖 + raceline 實測、PP vs MPC benchmark（待場地）

## License

MIT

## 致謝

- [MIT Racecar PF](https://github.com/mit-racecar/particle_filter)
- [TUM global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)
- [range_libc](https://github.com/kctess5/range_libc) by Corey Walsh
- [F1Tenth Foundation](https://f1tenth.org/)
