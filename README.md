# vector-tern

Autonomy stack for a 7" drone (Pixhawk 6C running PX4, Luxonis OAK-D, NVIDIA Orin Nano) for an indoor obstacle course with walls, hoops, obstacles, and mapping/localization tasks. The course is unseen until competition day.

This repo starts with **simulation-first verification of visual SLAM**. The simulator is Gazebo Harmonic with PX4 SITL. The same ROS 2 launch files and nodes run in simulation and on the drone.

> **Status:** initial scaffold. It has not yet been built or run end to end. See [TODOs](#todos).

## Architecture

```
OAK-D stereo + IMU ──► cuVSLAM ──► odom_to_px4 ──► PX4 EKF2 ◄── Pixhawk IMU/baro
        │                                              │
        └──► (todo) nvblox map / hoop detector ──► planner ──► offboard setpoints
```

In simulation, Gazebo provides the OAK-D-like sensors, and an odometry plugin publishes **ground truth** for scoring.

## Layout

```
docker/                 Dev image: ROS 2 Humble, Gazebo Harmonic, PX4 v1.15 SITL, XRCE-DDS agent, (optional) Isaac ROS
sim/models/vt_x500_oakd PX4 x500 + stereo/depth/IMU head + ground-truth odometry plugin
sim/tools/              gen_textures.py (feature-rich textures), gen_course.py (procedural courses + hoop ground truth)
ros2_ws/src/
  vt_bringup/           sim.launch.py, slam.launch.py, bridge/cuVSLAM/PX4 configs
  vt_px4_bridge/        Odometry (FLU, z-up) -> PX4 VehicleOdometry (FRD, z-down), with unit tests
  vt_offboard/          Scripted offboard missions: hover, square
  vt_eval/              Records estimate + ground truth to TUM files
scripts/                build_ws.sh, new_course.sh, eval_ate.sh
```

## Requirements

- Ubuntu 22.04 x86_64 with an **NVIDIA GPU**. cuVSLAM and nvblox need CUDA. macOS cannot run this stack.
- Docker with the NVIDIA Container Toolkit.

## Quickstart

```bash
# On the host
xhost +local:docker
INSTALL_ISAAC_ROS=1 docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm dev

# Inside the container (/workspace = this repo)
scripts/build_ws.sh && source ros2_ws/install/setup.bash
scripts/new_course.sh 0                      # -> sim/worlds/generated/course_0.sdf
```

Generate worlds **inside the container**, because the `.sdf` files contain absolute texture paths.

Then use one terminal for each of the following. Open more with `docker exec -it <container> bash`.

```bash
# 1. Simulator + PX4 + bridges
ros2 launch vt_bringup sim.launch.py world:=/workspace/sim/worlds/generated/course_0.sdf

# 2. VSLAM + odometry to PX4
#    Stage A (open loop): PX4 flies on ground truth, VSLAM only observes
ros2 launch vt_bringup slam.launch.py px4_odom_source:=ground_truth
#    Stage B (closed loop): PX4 flies on VSLAM
ros2 launch vt_bringup slam.launch.py px4_odom_source:=slam

# 3. Record for evaluation (Ctrl-C to save)
ros2 run vt_eval trajectory_recorder --ros-args -p use_sim_time:=true -p out_dir:=results/course0_square

# 4. Fly a scripted mission
ros2 run vt_offboard mission --ros-args -p use_sim_time:=true -p mission:=square

# 5. Score
scripts/eval_ate.sh results/course0_square
```

If you have no NVIDIA GPU, run `slam.launch.py run_slam:=false px4_odom_source:=ground_truth`. This still tests the sim, PX4 and offboard control.

## Conventions

- **Frames:** ROS uses REP-103 (FLU body, z-up world). PX4 uses FRD body and a z-down world. `vt_px4_bridge` publishes `POSE_FRAME_FRD`, so the VSLAM heading does not need to be aligned to north.
- **Sensor frames:** `base_link → oakd_link → {oakd_imu, oakd_left(_optical), oakd_right(_optical)}`. These are defined in three places that must agree: `model.sdf`, `sim.launch.py` (`STATIC_TFS`) and `cuvslam.yaml`.
- **IMU noise:** sim noise in `model.sdf` and filter noise in `cuvslam.yaml` are derived from the same densities. Change both together.
- **PX4 parameters:** set in `vt_bringup/config/px4_sitl.params` and applied at startup as `PX4_PARAM_*` environment variables.

## TODOs

### Phase 0: Decisions and hardware facts
- [ ] Confirm PX4 (assumed) vs ArduPilot on the Pixhawk 6C, and the PX4 version flashed on the kits. Keep `PX4_VERSION` and `PX4_MSGS_BRANCH` in the Dockerfile matched to it.
- [ ] Confirm the exact OAK-D model and whether it has an IMU. Update the resolution, HFOV and baseline in `model.sdf`.
- [ ] Dump the real OAK-D calibration (intrinsics and stereo/IMU extrinsics) with `depthai` and copy it into `model.sdf` and `STATIC_TFS`.
- [ ] Measure the real camera mount position on the frame and update `oakd_link` (currently `0.12 0 0.03` from `base_link`).
- [ ] Read the competition rules: extra sensors allowed? Number of laps or attempts? How is mapping scored? Course dimension limits? Use them to set ranges in `gen_course.py`.
- [ ] Choose a license (packages currently say `TODO`).

### Phase 1: Sim + VSLAM scored against ground truth
- [ ] Build the Docker image on the Ubuntu/NVIDIA machine and fix any build breakages. Watch the XRCE agent v2.4.2 Fast-DDS superbuild and the Isaac ROS apt repo/CUDA dependencies.
- [ ] Check that the world loads with `gz sdf -k` and in Gazebo. Check that textures render and that the hoop ring segments line up.
- [ ] Check that PX4 standalone mode binds to `vt_drone` in world `course`. Check that `PX4_PARAM_*` environment variables apply (`param show EKF2_EV_CTRL`).
- [ ] Check the bridged topics: `/vt/{left,right}/image_raw`, `camera_info`, `/vt/imu`, `/vt/ground_truth/odom`, `/clock`. Confirm that the image `frame_id`s are the optical frames (`gz_frame_id`).
- [ ] Verify the cuVSLAM parameter names in `cuvslam.yaml` against the installed Isaac ROS release. Confirm tracking in RViz.
- [ ] Ground truth: the odometry plugin reports the model frame, not `base_link`. Confirm the offset is negligible or correct for it.
- [ ] Stage A: PX4 on ground truth, record VSLAM, score with `eval_ate.sh` on hover, square and full-course flights.
- [ ] Add `yaw_spin`, figure-8 and fast-flight missions to `vt_offboard` (hoop-run comes with the planner in Phase 3).
- [ ] Add an OpenVINS or ORB-SLAM3 comparison baseline, so you can tell sim problems from algorithm problems.

### Phase 2: Closed loop + mapping
- [ ] Stage B: PX4 EKF2 on VSLAM only. Tune `EKF2_EV_DELAY` and the noise parameters. Pass criteria: hover drift < 5 cm over 60 s, ATE < 10 cm per lap.
- [ ] Increment `reset_counter` in `odom_to_px4` on VSLAM resets or relocalization jumps.
- [ ] Failure injection: camera blackout for 2 s, bare-wall pass, kidnapping (teleport the drone). Measure recovery.
- [ ] Add nvblox (depth + VSLAM pose → 3D map of distances to obstacles) and a mapping launch file. Export the map for the mapping deliverable.
- [ ] Add a time-sync stress test: inject a camera–IMU time offset and measure how much error it causes.

### Phase 3: Hoops + planning
- [ ] Synthetic hoop dataset from the sim with labels, plus real hoop photos. Train a detector and export it to TensorRT.
- [ ] Hoop tracker: one Kalman filter per hoop in the world frame, scored against `course_<seed>.json`.
- [ ] Mission state machine (explore → approach → pass → next → map → land) and a minimum-snap planner with map collision checks.
- [ ] Explore-then-exploit: slow mapping lap, save the cuVSLAM map, then relocalize and fly the optimized fast lap.

### Phase 4: Regression on held-out courses
- [ ] Headless scenario runner: N seeds × missions → results table (ATE, collisions, hoops passed, lap time).
- [ ] Split seeds into train, held-out and stress sets (`--light-scale`, `--bare-fraction`, tight gaps).
- [ ] Run nightly or on every major change.

### Phase 5: Hardware
- [ ] Hardware-in-the-loop: run the stack on the Orin Nano and stream sim sensors over Ethernet. Measure latency, GPU load and thermals in the MAXN power mode.
- [ ] Real OAK-D bring-up (`depthai-ros`), with the IR projector off or alternating for VSLAM. Map its topics to the sim topic names.
- [ ] Record hand-carried data through a mock course and score it offline. Tune sim noise, blur and lighting until real and sim failure modes match.
- [ ] Flight progression: shadow mode → tethered hover → single hoop → full mock course → second, unseen real course.
- [ ] Failsafes: VSLAM-loss handling (hover on flow/radar → land), geofence, RC kill switch.

### Optional
- [ ] Downward optical-flow and distance sensor as a VSLAM-loss fallback (native in PX4 EKF2).
- [ ] IWR6843ISK radar: simulate Doppler ego-velocity from a `gpu_lidar` plus ground-truth velocity, then evaluate it as a fallback.
- [ ] Learned policy on abstract inputs (depth + hoop poses + state), trained on generated courses, with the classical planner as fallback.
