# udl_disha

Visual-inertial odometry (VIO) on a simulated quadcopter, and what happens to it when things go wrong.

The drone is an ArduPilot SITL iris flying in a Gazebo Harmonic warehouse. It carries a forward-facing camera and its own IMU, and [OpenVINS](https://github.com/rpng/open_vins) estimates its motion from those two sensors. Gazebo gives exact ground truth, so the estimate can be scored, and the plan is to break the inputs on purpose (timing offsets, IMU noise, dropped or dark frames) and see how the error grows. The end goal is a small health monitor that says OK / DEGRADED / LOST before the estimate actually falls apart.

## Status

Work in progress.

- [x] Sim drone with a fixed forward camera, a VIO IMU and ground truth
- [x] Camera, IMU and ground truth available in ROS 2
- [x] OpenVINS built and configured for the sim camera and IMU
- [x] Repeatable scripted test flight
- [ ] OpenVINS running on the test flight, compared with ground truth
- [ ] Baseline accuracy (ATE / RPE)
- [ ] Stress tests: time offset, IMU noise, dropped frames, blur, darkness
- [ ] Health monitor node (C++)
- [ ] Monitor validation on the stress tests

## Layout

```
scripts/
  start_vio_sim.sh          start / stop the whole sim in terminal tabs
vin_ws/
  config/openvins_sim/      OpenVINS config for the sim camera and IMU
  src/gz_imu_bridge/        Gazebo IMU -> ROS 2 (see docs for why this exists)
  src/vio_flight/           scripted test flight over MAVROS
docs/
  how_it_works.md           how the pieces fit together
  maths.md                  the bits of maths behind the config
```

## Requirements

- Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic
- ArduPilot SITL, MAVROS, `ros_gz_bridge`
- Gazebo Python bindings: `python3-gz-transport13`, `python3-gz-msgs10`
- OpenVINS built in its own colcon workspace

Some pieces live outside this repo, in the `udl_aakash` workspace on my machine, and the scripts point at them by absolute path:

- ArduPilot source and build: `udl_aakash/tools/ardupilot`
- `ardupilot_gazebo` plugin, plus the model `iris_with_vio_camera` and world `iris_warehouse_vio.sdf` in its `models/` and `worlds/` folders
- OpenVINS workspace: `udl_aakash/vio_ws`

If you're setting this up elsewhere, change the paths at the top of `scripts/start_vio_sim.sh`.

## Running it

Run these from a normal terminal. VS Code installed as a snap breaks `gnome-terminal` when launched from its built-in terminal.

Start the sim (Gazebo, ArduPilot, MAVROS, camera bridge, IMU bridge):

```bash
bash scripts/start_vio_sim.sh --start
```

Running the script with no option just prints help. It never builds anything; ArduPilot is started with `--no-rebuild`.

Check the topics are there:

```bash
ros2 topic list | grep vio
# /vio/camera/image  /vio/camera/camera_info  /vio/imu  /vio/ground_truth/odom
```

Start OpenVINS with the sim config (in another terminal):

```bash
source /opt/ros/humble/setup.bash
source /data/projects/udl_aakash/vio_ws/install/setup.bash
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=$PWD/vin_ws/config/openvins_sim/estimator_config.yaml \
  max_cameras:=1 use_stereo:=false rviz_enable:=true
```

Fly the test pattern, a 2 m square at 1.5 m facing the shelves:

```bash
cd vin_ws/src/vio_flight
python3 vio_flight.py                       # defaults
python3 vio_flight.py --side 3 --loops 2    # bigger square, twice
```

Stop everything and close the tabs:

```bash
bash scripts/start_vio_sim.sh --stop
```

OpenVINS doesn't always exit on Ctrl-C, so check it's gone (`pgrep -l run_subscribe`) and kill it if needed.

## Notes

- The sim runs at roughly 0.5x real time on my laptop (i7-13620H, RTX 3050). ArduPilot runs in lock-step, so results are still correct, just slower.
- `vin_ws/src/vio_flight/mavros_adapter.py` is copied unchanged from my `ardupilot_autonomy` project.
