# How it works

## The pipeline

```
Gazebo (warehouse + iris_with_vio_camera)
  |-- camera 30 Hz ----> ros_gz_bridge ----> /vio/camera/image ----+
  |-- VIO IMU 200 Hz --> gz_imu_bridge.py -> /vio/imu ------------+--> OpenVINS --> estimated pose
  |-- ground truth ----> ros_gz_bridge ----> /vio/ground_truth/odom          (compared later)
  |
  +-- ArduPilot plugin <--> ArduPilot SITL <--> MAVROS <--> vio_flight.py
```

There are two separate loops here. ArduPilot flies the drone using its own sensors and simulated GPS. OpenVINS only watches the camera and the VIO IMU, and nothing it estimates goes back to the autopilot. That keeps the experiment clean: when VIO fails, the drone keeps flying the same path, so we can measure the failure without it changing the flight.

## The drone

`iris_with_vio_camera` is the stock `iris_with_ardupilot` with three things bolted on:

- **A forward camera, rigidly fixed.** 640x480 grayscale at 30 Hz with a 90 degree field of view. The existing camera models were pointing down or sat on a gimbal, and VIO needs the camera locked to the IMU. If the camera can move relative to the IMU, the geometry OpenVINS relies on is wrong all the time.
- **A second IMU, just for VIO.** ArduPilot's IMU is mounted upside down (rolled 180 degrees) and has its own job. A separate IMU at 200 Hz, aligned with the body, keeps the calibration simple. It also means noise can be added to it later without upsetting the autopilot.
- **A ground truth publisher.** Gazebo's odometry plugin publishes the true pose at 100 Hz.

Nothing is noisy yet: no image noise, no IMU noise. The baseline is meant to be clean so that the stress tests are the only thing changing.

The world is the warehouse from the gnc_autonomy project. Shelves, barrels, crates and brick walls give lots of corners to track, which the runway world didn't have.

## Why there's a custom IMU bridge

The installed `ros_gz_bridge` was built for Gazebo Garden, but the sim runs Harmonic. Images and odometry come through fine. IMU messages come through with the acceleration as zeros, which is useless for VIO.

Rather than replace the system bridge (an apt install, or a source build of `ros_gz`), `gz_imu_bridge.py` reads the IMU with Harmonic's own Python bindings and republishes it as `sensor_msgs/Imu`. It keeps Gazebo's sim-time stamp. It's about 60 lines and holds 200 Hz with no gaps at roughly 18% of one core.

## The test flight

Stress tests only mean something if every run flies the same path, so the flight is scripted. `vio_flight.py` uses `mavros_adapter.py` for the MAVROS calls and does:

1. wait for the MAVROS link, switch to GUIDED, arm
2. take off to 1.5 m
3. fly a 2 m square (forward, left, back, right) keeping the heading fixed, so the camera always faces the shelves
4. land and wait for disarm

The adapter's commands are fire-and-forget: `arm()` returns as soon as the request is sent, not when the drone is armed. So the script checks telemetry after every step and re-sends a command if nothing happens. Each step has a timeout; on a timeout it lands.

## Start / stop script

`start_vio_sim.sh --start` opens one terminal tab per process, in the same style as the older `start_all.sh`. ArduPilot runs from `runs/sitl` so its parameters and logs don't mix with other projects.

`--stop` finds the sim processes by what program they actually are (gz, arducopter, mavros_node, parameter_bridge, or Python running sim_vehicle / mavproxy / ros2 / the IMU bridge). Matching on command-line text alone turned out to be a bad idea, because it also hit unrelated shells that merely mentioned those names. Each process gets Ctrl-C first, then TERM, then KILL if it's still hanging around, and then the tabs are closed.
