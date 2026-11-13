# Plan

## Goal

A multi-sensor, integrity-aware estimator for a drone flying where GNSS is jammed or spoofed. It should keep track of where the drone is, know how far that estimate can be trusted, and make the autopilot act safely when it can't. Everything is built and demonstrated in SITL (Gazebo + ArduPilot) first, ending with one end-to-end demo.

## Who does what

- **Me**: I write it myself, typed in and understood line by line, with Claude explaining, deriving alongside and reviewing. This covers the filter maths.
- **Claude**: Claude writes it and I review it.
- **Shared**: Claude builds the scaffolding, I do the core part.

## Steps

| # | Step | What | Owner | Done when |
|---|---|---|---|---|
| **M0** | **Tooling** | | | |
| 0.1 | Recorder + scorer | Record a run, score vs ground truth (ATE / RPE / scale) | Claude | ✅ 3 cm ATE on first square |
| 0.2 | Bag record / replay | Fly once, replay into the estimator without the sim | Claude | Replays agree with each other and with the live run |
| 0.3 | Fault injector | IMU noise / bias, time offsets, dropped / frozen frames, blur, darkness | Claude | Each fault shown on one replay |
| 0.4 | Sim sensors | New model copy: GNSS, magnetometer, rangefinder, realistic camera and IMU noise | Claude | Topics in ROS 2 |
| 0.5 | Flight set | Easy, medium, hard and weak-motion flights, recorded as bags | Shared (I fly) | Bags for all four |
| 0.6 | Regression suite | Replay all bags, score, fail if worse than a threshold | Claude | One command, runs in CI |
| **M1** | **ESKF core** | | | |
| 1.1 | Derivation | Error state on SO(3), IMU biases, Jacobians for GNSS / baro / mag / range | Me | Written up in `docs/` |
| 1.2 | Filter code | C++ library: propagate, update, chi-square gating | Me | Unit tests pass |
| 1.3 | Headless sim | Fast Monte Carlo simulator | Claude | Hundreds of runs in minutes |
| 1.4 | ROS 2 wrapper + plots | Node around the library, NEES / error plots | Claude | Runs on sim bags |
| 1.5 | Consistency | Monte Carlo NEES, tune until consistent | Me | NEES inside bounds |
| **M2** | **Visual update** | | | |
| 2.1 | MSCKF update | Triangulation, nullspace projection, update (features from the OpenVINS tracker, `ov_core`) | Me | ATE close to OpenVINS |
| 2.2 | Online calibration | Camera-IMU time offset and extrinsics in the state | Me | Recovers injected 5-50 ms offsets |
| 2.3 | Integration | Wire the tracker in, configs, comparison vs OpenVINS | Claude | Side-by-side table |
| 2.4 | Public dataset check (optional) | EuRoC through the same regression suite | Claude | Numbers in the table |
| **M3** | **Integrity** | | | |
| 3.1 | Fault detection and exclusion | Chi-square tests per sensor, exclusion logic | Me | Catches injected faults |
| 3.2 | Protection levels | Error bound from the filter | Me | Error vs protection level plot, compared with 3σ |
| 3.3 | Health states | OK / DEGRADED / LOST to ROS 2 and MAVLink | Claude | Correct on fault bags |
| **M4** | **Closed loop with ArduPilot (SITL)** | | | |
| 4.1 | External nav into EKF3 | Pose + covariance + quality, frame conversion, separate param file | Claude | Loiter on the estimator |
| 4.2 | GNSS jamming | GPS off mid-flight, failsafe if health drops | Shared (I set the failsafe logic) | Drift: stock EKF3 vs this estimator |
| 4.3 | Slow GNSS spoofing | Ramp a GPS offset, detect it by cross-checking with VIO | Shared (I write the detector) | Detection time |
| **M5** | **Depth** | | | |
| 5.1 | FEJ vs invariant EKF | Second filter variant, compare consistency | Me | NEES comparison |
| 5.2 | Observability | What becomes unobservable as sensors drop out | Me | Analysis + plots |
| **M6** | **SITL demo and write-up** | | | |
| 6.1 | Demo scenario | One scripted flight: GPS spoofed, then jammed, then camera degraded; the monitor catches each and the drone stays safe | Shared | Runs end to end from one command |
| 6.2 | Video + plots | Screen capture, trajectory and integrity plots | Claude | In the repo |
| 6.3 | Write-up | Design doc: decisions, alternatives, dead ends, measured results | Shared (I write, Claude edits) | In the repo |
| Stretch | Absolute visual localisation | Downward camera frames matched to satellite imagery, fused as position updates | Shared | Position fix error |

Unit tests and CI run throughout, and every milestone ends with a measured result.

## What each milestone is for

**M0, Tooling.** Make every result repeatable. A flight is flown once and replayed as often as needed, faults are injected on demand, and every change is scored against the same set of flights automatically. Without this, a difference in error can't be told apart from a difference in the flight.

**M1, ESKF core.** An error-state filter on SO(3) fusing IMU, GNSS, barometer, magnetometer and rangefinder, derived from scratch. The filter has to be consistent, meaning its covariance matches its real error (checked with Monte Carlo NEES), because everything later relies on that covariance being honest.

**M2, Visual update.** Add the camera as a tightly coupled MSCKF update inside the same filter, with online estimation of the camera-IMU time offset and extrinsics. Accuracy should be close to OpenVINS on the same flights, and injected time offsets should be recovered.

**M3, Integrity.** Let the filter judge its own measurements and its own estimate. Chi-square tests detect and exclude faulty sensors, protection levels give an error bound that holds more reliably than 3σ, and a simple health state (OK / DEGRADED / LOST) is published for the autopilot to use.

**M4, Closed loop.** Put the estimator in charge of the drone through ArduPilot's EKF3 and show what happens when GNSS goes away or starts lying: with GPS jammed the drone keeps flying on the estimator, with GPS slowly spoofed the offset is caught by cross-checking against VIO, and failsafes trigger when health drops. Results are compared against stock EKF3.

**M5, Depth.** Understand why the filter behaves as it does. Compare first-estimates Jacobians against an invariant EKF for consistency, and show analytically and in plots which directions become unobservable as sensors drop out (yaw without magnetometer or GNSS, scale when hovering).

**M6, SITL demo and write-up.** One scripted flight that shows the whole system working: spoofing, then jamming, then a degraded camera, each detected and handled. Plus a video, plots, and a write-up of the decisions, the alternatives considered, what failed along the way and the measured results.

## Notes

- Baseline is OpenVINS.
- Integrity work is presented as engineering, not as a new research result. Related work: VINS-Integrity (IEEE 2025), IM-GIV, and the Zhu et al. survey (NAVIGATION 2022).

---

*Physical rig, after the SITL demo: Jetson Orin Nano, Intel RealSense D455, a mono global shutter camera and a Holybro rangefinder. The sim's sensor noise values are taken from these parts' datasheets so the move to hardware is shorter. Later steps: bench calibration, handheld recording, logging-only flights, then the estimator in the loop.*
