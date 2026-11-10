# The maths

Just enough to explain the numbers in `vin_ws/config/openvins_sim`. None of this is new; it's the standard VIO setup applied to this sim.

## Camera: pinhole model

A point at $(X, Y, Z)$ in the camera frame (z forward, x right, y down) lands on pixel

$$
u = f_x \frac{X}{Z} + c_x, \qquad v = f_y \frac{Y}{Z} + c_y
$$

Gazebo describes the camera by its horizontal field of view instead of a focal length. The edge of the image is at $u = W$, which is $W/2$ pixels from the centre at an angle of $\text{hfov}/2$, so

$$
f_x = \frac{W/2}{\tan(\text{hfov}/2)} = \frac{320}{\tan 45^\circ} = 320
$$

Pixels are square, so $f_y = f_x$, and the centre is $(c_x, c_y) = (320, 240)$. The sim lens is perfect, so the distortion terms are all zero. Gazebo reports 319.999 on `camera_info` and that's what goes in the config.

## Camera to IMU

OpenVINS wants the rotation from camera to IMU, $R_{CI}$, and the camera's position in the IMU frame, $p_C$.

The IMU frame is the drone body: x forward, y left, z up. The camera frame is z forward, x right, y down. The columns of $R_{CI}$ are just the camera axes written in IMU coordinates:

- camera x (right) = IMU $-y$
- camera y (down) = IMU $-z$
- camera z (forward) = IMU $+x$

$$
R_{CI} = \begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix},
\qquad p_C = \begin{bmatrix} 0.10 \\ 0 \\ -0.05 \end{bmatrix} \text{ m}
$$

A quick check: $\det R_{CI} = 1$ and $R_{CI} R_{CI}^T = I$, so it's a proper rotation. Getting this matrix wrong is the most common reason VIO won't work on a new setup, so it's worth checking by hand.

## IMU model

The IMU doesn't measure motion directly. It measures

$$
\omega_m = \omega + b_g + n_g, \qquad a_m = R^T (a - g) + b_a + n_a
$$

where $b$ are slowly drifting biases and $n$ is white noise. The accelerometer sees gravity too; at rest it reads $+9.8$ upward, not zero. OpenVINS has to know $|g|$ to separate gravity from real acceleration, so `gravity_mag` is set to Gazebo's 9.8 rather than the usual 9.81.

The config gives the noise as continuous-time densities. To get the noise on a single sample, multiply by $\sqrt{\text{rate}}$:

$$
\sigma_a = 2 \times 10^{-3} \cdot \sqrt{200} \approx 0.028 \ \text{m/s}^2
$$

The sim IMU is actually perfect. The values are kept non-zero (EuRoC's numbers) because they tell the filter how much to trust the IMU against the camera, and a filter that trusts one sensor completely tends to break.

## Why the time offset isn't estimated

If the camera clock is off by $t_d$, each image is matched to the IMU state at the wrong moment. While rotating at $\omega$, that's an attitude error of about $\omega \, t_d$, which shows up in the image as a shift of roughly

$$
\Delta u \approx f_x \, \omega \, t_d
$$

At 1 rad/s and $t_d = 10$ ms that's $320 \times 1 \times 0.01 \approx 3$ pixels. That's small, but it's systematic and it happens on every frame.

OpenVINS can estimate $t_d$ online, and normally you'd let it. Here it's turned off (`calib_cam_timeoffset: false`), because the time offset stress test injects 5, 10 and 20 ms on purpose. If the filter quietly corrected the offset, the test would show nothing. The same goes for the intrinsics and extrinsics: the sim values are exact, so there's nothing to learn and turning calibration off keeps the comparison honest.

## Why one camera is enough (and why it's harder)

With one camera, the images on their own fix the shape of the motion but not its size: everything could be twice as big and twice as far away. The IMU fixes the scale, because acceleration is measured in m/s². But that only works once the drone actually accelerates. Hovering perfectly still, scale is unobservable.

That's why OpenVINS waits for a "jerk" in the accelerometer before it initialises (`init_imu_thresh`), and why the takeoff matters. It's also why mono VIO is a good subject for this project: it has more ways to fail than stereo.

## Zero velocity updates

When the drone sits still, nothing moves in the image, so features have no parallax and can't be triangulated. The camera stops correcting the filter, and the IMU integration is left on its own. Position error from a small acceleration error grows with $t^2$, which is most likely what the runaway after landing was.

A zero velocity update (`try_zupt`) tells the filter what it can't see for itself. When features move less than `zupt_max_disparity` (0.5 px), OpenVINS treats the drone as stopped, so the IMU should read only its biases and gravity:

$$
\omega_m - b_g \approx 0, \qquad a_m - b_a \approx -R^T g
$$

One catch for the stress tests: a frozen camera also has zero feature motion, so it could look like the drone stopped.

## Scoring: alignment, ATE and RPE

OpenVINS's world frame starts where it initialises, with a heading it picks itself. Gravity pins down roll and pitch, but position and heading are unobservable from a camera and an IMU alone. So before comparing to ground truth, the estimate gets one rotation about the vertical, $R_z(\psi)$, and one shift $t$, chosen to minimise

$$
\sum_i \left\| R_z(\psi)\, p^{\text{est}}_i + t - p^{\text{gt}}_i \right\|^2
$$

With both point sets centred (subtract their means, giving $e_i$ and $g_i$), the best heading has a closed form:

$$
\psi = \operatorname{atan2}\!\Big(\sum_i e_{x,i} g_{y,i} - e_{y,i} g_{x,i},\ \sum_i e_{x,i} g_{x,i} + e_{y,i} g_{y,i}\Big)
$$

and then $t = \bar p^{\text{gt}} - R_z(\psi)\, \bar p^{\text{est}}$. Roll and pitch aren't aligned away, because a VIO that gets gravity wrong should be penalised for it.

**ATE** (absolute trajectory error) is the RMSE of the aligned position error, $\sqrt{\tfrac1N \sum \| p^{\text{al}}_i - p^{\text{gt}}_i \|^2}$, plus the same for the rotation angle between the estimated and true orientation. ATE is a single number for the whole run. It also depends on when the error happened, because an early error gets carried along for the rest of the flight.

**RPE** (relative pose error) looks at short pieces instead. For poses $i$ and $j$ one second apart, it compares how far the estimate says the drone moved with how far it really moved, both measured in the body frame at $i$:

$$
\Delta^{\text{est}} = R_i^{\text{est}\,T}\big(p^{\text{est}}_j - p^{\text{est}}_i\big), \qquad
e_{ij} = \big\| \Delta^{\text{est}} - \Delta^{\text{gt}} \big\|
$$

RPE doesn't need any alignment, and it shows local drift rate, which is what a fault would make worse.

**Scale error.** A mono VIO that gets scale wrong draws the right shape at the wrong size. To measure that, a separate similarity fit $p^{\text{gt}} \approx s R\, p^{\text{est}} + t$ is solved with Umeyama's SVD method, and $s - 1$ is reported. It is reported only and not applied to the ATE, so a scale error still counts as error.

## What OpenVINS does with all this

OpenVINS is an MSCKF, a Kalman filter that keeps a short sliding window of past camera poses (11 here, `max_clones`) instead of keeping every feature in the state. The IMU predicts forward between images. When a tracked feature has been seen from several poses in the window, it gets triangulated and used to correct all of those poses at once, then thrown away. A few long-lived features (`max_slam`) stay in the state as landmarks.

The filter's covariance, its own estimate of how unsure it is, is one of the signals the health monitor will watch. If it grows while the feature count drops, things are going bad.
