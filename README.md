# Differential-Drive Robot: Kinematic Modelling, Calibration & Odometry

A LEGO EV3 differential-drive robot used as a testbed for turning mathematical models into motion. The robot carries a downward-facing marker on its axle midpoint, so every run leaves a physical trace on paper. That trace is the ground truth the models are measured against.

The project covers four areas:

- **Trajectory generation** from closed-form kinematics: straight lines, circles, rectangles, and a Bernoulli lemniscate driven by continuously varying curvature.
- **Calibration** of the robot's effective track width from measured test circles.
- **Dead-reckoning odometry** that integrates wheel-encoder readings into a pose estimate, checked against the gyroscope and the drawn trace.
- **Reactive sensor-to-motor control**: four Braitenberg-vehicle behaviours driven by two light sensors.

Everything is written in Python on [ev3dev](https://www.ev3dev.org/) using `python-ev3dev2`.

---

## Hardware

| Component | Detail |
|---|---|
| Controller | LEGO Mindstorms EV3 brick running ev3dev |
| Drive | 2 x EV3 Large Motor (ports A and B), one per wheel |
| Wheels | 56 mm diameter, nominal track width 15.2 cm (wheel centre to wheel centre) |
| Third contact | Ball caster |
| Heading sensor | EV3 Gyro Sensor (port 1), mounted flat on the brick |
| Light sensors | 2 x EV3 Color Sensor in ambient mode (ports 2 and 3), front-mounted and symmetric about the centreline |
| Ground-truth tracer | Marker mounted vertically on the axle line, halfway between the wheels |

Design choices that matter for accuracy:

- **Marker on the axle midpoint.** The drawn line is the exact path of the robot's reference point, so no offset correction is needed.
- **Rigid motor mounts.** They keep the wheels aligned from run to run.
- **Narrow tyres.** They reduce scrubbing during differential turns.
- **Centred brick.** It balances the load between the two drive wheels.

---

## Mathematical model

### Frame and conventions

- **Units:** position in cm, orientation in radians internally (reported in degrees).
- **Reference point:** the midpoint of the wheel axle, which is also where the marker sits.
- **Frame:** the world frame is fixed at the start pose. $x$ points forward along the initial heading.
- **Sign convention for $\theta$:** positive is counter-clockwise, and headings are wrapped to $[-180º, 180º)$.

### Differential-drive kinematics

With wheel linear speeds $v_L, v_R$ and track width $W$:

$$
v = \frac{v_R + v_L}{2}, \qquad \omega = \frac{v_R - v_L}{W}
$$

The inverse, used to drive a desired motion:

$$
v_L = v - \omega \frac{W}{2}, \qquad v_R = v + \omega \frac{W}{2}
$$

Wheel speeds are converted to motor commands in degrees per second with wheel radius $r$:

$$
\dot\phi = \frac{v}{r} \cdot \frac{180}{\pi}
$$

### Circle of radius $R$

Both wheels must share one instantaneous centre of rotation, so their speeds are in the same ratio as their radii:

$$
\frac{v_{\text{inner}}}{v_{\text{outer}}} = \frac{R - W/2}{R + W/2}
$$

The arc is ended by the gyroscope rather than a timer. The robot stops a few degrees before 360º to allow for braking overshoot.

### Bernoulli lemniscate (figure-eight)

The curve is parameterised as

$$
x(u) = \frac{a\cos u}{1+\sin^2 u}, \qquad y(u) = \frac{a \sin u \cos u}{1+\sin^2 u}
$$

The parameter advances at a constant rate, $u = \pi/2 + \alpha t$ with $\alpha = 2\pi / T$, so one full figure-eight takes $T$ seconds. The robot starts at the crossing point. Differentiating the curve gives the path speed and the signed curvature:

$$
v(u) = \frac{a\,\alpha}{\sqrt{1+\sin^2 u}}, \qquad \kappa(u) = \frac{3\cos u}{a\sqrt{1+\sin^2 u}}
$$

The turn rate is then $\omega = v\kappa$, which feeds the inverse kinematics above. The robot never follows two stitched arcs; every wheel command comes directly from the curve's geometry.

Parameters used: $a = 50$ cm, $T = 45$ s, update interval $\Delta t = 30$ ms. Motor acceleration ramps are disabled for this mode so the wheels follow the commanded speeds immediately.

**Discretisation error.** Between updates the wheel speeds are held constant, so the robot actually traces a chain of short constant-curvature arcs that approximates the smooth curve. Shrinking $\Delta t$ reduces this error, but it doesn't touch the mechanical error sources discussed below.

### Dead-reckoning odometry

The robot runs a user-entered sequence of `(left %, right %, duration)` commands. While it drives, encoder readings are sampled every $\Delta t = 0.05$ s (20 Hz). Each sample is converted to wheel travel:

$$
d_L = \frac{\Delta\phi_L}{360}\,C, \qquad d_R = \frac{\Delta\phi_R}{360}\,C, \qquad C = \pi D_{\text{wheel}}
$$

$$
\Delta s = \frac{d_R + d_L}{2}, \qquad \Delta\theta = \frac{d_R - d_L}{W}
$$

The pose is advanced using the heading at the middle of the step. This is second-order accurate for arcs, where using the start-of-step heading would only be first-order:

$$
\theta_{\text{mid}} = \theta + \tfrac{1}{2}\Delta\theta
$$

$$
x \leftarrow x + \Delta s\cos\theta_{\text{mid}}, \qquad y \leftarrow y + \Delta s\sin\theta_{\text{mid}}, \qquad \theta \leftarrow \theta + \Delta\theta
$$

At the end of the sequence, the estimated heading is compared with the gyroscope's heading.

### Track-width calibration

Commanded circles of radius 50 cm came out with a consistent diameter of 106-108 cm. That points to the effective track width being larger than the nominal 15.2 cm, probably because the wide rubber tyres touch the ground over an area rather than a single point. Scaling the track width by the measured-to-commanded radius ratio gives:

$$
W_{\text{eff}} = W \cdot \frac{R_{\text{meas}}}{R_{\text{cmd}}} = 15.2 \cdot \frac{53.5}{50} \approx 16.3 \text{ cm}
$$

This corrected value is used when computing the circle's inner and outer wheel speeds. The same gyroscope-terminated run also gives an estimate of the effective wheel circumference:

$$
C_{\text{eff}} = \frac{\Delta\theta_{\text{gyro}} \cdot W_{\text{eff}}}{|n_R - n_L|}
$$

Here $n_L, n_R$ are the wheel revolutions counted during the arc.

### Braitenberg behaviours

Each light reading is normalised to $s \in [0, 1]$ using calibrated dark and bright levels, then mapped to wheel speeds. The four behaviours differ in two choices: which wheel each sensor drives, and whether brighter light means faster or slower.

| Behaviour | Wiring | Brighter light means | Result |
|---|---|---|---|
| Cowardice | Same side | Faster | Turns away from the light |
| Aggression | Crossed | Faster | Turns toward the light and speeds into it |
| Love | Same side | Slower | Approaches the light and stops facing it |
| Curiosity | Crossed | Slower | Approaches, then turns away and keeps exploring |

---

## Results

Every trajectory was run at least three times from the same start pose. The encoder-based measurement was compared against an independent physical measurement (ruler on the drawn trace).

### Straight line (1 m)

| Speed | Encoder error | Physical end error |
|---|---|---|
| 20% | < 0.04 cm | Stopped 2.0-2.8 cm short, 3.5-6.0 cm lateral drift |
| 40% | < 0.11 cm | Stopped 2-3 cm short, 2.0-5.5 cm lateral drift |
| 80% | < 0.06 cm | Stopped 2.7-3.0 cm short, 2.0-3.0 cm lateral drift |

The encoders report near-perfect execution because they only see wheel rotation. The drawn trace shows a repeatable undershoot and sideways drift, which is systematic bias from slip and small mechanical asymmetries, not random noise.

### Circle (R = 50 cm)

| Speed | Measured diameter | Start-to-end gap |
|---|---|---|
| 20% | 106.5-108 cm | 31-36 cm |
| 40% | 106-108 cm | 29-35 cm |
| 60% | 107-107.5 cm | 32-33.5 cm |

The consistent ~7% oversize is what motivated the track-width calibration above. Speed had little effect across 20-80%, so mechanical effects dominate over speed-dependent ones in this range.

### Rectangle (1 m x 0.5 m)

Closure error (final position vs. start) was 4.3 cm, 13.0 cm, and 27.0 cm across three runs. The error grows at each corner: every 90º pivot adds a small heading error, and the straight legs that follow turn that heading error into position error.

### Lemniscate (a = 50 cm)

Closure errors were 14 cm, 14 cm, and 12 cm (mean ≈ 13.3 cm). The close agreement between runs shows the curvature-driven controller is repeatable. The remaining error is mostly systematic.

### Dead-reckoning odometry

| Run | Estimated pose (x, y, θ) | Measured pose (x, y, θ) | Position error | Heading error |
|---|---|---|---|---|
| 1 | (105.8, 24.4, -88.7º) | (111.0, 21.0, -102.0º) | 6.2 cm | -13.3º |
| 2 | (105.3, 25.4, -91.4º) | (111.5, 22.0, -105.0º) | 7.1 cm | -13.6º |
| 3 | (105.8, 22.7, -88.8º) | (110.0, 20.0, -103.0º) | 5.0 cm | -14.2º |
| **Mean** | | | **6.1 cm** | **-13.7º** |

The heading error is nearly identical across runs, about 14º every time. That consistency points to a calibration issue rather than noise. The odometry loop still uses the nominal track width, and a width that is too small makes the estimate under-report how far the robot turned.

---

## Known limitations and next steps

- **Use the calibrated track width in the odometry loop.** Because the heading error is so consistent, this should remove most of it.
- **Close the loop on heading.** A gyro-based PID controller on straight segments and pivots would stop small heading errors from piling up over a rectangle.
- **Fuse gyro and encoders.** Combining the two with a complementary filter should beat either sensor alone.
- **Use a fixed-rate control loop.** The loops currently use `time.sleep`, so the real update interval drifts with processing time. Measuring the actual elapsed time and using it in each step would fix this.

---

## Running it

Requires an EV3 brick running ev3dev with `python-ev3dev2`, and the motors and sensors connected on the ports listed under Hardware.

```bash
# copy to the brick, then on the brick:
chmod +x main.py
./main.py
```

A terminal menu selects each behaviour: straight line, circle, rectangle, lemniscate, 90º turn, the four Braitenberg modes, and `cm` for command-sequence input with odometry.

---

## Team

Built by **Robert Polishchuk**, **Vladislav Marchenko**, and **Ruslan Kurbanov** at the University of Alberta.
