#!/usr/bin/env python3
"""
Differential-drive robot: kinematic modelling, calibration and odometry.

Runs on a LEGO EV3 brick under ev3dev. Provides open-loop trajectory
generation (straight line, circle, rectangle, Bernoulli lemniscate),
encoder-based dead-reckoning odometry, and four Braitenberg light-reactive
behaviours, all selectable from a terminal menu.

Conventions:
    - Position in cm, orientation in radians internally (reported in degrees).
    - Reference point is the midpoint of the wheel axle.
    - Positive theta is counter-clockwise; headings are wrapped to [-180, 180).
"""
import time, math
from ev3dev2.motor import MoveTank, LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3
from ev3dev2.sensor.lego import GyroSensor, ColorSensor

# ---------------------------------------------------------------------------
# Hardware setup
# ---------------------------------------------------------------------------

# Front-mounted light sensors, read as ambient light intensity
left_light = ColorSensor(INPUT_2)
right_light = ColorSensor(INPUT_3)

left_light.mode = 'COL-AMBIENT'
right_light.mode = 'COL-AMBIENT'

# Light calibration and speed limits for the Braitenberg behaviours
LIGHT_MIN = 2.0     # Ambient reading in a normally lit room (maps to 0)
LIGHT_MAx = 25.0    # Reading with a lamp held close (maps to 1)
MIN_SPEED = 15.0    # Motor speed (%) at the dark end of the range
MAx_SPEED = 50.0    # Motor speed (%) at the bright end of the range

gyro = GyroSensor(INPUT_1)
gyro.reset()

robot_drive = MoveTank(OUTPUT_A, OUTPUT_B)

left_motor = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_B)

# Acceleration ramps (ms to reach full speed) to limit wheel slip on start and stop
left_motor.ramp_up_sp = 2000
left_motor.ramp_down_sp = 2000

right_motor.ramp_up_sp = 2000
right_motor.ramp_down_sp = 2000

# ---------------------------------------------------------------------------
# Geometry and calibration
# ---------------------------------------------------------------------------

WHEEL_DIAMETER = 5.6    # cm
BASE_WIDTH = 15.2       # Nominal track width, wheel centre to wheel centre (cm)
OUTER_SPEED = 40        # Default drive speed (%)
TARGET_ANGLE = 87

CIRCUMFERENCE = WHEEL_DIAMETER * math.pi

# Effective track width, calibrated from test circles. Circles commanded at
# R = 50 cm measured 108, 107 and 106 cm across (mean R = 53.5 cm), so the
# nominal width is scaled by the measured-to-commanded radius ratio.
W_TRUE = BASE_WIDTH * ((108.0 + 107.0 + 106.0) / 3.0 / 2.0) / 50.0

required_rotations = 0.0

# ---------------------------------------------------------------------------
# Trajectories
# ---------------------------------------------------------------------------

def straight_line(distance_cm):
    """Drive straight for distance_cm using encoder-counted wheel rotations."""
    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED), SpeedPercent(OUTER_SPEED), distance_cm / CIRCUMFERENCE) 


def circle_calibrated(lead_deg=6.0):
    """
    Drive a full circle of radius 50 cm using the calibrated track width.

    Wheel speeds are set in the ratio of the inner and outer wheel radii so
    both wheels share one instantaneous centre of rotation. The gyro ends the
    arc lead_deg before 360 degrees to compensate for braking overshoot.
    Afterwards, the encoder and gyro readings are used to estimate the
    effective wheel circumference and each wheel's travelled distance.
    """
    print("\n=== Starting Circle ===")

    r_outer = 50.0 + (W_TRUE / 2.0) # Path radius of the outer (left) wheel
    r_inner = 50 - (W_TRUE / 2.0)   # Path radius of the inner (right) wheel
    inner_speed = OUTER_SPEED * (r_inner / r_outer) # Same angular velocity for both wheels about the centre

    gyro.reset()
    time.sleep(0.5)	# Allow the gyro to settle after reset

    start_l = left_motor.position	# Encoder baselines (deg)
    start_r = right_motor.position

    robot_drive.on(SpeedPercent(OUTER_SPEED), SpeedPercent(inner_speed))

    # Poll the gyro at ~100 Hz until the turn is nearly complete
    while abs(gyro.angle) < (360.0 - lead_deg):
        time.sleep(0.01)

    robot_drive.off(brake=True)
    time.sleep(0.3) # Let the robot come to rest before reading sensors

    ticks_l = left_motor.position - start_l     # Wheel rotation during the arc (deg)
    ticks_r = right_motor.position - start_r

    heading_rad = math.radians(abs(gyro.angle))	# Heading change measured by the gyro
    rot_diff = abs(ticks_r - ticks_l) / 360.0	# Difference in wheel revolutions
    
    # Effective circumference: C = (delta_theta * W) / (n_R - n_L).
    # Falls back to the nominal value if the wheels barely differed.
    if rot_diff > 0.01:
        circ_eff = (heading_rad * W_TRUE / rot_diff)
    else:
        circ_eff = CIRCUMFERENCE	

    left_cm = (ticks_l / 360.0) * circ_eff	# Distance travelled by each wheel (cm)
    right_cm = (ticks_r / 360.0) * circ_eff

    # Report measured distances against the ideal circle
    print("CIRCLE completed")
    print("Final heading: {} deg".format(gyro.angle))
    print("Effective wheel circumference: {:.2f} cm".format(circ_eff))
    print("Left  wheel: {:.2f} cm (target {:.2f})".format(left_cm, 2 * math.pi * r_outer))
    print("Right wheel: {:.2f} cm (target {:.2f})".format(right_cm, 2 * math.pi * r_inner))
    print("Centre path: {:.2f} cm (target {:.2f})".format((left_cm + right_cm) / 2.0, 2 * math.pi * 50))

def rectangle():
    """
    Drive a 100 cm x 50 cm rectangle as four straight legs joined by
    in-place 90 degree turns, then report the encoder-estimated linear
    distance for each wheel against the 300 cm perimeter.
    """
    print("\n=== Starting Rectangle ===")
    
    start_r_moto_pos = right_motor.position
    start_l_moto_pos = left_motor.position
    time.sleep(0.5)

    theoretical_linear_distance_cm = 300.0

    straight_line(100)
    turn_90()

    straight_line(50)
    turn_90()

    straight_line(100)
    turn_90()

    straight_line(50)
    turn_90()

    robot_drive.off(brake=True)
    time.sleep(0.5)

    # Encoder rotation spent on the four pivots, removed from the linear total
    turn_rotations_total = (BASE_WIDTH / (4.0 * WHEEL_DIAMETER)) * 4
    turn_degrees_total = turn_rotations_total * 360.0

    actual_left_linear_cm = ((abs(start_l_moto_pos) - turn_degrees_total) / 360.0) * CIRCUMFERENCE
    actual_right_linear_cm = ((abs(start_r_moto_pos) - turn_degrees_total) / 360.0) * CIRCUMFERENCE

    left_encoder_error_cm = abs(theoretical_linear_distance_cm - actual_left_linear_cm)
    right_encoder_error_cm = abs(theoretical_linear_distance_cm - actual_right_linear_cm)

    print("\nRECTANGLE completed")
    print("[Method 1] Left Wheel Estimated Linear: {:.2f} cm (Error: {:.2f} cm)".format(actual_left_linear_cm, left_encoder_error_cm))
    print("[Method 1] Right Wheel Estimated Linear: {:.2f} cm (Error: {:.2f} cm)".format(actual_right_linear_cm, right_encoder_error_cm))

def lemniscate():
    """
    Follow a Bernoulli lemniscate (figure-eight) with a = 50 cm.

    The curve parameter advances at a constant rate, u = pi/2 + alpha * t,
    so one full loop takes T seconds and the robot starts at the crossing
    point. At each step the path speed v(u) and signed curvature kappa(u)
    are computed analytically, converted to a turn rate omega = v * kappa,
    and mapped to wheel speeds through inverse differential-drive kinematics.

    Wheel speeds are held constant between updates (every dt seconds), so the
    robot traces a chain of short constant-curvature arcs. This is the source
    of the discretisation error.
    """
    a = 50.0                        # Lemniscate half-width (cm)
    W = BASE_WIDTH
    wheel_radius = WHEEL_DIAMETER / 2.0

    T = 45.0    # Duration of one full loop (s)
    dt = 0.03   # Control update interval (s)
    alpha = 2.0 * math.pi / T       # Rate of the curve parameter (rad/s)

    # Disable acceleration ramps so the wheels track rapidly changing commands
    robot_drive.left_motor.ramp_up_sp = 0
    robot_drive.left_motor.ramp_down_sp = 0
    robot_drive.right_motor.ramp_up_sp = 0
    robot_drive.right_motor.ramp_down_sp = 0

    start_time = time.time()

    while True:

        elapsed = time.time() - start_time

        if elapsed >= T:
            break

        # Curve parameter; starting at pi/2 places the robot at the crossing point
        u = math.pi / 2.0 + alpha * elapsed

        sin_u = math.sin(u)
        cos_u = math.cos(u)

        # Path speed: |dr/dt| = a * alpha / sqrt(1 + sin^2 u)
        v = (
            a * alpha
            / math.sqrt(1.0 + sin_u * sin_u)
        )

        # Signed curvature: kappa = 3 cos u / (a * sqrt(1 + sin^2 u)).
        # The sign flips at the crossing point, reversing the turn direction.
        kappa = (
            3.0 * cos_u
            / (a * math.sqrt(1.0 + sin_u * sin_u))
        )

        # Robot turn rate (rad/s)
        omega = v * kappa

        # Inverse differential-drive kinematics: v_L,R = v -/+ omega * W / 2
        v_l = v - omega * (W / 2.0)
        v_r = v + omega * (W / 2.0)

        # Wheel linear speed (cm/s) to motor angular speed (deg/s)
        dps_l = (v_l / wheel_radius) * 180.0 / math.pi
        dps_r = (v_r / wheel_radius) * 180.0 / math.pi

        robot_drive.on(
            SpeedDPS(dps_l),
            SpeedDPS(dps_r)
        )

        time.sleep(dt)

    robot_drive.off(brake=True)

    print("LEMNISCATE completed")

def turn_90():
    """
    Pivot 90 degrees in place by driving the wheels in opposite directions.

    Each wheel travels a quarter of the turning circle (pi * W / 4), which is
    W / (4 * D) wheel revolutions. The gyro reading is printed to compare the
    commanded and actual turn.
    """
    turn_rotations = BASE_WIDTH / (4.0 * WHEEL_DIAMETER)

    start_angle = gyro.angle

    # Reduced speed for the pivot to limit wheel slip
    robot_drive.on_for_rotations(
        SpeedPercent(-OUTER_SPEED / 3),
        SpeedPercent(OUTER_SPEED / 3),
        rotations=turn_rotations,
        brake=True
    )

    robot_drive.off(brake=True)

    print("Turned:", abs(gyro.angle - start_angle))

    time.sleep(0.2) # Let the robot come to rest before the next move

# ---------------------------------------------------------------------------
# Dead-reckoning odometry
# ---------------------------------------------------------------------------

def get_user_command_matrix(num_rows=3):
    """
    Read num_rows motor commands from the terminal.

    Each row is "left_speed right_speed duration", with speeds in percent
    and duration in seconds. Malformed rows and non-positive durations are
    rejected and re-prompted.

    Returns a list of [left, right, duration] rows.
    """
    commands = []
    print("[System] Enter motor commands for {} rows.".format(num_rows))

    for i in range(num_rows):
        while True:
            try:
                raw_input = input("[Input] Enter Row {} (Left Speed, Right Speed, Duration): ".format(i + 1))
                parts = raw_input.strip().split()

                if len(parts) != 3:
                    print("[Error] Row {} must contain exactly 3 numbers (received {}).".format(i + 1, len(parts)))
                    continue

                left = float(parts[0])
                right = float(parts[1])
                duration = float(parts[2])

                if duration <= 0:
                    print("[Error] Duration must be greater than 0 seconds (received {:.2f} s).".format(duration))
                    continue

                commands.append([left, right, duration])
                print("[Success] Row {} accepted: Left Speed: {:.2f}%, Right Speed: {:.2f}%, Duration: {:.2f} s".format(
                    i + 1, left, right, duration
                ))
                break

            except ValueError:
                print("[Error] Invalid numeric input: '{}'. Please enter numbers separated by spaces.".format(raw_input))

    print("[Summary] Matrix loaded with {} command row(s).".format(len(commands)))
    return commands


def execute_and_estimate_pose(commands):
    """
    Execute a command sequence while estimating pose by dead reckoning.

    Wheel encoders are sampled at 20 Hz. Each sample is converted to wheel
    travel, then to a centre displacement and heading change using
    differential-drive forward kinematics. The pose is advanced with the
    midpoint heading (second-order arc integration). At the end, the
    estimated heading is compared with the gyro reading.

    Returns (x, y, theta_deg, orientation_error_deg).
    """
    initial_x=0.0 
    initial_y=0.0
    initial_theta_deg=0.0

    gyro = GyroSensor()
    gyro.mode = 'GYRO-ANG'
    x = initial_x
    y = initial_y
    theta_rad = math.radians(initial_theta_deg)

    # Zero the gyro so its heading shares the odometry frame
    if gyro is not None:
        gyro.reset()

    print("[Odometry] Starting execution at Pose (x: {:.2f} cm, y: {:.2f} cm, theta: {:.2f} deg)".format(x, y, math.degrees(theta_rad)))

    # Encoder baselines (deg)
    prev_left_deg = robot_drive.left_motor.degrees
    prev_right_deg = robot_drive.right_motor.degrees

    for idx, row in enumerate(commands):
        right_power, left_power, duration = row[0], row[1], row[2]

        # Clamp to the valid motor range
        left_clamped = max(-100, min(100, left_power))
        right_clamped = max(-100, min(100, right_power))

        # Non-blocking start so the encoders can be sampled during motion
        robot_drive.on(SpeedPercent(left_clamped), SpeedPercent(right_clamped))

        sample_interval = 0.05  # Odometry update interval (s), 20 Hz
        elapsed = 0.0

        while elapsed < duration:
            time.sleep(sample_interval)
            elapsed += sample_interval

            curr_left_deg = robot_drive.left_motor.degrees
            curr_right_deg = robot_drive.right_motor.degrees

            # Encoder change since the last sample (deg)
            delta_left_deg = curr_left_deg - prev_left_deg
            delta_right_deg = curr_right_deg - prev_right_deg
            
            # Wheel travel since the last sample (cm)
            d_left = (delta_left_deg / 360.0) * CIRCUMFERENCE
            d_right = (delta_right_deg / 360.0) * CIRCUMFERENCE

            prev_left_deg = curr_left_deg
            prev_right_deg = curr_right_deg

            # Forward kinematics: centre displacement is the mean wheel travel
            d_center = (d_right + d_left) / 2.0

            # Heading change is the wheel travel difference over the track width
            delta_theta_rad = (d_right - d_left) / BASE_WIDTH

            # Midpoint heading gives second-order accuracy on arcs
            theta_mid_rad = theta_rad + (delta_theta_rad / 2.0)

            x += d_center * math.cos(theta_mid_rad)
            y += d_center * math.sin(theta_mid_rad)
            theta_rad += delta_theta_rad

    robot_drive.off(brake=True)

    # Wrap estimated heading to [-180, 180)
    estimated_theta_deg = (math.degrees(theta_rad) + 180) % 360 - 180

    # Gyro heading as the independent reference
    if gyro is not None:
        measured_theta_deg = (float(gyro.angle) + 180) % 360 - 180
    else:
        measured_theta_deg = 0.0

    orientation_error_deg =  (measured_theta_deg - estimated_theta_deg+ 180) % 360 - 180

    print("[Pose Estimate] x: {:.2f} cm | Y: {:.2f} cm | Theta: {:.2f} deg".format(
        x, y, estimated_theta_deg
    ))
    print("[Orientation] Measured Theta: {:.2f} deg | Estimated Theta: {:.2f} deg".format(
        measured_theta_deg, estimated_theta_deg
    ))
    print("[Orientation Error] Wrapped Error: {:.2f} deg".format(
        orientation_error_deg
    ))

    return x, y, estimated_theta_deg, orientation_error_deg

# ---------------------------------------------------------------------------
# Braitenberg behaviours
#
# Each light reading is normalised to [0, 1] between LIGHT_MIN and LIGHT_MAx,
# then mapped to a wheel speed. The behaviours differ in two choices: whether
# each sensor drives the wheel on its own side or the opposite side, and
# whether brighter light makes that wheel faster or slower. Each runs for 15 s
# with a 20 Hz update.
# ---------------------------------------------------------------------------

def cowardice():
    """Same-side wiring, brighter is faster: the robot turns away from light."""
    print("=== Cowardice ===")
    left_min_light = 0
    right_min_light = 0

    t0 = time.time()
    while time.time() - t0 < 15.0:
        left_min_light = (left_light.ambient_light_intensity - LIGHT_MIN) / (LIGHT_MAx - LIGHT_MIN)
        left_sensor = max(0.0, min(1.0, left_min_light))	# Normalised and clamped to [0, 1]

        right_min_light = (right_light.ambient_light_intensity - LIGHT_MIN) / (LIGHT_MAx - LIGHT_MIN)
        right_sensor = max(0.0, min(1.0, right_min_light))
 
        left_speed = MIN_SPEED + left_sensor * (MAx_SPEED - MIN_SPEED)	# Linear map to [MIN_SPEED, MAx_SPEED]
        right_speed = MIN_SPEED + right_sensor * (MAx_SPEED - MIN_SPEED)
 
        # The wheel nearer the light spins faster, steering the robot away from it
        robot_drive.on(SpeedPercent(left_speed), SpeedPercent(right_speed))
        time.sleep(0.05)
 
    robot_drive.off(brake=True)
    print("Cowardice finished")
 
def aggression():
    """Crossed wiring, brighter is faster: the robot turns toward light and speeds into it."""
    print("=== Aggression ===")
    left_min_light = 0
    right_min_light = 0

    t0 = time.time()
    while time.time() - t0 < 15.0:
        left_min_light = (left_light.ambient_light_intensity - LIGHT_MIN) / (LIGHT_MAx - LIGHT_MIN)
        left_sensor = max(0.0, min(1.0, left_min_light))	# Normalised and clamped to [0, 1]

        right_min_light = (right_light.ambient_light_intensity - LIGHT_MIN) / (LIGHT_MAx - LIGHT_MIN)
        right_sensor = max(0.0, min(1.0, right_min_light))
 
        # Crossed: each sensor drives the opposite wheel
        left_speed = MIN_SPEED + right_sensor * (MAx_SPEED - MIN_SPEED)
        right_speed = MIN_SPEED + left_sensor * (MAx_SPEED - MIN_SPEED)
 
        # The wheel farther from the light spins faster, steering the robot into it
        robot_drive.on(SpeedPercent(left_speed), SpeedPercent(right_speed))
        time.sleep(0.05)
 
    robot_drive.off(brake=True)
    print("Aggression finished")
 
def love():
    """Same-side wiring, brighter is slower: the robot approaches light and stops facing it."""
    print("=== Love ===")
    left_min_light = 0
    right_min_light = 0

    t0 = time.time()
    while time.time() - t0 < 15.0:
        left_min_light = (left_light.ambient_light_intensity - LIGHT_MIN) / (LIGHT_MAx - LIGHT_MIN)
        left_sensor = max(0.0, min(1.0, left_min_light))	# Normalised and clamped to [0, 1]

        right_min_light = (right_light.ambient_light_intensity - LIGHT_MIN) / (LIGHT_MAx - LIGHT_MIN)
        right_sensor = max(0.0, min(1.0, right_min_light))
 
        # Inverted map: full speed in the dark, stopped at full brightness
        left_speed = (1.0 - left_sensor) * MAx_SPEED
        right_speed = (1.0 - right_sensor) * MAx_SPEED
 
        # The wheel nearer the light slows, turning the robot toward it until it stops
        robot_drive.on(SpeedPercent(left_speed), SpeedPercent(right_speed))
        time.sleep(0.05)
 
    robot_drive.off(brake=True)
    print("Love finished")
 
def curiosity():
    """Crossed wiring, brighter is slower: the robot approaches light, then turns away and keeps exploring."""
    print("=== Curiosity ===")
    left_min_light = 0
    right_min_light = 0

    t0 = time.time()
    while time.time() - t0 < 15.0:
        left_min_light = (left_light.ambient_light_intensity - LIGHT_MIN) / (LIGHT_MAx - LIGHT_MIN)
        left_sensor = max(0.0, min(1.0, left_min_light))	# Normalised and clamped to [0, 1]

        right_min_light = (right_light.ambient_light_intensity - LIGHT_MIN) / (LIGHT_MAx - LIGHT_MIN)
        right_sensor = max(0.0, min(1.0, right_min_light))
 
        # Crossed and inverted: each sensor slows the opposite wheel
        left_speed = (1.0 - right_sensor) * MAx_SPEED
        right_speed = (1.0 - left_sensor) * MAx_SPEED
 
        # The wheel farther from the light slows, so the robot veers away and keeps exploring
        robot_drive.on(SpeedPercent(left_speed), SpeedPercent(right_speed))
        time.sleep(0.05)
 
    robot_drive.off(brake=True)
    print("Curiosity finished")
    
# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    """Terminal menu for selecting and running each behaviour."""
    while True:
        print("Select an action:")
        print("1: Straight Line")
        print("2: Circle")
        print("3: Rectangle")
        print("4: Lemniscate")
        print("5: Turn 90")
        print("6: Cowardice")
        print("7: Aggression")
        print("8: Love")
        print("9: Curiosity")
        print("command: command array input")
        print("cm: command array input and measure error")
                
        
        print("q: Quit")
        
        choice = input("Enter choice: ").strip()

        if choice == '1':
            drive_straight_gyro_pid(100)
        elif choice == '2':
            circle_calibrated()
        elif choice == '3':
            rectangle()
        elif choice.lower() == '4':
            lemniscate()
        elif choice.lower() == '5':
            turn_90()
        elif choice == '6':
            cowardice()
        elif choice == '7':
            aggression()
        elif choice == '8':
            love()
        elif choice == '9':
            curiosity()
        
        elif choice == 'cm':
            user_input = get_user_command_matrix()
            execute_and_estimate_pose(user_input)
                

        elif choice.lower() == 'q':
            print("Exiting...")
            break
        else:
            print("Invalid selection. Please choose 1, 2, 3, or q.")

if __name__ == '__main__':
    main()