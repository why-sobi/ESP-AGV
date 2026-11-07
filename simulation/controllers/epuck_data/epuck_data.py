from pathlib import Path
import sys

from controller import Robot
import pandas as pd

# 1. Workspace path resolution
controller_dir = Path(__file__).resolve().parent
root_dir = controller_dir.parent.parent.parent
data_dir = root_dir / "data"
data_dir.mkdir(parents=True, exist_ok=True)
csv_path = data_dir / "SensorData-v2(PD) (2).csv"

# 2. Initialize Webots Robot & Timestep
robot = Robot()
time_step = int(robot.getBasicTimeStep())

# E-puck wheel motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Position sensors
left_sensor = robot.getDevice("left wheel sensor")
right_sensor = robot.getDevice("right wheel sensor")
left_sensor.enable(time_step)
right_sensor.enable(time_step)

# IR Ground Sensors
gs_left = robot.getDevice("gs0")
gs_center = robot.getDevice("gs1")
gs_right = robot.getDevice("gs2")

gs_left.enable(time_step)
gs_center.enable(time_step)
gs_right.enable(time_step)

max_speed = 3.5                     # Straight-line wheel speed in rad/s.
min_speed = 2.5                     # Slow down on bends without making the inner wheel reverse.
max_wheel_speed = 5.0                # Keep commands below the E-puck motor limit.
line_threshold = 400.0              # Sensor readings below this are treated as seeing the dark line.
min_line_strength = 30.0             # Ignore weak threshold crossings caused by sensor noise.
kp = 1.2                            # Proportional gain: steering response to line offset.
kd = 0.015                          # Derivative gain: damping for rapid changes in line offset.
max_steering = 1.5                   # Bound differential steering and avoid abrupt spins.
derivative_limit = 5.0               # Bound the error derivative before it amplifies sensor noise.
derivative_smoothing = 0.25          # Low-pass weight for the derivative term.
previous_error = 0.0                # Last line offset, used by the derivative and recovery behavior.
filtered_error_rate = 0.0            # Smoothed line-offset rate used by the D term.
last_turn_direction = -1.0           # Remember the last turn; default to left if a side choice is ambiguous.
junction_choice = None               # Latched branch choice until the robot clears the junction.
junction_frames = 0                  # Consecutive frames needed to confirm a junction.
junction_clear_frames = 0            # Consecutive clear frames needed to release the junction latch.
junction_confirm_steps = 2
junction_release_steps = 3
controller_dt = time_step / 1000.0   # Webots step duration in seconds, for the error derivative.
batch_size = 500                    # Write to CSV in batches every 500 steps to protect data
data_log = []                       # Batching data before flushing

print(f"Starting simulation. Batch-saving data every {batch_size} steps...")

step_count = 0

while robot.step(time_step) != -1:
    current_time = robot.getTime()

    v_left      = gs_left.getValue()
    v_center    = gs_center.getValue()
    v_right     = gs_right.getValue()
    
    # Lower readings indicate the dark line, so convert them into line strengths.
    # Each line_* value is the sensor's positive amount below the background threshold.
    line_left       = max(0.0, line_threshold - v_left)
    line_center     = max(0.0, line_threshold - v_center)
    line_right      = max(0.0, line_threshold - v_right)
    line_strength   = line_left + line_center + line_right  # Total line signal; zero means no sensor sees it.

    left_active = line_left >= min_line_strength
    center_active = line_center >= min_line_strength
    right_active = line_right >= min_line_strength
    both_sides_active = left_active and right_active

    if junction_choice is None:
        if both_sides_active:
            junction_frames += 1
        else:
            junction_frames = 0

        if junction_frames >= junction_confirm_steps:
            if center_active:
                # At a three-way junction, continue forward through the center.
                junction_choice = "straight"
            else:
                # With only two side branches, continue the last turn direction.
                # If there is no useful history, the default is to choose left.
                junction_choice = "left" if last_turn_direction < 0 else "right"
    else:
        if both_sides_active:
            junction_clear_frames = 0
        else:
            junction_clear_frames += 1

        if junction_clear_frames >= junction_release_steps:
            junction_choice = None
            junction_frames = 0
            junction_clear_frames = 0

    if line_strength < min_line_strength:
        # Search slowly in the last known direction instead of driving blind
        # at the normal line-following speed.
        search_steering = max_steering * last_turn_direction * 0.5
        target_left = 1.0 + search_steering
        target_right = 1.0 - search_steering
        filtered_error_rate = 0.0
    else:
        if junction_choice == "straight":
            error = 0.0
        elif junction_choice == "left":
            error = -1.0
        elif junction_choice == "right":
            error = 1.0
        else:
            # Positive error means the line is to the robot's right.
            error = (line_right - line_left) / line_strength

        if junction_choice is None:
            raw_error_rate = (error - previous_error) / controller_dt
            raw_error_rate = max(-derivative_limit, min(derivative_limit, raw_error_rate))
            filtered_error_rate = (
                derivative_smoothing * raw_error_rate
                + (1.0 - derivative_smoothing) * filtered_error_rate
            )
        else:
            # A discrete branch choice should not create a derivative kick.
            filtered_error_rate = 0.0

        if error < 0.0:
            last_turn_direction = -1.0
        elif error > 0.0:
            last_turn_direction = 1.0

        previous_error = error

        # Reduce forward speed as the line moves farther from center.
        turn_amount = min(1.0, abs(error))
        base_speed = max_speed - (max_speed - min_speed) * turn_amount

        # PD is sufficient for lateral tracking; an integral term could wind up
        # during sensor bias or line loss.
        steering = kp * error + kd * filtered_error_rate
        steering = max(-max_steering, min(max_steering, steering))

        target_left = base_speed + steering
        target_right = base_speed - steering

    # Keep both targets in the forward, supported motor-speed range.
    target_left = max(0.0, min(max_wheel_speed, target_left))
    target_right = max(0.0, min(max_wheel_speed, target_right))

    left_motor.setVelocity(target_left)
    right_motor.setVelocity(target_right)

    # Collect telemetry
    data_log.append({
        "time": current_time,
        "target_vel_left": target_left,
        "target_vel_right": target_right,
        "actual_pos_left": left_sensor.getValue(),
        "actual_pos_right": right_sensor.getValue(),
        "gs_left": v_left,
        "gs_center": v_center,
        "gs_right": v_right
    })

    step_count += 1

    # Periodic batch flush to disk
    if step_count % batch_size == 0:
        df = pd.DataFrame(data_log)
        header = not csv_path.exists()
        df.to_csv(csv_path, mode='a', index=False, header=header)
        data_log.clear()
        print(f"[Batch Flush] Saved {step_count} total steps to {csv_path}")

if data_log:
    df = pd.DataFrame(data_log)
    header = not csv_path.exists()
    df.to_csv(csv_path, mode='a', index=False, header=header)
    print(f"[Final Flush] Saved remaining data to {csv_path}")
