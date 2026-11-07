from pathlib import Path
import sys
from controller import Robot
import pandas as pd

# 1. Workspace path resolution
controller_dir = Path(__file__).resolve().parent
root_dir = controller_dir.parent.parent.parent
data_dir = root_dir / "data"
data_dir.mkdir(parents=True, exist_ok=True)
csv_path = data_dir / "SensorData-v1.csv"

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

max_speed = 4.0
data_log = []
batch_size = 500     # Write to CSV in batches every 500 steps to protect data

print(f"Starting simulation. Batch-saving data every {batch_size} steps...")

step_count = 0

while robot.step(time_step) != -1:
    current_time = robot.getTime()

    v_left = gs_left.getValue()
    v_center = gs_center.getValue()
    v_right = gs_right.getValue()
    
    # Line following logic placeholder
    if v_center < 400:  
        target_left = max_speed
        target_right = max_speed
    elif v_left < 400:  
        target_left = max_speed * 0.2
        target_right = max_speed
    elif v_right < 400: 
        target_left = max_speed
        target_right = max_speed * 0.2
    else:
        target_left = max_speed * 0.5
        target_right = max_speed * 0.5

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
        # Append if file exists, write header only on first write
        header = not csv_path.exists()
        df.to_csv(csv_path, mode='a', index=False, header=header)
        data_log.clear()
        print(f"[Batch Flush] Saved {step_count} total steps to {csv_path}")
