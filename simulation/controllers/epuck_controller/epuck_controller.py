from controller import Robot

robot = Robot()
time_step = int(robot.getBasicTimeStep())

# E-puck wheel motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# E-puck's built-in ground IR sensors
gs_left = robot.getDevice("gs0")
gs_center = robot.getDevice("gs1")
gs_right = robot.getDevice("gs2")

gs_left.enable(time_step)
gs_center.enable(time_step)
gs_right.enable(time_step)

max_speed = 4.0

while robot.step(time_step) != -1:
    v_left = gs_left.getValue()
    v_center = gs_center.getValue()
    v_right = gs_right.getValue()
    
    # Live sensor data output
    print(f"[E-PUCK DATA] L: {v_left:.1f} | C: {v_center:.1f} | R: {v_right:.1f}")

    # Line following logic
    if v_center < 400:  # Center sees black line
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(max_speed)
    elif v_left < 400:  # Left sees line -> turn left
        left_motor.setVelocity(max_speed * 0.2)
        right_motor.setVelocity(max_speed)
    elif v_right < 400: # Right sees line -> turn right
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(max_speed * 0.2)
    else:
        left_motor.setVelocity(max_speed * 0.5)
        right_motor.setVelocity(max_speed * 0.5)