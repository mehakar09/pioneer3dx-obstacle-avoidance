# Author: Mehak Arora
# Controller: Webots Inbuilt Controller (Modified with Logger)
# Based on: Webots Pioneer 3-DX collision avoidance controller
# Modifications:
# - Added noise injection
# - Added unsafe proximity detection and counting
# - Added logging for experimental evaluation
#Webots inbuilt controller in python
# This controller does not include any reverse recovery behaviour
from controller import Robot
import csv
import os
import random

print("WEBOTS INBUILT-STYLE CONTROLLER (PYTHON LOGGER)")

#SETTINGS
NOISE_MODE =     True      # False = Normal, True = Noisy
NOISE_LEVEL = 15           # Noise range ±15

MAX_SPEED = 5.24           # same as Webots inbuilt controller
MAX_SENSOR_VALUE = 1024.0

MIN_DISTANCE = 1.0         # obstacle considered close if < 1 meter
WHEEL_WEIGHT_THRESHOLD = 100

UNSAFE_THRESHOLD = 900     # for unsafe proximity metric (according to the project)
TRIAL_DURATION = 10.0      # stop trial after 10 seconds (fair benchmark)

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# motors
left_motor = robot.getDevice("left wheel")
right_motor = robot.getDevice("right wheel")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# sonar sensors (so0 to so15)
sonars = []
for i in range(16):
    s = robot.getDevice(f"so{i}")
    s.enable(timestep)
    sonars.append(s)

# INBUILT WEIGHTS (same idea as C controller)
# Each sensor influences left/right wheel turning
wheel_weights = [
    [150, 0], [200, 0], [300, 0], [600, 0],
    [0, 600], [0, 300], [0, 200], [0, 150],
    [0, 0],   [0, 0],   [0, 0],   [0, 0],
    [0, 0],   [0, 0],   [0, 0],   [0, 0]
]

#STATES
FORWARD = 0
LEFT = 1
RIGHT = 2
state = FORWARD

# LOGGING 
start_time = robot.getTime()

max_front = 0.0
unsafe_proximity = 0

# Unsafe proximity event count
unsafe_count = 0
was_unsafe_last_step = False

# CSV files
csv_file = "webots_inbuilt_noisy.csv" if NOISE_MODE else "webots_inbuilt_normal.csv"
file_exists = os.path.isfile(csv_file)

#MAIN LOOP
while robot.step(timestep) != -1:
    # read sensor values
    values = [s.getValue() for s in sonars]

    # noise injection (optional experiment)
    if NOISE_MODE:
        values = [v + random.uniform(-NOISE_LEVEL, NOISE_LEVEL) for v in values]

    # FRONT METRICS (same as the project)
    # we measure front using sensors 2-5 for fairness
    front = max(values[2], values[3], values[4], values[5])

    # closest approach tracking (raw max front)
    if front > max_front:
        max_front = front

    # UNSAFE PROXIMITY FLAG + COUNT
    is_unsafe_now = (front > UNSAFE_THRESHOLD)

    # 0/1 flag
    if is_unsafe_now:
        unsafe_proximity = 1

    # count only when entering unsafe zone
    if is_unsafe_now and (not was_unsafe_last_step):
        unsafe_count += 1

    was_unsafe_last_step = is_unsafe_now

    # INBUILT CONTROLLER LOGIC
    wheel_total = [0.0, 0.0]

    for i in range(16):
        sensor_value = values[i]

        # convert sensor value to distance (approx, like in C code)
        if sensor_value <= 0.0:
            speed_modifier = 0.0
        else:
            distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE))

            if distance < MIN_DISTANCE:
                speed_modifier = 1.0 - (distance / MIN_DISTANCE)
            else:
                speed_modifier = 0.0

        wheel_total[0] += wheel_weights[i][0] * speed_modifier
        wheel_total[1] += wheel_weights[i][1] * speed_modifier

    # decide speeds using state machine
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    if state == FORWARD:
        if wheel_total[0] > WHEEL_WEIGHT_THRESHOLD:
            left_speed = 0.7 * MAX_SPEED
            right_speed = -0.7 * MAX_SPEED
            state = LEFT

        elif wheel_total[1] > WHEEL_WEIGHT_THRESHOLD:
            left_speed = -0.7 * MAX_SPEED
            right_speed = 0.7 * MAX_SPEED
            state = RIGHT

        else:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED

    elif state == LEFT:
        if wheel_total[0] > WHEEL_WEIGHT_THRESHOLD or wheel_total[1] > WHEEL_WEIGHT_THRESHOLD:
            left_speed = 0.7 * MAX_SPEED
            right_speed = -0.7 * MAX_SPEED
        else:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED
            state = FORWARD

    elif state == RIGHT:
        if wheel_total[0] > WHEEL_WEIGHT_THRESHOLD or wheel_total[1] > WHEEL_WEIGHT_THRESHOLD:
            left_speed = -0.7 * MAX_SPEED
            right_speed = 0.7 * MAX_SPEED
        else:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED
            state = FORWARD

    # apply motor speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    #STOP AFTER FIXED TIME
    if robot.getTime() - start_time >= TRIAL_DURATION:
        total_time = robot.getTime() - start_time

        print("TRIAL COMPLETED (WEBOTS INBUILT LOGGER)")
        print(f"Noise Mode = {NOISE_MODE}")
        print(f"Final Time = {total_time:.2f} sec")
        print(f"Closest Approach (max_front raw) = {max_front:.1f}")
        print(f"Unsafe Proximity (0/1) = {unsafe_proximity}")
        print(f"Unsafe Proximity Count = {unsafe_count}")

        with open(csv_file, "a", newline="") as f:
            writer = csv.writer(f)

            if not file_exists:
                writer.writerow([
                    "NoiseMode",
                    "NoiseLevel",
                    "TrialTime_sec",
                    "ClosestApproach_raw",
                    "UnsafeProximity_0or1",
                    "UnsafeProximityCount"
                ])
                file_exists = True

            writer.writerow([
                NOISE_MODE,
                NOISE_LEVEL if NOISE_MODE else 0,
                round(total_time, 2),
                round(max_front, 1),
                unsafe_proximity,
                unsafe_count
            ])

        break

