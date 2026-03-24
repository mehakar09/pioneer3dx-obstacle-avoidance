# Author: Mehak Arora
# Controller: Baseline Controller with Logging
# Description:
# This controller implements a basic obstacle avoidance strategy for the Pioneer 3-DX robot.
# It uses front sonar sensors (so3, so4) for detection and simple turning behaviour.
# Logging includes trial time, closest approach, unsafe proximity flag, and unsafe proximity count.
# The baseline controller does not include any reverse recovery behaviour
from controller import Robot
import csv
import os
import random

# SETTINGS
NOISE_MODE = True         # False = normal, True = noisy
NOISE_LEVEL = 15            

MAX_SPEED = 2.0
DETECT_THRESHOLD = 300
UNSAFE_THRESHOLD = 900      # unsafe proximity flag

#Initialize ROBOT
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# motors
left_motor = robot.getDevice("left wheel")
right_motor = robot.getDevice("right wheel")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# sonar sensors
sonars = []
for i in range(16):
    s = robot.getDevice(f"so{i}")
    s.enable(timestep)
    sonars.append(s)

# states
FORWARD = 0
TURN = 1
ESCAPE = 2
state = FORWARD

turn_steps = 0
escape_steps = 0
cooldown_steps = 0
turn_left = True

#LOGGING
start_time = robot.getTime()
max_front = 0.0
unsafe_proximity = 0

# NEW: unsafe proximity count
unsafe_count = 0
was_unsafe_last_step = False

escape_completed_once = False

#CSV FILE
if NOISE_MODE:
    csv_file = "baseline_noisy.csv"
else:
    csv_file = "baseline_normal.csv"

file_exists = os.path.isfile(csv_file)

# MAIN LOOP
while robot.step(timestep) != -1:
    values = [s.getValue() for s in sonars]

    # Noise injection experiment
    if NOISE_MODE:
        values = [v + random.uniform(-NOISE_LEVEL, NOISE_LEVEL) for v in values]

    # front sensors (baseline uses only so3 and so4)
    so3 = values[3]
    so4 = values[4]
    front = max(so3, so4)

    # record closest approach
    if front > max_front:
        max_front = front

    #UNSAFE PROXIMITY FLAG AND COUNT
    is_unsafe_now = (front > UNSAFE_THRESHOLD)

    # 0/1 flag
    if is_unsafe_now:
        unsafe_proximity = 1

    # count only on entry to unsafe zone
    if is_unsafe_now and (not was_unsafe_last_step):
        unsafe_count += 1

    was_unsafe_last_step = is_unsafe_now

    # cooldown
    if cooldown_steps > 0:
        cooldown_steps -= 1

    # default speeds
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    # baseline movement logic
    if state == FORWARD:
        if cooldown_steps == 0 and front > DETECT_THRESHOLD:
            turn_left = (so4 > so3)
            turn_steps = 25
            state = TURN

    elif state == TURN:
        if turn_left:
            left_speed = -MAX_SPEED
            right_speed = MAX_SPEED
        else:
            left_speed = MAX_SPEED
            right_speed = -MAX_SPEED

        turn_steps -= 1
        if turn_steps <= 0:
            escape_steps = 80
            cooldown_steps = 80
            state = ESCAPE

    elif state == ESCAPE:
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
        escape_steps -= 1

        if escape_steps <= 0:
            escape_completed_once = True
            state = FORWARD

    # apply speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # Print the readings
    if escape_completed_once:
        total_time = robot.getTime() - start_time

        print("TRIAL COMPLETED (BASELINE)")
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

