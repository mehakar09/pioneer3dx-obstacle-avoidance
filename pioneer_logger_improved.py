from controller import Robot
import csv
import os
import random

# ---------------- SETTINGS ----------------
NOISE_MODE = True  # ✅ False = normal, True = noisy
NOISE_LEVEL = 15    # ✅ try 10–20

MAX_SPEED = 2.0

DETECT_THRESHOLD = 220
SLOWDOWN_THRESHOLD = 160
SLOW_SPEED = 1.0

UNSAFE_THRESHOLD = 900      # ✅ unsafe proximity flag

# anti-stuck tuning
TOO_CLOSE_THRESHOLD = 850   # triggers reverse recovery

# ---------------- INIT ROBOT ----------------
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

# ---------------- STATES ----------------
FORWARD = 0
TURN = 1
ESCAPE = 2
REVERSE = 3

state = FORWARD

turn_steps = 0
escape_steps = 0
reverse_steps = 0
cooldown_steps = 0
turn_left = True

# ---------------- LOGGING ----------------
start_time = robot.getTime()
max_front = 0.0
unsafe_proximity = 0

# ✅ NEW: unsafe proximity count
unsafe_count = 0
was_unsafe_last_step = False

reverse_count = 0
escape_completed_once = False

# ---------------- CSV FILE ----------------
if NOISE_MODE:
    csv_file = "improved_noisy.csv"
else:
    csv_file = "improved_normal.csv"

file_exists = os.path.isfile(csv_file)

# ---------------- MAIN LOOP ----------------
while robot.step(timestep) != -1:
    values = [s.getValue() for s in sonars]

    # ✅ Noise injection experiment
    if NOISE_MODE:
        values = [v + random.uniform(-NOISE_LEVEL, NOISE_LEVEL) for v in values]

    # ✅ improved front sensing uses more sensors
    front_left = max(values[2], values[3])
    front_right = max(values[4], values[5])
    front = max(front_left, front_right)

    # side sensing (optional for future tuning)
    left_side = max(values[0], values[1])
    right_side = max(values[6], values[7])

    # ✅ record closest approach
    if front > max_front:
        max_front = front

    # ---------------- UNSAFE PROXIMITY FLAG + COUNT ----------------
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

    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    # ✅ anti-stuck reverse if too close
    if front > TOO_CLOSE_THRESHOLD and state != REVERSE:
        reverse_steps = 25
        state = REVERSE
        reverse_count += 1

    # ---------------- CONTROL ----------------
    if state == FORWARD:
        # slowdown near obstacle
        if front > SLOWDOWN_THRESHOLD:
            left_speed = SLOW_SPEED
            right_speed = SLOW_SPEED

        # start turning earlier
        if cooldown_steps == 0 and front > DETECT_THRESHOLD:
            # turn away from stronger side
            turn_left = (front_right > front_left)
            turn_steps = 45
            state = TURN

    elif state == REVERSE:
        left_speed = -MAX_SPEED
        right_speed = -MAX_SPEED
        reverse_steps -= 1

        if reverse_steps <= 0:
            turn_steps = 50
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
            escape_steps = 60
            cooldown_steps = 60
            state = ESCAPE

    elif state == ESCAPE:
        # if still blocked, keep turning, else go forward
        if front > DETECT_THRESHOLD:
            left_speed = -MAX_SPEED
            right_speed = MAX_SPEED
        else:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED

        escape_steps -= 1
        if escape_steps <= 0:
            escape_completed_once = True
            state = FORWARD

    # apply motor speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # ---------------- STOP + PRINT + SAVE ----------------
    if escape_completed_once:
        total_time = robot.getTime() - start_time

        print("✅ TRIAL COMPLETED (IMPROVED + ANTI-STUCK)")
        print(f"Noise Mode = {NOISE_MODE}")
        print(f"Final Time = {total_time:.2f} sec")
        print(f"Closest Approach (max_front raw) = {max_front:.1f}")
        print(f"Unsafe Proximity (0/1) = {unsafe_proximity}")
        print(f"Unsafe Proximity Count = {unsafe_count}")
        print(f"Reverse Count = {reverse_count}")

        with open(csv_file, "a", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow([
                    "NoiseMode",
                    "NoiseLevel",
                    "TrialTime_sec",
                    "ClosestApproach_raw",
                    "UnsafeProximity_0or1",
                    "UnsafeProximityCount",
                    "ReverseCount"
                ])
                file_exists = True

            writer.writerow([
                NOISE_MODE,
                NOISE_LEVEL if NOISE_MODE else 0,
                round(total_time, 2),
                round(max_front, 1),
                unsafe_proximity,
                unsafe_count,
                reverse_count
            ])

        break

# from controller import Robot
# import csv
# import os
# import random

# # ---------------- SETTINGS ----------------
# NOISE_MODE = False  # ✅ False = normal, True = noisy
# NOISE_LEVEL = 15            # ✅ try 10–20

# MAX_SPEED = 2.0

# DETECT_THRESHOLD = 220
# SLOWDOWN_THRESHOLD = 160
# SLOW_SPEED = 1.0

# UNSAFE_THRESHOLD = 900      # ✅ unsafe proximity flag

# # anti-stuck tuning
# TOO_CLOSE_THRESHOLD = 850   # triggers reverse recovery

# # ---------------- INIT ROBOT ----------------
# robot = Robot()
# timestep = int(robot.getBasicTimeStep())

# # motors
# left_motor = robot.getDevice("left wheel")
# right_motor = robot.getDevice("right wheel")
# left_motor.setPosition(float('inf'))
# right_motor.setPosition(float('inf'))

# # sonar sensors
# sonars = []
# for i in range(16):
#     s = robot.getDevice(f"so{i}")
#     s.enable(timestep)
#     sonars.append(s)

# # ---------------- STATES ----------------
# FORWARD = 0
# TURN = 1
# ESCAPE = 2
# REVERSE = 3

# state = FORWARD

# turn_steps = 0
# escape_steps = 0
# reverse_steps = 0
# cooldown_steps = 0
# turn_left = True

# # ---------------- LOGGING ----------------
# start_time = robot.getTime()
# max_front = 0.0
# unsafe_proximity = 0
# reverse_count = 0           # ✅ NEW metric
# escape_completed_once = False

# # ---------------- CSV FILE ----------------
# if NOISE_MODE:
#     csv_file = "improved_noisy.csv"
# else:
#     csv_file = "improved_normal.csv"

# file_exists = os.path.isfile(csv_file)

# # ---------------- MAIN LOOP ----------------
# while robot.step(timestep) != -1:
#     values = [s.getValue() for s in sonars]

#     # ✅ Noise injection experiment
#     if NOISE_MODE:
#         values = [v + random.uniform(-NOISE_LEVEL, NOISE_LEVEL) for v in values]

#     # ✅ improved front sensing uses more sensors
#     front_left = max(values[2], values[3])
#     front_right = max(values[4], values[5])
#     front = max(front_left, front_right)

#     # side sensing (optional for future tuning)
#     left_side = max(values[0], values[1])
#     right_side = max(values[6], values[7])

#     # record closest approach
#     if front > max_front:
#         max_front = front

#     # unsafe proximity flag
#     if front > UNSAFE_THRESHOLD:
#         unsafe_proximity = 1

#     # cooldown
#     if cooldown_steps > 0:
#         cooldown_steps -= 1

#     left_speed = MAX_SPEED
#     right_speed = MAX_SPEED

#     # ✅ anti-stuck reverse if too close
#     if front > TOO_CLOSE_THRESHOLD and state != REVERSE:
#         reverse_steps = 25
#         state = REVERSE
#         reverse_count += 1   # ✅ NEW

#     # ---------------- CONTROL ----------------
#     if state == FORWARD:
#         # slowdown near obstacle
#         if front > SLOWDOWN_THRESHOLD:
#             left_speed = SLOW_SPEED
#             right_speed = SLOW_SPEED

#         # start turning earlier
#         if cooldown_steps == 0 and front > DETECT_THRESHOLD:
#             # turn away from stronger side
#             turn_left = (front_right > front_left)
#             turn_steps = 45
#             state = TURN

#     elif state == REVERSE:
#         left_speed = -MAX_SPEED
#         right_speed = -MAX_SPEED
#         reverse_steps -= 1

#         if reverse_steps <= 0:
#             turn_steps = 50
#             state = TURN

#     elif state == TURN:
#         if turn_left:
#             left_speed = -MAX_SPEED
#             right_speed = MAX_SPEED
#         else:
#             left_speed = MAX_SPEED
#             right_speed = -MAX_SPEED

#         turn_steps -= 1
#         if turn_steps <= 0:
#             escape_steps = 60
#             cooldown_steps = 60
#             state = ESCAPE

#     elif state == ESCAPE:
#         # if still blocked, keep turning, else go forward
#         if front > DETECT_THRESHOLD:
#             left_speed = -MAX_SPEED
#             right_speed = MAX_SPEED
#         else:
#             left_speed = MAX_SPEED
#             right_speed = MAX_SPEED

#         escape_steps -= 1
#         if escape_steps <= 0:
#             escape_completed_once = True
#             state = FORWARD

#     # apply motor speeds
#     left_motor.setVelocity(left_speed)
#     right_motor.setVelocity(right_speed)

#     # ---------------- STOP + PRINT + SAVE ----------------
#     if escape_completed_once:
#         total_time = robot.getTime() - start_time

#         print("✅ TRIAL COMPLETED (IMPROVED + ANTI-STUCK)")
#         print(f"Noise Mode = {NOISE_MODE}")
#         print(f"Final Time = {total_time:.2f} sec")
#         print(f"Closest Approach (max_front raw) = {max_front:.1f}")
#         print(f"Unsafe Proximity (0/1) = {unsafe_proximity}")
#         print(f"Reverse Count = {reverse_count}")

#         with open(csv_file, "a", newline="") as f:
#             writer = csv.writer(f)
#             if not file_exists:
#                 writer.writerow([
#                     "NoiseMode",
#                     "NoiseLevel",
#                     "TrialTime_sec",
#                     "ClosestApproach_raw",
#                     "UnsafeProximity_0or1",
#                     "ReverseCount"
#                 ])
#                 file_exists = True

#             writer.writerow([
#                 NOISE_MODE,
#                 NOISE_LEVEL if NOISE_MODE else 0,
#                 round(total_time, 2),
#                 round(max_front, 1),
#                 unsafe_proximity,
#                 reverse_count
#             ])

#         break
