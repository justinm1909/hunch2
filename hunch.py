import time
import math
from mpu6050 import mpu6050
from adafruit_servokit import ServoKit
from pynput.keyboard import Listener, Key
 
sensor = mpu6050(0x68)
kit = ServoKit(channels=16)
 
SERVOS = [0, 1, 2]
 
CENTER = 90
GAIN = 2.4
MAX_DEFLECT = 75
DEADBAND = 4
 
BASELINE_TIME = 3.0
IMPACT_MARGIN = 2.8
WAIT_TIME = 2.0
CONFIRM_SAMPLES = 2
 
baseline_samples = []
baseline_g = None
baseline_start = None
armed = False
 
impact_detected = False
impact_time = 0
confirm_count = 0
 
servo_dirs = [
    math.radians(0),
    math.radians(120),
    math.radians(240)
]
 
print("Initializing servos...")
for s in SERVOS:
    kit.servo[s].actuation_range = 180
    kit.servo[s].set_pulse_width_range(650, 2350)
    kit.servo[s].angle = CENTER
 
 
def clamp(v):
    return max(0, min(180, v))
 
 
def on_press(key):
    if key in (Key.shift, Key.shift_l, Key.shift_r):
        print("SHIFT pressed — capturing zero")
        return False
 
 
print("Hold robot perfectly upright.")
print("Press SHIFT to capture reference angle.")
 
with Listener(on_press=on_press) as listener:
    listener.join()
 
time.sleep(1)
 
print("Reading reference accelerometer values...")
while True:
    try:
        accel = sensor.get_accel_data()
        break
    except OSError:
        continue
 
ref_pitch = math.atan2(accel['x'], accel['z'])
ref_roll = math.atan2(accel['y'], accel['z'])
 
print(f"Reference pitch: {math.degrees(ref_pitch):.2f}°")
print(f"Reference roll : {math.degrees(ref_roll):.2f}°")
 
for s in SERVOS:
    kit.servo[s].angle = CENTER
 
print("Robot centered.")
print("Learning normal acceleration... DO NOT TOUCH ROBOT")
 
while True:
 
    try:
        accel = sensor.get_accel_data()
    except OSError:
        continue
 
    g = math.sqrt(accel['x']**2 + accel['y']**2 + accel['z']**2)
 
    # ---------- BASELINE ----------
    if not armed:
 
        if baseline_start is None:
            baseline_start = time.time()
 
        baseline_samples.append(g)
 
        if time.time() - baseline_start < BASELINE_TIME:
            print(f"Learning baseline: {g:.2f}g")
            continue
 
        baseline_g = sum(baseline_samples) / len(baseline_samples)
 
        print(f"BASELINE LOCKED: {baseline_g:.2f}g")
        print("Impact detector ARMED")
 
        armed = True
        continue
 
    # ---------- IMPACT CONFIRMATION ----------
    if not impact_detected:
 
        if g > baseline_g + IMPACT_MARGIN:
            confirm_count += 1
            print(f"Impact check {confirm_count}/{CONFIRM_SAMPLES} | g={g:.2f}")
        else:
            confirm_count = 0
            continue
 
        if confirm_count < CONFIRM_SAMPLES:
            continue
 
        impact_detected = True
        impact_time = time.time()
        print("IMPACT CONFIRMED — waiting 2 seconds")
        continue
 
    # ---------- WAIT ----------
    if time.time() - impact_time < WAIT_TIME:
        continue
 
    # ---------- BALANCE ----------
    pitch = math.atan2(accel['x'], accel['z']) - ref_pitch
    roll = math.atan2(accel['y'], accel['z']) - ref_roll
 
    tilt_mag = math.sqrt(pitch**2 + roll**2)
 
    print(
        f"Pitch: {math.degrees(pitch):6.2f}° | "
        f"Roll: {math.degrees(roll):6.2f}° | "
        f"Tilt mag: {math.degrees(tilt_mag):6.2f}°"
    )
 
    if tilt_mag < math.radians(DEADBAND):
        print("Within deadband — centering servos")
        for s in SERVOS:
            kit.servo[s].angle = CENTER
        time.sleep(0.02)
        continue
 
    tilt_angle = math.atan2(roll, pitch)
    print(f"Tilt angle: {math.degrees(tilt_angle):.2f}°")
 
    for i, dir_angle in enumerate(servo_dirs):
        projection = math.cos(tilt_angle - dir_angle)
        correction = projection * tilt_mag * GAIN * 180 / math.pi
        correction = max(-MAX_DEFLECT, min(MAX_DEFLECT, correction))
 
        target = CENTER - correction
        kit.servo[i].angle = clamp(target)
 
        print(
            f"Servo {i}: "
            f"Projection={projection:+.2f}, "
            f"Correction={correction:+.2f}, "
            f"Target={target:.2f}"
        )
 
    print("-" * 50)
 
    time.sleep(0.02)
