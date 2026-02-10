"""
Will Cogley's Eye Mechanism control code (BASE)
Requires the micropython-servo and picozero libraries
https://pypi.org/project/micropython-servo/
https://pypi.org/project/picozero/

This file is based on the user's pasted "manual/random + potentiometer" version.
Edits marked with:  # [MOD] ...
"""

import time
from machine import Pin, ADC, UART  # [MOD] add UART for Grove Vision AI
from servo import Servo
from picozero import Button
import random
import ujson  # [MOD] parse Grove Vision JSON

# ----------------------------
# Switches / potentiometers
# ----------------------------
enable = Pin(6, Pin.IN, Pin.PULL_UP)
mode = Pin(7, Pin.IN, Pin.PULL_UP)
blink_pin = Pin(9, Pin.IN, Pin.PULL_UP)
UD = ADC(26)
trim = ADC(27)
LR = ADC(28)

# ----------------------------
# Grove Vision AI (UART)
# ----------------------------
uart = UART(0, baudrate=921600, tx=Pin(0), rx=Pin(1))  # [MOD] Grove Vision UART
FACE_SCORE_THRESHOLD = 30  # [MOD] set score threshold to 30%

def set_score_threshold(th=FACE_SCORE_THRESHOLD):
    """Send AT+TSCORE to Grove Vision (if supported by firmware)."""
    try:
        while uart.any():
            uart.read()
        uart.write("AT+TSCORE={}\r".format(int(th)))  # [MOD]
        time.sleep_ms(50)
        while uart.any():
            uart.read()
    except Exception:
        pass

set_score_threshold()  # [MOD] apply once at boot

# ----------------------------
# Servos
# ----------------------------
servos = {
    "LR": Servo(pin_id=10),
    "UD": Servo(pin_id=11),
    "TL": Servo(pin_id=12),
    "BL": Servo(pin_id=13),
    "TR": Servo(pin_id=14),
    "BR": Servo(pin_id=15),
}

# Min, Max
# [MOD] tighten LR/UD range to reduce "looking into corners"
# [MOD] reduce "fully open" eyelid angles by ~10° so lids can fully close
servo_limits = {
    "LR": (65, 115),     # [MOD] was (40, 140)
    "UD": (65, 115),     # [MOD] was (40, 140)

    "TL": (90, 160),     # [MOD] was (90, 170)
    "BL": (90, 20),      # [MOD] was (90, 10)  (reversed range)
    "TR": (90, 20),      # [MOD] was (90, 10)  (reversed range)
    "BR": (90, 150),     # [MOD] was (90, 160)
}

def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

# ----------------------------
# Base functions (mostly unchanged)
# ----------------------------
def calibrate():
    for _, servo in servos.items():
        servo.write(90)

def neutral():
    for _, servo in servos.items():
        servo.write(90)

    lids = list(servos.keys())[-4:]
    for s in lids:
        max_angle = servo_limits[s][1]
        servos[s].write(max_angle)

def blink_close():
    """Close lids to the 'closed' position (close = 90 for all lids in this build)."""
    lids = list(servos.keys())[-4:]
    for s in lids:
        close_angle = servo_limits[s][0]
        servos[s].write(close_angle)

def control_ud_and_lids(ud_angle):
    """Moves UD servo and makes eyelids follow UD instantly based on UD position."""
    ud_min, ud_max = servo_limits["UD"]
    tl_min, tl_max = servo_limits["TL"]
    tr_min, tr_max = servo_limits["TR"]
    bl_min, bl_max = servo_limits["BL"]
    br_min, br_max = servo_limits["BR"]

    ud_progress = (ud_angle - ud_min) / (ud_max - ud_min + 1e-6)

    tl_target = tl_max - ((tl_max - tl_min) * (0.8 * (1 - ud_progress)))
    tr_target = tr_max + ((tr_min - tr_max) * (0.8 * (1 - ud_progress)))
    bl_target = bl_max + ((bl_min - bl_max) * (0.4 * (ud_progress)))
    br_target = br_max - ((br_max - br_min) * (0.4 * (ud_progress)))

    servos["UD"].write(int(ud_angle))
    servos["TL"].write(int(tl_target))
    servos["TR"].write(int(tr_target))
    servos["BL"].write(int(bl_target))
    servos["BR"].write(int(br_target))

def scale_potentiometer(pot_value, servo, reverse=False):
    in_min = 300
    in_max = 65300
    min_limit = servo_limits[servo][0]
    max_limit = servo_limits[servo][1]

    scaled_value = min_limit + (pot_value - in_min) * (max_limit - min_limit) / (in_max - in_min)
    if reverse:
        scaled_value = max_limit - (scaled_value - min_limit)
    return scaled_value

def update_eyelid_limits(trim_value):
    trim_min = 7000
    trim_max = 14500

    # [MOD] shift max/open range ~10° towards "less open"
    TL_max_range = (120, 160)   # [MOD] was (130, 170)
    BR_max_range = (120, 160)   # [MOD] was (130, 170)
    BL_max_range = (60, 20)     # [MOD] was (50, 10) reversed
    TR_max_range = (60, 20)     # [MOD] was (50, 10) reversed

    trim_progress = (trim_value - trim_min) / (trim_max - trim_min + 1e-6)
    trim_progress = max(0, min(1, trim_progress))

    servo_limits["TL"] = (90, TL_max_range[0] + (TL_max_range[1] - TL_max_range[0]) * trim_progress)
    servo_limits["BR"] = (90, BR_max_range[0] + (BR_max_range[1] - BR_max_range[0]) * trim_progress)
    servo_limits["BL"] = (90, BL_max_range[0] + (BL_max_range[1] - BL_max_range[0]) * trim_progress)
    servo_limits["TR"] = (90, TR_max_range[0] + (TR_max_range[1] - TR_max_range[0]) * trim_progress)

# ----------------------------
# [MOD] AI tracking + blink state machine
# ----------------------------

# Blink tuning (fast + fully closes)
BLINK_MIN_MS = 1200
BLINK_MAX_MS = 3000
BLINK_HOLD_MS = 180       # [MOD] keep closed long enough so it actually reaches closed
BLINK_RECOVERY_MS = 140   # [MOD] reopen settle
blink_next_ms = time.ticks_ms() + random.randint(BLINK_MIN_MS, BLINK_MAX_MS)

blink_state = 0   # [MOD] 0=open idle, 1=closed hold, 2=recovery
blink_t0 = time.ticks_ms()

# Tracking tuning
MEAS_ALPHA = 0.25
CENTER_BETA = 0.02
Kp_x = 0.032
Kp_y = 0.030
MAX_STEP_DEG = 1.1
deadzone_x = 10
deadzone_y = 12
LOST_TIMEOUT_MS = 250

sx = 0.0
sy = 0.0
cx0 = None
cy0 = None
last_seen_ms = 0

x_target = 90.0
y_target = 90.0

def get_camera_boxes(timeout_ms=15):
    """Ask Grove Vision for a frame and return boxes list (or None)."""
    while uart.any():
        uart.read()
    try:
        uart.write("AT+INVOKE=1,0,1\r")  # [MOD]
    except:
        return None

    start = time.ticks_ms()
    buf = bytearray()
    depth = 0
    in_json = False
    BUF_MAX = 4096

    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        if uart.any():
            data = uart.read()
            if not data:
                continue
            for ch in data:
                if ch == ord("{"):
                    in_json = True
                    depth += 1
                if in_json:
                    buf.append(ch)
                    if len(buf) > BUF_MAX:
                        buf = bytearray()
                        depth = 0
                        in_json = False
                        break
                if ch == ord("}") and in_json:
                    depth -= 1
                    if depth == 0:
                        try:
                            js = ujson.loads(buf)
                            if js.get("type") == 1:
                                return js.get("data", {}).get("boxes", [])
                        except:
                            pass
                        buf = bytearray()
                        in_json = False
    return None

def pick_target_box(boxes):
    """Pick largest-area box (more stable than 'last')."""
    best = None
    best_area = -1.0
    for b in boxes:
        if b and len(b) >= 4:
            try:
                w = float(b[2]); h = float(b[3])
                area = w * h
                if area > best_area:
                    best_area = area
                    best = b
            except:
                pass
    return best if best is not None else (boxes[-1] if boxes else None)

def extract_center_xy(b):
    """Return center (cx, cy) in pixels from a box."""
    if (not b) or (len(b) < 2):
        return 0.0, 0.0, False
    try:
        x0 = float(b[0])
        y0 = float(b[1])
        if len(b) >= 4:
            w = float(b[2]); h = float(b[3])
            if w > 1.0 and h > 1.0 and w < 10000 and h < 10000:
                return x0 + 0.5 * w, y0 + 0.5 * h, True
        return x0, y0, True
    except:
        return 0.0, 0.0, False

def step_servo_target(cur, err, kp, deadzone, max_step, lo, hi):
    """One small step towards reducing pixel error."""
    if abs(err) <= deadzone:
        return cur
    step = kp * err
    if abs(step) < 1.0:
        step = 1.0 if step > 0 else -1.0
    step = clamp(step, -max_step, max_step)
    return clamp(cur + step, lo, hi)

def blink_update(now_ms):
    """Fast human-like blink that really closes."""
    global blink_state, blink_t0, blink_next_ms

    if blink_state == 0:
        if now_ms >= blink_next_ms:
            blink_close()
            blink_state = 1
            blink_t0 = now_ms
        return

    if blink_state == 1:
        # [MOD] keep commanding closed during hold (prevents "only moved 1°")
        blink_close()
        if time.ticks_diff(now_ms, blink_t0) >= BLINK_HOLD_MS:
            blink_state = 2
            blink_t0 = now_ms
        return

    if blink_state == 2:
        # [MOD] reopen by restoring lids to follow current UD (prevents twitch)
        control_ud_and_lids(int(y_target))
        if time.ticks_diff(now_ms, blink_t0) >= BLINK_RECOVERY_MS:
            blink_state = 0
            blink_next_ms = now_ms + random.randint(BLINK_MIN_MS, BLINK_MAX_MS)
        return

# Start in a reasonable pose
neutral()
x_target = 90.0
y_target = 90.0
control_ud_and_lids(int(y_target))

while True:
    mode_state = not mode.value()
    enable_state = not enable.value()

    if mode_state == 1:  # calibration / hold
        calibrate()
        time.sleep_ms(500)
        continue

    # ----------------------------
    # [MOD] Auto mode => AI tracking
    # ----------------------------
    if enable_state == 0:
        now = time.ticks_ms()

        # [MOD] allow trim to adjust eyelid openness even in AI mode
        trim_value = trim.read_u16()
        update_eyelid_limits(trim_value)

        # [MOD] blink timing/state
        blink_update(now)

        # [MOD] tracking
        boxes = get_camera_boxes(timeout_ms=15)
        if boxes and len(boxes) > 0:
            b = pick_target_box(boxes)
            cx, cy, ok = extract_center_xy(b)
            if ok:
                if cx0 is None:
                    cx0 = cx; cy0 = cy
                    sx = cx; sy = cy

                sx = MEAS_ALPHA * cx + (1.0 - MEAS_ALPHA) * sx
                sy = MEAS_ALPHA * cy + (1.0 - MEAS_ALPHA) * sy

                cx0 = cx0 + CENTER_BETA * (sx - cx0)
                cy0 = cy0 + CENTER_BETA * (sy - cy0)

                err_x = (cx0 - sx)
                err_y = (cy0 - sy)

                x_target = step_servo_target(
                    x_target, err_x, Kp_x, deadzone_x, MAX_STEP_DEG,
                    servo_limits["LR"][0], servo_limits["LR"][1]
                )
                y_target = step_servo_target(
                    y_target, err_y, Kp_y, deadzone_y, MAX_STEP_DEG,
                    servo_limits["UD"][0], servo_limits["UD"][1]
                )

                servos["LR"].write(int(x_target))

                # [MOD] during closed hold, don't let UD->lids mapping fight blink
                if blink_state != 1:
                    control_ud_and_lids(int(y_target))
                else:
                    servos["UD"].write(int(y_target))

                last_seen_ms = now

        # [MOD] if lost, slowly return towards center
        if time.ticks_diff(now, last_seen_ms) > LOST_TIMEOUT_MS:
            x_target = step_servo_target(x_target, (90 - x_target), 1.0, 0.0, 0.9,
                                         servo_limits["LR"][0], servo_limits["LR"][1])
            y_target = step_servo_target(y_target, (90 - y_target), 1.0, 0.0, 0.9,
                                         servo_limits["UD"][0], servo_limits["UD"][1])
            servos["LR"].write(int(x_target))
            if blink_state != 1:
                control_ud_and_lids(int(y_target))
            else:
                servos["UD"].write(int(y_target))

        time.sleep_ms(5)
        continue

    # ----------------------------
    # Controller mode (original)
    # ----------------------------
    UD_value = UD.read_u16()
    trim_value = trim.read_u16()
    LR_value = LR.read_u16()
    blink_btn = not blink_pin.value()

    update_eyelid_limits(trim_value)

    if blink_btn == 0:
        blink_close()
    else:
        servos["LR"].write(int(scale_potentiometer(LR_value, "LR", reverse=True)))
        control_ud_and_lids(int(scale_potentiometer(UD_value, "UD")))

    time.sleep_ms(10)
