# main.py
# Author: koala
#
# Update (blink-swap + tracking fix):
# - Replaced the blink state machine with the "hard blink" style (full close → hold → reopen) like your other working code.
# - Fixed tracking reliability by using a *stateful* UART JSON parser (no per-frame flush). This avoids discarding late camera replies.
# - HOLD mode (Pin7 LOW): eyelids closed + LR/UD forced to center.
# - MOVE mode (Pin7 HIGH): face tracking + blinking.

import time
import random
import ujson
from machine import Pin, UART
from servo import Servo

# -----------------------------
# Hardware pins
# -----------------------------
mode_pin = Pin(7, Pin.IN, Pin.PULL_UP)     # HIGH=move, LOW=hold
blink_pin = Pin(9, Pin.IN, Pin.PULL_UP)    # optional manual blink button (active low)

uart = UART(0, baudrate=921600, tx=Pin(0), rx=Pin(1))

try:
    led = Pin(25, Pin.OUT)
except:
    led = Pin("LED", Pin.OUT)

# -----------------------------
# Grove Vision settings
# -----------------------------
FACE_SCORE_THRESHOLD = 30
INVOKE_CMD = b"AT+INVOKE=1,0,1\r"
TSCORE_CMD = "AT+TSCORE={}\r".format(FACE_SCORE_THRESHOLD).encode()

INVOKE_INTERVAL_MS = 80   # faster invoke = faster tracking response
BUF_MAX = 4096

# -----------------------------
# Servo setup (uses your servo.py)
# -----------------------------
servos = {
    "LR": Servo(pin_id=10),
    "UD": Servo(pin_id=11),
    "TL": Servo(pin_id=12),
    "BL": Servo(pin_id=13),
    "TR": Servo(pin_id=14),
    "BR": Servo(pin_id=15),
}

servo_limits = {
    "LR": (40, 140),
    "UD": (40, 140),
    "TL": (90, 170),
    "BL": (90, 10),
    "TR": (90, 10),
    "BR": (90, 160),
}

LID_NAMES = ("TL", "TR", "BL", "BR")

CENTER_LR = 90
CENTER_UD = 90

# Soft limits to prevent over-rotating to corners
LR_SOFT_MIN = 65
LR_SOFT_MAX = 115
UD_SOFT_MIN = 65
UD_SOFT_MAX = 115

# -----------------------------
# Blink (hard blink) parameters
# -----------------------------
# These are the "open" angles. We'll apply OPEN_REDUCE so the default "open" is slightly less open,
# helping the close reach full closure.
OPEN_POS_BASE = {"TL": 160, "TR": 20, "BL": 20, "BR": 150}
CLOSE_POS = {"TL": 90, "TR": 90, "BL": 90, "BR": 90}
OPEN_REDUCE = 10  # user request: "open angle reduce 10°"

BLINK_MIN_MS = 1200
BLINK_MAX_MS = 3000
BLINK_HOLD_MS = 120          # faster blink
BLINK_RECOVERY_MS = 90       # faster reopen

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

def open_pos():
    # TL/BR: smaller = less open (toward close); TR/BL: larger = less open (toward close)
    tl = OPEN_POS_BASE["TL"] - OPEN_REDUCE
    br = OPEN_POS_BASE["BR"] - OPEN_REDUCE
    tr = OPEN_POS_BASE["TR"] + OPEN_REDUCE
    bl = OPEN_POS_BASE["BL"] + OPEN_REDUCE

    # clamp to per-servo limits
    tl = int(clamp(tl, min(servo_limits["TL"]), max(servo_limits["TL"])))
    br = int(clamp(br, min(servo_limits["BR"]), max(servo_limits["BR"])))
    tr = int(clamp(tr, min(servo_limits["TR"]), max(servo_limits["TR"])))
    bl = int(clamp(bl, min(servo_limits["BL"]), max(servo_limits["BL"])))
    return {"TL": tl, "TR": tr, "BL": bl, "BR": br}

def lids_write(tgts):
    servos["TL"].write(int(tgts["TL"]))
    servos["TR"].write(int(tgts["TR"]))
    servos["BL"].write(int(tgts["BL"]))
    servos["BR"].write(int(tgts["BR"]))

LID_STATE_OPEN_IDLE = 0
LID_STATE_CLOSED_HOLD = 1
LID_STATE_RECOVERY = 2

lid_state = LID_STATE_OPEN_IDLE
lid_t0 = time.ticks_ms()
blink_next_time = time.ticks_ms() + random.randint(BLINK_MIN_MS, BLINK_MAX_MS)

def blink_update(now_ms, move_mode=True):
    global lid_state, lid_t0, blink_next_time

    if not move_mode:
        lids_write(CLOSE_POS)
        lid_state = LID_STATE_OPEN_IDLE
        blink_next_time = now_ms + 800
        return

    # manual blink (active low) also triggers a blink
    manual = (blink_pin.value() == 0)

    if lid_state == LID_STATE_OPEN_IDLE:
        lids_write(open_pos())
        if manual or (now_ms >= blink_next_time):
            lids_write(CLOSE_POS)
            lid_state = LID_STATE_CLOSED_HOLD
            lid_t0 = now_ms
        return

    if lid_state == LID_STATE_CLOSED_HOLD:
        lids_write(CLOSE_POS)
        if time.ticks_diff(now_ms, lid_t0) >= BLINK_HOLD_MS:
            lids_write(open_pos())
            lid_state = LID_STATE_RECOVERY
            lid_t0 = now_ms
        return

    if lid_state == LID_STATE_RECOVERY:
        lids_write(open_pos())
        if time.ticks_diff(now_ms, lid_t0) >= BLINK_RECOVERY_MS:
            lid_state = LID_STATE_OPEN_IDLE
            blink_next_time = now_ms + random.randint(BLINK_MIN_MS, BLINK_MAX_MS)
        return

    lid_state = LID_STATE_OPEN_IDLE
    blink_next_time = now_ms + random.randint(BLINK_MIN_MS, BLINK_MAX_MS)

# -----------------------------
# Tracking parameters
# -----------------------------
MEAS_ALPHA = 0.25
CENTER_BETA = 0.02

Kp_x = 0.032
Kp_y = 0.030
MAX_STEP_DEG = 1.2

deadzone_x = 10
deadzone_y = 12

LOST_TIMEOUT_MS = 350

# -----------------------------
# Robust stateful UART JSON parser
# (Fixes "no tracking" caused by flushing and timing mismatch)
# -----------------------------
_rx_buf = bytearray()
_depth = 0
_in_json = False
_last_invoke_ms = 0

def camera_send_invoke(now_ms):
    global _last_invoke_ms
    if time.ticks_diff(now_ms, _last_invoke_ms) >= INVOKE_INTERVAL_MS:
        try:
            uart.write(INVOKE_CMD)
        except:
            pass
        _last_invoke_ms = now_ms

def camera_feed_and_parse():
    global _rx_buf, _depth, _in_json

    while uart.any():
        data = uart.read()
        if not data:
            break
        for ch in data:
            if ch == ord("{"):
                _in_json = True
                _depth += 1
            if _in_json:
                _rx_buf.append(ch)
                if len(_rx_buf) > BUF_MAX:
                    _rx_buf = bytearray()
                    _depth = 0
                    _in_json = False
                    break
            if ch == ord("}") and _in_json:
                _depth -= 1
                if _depth == 0:
                    # one full JSON message completed
                    try:
                        js = ujson.loads(_rx_buf)
                        _rx_buf = bytearray()
                        _in_json = False
                        if js.get("type") == 1:
                            return js.get("data", {}).get("boxes", None)
                    except:
                        _rx_buf = bytearray()
                        _in_json = False
    return None

def pick_target_box(boxes):
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
    if best is not None:
        return best
    try:
        return boxes[-1]
    except:
        return None

def extract_center_xy(b):
    if (not b) or (len(b) < 2):
        return 0.0, 0.0, False
    try:
        x0 = float(b[0])
        y0 = float(b[1])
        if len(b) >= 4:
            w = float(b[2])
            h = float(b[3])
            if w > 1.0 and h > 1.0 and w < 10000 and h < 10000:
                return x0 + 0.5 * w, y0 + 0.5 * h, True
        return x0, y0, True
    except:
        return 0.0, 0.0, False

def move_axis(cur_angle, error, kp, deadzone, soft_min, soft_max):
    if abs(error) <= deadzone:
        return cur_angle
    step = kp * error
    if abs(step) < 1.0:
        step = 1.0 if step > 0 else -1.0
    step = clamp(step, -MAX_STEP_DEG, MAX_STEP_DEG)
    return clamp(cur_angle + step, soft_min, soft_max)

def center_all(close_lids=True):
    servos["LR"].write(CENTER_LR)
    servos["UD"].write(CENTER_UD)
    if close_lids:
        lids_write(CLOSE_POS)
    else:
        lids_write(open_pos())

# -----------------------------
# Init
# -----------------------------
# Set threshold once, and DO NOT flush UART continuously (we must keep late replies).
try:
    uart.write(TSCORE_CMD)
except:
    pass
time.sleep_ms(80)

center_all(close_lids=False)

sx = 0.0
sy = 0.0
cx0 = None
cy0 = None
last_seen_ms = 0

lr_ang = float(CENTER_LR)
ud_ang = float(CENTER_UD)

prev_move_mode = None

# -----------------------------
# Main loop
# -----------------------------
while True:
    now = time.ticks_ms()
    is_move_mode = (mode_pin.value() == 1)

    if prev_move_mode is None:
        prev_move_mode = is_move_mode
    entered_hold = (prev_move_mode is True) and (is_move_mode is False)
    prev_move_mode = is_move_mode

    if is_move_mode:
        led.value(1 if ((now // 120) % 2 == 0) else 0)

        # 1) send invoke periodically (non-blocking)
        camera_send_invoke(now)

        # 2) read & parse any arriving JSON (stateful)
        boxes = camera_feed_and_parse()
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

                # error sign: move towards the face centre
                err_x = (cx0 - sx)
                err_y = (cy0 - sy)

                lr_ang = move_axis(lr_ang, err_x, Kp_x, deadzone_x, LR_SOFT_MIN, LR_SOFT_MAX)
                ud_ang = move_axis(ud_ang, err_y, Kp_y, deadzone_y, UD_SOFT_MIN, UD_SOFT_MAX)

                servos["LR"].write(int(lr_ang))
                servos["UD"].write(int(ud_ang))

                last_seen_ms = now

        # lost -> slowly recenter
        if time.ticks_diff(now, last_seen_ms) > LOST_TIMEOUT_MS:
            lr_ang = lr_ang + clamp(CENTER_LR - lr_ang, -0.9, 0.9)
            ud_ang = ud_ang + clamp(CENTER_UD - ud_ang, -0.9, 0.9)
            servos["LR"].write(int(lr_ang))
            servos["UD"].write(int(ud_ang))
            cx0 = None
            cy0 = None

        # blink runs independently from tracking
        blink_update(now, move_mode=True)

    else:
        led.value(1)

        if entered_hold:
            # reset tracking when entering hold
            cx0 = None
            cy0 = None
            last_seen_ms = 0
            lr_ang = float(CENTER_LR)
            ud_ang = float(CENTER_UD)

        # HOLD behaviour you requested: close + center all
        servos["LR"].write(CENTER_LR)
        servos["UD"].write(CENTER_UD)
        blink_update(now, move_mode=False)

    time.sleep_ms(2)
