# Author: koala
import time
import random
import ujson
from machine import Pin, PWM, UART

try:
    led = Pin(25, Pin.OUT)
except:
    led = Pin("LED", Pin.OUT)
led.value(1)

time.sleep(2)
led.value(0)
time.sleep(0.5)
led.value(1)

mode_pin = Pin(7, Pin.IN, Pin.PULL_UP)
uart = UART(0, baudrate=921600, tx=Pin(0), rx=Pin(1))

FACE_SCORE_THRESHOLD = 30

def set_score_threshold(th=FACE_SCORE_THRESHOLD):
    try:
        while uart.any():
            uart.read()
        uart.write("AT+TSCORE={}\r".format(int(th)))
        time.sleep_ms(50)
        while uart.any():
            uart.read()
    except Exception:
        pass

set_score_threshold()

def clamp(x, a, b):
    return a if x < a else (b if x > b else x)

class SmoothServo:
    def __init__(
        self,
        pin_num, min_limit, max_limit,
        max_speed=600.0, max_accel=2500.0,
        snap_dist=0.5, snap_vel=10.0,
        enable_relax=False, relax_delay_ms=250
    ):
        self.pwm = PWM(Pin(pin_num))
        self.pwm.freq(50)

        self.min_limit = float(min_limit)
        self.max_limit = float(max_limit)
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)

        self.pos = 90.0
        self.target = 90.0
        self.vel = 0.0

        self.last_duty = -1

        self.snap_dist = float(snap_dist)
        self.snap_vel = float(snap_vel)

        self.enable_relax = bool(enable_relax)
        self.relax_delay_ms = int(relax_delay_ms)
        self._settled_since = None
        self._relaxed = False

        self._write_pwm(self.pos)

    def set_target(self, angle):
        new_target = clamp(float(angle), self.min_limit, self.max_limit)
        if self._relaxed:
            self._wake_pwm()
        if abs(new_target - self.target) > 0.1:
            self.target = new_target

    def force_angle(self, angle):
        if self._relaxed:
            self._wake_pwm()
        a = clamp(float(angle), self.min_limit, self.max_limit)
        self.pos = a
        self.target = a
        self.vel = 0.0
        self._write_pwm(self.pos)

    def _relax_pwm(self):
        self.pwm.duty_u16(0)
        self.last_duty = 0
        self._relaxed = True

    def _wake_pwm(self):
        self._relaxed = False
        self.last_duty = -1
        self._write_pwm(self.pos)

    def update(self, dt, now_ms=None, allow_relax=True):
        if dt <= 0:
            return
        if self._relaxed:
            return

        error = self.target - self.pos
        dist = abs(error)

        if dist <= self.snap_dist and abs(self.vel) <= self.snap_vel:
            if self.pos != self.target:
                self.pos = self.target
                self.vel = 0.0
                self._write_pwm(self.pos)

            if self.enable_relax and allow_relax and (now_ms is not None):
                if self._settled_since is None:
                    self._settled_since = now_ms
                elif time.ticks_diff(now_ms, self._settled_since) >= self.relax_delay_ms:
                    self._relax_pwm()
            return
        else:
            self._settled_since = None

        desired_dir = 1.0 if error > 0 else -1.0
        stopping_dist = (self.vel * self.vel) / (2.0 * self.max_accel + 1e-6)

        if (self.vel * desired_dir < 0) or (dist <= stopping_dist):
            accel = (-1.0 if self.vel > 0 else 1.0) * self.max_accel
        else:
            accel = desired_dir * self.max_accel

        self.vel += accel * dt
        self.vel = clamp(self.vel, -self.max_speed, self.max_speed)
        self.pos += self.vel * dt
        self.pos = clamp(self.pos, self.min_limit, self.max_limit)
        self._write_pwm(self.pos)

    def _write_pwm(self, angle):
        us = 500.0 + (2000.0 * angle / 180.0)
        duty = int(us * 65535 / 20000)
        if duty != self.last_duty:
            self.pwm.duty_u16(duty)
            self.last_duty = duty

RELAX_EYELIDS = False
RELAX_DELAY_MS = 220

servos = {
    "LR": SmoothServo(10, 65, 115, max_speed=400, max_accel=1000,
                      snap_dist=0.6, snap_vel=12.0, enable_relax=False),
    "UD": SmoothServo(11, 65, 115, max_speed=400, max_accel=1000,
                      snap_dist=0.6, snap_vel=12.0, enable_relax=False),

    "TL": SmoothServo(12, 70, 170, max_speed=9999, max_accel=30000,
                      snap_dist=1.6, snap_vel=90.0, enable_relax=RELAX_EYELIDS, relax_delay_ms=RELAX_DELAY_MS),
    "TR": SmoothServo(14, 10, 110,  max_speed=9999, max_accel=30000,
                      snap_dist=1.6, snap_vel=90.0, enable_relax=RELAX_EYELIDS, relax_delay_ms=RELAX_DELAY_MS),
    "BL": SmoothServo(13, 0, 110,   max_speed=9999, max_accel=30000,
                      snap_dist=1.6, snap_vel=90.0, enable_relax=RELAX_EYELIDS, relax_delay_ms=RELAX_DELAY_MS),
    "BR": SmoothServo(15, 70, 160, max_speed=9999, max_accel=30000,
                      snap_dist=1.6, snap_vel=90.0, enable_relax=RELAX_EYELIDS, relax_delay_ms=RELAX_DELAY_MS),
}

LID_NAMES = ("TL", "TR", "BL", "BR")

EYE_POS = {
    "OPEN_TL": 155,
    "OPEN_TR": 25,
    "OPEN_BL": 15,
    "OPEN_BR": 150,
    "CLOSE_VAL": 90,
    "CENTER_LR": 90,
    "CENTER_UD": 90
}

OPEN_TRIM = {"TL": 0.0, "TR": 0.0, "BL": 0.0, "BR": 0.0}

FLIP_X = False
FLIP_Y = False
X_SIGN = 1.0
Y_SIGN = 1.0

MEAS_ALPHA = 0.25
CENTER_BETA = 0.02

Kp_x = 0.032
Kp_y = 0.030
MAX_STEP_DEG = 1.1

deadzone_x = 10
deadzone_y = 12

X_AUTO_GAIN = True
X_GAIN_TARGET_ERR = 25.0
X_GAIN_MAX = 5.0

LOST_TIMEOUT_MS = 250

last_time = time.ticks_ms()

BLINK_MIN_MS = 1200
BLINK_MAX_MS = 3000

BLINK_HOLD_MS = 180
BLINK_RECOVERY_MS = 140
BLINK_OPEN = {"TL": 160, "TR": 20, "BL": 20, "BR": 150}
BLINK_CLOSE = {"TL": 90, "TR": 90, "BL": 90, "BR": 90}


last_time = time.ticks_ms()

def lid_targets_open_final():
    return BLINK_OPEN

def lid_targets_close():
    return BLINK_CLOSE

def lids_force_angles(tgts):
    for k in LID_NAMES:
        servos[k].force_angle(tgts[k])

LID_STATE_OPEN_IDLE   = 0
LID_STATE_CLOSED_HOLD = 1
LID_STATE_RECOVERY    = 2

lid_state = LID_STATE_OPEN_IDLE
lid_t0 = time.ticks_ms()
blink_next_time = time.ticks_ms() + random.randint(BLINK_MIN_MS, BLINK_MAX_MS)

lids_force_angles(lid_targets_open_final())

def blink_update(now_ms, move_mode=True):
    global lid_state, lid_t0, blink_next_time

    if not move_mode:
        lids_force_angles(lid_targets_close())
        lid_state = LID_STATE_OPEN_IDLE
        blink_next_time = now_ms + 800
        return False

    if lid_state == LID_STATE_OPEN_IDLE:
        if now_ms >= blink_next_time:
            lids_force_angles(lid_targets_close())
            lid_state = LID_STATE_CLOSED_HOLD
            lid_t0 = now_ms
            return False
        return True

    if lid_state == LID_STATE_CLOSED_HOLD:
        lids_force_angles(lid_targets_close())
        if time.ticks_diff(now_ms, lid_t0) >= BLINK_HOLD_MS:
            lids_force_angles(lid_targets_open_final())
            lid_state = LID_STATE_RECOVERY
            lid_t0 = now_ms
        return False

    if lid_state == LID_STATE_RECOVERY:
        lids_force_angles(lid_targets_open_final())
        if time.ticks_diff(now_ms, lid_t0) >= BLINK_RECOVERY_MS:
            lid_state = LID_STATE_OPEN_IDLE
            blink_next_time = now_ms + random.randint(BLINK_MIN_MS, BLINK_MAX_MS)
        return False

    lid_state = LID_STATE_OPEN_IDLE
    blink_next_time = now_ms + random.randint(BLINK_MIN_MS, BLINK_MAX_MS)
    return True

def get_camera_data(timeout_ms=15):
    while uart.any():
        uart.read()
    try:
        uart.write("AT+INVOKE=1,0,1\r")
    except:
        pass

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

def move_target(servo_name, error, kp, deadzone, max_step_deg):
    if error is None:
        return
    if abs(error) <= deadzone:
        return

    step = kp * error
    if abs(step) < 1.0:
        step = 1.0 if step > 0 else -1.0
    step = clamp(step, -max_step_deg, max_step_deg)
    servos[servo_name].set_target(servos[servo_name].target + step)

sx = 0.0
sy = 0.0
cx0 = None
cy0 = None
x_err_ema = 0.0
last_seen_ms = 0

prev_move_mode = None
HOLD_FORCE_CENTER = True

while True:
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 1000.0
    last_time = now
    if dt < 0:
        dt = 0.0
    if dt > 0.05:
        dt = 0.05

    is_move_mode = (mode_pin.value() == 1)

    if prev_move_mode is None:
        prev_move_mode = is_move_mode
    entered_hold = (prev_move_mode is True) and (is_move_mode is False)
    entered_move = (prev_move_mode is False) and (is_move_mode is True)
    prev_move_mode = is_move_mode

    if is_move_mode:
        led.value(1 if ((now // 100) % 2 == 0) else 0)

        allow_relax_eyelids = blink_update(now, move_mode=True)

        boxes = get_camera_data(timeout_ms=15)
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

                if FLIP_X:
                    sx = 2.0 * cx0 - sx
                if FLIP_Y:
                    sy = 2.0 * cy0 - sy

                error_x = (cx0 - sx) * X_SIGN
                error_y = (cy0 - sy) * Y_SIGN

                if X_AUTO_GAIN:
                    x_err_ema = 0.95 * x_err_ema + 0.05 * abs(error_x)
                    denom = max(x_err_ema, 1e-6)
                    gain = clamp(X_GAIN_TARGET_ERR / denom, 1.0, X_GAIN_MAX)
                    error_x = error_x * gain

                move_target("LR", error_x, Kp_x, deadzone_x, MAX_STEP_DEG)
                move_target("UD", error_y, Kp_y, deadzone_y, MAX_STEP_DEG)

                last_seen_ms = now

        if time.ticks_diff(now, last_seen_ms) > LOST_TIMEOUT_MS:
            servos["LR"].set_target(servos["LR"].target + clamp(EYE_POS["CENTER_LR"] - servos["LR"].target, -0.9, 0.9))
            servos["UD"].set_target(servos["UD"].target + clamp(EYE_POS["CENTER_UD"] - servos["UD"].target, -0.9, 0.9))

    else:
        led.value(1)

        if entered_hold:
            if HOLD_FORCE_CENTER:
                servos["LR"].force_angle(EYE_POS["CENTER_LR"])
                servos["UD"].force_angle(EYE_POS["CENTER_UD"])
            else:
                servos["LR"].set_target(EYE_POS["CENTER_LR"])
                servos["UD"].set_target(EYE_POS["CENTER_UD"])

            cx0 = None
            cy0 = None
            x_err_ema = 0.0
            last_seen_ms = 0

        allow_relax_eyelids = blink_update(now, move_mode=False)
        servos["LR"].set_target(EYE_POS["CENTER_LR"])
        servos["UD"].set_target(EYE_POS["CENTER_UD"])

    for name, s in servos.items():
        if name in LID_NAMES:
            s.update(dt, now_ms=now, allow_relax=allow_relax_eyelids)
        else:
            s.update(dt, now_ms=now, allow_relax=True)

    time.sleep_us(500)
