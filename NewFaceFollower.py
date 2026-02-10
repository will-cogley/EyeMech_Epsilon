"""
Will Cogley's Eye Mechanism control code
Requires the micropython-servo and picozero libraries
https://pypi.org/project/micropython-servo/
https://pypi.org/project/picozero/
"""

import time
from machine import Pin, I2C, ADC, UART
from servo import Servo
from picozero import Button
import time, random, ujson, urandom, sys, select, uselect, math
#test change

# Set up the switches and potentiometers
enable = Pin(6, Pin.IN, Pin.PULL_UP)
mode = Pin(7, Pin.IN, Pin.PULL_UP)
blink_pin = Pin(9, Pin.IN, Pin.PULL_UP)
UD = ADC(26)
trim = ADC(27)
LR = ADC(28)
uart1 = UART(0, baudrate=921600, tx=Pin(0), rx=Pin(1))

INVOKE_INTERVAL_MS = 500 # Adjust this: faster interval means more frames/sec
INVOKE_CMD = b"AT+INVOKE=1,0,1\r"
cbuf = []
readflag = True
staticflag = False
last_boxes = None
deadzone = 8
x_offset = 0
y_offset = 0
x_adj_factor = 1
y_adj_factor = 1
pixel_centre = 112
tl_target = 90
tr_target = 90
bl_target = 90
br_target = 90


# Define servos
servos = {
    "LR": Servo(pin_id=10),
    "UD": Servo(pin_id=11),
    "TL": Servo(pin_id=12),
    "BL": Servo(pin_id=13),
    "TR": Servo(pin_id=14),
    "BR": Servo(pin_id=15),
}

# Min, Max
servo_limits = {
    "LR": (40, 140),   
    "UD": (40, 140),
    "TL": (90, 170),
    "BL": (90, 10),
    "TR": (90, 10),
    "BR": (90, 160),
}

def continuous_read():
    global cbuf, readflag, staticflag, last_boxes, x_offset, y_offset
    if readflag == True:
        while uart1.any():
            uart1.read()
        uart1.write(INVOKE_CMD)
        cbuf = b""
        readflag = False
    if readflag == False:
        if uart1.any():
            data = uart1.read()
            for ch in data:
                cbuf += bytes([ch]) 
            if b'"resolution"' in cbuf:
                key = b'"boxes":'
                i = cbuf.find(key)
                if i != -1:
                    boxes_part = cbuf[i + len(key):]
                    boxes_part = boxes_part[:boxes_part.find(b']') + 1]
                    boxes_part = boxes_part.strip()
                    if boxes_part != b'[]' and boxes_part != last_boxes:
                        staticflag = False
                        boxes_str = boxes_part.decode('utf-8').strip('[]')
                        numbers = [int(n) for n in boxes_str.split(',')]
                        x_offset, y_offset = numbers[0] - pixel_centre, numbers[1] - pixel_centre
#                         print("x: ", x_offset, "y: ", y_offset)
                        last_boxes = boxes_part
                    else:
                        x_offset, y_offset = 0, 0
                        staticflag = True
                cbuf = b""
                readflag = True

# Set all servos to central position for assembly
def calibrate():
    for name, servo in servos.items():
#         print("neutral position")  # Debug output
        servo.write(90)

# Neutral pose
def neutral():
    eyes = list(servos.keys())[2:]  # Get last 4 servo names
    for name, servo in servos.items():
        servo.write(90)
    lids = list(servos.keys())[-4:]  # Get last 4 servo names
    for servo in lids:
        max_angle = servo_limits[servo][1]  # Get min angle
        servos[servo].write(max_angle)  # Move to min      
        

    
def blink():
    lids = list(servos.keys())[-4:]  # Get last 4 servo names
    for servo in lids:
        min_angle = servo_limits[servo][0]  # Get min angle
        servos[servo].write(min_angle)  # Move to min
        
def control_ud_and_lids(ud_angle): # Moves UD servo and makes TL/TR follow instantly based on UD's position
    global tl_target, tr_target, bl_target, br_target
    # Get limits
    ud_min, ud_max = servo_limits["UD"]
    tl_min, tl_max = servo_limits["TL"]
    tr_min, tr_max = servo_limits["TR"]
    bl_min, bl_max = servo_limits["BL"]
    br_min, br_max = servo_limits["BR"]

    # Normalize UD position to a 0-1 range
    ud_progress = (ud_angle - ud_min) / (ud_max - ud_min)  # 0 (min) → 1 (max)
    
    # Find target positions
    tl_target = tl_max - ((tl_max - tl_min)*(0.8*(1-ud_progress)))
    tr_target = tr_max + ((tr_min - tr_max)*(0.8*(1-ud_progress)))
    bl_target = bl_max + ((bl_min - bl_max)*(0.4*(ud_progress)))
    br_target = br_max - ((br_max - br_min)*(0.4*(ud_progress)))
   
# Move all servos instantly
    servos["UD"].write(ud_angle)
    servos["TL"].write(tl_target)
    servos["TR"].write(tr_target)
    servos["BL"].write(bl_target)
    servos["BR"].write(br_target)

def scale_potentiometer(pot_value, servo, reverse=False):
    # Scale potentiometer value to UD range (50 to 130)
    in_min = 300
    in_max = 65300
    min_limit = servo_limits[servo][0]
    max_limit = servo_limits[servo][1]
    
    # Scale the potentiometer input to the servo range
    scaled_value = min_limit + (pot_value - in_min) * (max_limit - min_limit) / (in_max - in_min)
    if reverse:
       scaled_value = max_limit - (scaled_value - min_limit)
    return scaled_value


def map_value(value, in_min, in_max, out_min, out_max):
    # Map the value
    mapped = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return mapped
        
x_target = 90
y_target = 90
adjustment_factor = 0

neutral()

blink_time = 50
blinking = False
        
while True:
    continuous_read() # Updates x and y offset by looking at camera
    if not blinking and random.randrange(500) == 0:
        blinking = True
        blink_counter = blink_time
        
    # --- blinking state ---
    if blinking:
        if blink_counter > blink_time - 10:
            # closed
            servos["TL"].write(servo_limits["TL"][0])
            servos["TR"].write(servo_limits["TR"][0])
            servos["BL"].write(servo_limits["BL"][0])
            servos["BR"].write(servo_limits["BR"][0])

        elif blink_counter > 0:
            # reopen
            servos["TL"].write(tl_target)
            servos["TR"].write(tr_target)
            servos["BL"].write(bl_target)
            servos["BR"].write(br_target)

        blink_counter -= 1

        if blink_counter == 0:
            blinking = False
    else:
        if (x_offset < -deadzone or x_offset > deadzone) and not staticflag:
            x_adj_value = map_value(x_offset, -110, 110, x_adj_factor, -x_adj_factor)
            x_target = max(servo_limits["LR"][0],
                           min(x_target + x_adj_value, servo_limits["LR"][1]))
            servos["LR"].write(x_target)

        if (y_offset < -deadzone or y_offset > deadzone) and not staticflag:
            y_adj_value = map_value(y_offset, -110, 110, y_adj_factor, -y_adj_factor)
            y_target = max(servo_limits["UD"][0],
                           min(y_target + y_adj_value, servo_limits["UD"][1]))
            control_ud_and_lids(y_target)
    time.sleep_ms(1)

