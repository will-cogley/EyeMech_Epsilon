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
last_boxes = None
deadzone = 10
x_offset = 0
y_offset = 0

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
    global cbuf, readflag, last_boxes, x_offset, y_offset
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
                        boxes_str = boxes_part.decode('utf-8').strip('[]')
                        numbers = [int(n) for n in boxes_str.split(',')]
                        x_offset, y_offset = numbers[0], numbers[1]
#                         print("x: ", x_offset, "y: ", y_offset)
                        last_boxes = boxes_part
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
        
def move_servo_random(servo):
    """Moves the servo to a random position within its defined range."""
    min_angle, max_angle = servo_limits[servo]  # Get range from dictionary
    random_angle = random.randint(min_angle, max_angle)  # Get random value
    servos[servo].write(random_angle)
    print(f"Moved {servo} to {random_angle}°")  # Debug output
#     move_servo_random("LR")

def move_servo_extremes(servo, delay): # Moves the servo between its min and max values
    min_angle, max_angle = servo_limits[servo]
    servos[servo].write(min_angle)  # Move to min position
    print(min_angle)
    time.sleep_ms(delay)
    servos[servo].write(max_angle)  # Move to max position
    print(max_angle)
    time.sleep_ms(delay)
    
def blink():
    lids = list(servos.keys())[-4:]  # Get last 4 servo names
    for servo in lids:
        min_angle = servo_limits[servo][0]  # Get min angle
        servos[servo].write(min_angle)  # Move to min
        
def control_ud_and_lids(ud_angle): # Moves UD servo and makes TL/TR follow instantly based on UD's position
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

def update_eyelid_limits(trim_value):
    # Define the trim value range
    trim_min = 7000
    trim_max = 14500
    
    # Target max ranges for each eyelid servo
    TL_max_range = (130, 170)  # from most closed to most open
    BR_max_range = (130, 170)
    BL_max_range = (50, 10)    # reversed range
    TR_max_range = (50, 10)
    
    # Scale trim_value to target max ranges
    trim_progress = (trim_value - trim_min) / (trim_max - trim_min)  # 0 (closed) to 1 (open)
    trim_progress = max(0, min(1, trim_progress))  # Clamp to [0, 1] range

    # Update servo limits
    servo_limits["TL"] = (90, TL_max_range[0] + (TL_max_range[1] - TL_max_range[0]) * trim_progress)
    servo_limits["BR"] = (90, BR_max_range[0] + (BR_max_range[1] - BR_max_range[0]) * trim_progress)
    servo_limits["BL"] = (90, BL_max_range[0] + (BL_max_range[1] - BL_max_range[0]) * trim_progress)
    servo_limits["TR"] = (90, TR_max_range[0] + (TR_max_range[1] - TR_max_range[0]) * trim_progress)

#     print("Updated servo limits:")
#     print(servo_limits)

def map_value(value, in_min, in_max, out_min, out_max):
    # Map the value
    mapped = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return mapped
        
xt = 90

        
while True:
    continuous_read()
#     targ = int(map_value(x_offset, 0, 225, 40, 140))
#     print(targ)
#     print(".", end="")
    mode_state = not mode.value()
    enable_state = not enable.value()
    if mode_state == 1: # Enter calibration mode when switch is in hold position
        if x_offset < 112 - deadzone:
            xt += 1
            servos["LR"].write(xt)
        if x_offset > 112 + deadzone:
            xt -= 1
            servos["LR"].write(xt)
#         calibrate()
#         time.sleep_ms(500)
        time.sleep_ms(2)
    elif enable_state == 0: # Auto mode
        time.sleep_ms(2)
#         if 112 - deadzone <= x_offset <= 112 + deadzone:
#             targ = map_value(x_offset, 0, 225, 40, 140)
#             if x_offset != old_xt:
#                 print(x_offset)
#                 servos["LR"].write(targ)
#             old_xt = x_offset
    
                
#             if 112 - deadzone <= y_offset <= 112 + deadzone:
#                 print("y offset")
#             time.sleep_ms(2)
            
#             command = random.randint(0,2)
#             if command == 0:
#                 blink()
#                 time.sleep_ms(100)
#             elif command == 1:
#                 blink()
#                 time.sleep_ms(100)
#                 control_ud_and_lids(random.randint(servo_limits["UD"][0],servo_limits["UD"][1]))
#                 servos["LR"].write(random.randint(servo_limits["LR"][0],servo_limits["LR"][1]))
#                 time.sleep_ms(random.randint(300,1000))
#             elif command == 2:
#                 control_ud_and_lids(random.randint(servo_limits["UD"][0],servo_limits["UD"][1]))
#                 servos["LR"].write(random.randint(servo_limits["LR"][0],servo_limits["LR"][1]))
#                 time.sleep_ms(random.randint(200,400))
#         elif enable_state == 1: # Controller mode
#             # Reading sensors
#             UD_value = UD.read_u16()
#             trim_value = trim.read_u16()
#             LR_value = LR.read_u16()
#             blink_state = not blink_pin.value()
#     
#             update_eyelid_limits(trim_value)
            
#             if blink_state == 0:
#                 blink()
#             else:
#                 servos["LR"].write(scale_potentiometer(LR_value, "LR", reverse = True))
#                 control_ud_and_lids(scale_potentiometer(UD_value, "UD"))
#             time.sleep_ms(10)
        
        

        



