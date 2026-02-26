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
led_red = Pin(25, Pin.OUT)
enable = Pin(6, Pin.IN, Pin.PULL_UP)
mode = Pin(7, Pin.IN, Pin.PULL_UP)
rx_pin = Pin(1, Pin.IN, Pin.PULL_UP)
blink_pin = Pin(9, Pin.IN, Pin.PULL_UP)
UD = ADC(26)
trim = ADC(27)
LR = ADC(28)
#uart1 = UART(0, baudrate=921600, tx=Pin(0), rx=Pin(1))

INVOKE_INTERVAL_MS = 500 # Adjust this: faster interval means more frames/sec
INVOKE_CMD = b"AT+INVOKE=1,0,1\r"
cbuf = []
readflag = True
staticflag = False
last_boxes = None
detected_flag = False
static_target = 200
static_timer = static_target
idle_flag = False
idle_pause = 300


deadzone = 8
x_offset = 0
y_offset = 0
x_adj_factor = 10
y_adj_factor = 10
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
            time.sleep_ms(5)

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
        max_angle = servo_limits[servo][1]  # Get max angle
        servos[servo].write(max_angle)  # Move to min      
          
def blink():
    servos["TL"].write(servo_limits["TL"][0])
    servos["TR"].write(servo_limits["TR"][0])
    servos["BL"].write(servo_limits["BL"][0])
    servos["BR"].write(servo_limits["BR"][0])
        
def open_lid():        
    servos["TL"].write(tl_target)
    servos["TR"].write(tr_target)
    servos["BL"].write(bl_target)
    servos["BR"].write(br_target)  
        
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

class Comms:
    def __init__(self):
        # Hardware Initialization
        self.grove = UART(0, baudrate=921600, tx=Pin(0), rx=Pin(1))
        #self.esp = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
        
        # Constants
        self.INVOKE_CMD = b"AT+INVOKE=1,0,1\r"
        self.pixel_centre = 112
        self.deadzone = 20
        self.x_adj_factor = 10
        self.y_adj_factor = 10
        self.staticflag = False
        
        # Buffers and State
        self.cbuf = b""
        self.rx_buffer = b""
        self.readflag = True
        self.last_boxes = None
        
        self.grove.write(self.INVOKE_CMD)
        self.grove_vision_module = False

    def map_value(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def grove_read(self):
        # State 1: Requesting data
        if self.readflag:
            # Flush existing buffer quickly
            while self.grove.any():
                self.grove.read()
                self.grove_vision_module = True
            self.grove.write(self.INVOKE_CMD)
            self.cbuf = b""
            self.readflag = False
            return None # Return early, data isn't ready yet

        # State 2: Receiving data
        if self.grove.any():
            # PERFORMANCE: Read chunks directly instead of looping char by char
            self.cbuf += self.grove.read()
            
            # Wait until the end of the JSON packet arrives
            if b'"resolution"' in self.cbuf:
                key = b'"boxes":'
                i = self.cbuf.find(key)
                
                if i != -1:
                    boxes_part = self.cbuf[i + len(key):]
                    
                    # Ensure we have the closing bracket before slicing
                    end_idx = boxes_part.find(b']')
                    if end_idx != -1:
                        boxes_part = boxes_part[:end_idx + 1].strip()
                        
                        # Process if we have valid, new box data
                        if boxes_part != b'[]' and boxes_part != self.last_boxes:
                            self.staticflag = False
                            self.last_boxes = boxes_part
                            boxes_str = boxes_part.decode('utf-8').strip('[]')
                            
                            try:
                                numbers = [int(n) for n in boxes_str.split(',')]
                                x_offset = numbers[0] - self.pixel_centre
                                y_offset = numbers[1] - self.pixel_centre
                                
                                # Reset for next read
                                self.cbuf = b""
                                self.readflag = True
                                return x_offset, y_offset
                                
                            except ValueError:
                                # Handle cases where split data isn't a perfect integer
                                pass
                        else:
                            # Reset for next read (static or empty data)
                            self.cbuf = b""
                            self.readflag = True
                            self.staticflag = True
                            return None
                            
        return None

led_red.value(1)
x_target = 90
y_target = 90
adjustment_factor = 0

# print("Hello")

blink()
time.sleep_ms(50)
neutral()
uart1.write(INVOKE_CMD)
time.sleep_ms(250)
# time.sleep_ms(2500)


# print("Neutral")


blink_time = 200
blink_time_half = blink_time / 2
blinking = False

#Detect if
comms = Comms()
time.sleep_ms(100) #wait for a short time to see if grove_vision_module flag has been set
comms.grove_read()
if comms.grove_vision_module == True:
    work_mode = "tracking"
else:
    work_mode = "auto"
    
led_red.value(0)

while True:
    continuous_read() # Updates x and y offset by looking at camera
#     print(static_timer)
#     if staticflag == False:
#         static_timer = static_target
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
        
#         print(blink_counter)
        if blink_counter == 0:
            blinking = False
    else: # Track Mode
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
#     if staticflag == True:
#         if static_timer == 0: # Idle Mode
#             if idle_flag == False:
#                 idle_flag = True
#                 idle_pause = (random.randint(100,1000))
#                 blink()
#                 time.sleep_ms(80)
#                 servos["TL"].write(tl_target)
#                 servos["TR"].write(tr_target)
#                 servos["BL"].write(bl_target)
#                 servos["BR"].write(br_target)
#                 servos["LR"].write(random.randint(servo_limits["LR"][0]+30, servo_limits["LR"][1]-30))
#                 control_ud_and_lids(random.randint(servo_limits["UD"][0]+50, servo_limits["UD"][1]-30))
#             else:
#                 idle_pause -= 1
#                 time.sleep_ms(2)
#                 if idle_pause <= 0:
#                     idle_flag = False
#         else:
#             static_timer -= 1
    time.sleep_ms(1)

