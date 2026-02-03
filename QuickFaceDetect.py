from machine import Pin, I2C, UART
# import ssd1306
import time, random, ujson, urandom, sys, select, uselect, math

# --- OLED Setup ---
# i2c = I2C(1, scl=Pin(11), sda=Pin(10))
# oled = ssd1306.SSD1306_I2C(128, 32, i2c)

# --- UART Setup ---
# UART(0) is usually TX=GP0, RX=GP1
# UART(1) is usually TX=GP4, RX=GP5
uart1 = UART(0, baudrate=921600, tx=Pin(0), rx=Pin(1))
# uart1 = UART(1, baudrate=921600, tx=Pin(4), rx=Pin(5))

INVOKE_INTERVAL_MS = 500 # Adjust this: faster interval means more frames/sec

INVOKE_CMD = b"AT+INVOKE=1,0,1\r"

# Simple 4-line scroll buffer
MAX_LINES = 4
buf = []
cbuf = []
readflag = True
last_boxes = None
x_offset = 0
y_offset = 0

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
                        print("x: ", x_offset, "y: ", y_offset)
                        last_boxes = boxes_part
                cbuf = b""
                readflag = True
                

while True:
    continuous_read()
#     print(".", end="")
    time.sleep_ms(2)
# def invoke_once(loop_start_ms, timeout_ms=100):
#     global dt, last_time
#     # flush leftover data
#     while uart1.any():
#         uart1.read()
#     # request inference
#     uart1.write(INVOKE_CMD)
#     # start timing and set buffer
#     buf = b""
#     depth = 0
#     # read message
#     while time.ticks_diff(time.ticks_ms(), loop_start_ms) < timeout_ms:
#         now = time.ticks_ms()
#         dt = time.ticks_diff(now, last_time) / 1000.0
#         last_time = now
#         if uart1.any():
#             data = uart1.read()
#             for ch in data:
#                 buf += bytes([ch])
# 
#                 if ch == ord("{"):
#                     depth += 1
#                 elif ch == ord("}"):
#                     depth -= 1
#                     if depth == 0:
#                         # complete JSON object received
#                         print("RAW MESSAGE:")
#                         print(buf)
#                         print("----")
#                         buf = b""  # reset for next message
                        
                    
#         if uart1.any():
#             data = uart1.read()
#             for ch in data:
#                 buf += bytes([ch])
#                 if ch == ord("{"):
#                     depth += 1
#                 elif ch == ord("}"):
#                     depth -= 1
#                     if depth == 0:
#                         raw = buf[buf.find(b"{"):]
#                         try:
#                             js = ujson.loads(raw)
#                             buf = b""  # reset
#                             if js.get("type") == 1:
#                                 # Always return list of boxes (may be empty)
#                                 return js["data"].get("boxes", [])
#                             # if type==0, just ignore and wait
#                         except Exception as e:
#                             # corrupted? reset buffer
# #                             buf = b""
# #         else:
#             time.sleep_ms(2)
#     return None  # timeout
# 
# last_time = time.ticks_ms()
# last_switch = last_time
    
# oled_print("UART active")

# print("USB serial monitor active. Type messages below...")

#     now = time.ticks_ms()
#     dt = time.ticks_diff(now, last_time) / 1000.0
#     last_time = now
#     print(invoke_once(now))
#     time.sleep_ms(1000)

#     if uart.any():               # check if any data received
#         line = uart.readline()
#         if line:
#             line = line.decode('utf-8').strip()  # decode bytes
# #             ts = time.ticks_ms()
# #             msg = f"{ts//1000}.{ts%1000:03d}: {line}"
# #             oled_print(line)
#             print("RX:", line)   # optional: also print to USB serial

# def oled_print(line):
#     buf.append(line)
#     if len(buf) > MAX_LINES:
#         buf.pop(0)
#     oled.fill(0)
#     for i, t in enumerate(buf):
#         oled.text(t, 0, i*8)
#     oled.show()
    



