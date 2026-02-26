# EYEMECH ε3.2 Assembly Guide  
**NM Robotics**

Fully 3D-printable animatronic eye mechanism compatible with hobby electronics and micro servos.

---

## Overview

The **EYEMECH ε3.2** mechanism can be entirely 3D printed and built using common hobby electronics.

Recommended components and community:

- Website: https://nmrobots.com/
- Discord: https://discord.gg/qXKNRgHKNB  

---

# Parts, Tools & Requirements

## Skill Level

Suitable for experienced beginners in 3D printing and electronics.

- Some dexterity required
- Support removal can be delicate

---

## Required Parts

### Servos (6x Micro Servos)

Choose one:

- **M0090** — Quiet, strong, sturdy - our new favourite and possibly our new standard in the future
- **SG90** — Low cost, weaker/slower  
- **MG90s** — Recommended balance of cost and power  

> Other similar-form servos may work but are not fully tested.

---

### Eyes (2x)

Choose one:

- Realistic Eyes  
- Ultra Realistic Eyes  
- DIY 3D printed eyes  

---

### Controller

- 1x Eye Mechanism Controller Board  
- Optional: 1x Handheld Controller Board  

---

### Budget Alternative

Instead of the official board:

- Raspberry Pi Pico  
- Breadboard  
- Jumper wires  
- Two-way switch  

Note: Reliability may be reduced compared to the dedicated board.

---

## Tools

- Reliable 3D printer  
- PLA filament (quality recommended)  
- Scraper tool (for supports)  
- **Data-capable USB cable** (not power-only)

---

# Selecting a Design

Two variants are available:

| Factor | ε2 (Snap-fit) | ε3.2-PIP (Print-in-place) |
|--------|---------------|---------------------------|
| Ease of Printing | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| Post-processing | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| Ease of Assembly | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| Accuracy | ⭐⭐⭐⭐ | ⭐⭐⭐ |

### Recommendation

If unsure:

- Use **MG90s servos**
- Choose **ε3.2-PIP (Print-in-place)**

---

## Servo Fit Test

Due to servo manufacturing variations:

1. Print the **servo fitting block**
2. Test which hole fits best
3. Select the matching lettered parts
4. Fit should be tight but not damaging

---

# Manufacture

All parts fit on a Bambu X-1 Carbon or P1 series printer.

### Print Settings

- PLA recommended
- 0.2mm layer height
- Organic/tree supports recommended
- Ensure pivot rods/rings are parallel to bed
- For ε2: pivot openings facing down

---

# Post-Processing

- Remove supports carefully
- Avoid stressing delicate pivot features

---

# Assembly (ε3.2-PIP)

Video guide:  
https://youtu.be/MeHLouL9ltw  

---

## Assembly Overview

1. Insert servo into **X-Axis Subassembly A**
2. Clip **Y-Axis Subassembly** on top
3. Attach central pivot holders
4. Attach **X-Axis Subassembly B** (leave loose)
5. Snap assembly into main base
6. Attach eyes
7. Attach eyelids (top = short levers, bottom = long)
8. Snap outer eyelid pivots underneath base
9. Install remaining 5 servos
10. Secure with retainers
11. Insert PCB holders
12. Snap PCB into place

---

# Wiring & Code

Compatible with:

- NM Robotics β1.2 EyeMech Board  
- Raspberry Pi Pico  

---

## Firmware Installation

1. Download MicroPython firmware:  
   https://micropython.org/download/RPI_PICO/

2. Install Thonny IDE:  
   https://thonny.org/

3. Hold **BOOT** while plugging Pico into PC  
4. Copy `.uf2` firmware to device  

Official guide:  
https://www.raspberrypi.com/documentation/microcontrollers/micropython.html

---

## Required Libraries

Install via Thonny:

- picozero  
- micropython-servo  

Steps:

1. Select interpreter  
2. Tools → Manage Packages  
3. Install from local file  

---

## Uploading Code

1. Open `EyeMechEpsilon3.py`
2. Run to confirm no errors
3. Save as `main.py` on Pico

This enables auto-run on boot.

---

## Servo Wiring (Without Official Board)

| Pico Pin | Connect To |
|----------|------------|
| VBUS (5V) | Servo Power (Red) |
| GND | Servo GND (Black/Brown) |
| GPIO10 | Left/Right |
| GPIO11 | Up/Down |
| GPIO12 | Top Left |
| GPIO13 | Bottom Left |
| GPIO14 | Top Right |
| GPIO15 | Bottom Right |
| GPIO7 | Mode Switch → GND |

---

# Calibration

1. Set board to **Hold**
2. Power on
3. Eyes centered and forward
4. Lids closed meeting in middle
5. Secure servo horns

Up/Down servo may need temporary removal for access.

---

# Operation

- **Move Mode** → Random movement  
- **Controller Connected** → Manual mode  
  - Joystick = Eye direction  
  - Button = Blink  
  - Rear slider = Default eyelid openness  

---

# Support

- https://nmrobots.com/
- https://discord.gg/qXKNRgHKNB
