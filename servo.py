from machine import Pin, PWM

class Servo:
    def __init__(self, pin_id, freq=50, min_us=500, max_us=2500):
        self.pwm = PWM(Pin(pin_id))
        self.pwm.freq(freq)
        self.min_us = min_us
        self.max_us = max_us

    def write(self, angle):
        angle = int(angle)
        if angle < 0:
            angle = 0
        if angle > 180:
            angle = 180

        us = self.min_us + (self.max_us - self.min_us) * angle / 180.0
        duty = int(us * self.pwm.freq() * 65535 / 1000000)
        if duty < 0:
            duty = 0
        if duty > 65535:
            duty = 65535
        self.pwm.duty_u16(duty)

    def deinit(self):
        try:
            self.pwm.deinit()
        except:
            pass
