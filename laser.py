import serial
import time


class Laser:
    def __init__(self):
        self.status = 0

    def laser_on(self):
        self.status = 1
        print("[ACTION] Laser on")

    def laser_off(self):
        self.status = 0
        print("[ACTION] Laser off")
