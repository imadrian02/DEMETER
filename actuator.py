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


class XGantry:

    def __init__(self):
        self.laser = Laser()
        self.status = 0
        self.complete = 0
        self.pos = 0
        self.job = {"Target": None, "TargetX": None, "TargetY": None, "Treat": False, "LastSeen": 0}

    def updateJob(self, target, targetx, targety):
        self.status = 1
        self.job = {"Target": target, "TargetX": targetx, "TargetY": targety, "Treat": False, "LastSeen": 0}
        print(f'[Update] Job received:')
        print(self.job)

    def moveTo(self, targetx):

        self.pos = targetx
        print(f'[Action] Move to x: {self.pos}')

    def reset(self):
        self.status = 0
        self.job = {"Target": None, "TargetX": None, "TargetY": None, "Treat": False, "LastSeen": 0}
        print("[Update] Gantry reset")

    def treat(self, cycles=0, delay=0):
        duration = 1

        time.sleep(delay)

        Laser.laser_on(self.laser)

        for cycle in range(cycles):
            print(f"[Action] Laser blasting for {int(cycle+1)*duration} seconds")
            time.sleep(duration)

        Laser.laser_off(self.laser)
        print(f"[Update] Treatment Done! Gantry will now reset")
        self.reset()
        self.complete += 1