import serial


class XYGantry:

    def __init__(self, port='COM5', baud_rate=9600):
        self.available = True
        self.complete = 0
        self.job = {"Target": None, "X": None, "Y": None, "TreatTime": None}
        self.frame_gantryx_ratio = 1.56250
        self.frame_gantryy_ratio = 0.93750
        self.arduino = serial.Serial(port, baud_rate, timeout=1)

    def reset(self):
        self.available = True
        self.job = {"Target": None, "X": None, "Y": None, "TreatTime": None}

    def hard_reset(self):
        self.available = True
        self.job = {"Target": None, "X": None, "Y": None, "TreatTime": None}
        msg = '0,0,0'
        self.arduino.write(msg.encode())
        print(f"[INFO] RESET gantry, sending: '{msg}' ..")

    def job_update(self, target, x, y, treat_time):
        self.available = False
        self.job = {"Target": target, "X": x, "Y": y, "TreatTime": treat_time}
        print(f"[INFO] Job received: {self.job}")

    def treat(self):
        msg = f'{int(self.job["X"]*self.frame_gantryx_ratio)},{int(self.job["Y"]*self.frame_gantryy_ratio)},{self.job["TreatTime"]}'
        self.arduino.write(msg.encode())
        print(f"[INFO] Sending move command: '{msg}' ..")
        print(f"[INFO] Treating target {self.job['Target']} ..")

    def job_complete(self):
        response = self.arduino.readline().decode().strip()
        if response == "OK":
            print(f"[INFO] {self.job['Target']} completed.")
            self.reset()
            self.complete += 1
            return True
        else:
            return False
