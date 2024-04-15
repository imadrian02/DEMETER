import serial


class XYGantry:

    def __init__(self, port='COM13', baud_rate=9600):
        self.available = True
        self.complete = 0
        self.job = {"Target": None, "X": None, "Y": None, "TreatTime": None}
        self.frame_gantryx_ratio = 1.32353
        self.frame_gantryy_ratio = 1.41667
        self.arduino = serial.Serial(port, baud_rate, timeout=1)

    def reset(self):
        self.available = True
        self.job = {"Target": None, "X": None, "Y": None, "TreatTime": None}

    def hard_reset(self):
        self.reset()
        msg = '0,0,0'
        self.send_command(msg)

    def job_update(self, target, x, y, treat_time):
        self.available = False
        self.job = {"Target": target, "X": x, "Y": y, "TreatTime": treat_time}
        print(f"[INFO] Job received: {self.job}")

    def treat(self):
        msg = f'{int((680-self.job["X"]+25)*self.frame_gantryx_ratio)},{int((480-self.job["Y"]-60)*self.frame_gantryy_ratio)},{self.job["TreatTime"]}'
        self.send_command(msg)
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

    def send_command(self,msg):
        self.arduino.write(msg.encode())
        print(f"[INFO] Sending move command: '{msg}' ..")
        while not self.job_complete():
            pass


