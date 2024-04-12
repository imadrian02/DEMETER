import serial


class XYGantry:

    def __init__(self, port='/dev/ttyUSB0', baud_rate=115200):
        self.available = True
        self.complete = 0
        self.job = {"Target": None, "X": None, "Y": None, "TreatTime": None}
        self.frame_gantryx_ratio = 1.32353
        self.frame_gantryy_ratio = 1.41667
        self.arduino = serial.Serial(port, baud_rate)

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
        x = int((680-self.job["X"]+25)*self.frame_gantryx_ratio)
        y = int((480-self.job["Y"]-60)*self.frame_gantryy_ratio)
        msg = f'{x},{y},{self.job["TreatTime"]}'
        if x < 900 and y < 400:
            self.send_command(msg)
            print(f"[INFO] Treating target {self.job['Target']} ..")
        else:
            self.job_complete()

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


