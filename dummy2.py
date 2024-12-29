import sys
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtGui import QPixmap, QImage
from idp8 import Ui_MainWindow
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import Qt

# import cv2

ser = serial.Serial(r'/dev/ttyACM0', 115200)

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # # Create a QTimer to periodically update the displayed frame
        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.update_picture)
        # self.timer.start(30)  # Update frame every 30 milliseconds

        self.ui.start_btn.clicked.connect(lambda: self.send_command('K'))
        self.ui.stop_btn.clicked.connect(lambda: self.send_command('S'))
        self.ui.Estop_btn.clicked.connect(lambda: self.send_command('E'))
        self.ui.manual_btn.clicked.connect(lambda: self.send_command('M'))
        self.ui.auto_btn.clicked.connect(lambda: self.send_command('A'))

        self.ui.forward_btn.clicked.connect(lambda: self.send_command('F'))
        self.ui.backward_btn.clicked.connect(lambda: self.send_command('B'))
        self.ui.left_btn.clicked.connect(lambda: self.send_command('L'))
        self.ui.right_btn.clicked.connect(lambda: self.send_command('R'))

        self.setup_checkable_buttons()

    def setup_checkable_buttons(self):
        # Set buttons as checkable
        self.ui.auto_btn.setCheckable(True)
        self.ui.manual_btn.setCheckable(True)
        self.ui.start_btn.setCheckable(True)
        self.ui.stop_btn.setCheckable(True)
        self.ui.Estop_btn.setCheckable(True)

        # Connect state change signals
        self.ui.auto_btn.toggled.connect(self.on_auto_toggled)
        self.ui.manual_btn.toggled.connect(self.on_manual_toggled)
        self.ui.start_btn.toggled.connect(self.start_toggled)
        self.ui.stop_btn.toggled.connect(self.stop_toggled)
        self.ui.Estop_btn.toggled.connect(self.estop_toggled)

    def estop_toggled(self, checked):
        if checked:
            self.ui.Estop_btn.setStyleSheet("background-color: #CCC;")
            self.ui.stop_btn.setChecked(False)
            self.ui.start_btn.setChecked(False)
            self.ui.manual_btn.setChecked(False)
            self.ui.auto_btn.setChecked(False)
            self.ui.stop_btn.setStyleSheet("")
            self.ui.start_btn.setStyleSheet("")
            self.ui.manual_btn.setStyleSheet("background-color: #FFFF99;")
            self.ui.auto_btn.setStyleSheet("background-color: #FFFF99;")

    def start_toggled(self, checked):
        if checked:
            self.ui.start_btn.setStyleSheet("background-color: #CCC;")
            self.ui.stop_btn.setChecked(False)
            self.ui.stop_btn.setStyleSheet("")
            self.ui.Estop_btn.setChecked(False)
            self.ui.Estop_btn.setStyleSheet("background-color: #FF3333;")
        else:
            self.ui.start_btn.setStyleSheet("")

    def stop_toggled(self, checked):
        if checked:
            self.ui.stop_btn.setStyleSheet("background-color: #CCC;")
            self.ui.start_btn.setChecked(False)
            self.ui.start_btn.setStyleSheet("")
            self.ui.Estop_btn.setChecked(False)
            self.ui.Estop_btn.setStyleSheet("background-color: #FF3333;")
        else:
            self.ui.stop_btn.setStyleSheet("")

    def on_auto_toggled(self, checked):
        if checked:
            self.ui.auto_btn.setStyleSheet("background-color: #CCC;")
            self.ui.manual_btn.setChecked(False)
            self.ui.manual_btn.setStyleSheet("background-color: #FFFF99;")
            self.ui.Estop_btn.setChecked(False)
            self.ui.Estop_btn.setStyleSheet("background-color: #FF3333;")
        else:
            self.ui.auto_btn.setStyleSheet("background-color: #FFFF99;")

    def on_manual_toggled(self, checked):
        if checked:
            self.ui.manual_btn.setStyleSheet("background-color: #CCC;")
            self.ui.auto_btn.setChecked(False)
            self.ui.auto_btn.setStyleSheet("background-color: #FFFF99;")
            self.ui.Estop_btn.setChecked(False)
            self.ui.Estop_btn.setStyleSheet("background-color: #FF3333;")
        else:
            self.ui.manual_btn.setStyleSheet("background-color: #FFFF99;")

    # def display_image(self):
    #     # ̣ # Create a label that can contain an image
    #     # self.image_label = QLabel(self.ui.picture_frame)
    #     # self.image_label.setGeometry(0, 0, 471, 241)  # Adjust dimensions if necessary
    #     # self.image_label.setScaledContents(True)

    #     # # Load an image
    #     # pixmap = QPixmap(image_path)
    #     # if pixmap.isNull():
    #     #     print(f"Failed to load image: {image_path}")
    #     # else:
    #     #     self.image_label.setPixmap(pixmap)
    #     global weed_frame
    #     if weed_frame is not None:
    #     # Convert the OpenCV frame to QImage
    #         height, width, channels = weed_frame.shape
    #         bytes_per_line = channels * width
    #         q_img = QImage(weed_frame.data, width, height, bytes_per_line, QImage.Format_RGB888)

    #         # Convert QImage to QPixmap and display
    #         pixmap = QPixmap.fromImage(q_img)
    #         self.image_label.setPixmap(pixmap)
    #     else:
    #         print("No frame available.")

    # def update_frame(self):
    #     self.display_image()

    def send_command(self, command):
        global pause
        global auto
        data_to_send = f'{command}'
        print(f"Sending command: {data_to_send}")  # Debug message
        try:
            ser.write(data_to_send.encode())
            print(f"Command {data_to_send} sent successfully")  # Debug message
            if data_to_send == 'A':
                auto = True
                pause = False
                print("[INFO] Auto mode activated")
            elif data_to_send == 'M':
                auto = False
                pause = False
                print("[INFO] Manual mode activated")
        except serial.SerialException as e:
            print(f"Error sending command: {e}")

    def keyPressEvent(self, event):
        # Check if the first page is currently visible
        button = None
        if event.key() == Qt.Key_W:
            button = self.ui.forward_btn
            self.ui.forward_btn.click()
        elif event.key() == Qt.Key_A:
            button = self.ui.left_btn
            self.ui.left_btn.click()
        elif event.key() == Qt.Key_S:
            button = self.ui.backward_btn
            self.ui.backward_btn.click()
        elif event.key() == Qt.Key_D:
            button = self.ui.right_btn
            self.ui.right_btn.click()
        elif event.key() == Qt.Key_Z:
            button = self.ui.start_btn
            self.ui.start_btn.click()
        elif event.key() == Qt.Key_X:
            button = self.ui.stop_btn
            self.ui.stop_btn.click()
        elif event.key() == Qt.Key_C:
            button = self.ui.Estop_btn
            self.ui.Estop_btn.click()

        if button:
            # Apply hover style
            original_style = button.styleSheet()
            hover_style = original_style + "background-color: black; color: white;"
            button.setStyleSheet(hover_style)

            # Simulate button click if necessary
            # button.click()  # Uncomment this line if you want to simulate the click as well

            # Timer to reset the style after 150 milliseconds
            QTimer.singleShot(150, lambda: button.setStyleSheet(original_style))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())