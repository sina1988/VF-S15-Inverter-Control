from PyQt5 import QtWidgets, QtCore
import sys
import serial
import re
import time
import threading
import os
import csv
from datetime import datetime



GEAR_RATIO = 10.07
MOTOR_POLES = 4
MIN_FREQ = 5.0
MAX_FREQ = 50.0
SERIAL_DEVICE = '/dev/ttyUSB0'  # check which one it is in windows try COM1, 2, ...
def hz_to_hex(hz):
    return format(int(hz * 100), '04X')

def rpm_to_hex(rpm):
    return format(int(rpm), '04X')

def parse_response(response, prefix):
    match = re.search(r'\(' + prefix + r'([0-9A-Fa-f]{4})\)', response)
    if match:
        return int(match.group(1), 16)
    return None

class Logger:
    def __init__(self):
        self.folder = "./logs"
        os.makedirs(self.folder, exist_ok=True)
        self.writer = None
        self.logfile = None
        self.logging_enabled = False

    def start_logging(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.folder, f"inverter_log_{timestamp}.csv")
        self.logfile = open(filename, "w", newline="")
        self.writer = csv.writer(self.logfile)
        self.writer.writerow(["Timestamp", "Freq (Hz)", "Current (A)", "Voltage (V)", "Motor RPM", "Shaft RPM"])
        self.logging_enabled = True
        print(f"[LOG] Logging started: {filename}")

    def stop_logging(self):
        if self.logfile:
            self.logfile.close()
            print("[LOG] Logging stopped.")
        self.logging_enabled = False

    def log(self, freq, current, voltage, motor_rpm, shaft_rpm):
        if self.logging_enabled and self.writer:
            # now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # ms precision
            self.writer.writerow([now, freq, current, voltage, motor_rpm, shaft_rpm])

class InverterControl(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        try:
                self.ser = serial.Serial('/dev/ttyUSB0', 19200, bytesize=8,
                                        parity=serial.PARITY_EVEN, stopbits=1, timeout=0.2)
        except serial.SerialException as e:
            QtWidgets.QMessageBox.critical(None, "Inverter Connection Error",
                                        f"Could not open /dev/ttyUSB0.\n\nError: {e}")
            # sys.exit(1)

        # Try communicating with inverter
        try:
            test_response = self.read_command('(RFD00)')
            val = parse_response(test_response, 'RFD00')
            if val is None:
                raise ValueError("No valid response from inverter.")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Inverter Not Responding",
                                        f"Inverter connected, but not responding to commands.\n\nError: {e}")
            # sys.exit(1)
    
        self.freq = MIN_FREQ
        self.logger = Logger()
        self.initUI()
        self.rs485_checkbox.setChecked(False)  # Ensure GUI starts in manual mode
        self.toggle_control_mode(QtCore.Qt.Unchecked)
        self.rated_current = self.get_rated_current()
        self.rated_voltage = self.get_rated_voltage()
        self.monitoring_enabled = True
        self.start_monitoring()

    def initUI(self):
        self.setWindowTitle("Toshiba VF-S15 Control Panel")
        # self.showFullScreen()
        self.resize(1200, 800)  # or whatever size you like
        self.setWindowFlags(QtCore.Qt.Window)  # enables minimize/exit buttons
        


    



        layout = QtWidgets.QGridLayout(self)
        labels = ["Output Frequency (Hz):", "Output Current (A):", "Output Voltage (V):",
                  "Motor RPM:", "Shaft RPM:"]
        self.displays = {}

        for i, label in enumerate(labels):
            layout.addWidget(QtWidgets.QLabel(label), i, 0)
            lcd = QtWidgets.QLCDNumber()
            lcd.setDigitCount(7)
            lcd.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
            lcd.setFixedHeight(60)       # smaller height
            lcd.setFixedWidth(480)       # optional: control width
            lcd.setStyleSheet("color: lime; background-color: black; border: 2px solid lime;")
            layout.addWidget(lcd, i, 1)
            self.displays[label] = lcd

        self.freq_input = QtWidgets.QLineEdit()
        self.rpm_input = QtWidgets.QLineEdit()
        layout.addWidget(QtWidgets.QLabel("Set Frequency (Hz):"), 5, 0)
        layout.addWidget(self.freq_input, 5, 1)
        layout.addWidget(QtWidgets.QPushButton("Set", clicked=self.set_frequency), 5, 2)
        layout.addWidget(QtWidgets.QPushButton("−", clicked=lambda: self.adjust_frequency(-1)), 5, 3)
        layout.addWidget(QtWidgets.QPushButton("+", clicked=lambda: self.adjust_frequency(1)), 5, 4)

        layout.addWidget(QtWidgets.QLabel("Set RPM:"), 6, 0)
        layout.addWidget(self.rpm_input, 6, 1)
        layout.addWidget(QtWidgets.QPushButton("Set", clicked=self.set_rpm), 6, 2)

        layout.addWidget(QtWidgets.QPushButton("Start", clicked=self.start_motor), 7, 0)
        layout.addWidget(QtWidgets.QPushButton("Stop", clicked=self.stop_motor), 7, 1)

        self.rs485_checkbox = QtWidgets.QCheckBox("Use RS485 Control")
        self.rs485_checkbox.setChecked(True)
        self.rs485_checkbox.stateChanged.connect(self.toggle_control_mode)
        layout.addWidget(self.rs485_checkbox, 8, 0, 1, 2)

        self.log_button = QtWidgets.QPushButton("Start Logging")
        self.log_button.setCheckable(True)
        self.log_button.toggled.connect(self.toggle_logging)
        layout.addWidget(self.log_button, 9, 0, 1, 2)
        exit_button = QtWidgets.QPushButton("Exit")
        exit_button.clicked.connect(QtWidgets.qApp.quit)
        layout.addWidget(exit_button, 10, 0)
        
        # Emergency Stop and Reset buttons
        self.em_stop_btn = QtWidgets.QPushButton("EMC OFF")
        self.em_stop_btn.setCheckable(True)
        self.em_stop_btn.setStyleSheet("background-color: grey; color: black; font-weight: bold;")
        self.em_stop_btn.toggled.connect(self.toggle_emergency_stop)
        layout.addWidget(self.em_stop_btn, 7, 2)


        reset_btn = QtWidgets.QPushButton("Reset Inverter")
        reset_btn.setStyleSheet("background-color: orange; color: black; font-weight: bold;")
        reset_btn.clicked.connect(self.reset_inverter)
        layout.addWidget(reset_btn, 7, 3)

        self.reverse_state = False  # default direction: forward
        self.reverse_toggle = QtWidgets.QPushButton("Set Reverse")
        self.reverse_toggle.setCheckable(True)
        self.reverse_toggle.setStyleSheet("background-color: darkgray;")
        self.reverse_toggle.toggled.connect(self.toggle_direction)
        layout.addWidget(self.reverse_toggle, 8, 2)



    def write_command(self, cmd):
        self.ser.write((cmd + '\r').encode('ascii'))
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def read_command(self, cmd):
        self.ser.reset_input_buffer()
        self.ser.write((cmd + '\r').encode('ascii'))
        response = b''
        timeout = time.time() + 1.0
        while time.time() < timeout:
            chunk = self.ser.read(64)
            if chunk:
                response += chunk
                if b')' in response:
                    break
            else:
                time.sleep(0.01)
        return response.decode('ascii', errors='ignore')

    def start_monitoring(self):
        def poll():
            while True:
                if not self.monitoring_enabled:
                    time.sleep(0.1)
                    continue
                try:
                    freq_val = parse_response(self.read_command('(RFD00)'), 'RFD00')
                    curr_val = parse_response(self.read_command('(RFD03)'), 'RFD03')
                    volt_val = parse_response(self.read_command('(RFD05)'), 'RFD05')

                    freq_hz = freq_val * 0.01 if freq_val else 0
                    amps = (curr_val * 0.01 * self.rated_current / 100) if curr_val else 0
                    volts = (volt_val * 0.01 * self.rated_voltage / 100) if volt_val else 0
                    rpm = (freq_hz * 120) / MOTOR_POLES
                    shaft_rpm = rpm / GEAR_RATIO

                    self.displays["Output Frequency (Hz):"].display(freq_hz)
                    self.displays["Output Current (A):"].display(amps)
                    self.displays["Output Voltage (V):"].display(volts)
                    self.displays["Motor RPM:"].display(rpm)
                    self.displays["Shaft RPM:"].display(shaft_rpm)

                    self.logger.log(freq_hz, amps, volts, rpm, shaft_rpm)

                except Exception as e:
                    print(f"[Monitor Error] {e}")
                # time.sleep(1)
                time.sleep(0.1)  # logs every 100 ms


        threading.Thread(target=poll, daemon=True).start()

    def set_frequency(self):
        try:
            hz = float(self.freq_input.text())
            if MIN_FREQ <= hz <= MAX_FREQ:
                self.freq = hz
                hexval = hz_to_hex(hz)
                self.write_command(f'(PFA01{hexval})')
        except ValueError:
            pass

    def set_rpm(self):
        try:
            rpm = float(self.rpm_input.text())
            hz = (rpm * MOTOR_POLES) / 120.0
            if MIN_FREQ <= hz <= MAX_FREQ:
                hexval = rpm_to_hex(rpm)
                self.freq = hz
                self.write_command(f'(PFA13{hexval})')
        except ValueError:
            pass

    def adjust_frequency(self, step):
        new_freq = max(MIN_FREQ, min(MAX_FREQ, self.freq + step))
        self.freq_input.setText(f"{new_freq:.2f}")
        self.set_frequency()

    def start_motor(self):
        # self.write_command('(PFA00C400)')
        if self.rs485_checkbox.isChecked():
            self.write_command('(PFA00C400)')
        else:
            print("[INFO] RS485 control disabled. Start command blocked in manual mode.")

    def stop_motor(self):
        self.write_command('(PFA00C000)')

    def toggle_emergency_stop(self, checked):
        if checked:
            print("[EMC] Emergency Stop ACTIVATED.")
            self.write_command('(PFA009000)')  # Emergency stop
            self.em_stop_btn.setText("EMC ON")
            self.em_stop_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        else:
            print("[EMC] Emergency Stop RELEASED.")
            self.write_command('(PFA00A000)')  # Reset
            time.sleep(0.3)
            self.write_command('(PFA000000)')  # Clear FA00
            time.sleep(0.3)
            if self.rs485_checkbox.isChecked():
                self.write_command('(PFA000400)')  # Restore RS485 control
            #----exit if----    
            self.em_stop_btn.setText("EMC OFF")
            self.em_stop_btn.setStyleSheet("background-color: grey; color: black; font-weight: bold;")



    def reset_inverter(self):
        print("[COMMAND] Reset issued.")
        self.write_command('(PFA00A000)')  # Bit 13 set --> Reset
        time.sleep(0.3)
        self.write_command('(PFA000000)')  # Clear FA00 after reset


    #self.monitoring_enabled = True  # Resume polling
    def toggle_control_mode(self, state):
        self.monitoring_enabled = False

        if state == QtCore.Qt.Checked:
            print("[INFO] Switching to RS485 control (no RUN)...")
            self.write_command('(PFA00C000)')   # Ensure STOP first
            time.sleep(0.2)
            self.write_command('(PFA000400)')   # Bit 15 only (RS485 command), no freq, no run
            print("[INFO] RS485 command enabled (motor still stopped).")
        else:
            print("[INFO] Releasing RS485 control (manual mode)...")
            self.write_command('(PFA00C000)')  # Stop inverter
            time.sleep(0.2)
            self.write_command('(PFA000000)')  # Release FA00 bits
            print("[INFO] Manual/analog input mode is now active.")

        self.monitoring_enabled = True






    def toggle_logging(self, checked):
        if checked:
            self.logger.start_logging()
            self.log_button.setText("Stop Logging")
        else:
            self.logger.stop_logging()
            self.log_button.setText("Start Logging")

    def get_rated_current(self):
        response = self.read_command('(RFE70)')
        val = parse_response(response, 'RFE70')
        return val * 0.01 if val else 9.5

    def get_rated_voltage(self):
        response = self.read_command('(RFE71)')
        val = parse_response(response, 'RFE71')
        return val * 0.01 if val else 400.0

    def toggle_direction(self, checked):
        try:
            freq_val = parse_response(self.read_command('(RFD00)'), 'RFD00')
            freq_hz = freq_val * 0.01 if freq_val else 0
            motor_running = freq_hz > 0.5

            if motor_running:
                print("[Direction] Motor running. Reducing frequency before switching...")
                # Set frequency to MIN_FREQ
                hexval = hz_to_hex(MIN_FREQ)
                self.write_command(f'(PFA01{hexval})')
                time.sleep(1.0)  # give time for slowdown

            # Set direction bit
            if checked:
                print("[Direction] Switching to REVERSE.")
                self.reverse_toggle.setText("Set Forward")
                self.reverse_toggle.setStyleSheet("background-color: orange;")
                self.write_command('(PFA000200)')  # bit 9: REV run enable
            else:
                print("[Direction] Switching to FORWARD.")
                self.reverse_toggle.setText("Set Reverse")
                self.reverse_toggle.setStyleSheet("background-color: darkgray;")
                self.write_command('(PFA000000)')  # clear bit 9 (Forward by default)
            
        except Exception as e:
            print(f"[Direction Toggle Error] {e}")



def main():
    app = QtWidgets.QApplication(sys.argv)
    window = InverterControl()
    # window.show()
    # sys.exit(app.exec_())
    def on_exit():
        if window.rs485_checkbox.isChecked():
            print("[EXIT] Reverting to manual mode before exit.")
            window.write_command('(PFA00C000)')  # Ensure STOP
            time.sleep(0.2)
            window.write_command('(PFA000000)')  # Release RS485 control

        # Release EMC stop if active
        print("[EXIT] Releasing EMC stop if active.")
        window.write_command('(PFA00A000)')  # Reset
        time.sleep(0.3)
        window.write_command('(PFA000000)')  # Clear FA00

    app.aboutToQuit.connect(on_exit)


    window.show()
    sys.exit(app.exec_())





main()
