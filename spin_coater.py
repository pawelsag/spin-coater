#!/usr/bin/env python3

import sys
import time
import socket

from PyQt6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QPushButton,
    QWidget,
    QLineEdit,
    QFormLayout,
    QLabel
)
from PyQt6.QtGui import QIntValidator,QFont
from PyQt6.QtCore import Qt, QThread
min_duty_cycle = 49
max_duty_cycle = 98
current_rpm = 0
max_rpm_value = 7000


class spin_coater():

    class CountdownThread(QThread):

        def run(self):
            seconds_left = self.parent.spin_time_int
            while seconds_left > 0 and self.parent.spin_started:
                time.sleep(1)
                seconds_left -= 1
                self.parent.spin_time_left_label.setText(f"{seconds_left} sec")

    class WorkerThread(QThread):

        def run(self):
            while self.parent.connected:
                data = self.parent.connection_sock.recv(64)
                if len(data) == 0:
                    self.parent.__net_connection_change_callback(shutdown=True)
                    return

                data = list(filter(None,data.decode("utf-8").split('\n')))
                if not data:
                    print("Empty packet received")
                    continue

                for entry in data:
                    if ":" in entry:
                        key,value = entry.split(":")
                    else:
                        key = entry

                    if key.strip() == "rpm":
                        self.parent.current_rpm_label.setText(value.strip())
                        print(data)
                    elif key.strip() == "pwm":
                        self.parent.current_duty_cycle.setText(value.strip())
                        print(f"Duty Cycle: {value.strip()}")
                    elif key.strip() == "spin_started":
                        self.parent.spin_apply_button.setText("Stop spinning")
                        self.parent.spin_started = True
                        self.parent.countdownThread = spin_coater.CountdownThread()
                        self.parent.countdownThread.parent = self.parent
                        self.parent.countdownThread.start()
                        print(f"Duty Cycle: {value.strip()}")
                    elif key.strip() == "spin_stopped":
                        self.parent.spin_apply_button.setText("Start spinning")
                        self.parent.spin_started = False
                        self.parent.spin_time_left_label.setText(f"0 sec")
                    else:
                        print(f"Unknown format: {entry}, {key}")

    def __init__(self):
        self.connected = False
        self.spin_started = False

    def __duty_cycle_changed_callback(self):
        duty_cycle_int = int(self.duty_cycle_input.text())
        self.duty_cycle_input.setText("")
        if duty_cycle_int < min_duty_cycle:
            duty_cycle_int = min_duty_cycle
            print(f"Setting duty cycle to min={duty_cycle_int}")
        elif duty_cycle_int > max_duty_cycle:
            print(f"Duty cycle should be in scope ({min_duty_cycle} <= x <= {max_duty_cycle})")
            return

        if self.connected:
            self.connection_sock.send(f"pwm: {duty_cycle_int}\n".encode('utf-8'));

    def __net_connection_change_callback(self, shutdown=False):
        if self.connected or shutdown:
            self.connected = False
            self.net_connect.setText("Connect")
            self.connection_sock.close()
            return

        ip = self.net_ip_input.text()
        port = self.net_port_input.text()
        self.connection_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.connection_sock.connect((ip, int(port)))
        except:
            print(f"Couldn't connect to IPv4: {ip}, port: {port}")
            return

        self.connected = True
        self.net_connect.setText("Disconnect")

        self.workerThread = self.WorkerThread()
        self.workerThread.parent = self
        self.workerThread.finished.connect(self.workerThread.deleteLater)
        self.workerThread.start()

    def __change_spin_state(self):
        if self.spin_started:
            self.connection_sock.send(f"spin_stop\n".encode('utf-8'));
            return

        self.spin_time_int = int(self.spin_time_input.text())
        self.spin_rpm_int = int(self.spin_rpm_input.text())
        self.spin_time_input.setText("")
        self.spin_rpm_input.setText("")
        if self.spin_rpm_int > max_rpm_value :
            print(f"Max RPM value is {max_rpm_value}.")
            return

        if self.connected:
            self.connection_sock.send(f"spin_start: {self.spin_time_int} {self.spin_rpm_int}\n".encode('utf-8'));

    def start_gui(self):
        self.app = QApplication([])
        self.window = QWidget()
        self.window.setWindowTitle("Spin Couter App")
        
        # Network management ui section
        self.net_ip_input = QLineEdit()
        self.net_ip_input.setPlaceholderText("Server IPv4")
        self.net_port_input = QLineEdit()
        self.net_port_input.setPlaceholderText("Server port")
        self.net_port_input.setValidator(QIntValidator())
        self.net_connect = QPushButton("Connect")
        self.net_connect.clicked.connect(self.__net_connection_change_callback)
        network_layout = QHBoxLayout()
        network_layout.addWidget(self.net_ip_input)
        network_layout.addWidget(self.net_port_input)
        network_layout.addWidget(self.net_connect)

        # PWM management ui section
        self.duty_cycle_input = QLineEdit()
        self.duty_cycle_input.setValidator(QIntValidator())
        self.duty_cycle_input.setMaxLength(3)
        self.duty_cycle_input.setPlaceholderText("PWM duty - debug mode")
        self.duty_cycle_input.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.duty_cycle_input.returnPressed.connect(self.__duty_cycle_changed_callback)

        self.current_duty_cycle = QLabel()
        self.current_duty_cycle.setText(f"{min_duty_cycle}")

        self.current_rpm_label = QLabel()
        self.current_rpm_label.setText(f"{current_rpm}")

        self.spin_time_left_label = QLabel()
        self.spin_time_left_label.setText("0 sec")

        # Time managment UI
        self.spin_rpm_input = QLineEdit()
        self.spin_rpm_input.setPlaceholderText(f"Spin RPM (max: {max_rpm_value})")
        self.spin_rpm_input.setValidator(QIntValidator())
        self.spin_time_input = QLineEdit()
        self.spin_time_input.setPlaceholderText("Spin time (seconds)")
        self.spin_time_input.setValidator(QIntValidator())
        self.spin_apply_button = QPushButton("Start spinning")
        self.spin_apply_button.clicked.connect(self.__change_spin_state)
        spin_layout = QHBoxLayout()
        spin_layout.addWidget(self.spin_rpm_input)
        spin_layout.addWidget(self.spin_time_input)
        spin_layout.addWidget(self.spin_apply_button)


        # Main grid ui section
        self.flo = QFormLayout()
        self.flo.addRow(f"Network:", network_layout)
        self.flo.addRow(f"Main Spin settings:", spin_layout)
        self.flo.addRow(f"Duty cycle ({min_duty_cycle} < {max_duty_cycle}):", self.duty_cycle_input)
        self.flo.addRow("Current PWM duty cycle: ", self.current_duty_cycle)
        self.flo.addRow("Current RPM: ", self.current_rpm_label)
        self.flo.addRow("Spin time left: ", self.spin_time_left_label)

        self.window.setLayout(self.flo)
        self.window.show()

        sys.exit(self.app.exec())


if __name__ == "__main__":
    sc = spin_coater()
    sc.start_gui()
