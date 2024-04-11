#!/usr/bin/env python3

import sys
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

class spin_couter():

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

                key,value = data[-1].split(":")
                if key.strip() == "rpm":
                    self.parent.current_rpm_label.setText(value.strip())
                elif key.strip() == "pwm":
                    self.parent.current_duty_cycle.setText(value.strip())
                    print(f"Duty Cycle: {value.strip()}")
                else:
                    print(f"Unknown format: {data[-1]}")

    def __init__(self):
        self.connected = False

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
        self.duty_cycle_input.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.duty_cycle_input.returnPressed.connect(self.__duty_cycle_changed_callback)

        self.current_duty_cycle = QLabel()
        self.current_duty_cycle.setText(f"{min_duty_cycle}")

        self.current_rpm_label = QLabel()
        self.current_rpm_label.setText(f"{current_rpm}")

        # Time managment UI

        # Main grid ui section
        self.flo = QFormLayout()
        self.flo.addRow(f"Network:", network_layout)
        self.flo.addRow(f"Duty cycle ({min_duty_cycle} < {max_duty_cycle}):", self.duty_cycle_input)
        self.flo.addRow("Current PWM duty cycle: ", self.current_duty_cycle)
        self.flo.addRow("Current RPM: ", self.current_rpm_label)

        self.window.setLayout(self.flo)
        self.window.show()

        sys.exit(self.app.exec())


if __name__ == "__main__":
    sc = spin_couter()
    sc.start_gui()
