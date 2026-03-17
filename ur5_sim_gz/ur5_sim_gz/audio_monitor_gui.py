#!/usr/bin/env python3
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Float32, Int16MultiArray

import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore

RATE = 16000
CHUNK = 1024
HISTORY = RATE * 2
FPS = 60


class AudioMonitorNode(Node):
    def __init__(self):
        super().__init__('audio_monitor_gui')

        self.create_subscription(
            Int16MultiArray,
            '/voice/audio_monitor',
            self.audio_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/voice/noise_rms',
            self.rms_callback,
            10
        )

        self.ring = np.zeros(HISTORY, dtype=np.int16)
        self.ptr = 0

        self.latest = np.zeros(CHUNK, dtype=np.int16)
        self.rms = 0.0

    # -------------------------------------------------

    def audio_callback(self, msg):
        data = np.array(msg.data, dtype=np.int16)

        if len(data) < 10:
            return

        self.latest = data.copy()
        n = len(data)

        if n >= HISTORY:
            self.ring[:] = data[-HISTORY:]
            self.ptr = 0
            return

        end = self.ptr + n

        if end < HISTORY:
            self.ring[self.ptr:end] = data
        else:
            part = HISTORY - self.ptr
            self.ring[self.ptr:] = data[:part]
            self.ring[:n - part] = data[part:]

        self.ptr = (self.ptr + n) % HISTORY

    def rms_callback(self, msg):
        self.rms = msg.data

class AudioWindow(QtWidgets.QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.setWindowTitle("REALTIME STT Audio Monitor")
        self.resize(1300, 760)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        pg.setConfigOptions(antialias=True)

        # ===== WAVE =====
        self.pw = pg.PlotWidget(title="Realtime Waveform")
        self.pw.setYRange(-20000, 20000)
        self.pw.showGrid(x=True, y=True)
        self.curve_wave = self.pw.plot(pen=pg.mkPen('y', width=1))

        # ===== FFT =====
        self.pf = pg.PlotWidget(title="FFT Spectrum")
        self.pf.setXRange(0, 8000)
        self.pf.setYRange(0, 110)
        self.pf.showGrid(x=True, y=True)
        self.curve_fft = self.pf.plot(pen=pg.mkPen('c', width=1))

        # ===== INFO =====
        self.label = QtWidgets.QLabel("RMS: 0")
        self.label.setStyleSheet("font-size:16px; color: lime")

        self.indicator = QtWidgets.QLabel("● AUDIO")
        self.indicator.setStyleSheet("font-size:18px; color: gray")

        layout.addWidget(self.pw)
        layout.addWidget(self.pf)
        layout.addWidget(self.label)
        layout.addWidget(self.indicator)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(int(1000 / FPS))

    # -------------------------------------------------

    def update_plot(self):
        ptr = self.node.ptr
        ring = self.node.ring

        view = np.roll(ring, -ptr)
        self.curve_wave.setData(view)

        # FFT
        s = view[::4]
        fft = np.abs(np.fft.rfft(s))
        fft[fft == 0] = 1e-12

        fft_db = 20 * np.log10(fft)
        freq = np.fft.rfftfreq(len(s), 1.0 / RATE)

        self.curve_fft.setData(freq, fft_db)

        self.label.setText(f"RMS: {int(self.node.rms)}")

        # indicator
        if self.node.rms > 80:
            self.indicator.setStyleSheet("font-size:18px; color: lime")
        else:
            self.indicator.setStyleSheet("font-size:18px; color: gray")

def main():
    rclpy.init()

    node = AudioMonitorNode()

    app = QtWidgets.QApplication(sys.argv)
    win = AudioWindow(node)
    win.show()

    ros_timer = QtCore.QTimer()
    ros_timer.timeout.connect(
        lambda: rclpy.spin_once(node, timeout_sec=0)
    )
    ros_timer.start(5)

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
