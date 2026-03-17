#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import json
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,QTextEdit, QPushButton)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QPoint
from PyQt5.QtGui import QTextCursor

# TIME
def now_time():
    return datetime.now().strftime("%H:%M:%S")

# ROS LOGGER NODE
class VoiceLoggerNode(Node):

    def __init__(self, log_signal, gui_signal):
        super().__init__("voice_logger_node")

        self.log_signal = log_signal
        self.gui_signal = gui_signal

        self.declare_parameter("log_dir", os.path.expanduser("~/ur5_ws/logs"))
        log_dir = self.get_parameter("log_dir").value
        os.makedirs(log_dir, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.json_path = os.path.join(log_dir, f"voice_log_{stamp}.jsonl")
        self.csv_path = os.path.join(log_dir, f"voice_log_{stamp}.csv")

        self.json_file = open(self.json_path, "a", encoding="utf-8")
        self.csv_file = open(self.csv_path, "a", encoding="utf-8", newline="")
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=["ts", "topic", "type", "data"]
        )
        if os.path.getsize(self.csv_path) == 0:
            self.csv_writer.writeheader()

        topics = [
            "control/text_raw",
            "control/dialog_event",
            "gui_control/gui_event",
            "sound/beep_event",
            "text_to_speech/tts_event",
            "voice/wake_detected",
            "voice/heard_text",
            "Neural_parser/nlu_event",
            "mapper/mapper_event",

        ]

        for t in topics:
            self.create_subscription(String, t, self._create_string_cb(t), 10)

        self.create_subscription(Bool,"control/tts_done",self._create_bool_cb("voice/tts_done"),10)
        self.create_subscription(Bool,"gui_control/logger_enable",self._gui_toggle_cb,10)

    def _log(self, topic, msg_type, data):

        event = {
            "ts": now_time(),
            "topic": topic,
            "type": msg_type,
            "data": data
        }

        self.json_file.write(json.dumps(event) + "\n")
        self.json_file.flush()

        self.csv_writer.writerow(event)
        self.csv_file.flush()

        if self.log_signal:
            self.log_signal.emit(topic, str(data))

    def _create_string_cb(self, topic):
        def cb(msg):
            self._log(topic, "String", msg.data)
        return cb

    def _create_bool_cb(self, topic):
        def cb(msg):
            self._log(topic, "Bool", msg.data)
        return cb
    
    def _gui_toggle_cb(self, msg):
        if self.gui_signal:
            self.gui_signal.emit(bool(msg.data))


# ROS THREAD
class RosThread(QThread):
    log_signal = pyqtSignal(str, str)
    gui_signal = pyqtSignal(bool)

    def run(self):
        rclpy.init()
        self.node = VoiceLoggerNode(self.log_signal, self.gui_signal)  
        rclpy.spin(self.node)
        rclpy.shutdown()

# LUXURY DASHBOARD
class LuxuryDashboard(QWidget):

    def __init__(self):
        super().__init__()

        # Frameless Window
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.resize(1100, 700)

        self.container = QWidget(self)
        self.container.setGeometry(0, 0, 1100, 700)

        self.container.setStyleSheet("""
            QWidget {
                background-color: #1C1C1E;
                border-radius: 20px;
                color: #EAEAEA;
                font-family: Consolas, monospace;
                font-size: 14px;
            }
            QTextEdit {
                background-color: #242426;
                border-radius: 16px;
                padding: 16px;
                border: none;
            }
            QPushButton {
                background-color: #2C2C2E;
                border-radius: 12px;
                padding: 6px 18px;
            }
            QPushButton:hover {
                background-color: #3A3A3C;
            }
        """)

        layout = QVBoxLayout(self.container)
        layout.setContentsMargins(25, 25, 25, 25)

        # Title Bar
        title_layout = QHBoxLayout()

        self.title_btn = QPushButton("UR5 Voice Monitor")
        self.title_btn.setEnabled(False)
        self.title_btn.setStyleSheet("background: transparent; font-size: 16px;")

        self.close_btn = QPushButton("✕")
        self.close_btn.setFixedWidth(40)
        self.close_btn.clicked.connect(self.close)

        title_layout.addWidget(self.title_btn)
        title_layout.addStretch()
        title_layout.addWidget(self.close_btn)

        # Log View
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)

        # Bottom Bar
        bottom_layout = QHBoxLayout()
        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(self.log_view.clear)
        bottom_layout.addWidget(clear_btn)
        bottom_layout.addStretch()

        layout.addLayout(title_layout)
        layout.addWidget(self.log_view)
        layout.addLayout(bottom_layout)

        # ROS Thread
        self.ros_thread = RosThread()
        self.ros_thread.log_signal.connect(self.append_log)
        self.ros_thread.gui_signal.connect(self.toggle_gui)
        self.ros_thread.start()

        self.old_pos = None
        self.hide() 

    # Drag window
    def mousePressEvent(self, event):
        self.old_pos = event.globalPos()

    def mouseMoveEvent(self, event):
        delta = QPoint(event.globalPos() - self.old_pos)
        self.move(self.x() + delta.x(), self.y() + delta.y())
        self.old_pos = event.globalPos()

    def append_log(self, topic, msg):
        if topic.startswith("control"):
            color = "#C084FC"   # Purple
        elif topic.startswith("voice"):
            color = "#FACC15"   # Yellow
        elif topic.startswith("sound"):
            color = "#22D3EE"
        elif topic.startswith("text_to_speech"):
            color = "#6EE7B7"
        elif topic.startswith("Neural_parser"):
            color = "#FDBA74"
        elif topic.startswith("mapper"):
            color = "#E0F2FE"
        else:
            color = "#B0B0B0"   # Default gray
        line = f'<span style="color:{color}">[{now_time()}] {topic} → {msg}</span>'
        self.log_view.append(line)
        self.log_view.moveCursor(QTextCursor.End)

    def toggle_gui(self, state: bool):
        if state:
            self.setWindowState(Qt.WindowNoState)
            self.show()
            self.raise_()
            self.activateWindow()
        else:
            self.hide()

# MAIN
def main():
    app = QApplication(sys.argv)
    window = LuxuryDashboard()
    window.hide()  
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

    