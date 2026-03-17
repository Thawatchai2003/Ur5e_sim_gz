#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray, String

import numpy as np
import speech_recognition as sr
import io
import wave
import time


RATE = 16000


class GoogleSTTNode(Node):

    def __init__(self):
        super().__init__('google_stt_node')

        # รับ 6ch ดิบจาก ReSpeaker
        self.sub = self.create_subscription(
            UInt8MultiArray,
            '/respeaker/audio_raw',
            self.audio_cb,
            10
        )

        self.pub = self.create_publisher(String, '/voice/text', 10)

        self.recognizer = sr.Recognizer()

        self.buffer = bytearray()
        self.last_voice_time = time.time()

        # ปรับได้ตามห้อง
        self.VAD_THRESHOLD = 300
        self.SILENCE_SEC = 0.8

        self.get_logger().info("✅ Google STT Node Ready (6ch → ch0)")

    # ------------------------------------------------

    def audio_cb(self, msg):

        # ----- 1) แปลง bytes → int16 -----
        data = np.frombuffer(bytes(msg.data), dtype=np.int16)

        # ----- 2) บังคับตัดให้หาร 6 ลงตัว -----
        frames = len(data) // 6
        if frames == 0:
            return

        data = data[:frames*6]

        # ----- 3) reshape 6 channel -----
        audio6 = data.reshape(frames, 6)

        # ----- 4) เอาเฉพาะ ch0 (beamformed) -----
        ch0 = audio6[:,0]

        # ----- 5) VAD ง่าย ๆ -----
        energy = int(np.abs(ch0).mean())

        if energy > self.VAD_THRESHOLD:
            self.buffer.extend(ch0.tobytes())
            self.last_voice_time = time.time()

        # ----- 6) ถ้าเงียบเกิน → ส่ง STT -----
        if time.time() - self.last_voice_time > self.SILENCE_SEC:
            if len(self.buffer) > 2000:
                self.process_stt()

            self.buffer = bytearray()

    # ------------------------------------------------

    def process_stt(self):

        try:
            wav_bytes = self.to_wav(self.buffer)

            with sr.AudioFile(io.BytesIO(wav_bytes)) as source:
                audio = self.recognizer.record(source)

            text = self.recognizer.recognize_google(
                audio,
                language="th-TH"
            )

            self.get_logger().info(f"🗣 {text}")

            msg = String()
            msg.data = text
            self.pub.publish(msg)

        except sr.UnknownValueError:
            pass

        except Exception as e:
            self.get_logger().warn(f"STT error: {e}")

    # ------------------------------------------------

    def to_wav(self, raw: bytes):

        buf = io.BytesIO()

        with wave.open(buf, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(RATE)
            wf.writeframes(raw)

        return buf.getvalue()


def main(args=None):
    rclpy.init(args=args)
    node = GoogleSTTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
