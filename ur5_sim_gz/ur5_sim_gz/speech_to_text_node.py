#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String, Bool, Int16MultiArray
import speech_recognition as sr
import threading
import time
import queue
import numpy as np

RATE = 16000

class STTState(Enum):
    IDLE = 0
    LISTEN = 1
    DETECT = 2
    COLLECT = 3
    RECOGNIZE = 4
    WAIT_TTS_CLEAR = 5

class VADMode(Enum):
    FIXED = 0
    ADAPTIVE = 1
    OFF = 2
    NEURAL = 3
    HYBRID = 4

class AutoCalibrationMode(Enum):
    OFF = 0
    MANUAL = 1
    AUTO = 2

class DynamicMode(Enum):
    OFF = 0
    ON = 1

class EnvironmentMode(Enum):
    OFF = 0
    ON = 1

class AutoCalibration:
    def __init__(self, node):
        self.node = node
        self.mode = AutoCalibrationMode.OFF
        self.duration = 2.0

    def set_mode(self, mode: AutoCalibrationMode):
        self.mode = mode
        self.node.get_logger().info(f"AUTO CALIB MODE → {mode.name}")

    def run(self):
        if self.mode == AutoCalibrationMode.OFF:
            self.node.get_logger().warning("Auto Calibration OFF")
            return
        self.node.get_logger().info("=== AUTO NOISE CALIBRATION START ===")
        self.node.get_logger().info("Stay silent...")
        start = time.time()
        samples = []

        while time.time() - start < self.duration:
            try:
                data = self.node.audio_queue.get(timeout=0.1)
                arr = np.frombuffer(data, dtype=np.int16)
                if len(arr) > 0:
                    rms = np.sqrt(np.mean(arr.astype(np.float32)**2))
                    samples.append(rms)
            except queue.Empty:
                pass

        if len(samples) < 5:
            self.node.get_logger().warning("Calibration failed")
            return
        median_noise = float(np.median(samples))
        median_noise = max(50.0, min(median_noise, 2000.0))
        self.node.noise_floor = median_noise
        self.node.get_logger().info(f"CALIB DONE | Noise floor = {self.node.noise_floor:.1f}")

class DynamicThreshold:

    def __init__(self, node):
        self.node = node
        self.mode = DynamicMode.OFF

    def set_mode(self, mode: DynamicMode):
        self.mode = mode
        self.node.get_logger().info(f"DYNAMIC MODE → {mode.name}")

    def get_threshold(self):
        if self.mode == DynamicMode.OFF:
            return 250.0

        # ON mode
        base = self.node.noise_floor
        dynamic_th = base * 1.2
        return dynamic_th

class EnvironmentClassifier:

    def __init__(self, node):
        self.node = node
        self.mode = EnvironmentMode.OFF
        self.current_env = "NORMAL"

    def set_mode(self, mode: EnvironmentMode):
        self.mode = mode
        self.node.get_logger().info(f"ENVIRONMENT MODE → {mode.name}")

    def update(self):

        if self.mode == EnvironmentMode.OFF:
            return self.current_env

        if len(self.node.rms_history) < 15:
            return self.current_env
        mean_rms = np.mean(self.node.rms_history)
        var_rms = np.var(self.node.rms_history)

        if mean_rms < 200 and var_rms < 2000:
            self.current_env = "QUIET"

        elif mean_rms > 600 or var_rms > 20000:
            self.current_env = "NOISY"

        else:
            self.current_env = "NORMAL"

        return self.current_env

class GoogleSTTNode(Node):
    def __init__(self):
        super().__init__('google_stt_node')
        # CORE AUDIO PARAMETERS
        self.start_rms = 900
        self.continue_rms = 700
        self.min_samples = 2000
        self.silence_sec = 0.7
        self.max_collect_sec = 2.2
        self.listen_timeout = 6.0
        self.min_audio_sec = 0.05
        self.min_rms = 800
        self.rms_ratio = 0.25

        # TIMING / LIMIT CONTROL
        self.google_timeout = 3.0
        self.duplicate_sec = 0.8
        self.err_interval = 5.0
        self.calib_interval = 20.0

        # RUNTIME STATE VARIABLES
        self.start_time = 0.0
        self.last_voice_time = 0.0
        self.listen_window_start = 0.0
        self.last_calib_time = 0.0
        self._need_flush = False
        self._in_recognize = False
        self._tts_timer = None
        self._last_collect_speech = None

        # SYSTEM FLAGS / MODE STATE
        self.armed = False
        self.woke_up = False
        self.paused = False
        self.one_shot = False
        self.use_wake_mode = True
        self.debug = True
        self.debug_env = False
        self.debug_stt = False
        self.state = STTState.IDLE
        self.vad_mode = VADMode.HYBRID

        # NOISE MODEL
        self.noise_floor = 300.0
        self.noise_alpha = 0.98
        self.min_noise_floor = 50.0
        self.max_noise_floor = 5000.0

        # DUPLICATE FILTER
        self.last_dup = ""
        self.last_dup_ts = 0.0
        self.last_err_ts = 0.0

        # ROS INTERFACE (SUB / PUB)
        self.sub_audio = self.create_subscription(Int16MultiArray, '/audio/mono', self.audio_callback, 10)
        self.sub_armed = self.create_subscription(Bool, 'control/armed', self.armed_callback, 10)
        self.sub_control = self.create_subscription(String, 'control/stt_control', self.control_callback, 10)
        self.sub_dialog_event = self.create_subscription(String, 'control/dialog_event', self.dialog_event_callback, 10)
        self.pub = self.create_publisher(String, 'voice/heard_text', 10)
        self.pub_debug = self.create_publisher(String, 'voice/stt_debug', 10)
        self.pub_wake = self.create_publisher(Bool, 'voice/wake_detected', 10)

        #  AUDIO ENGINE
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 300
        self.recognizer.dynamic_energy_threshold = False
        self.audio_queue = queue.Queue()

        #  FEATURE MODULES
        self.dynamic = DynamicThreshold(self)
        self.dynamic.set_mode(DynamicMode.ON)
        self.auto_calib = AutoCalibration(self)
        self.auto_calib.set_mode(AutoCalibrationMode.AUTO)
        self.env_classifier = EnvironmentClassifier(self)
        self.env_classifier.set_mode(EnvironmentMode.ON)
        self.rms_history = []

        #  NEURAL VAD (OPTIONAL)
        self.vad_model = None
        self.get_speech_timestamps = None

        try:
            import torch
            self.torch = torch

            from silero_vad import load_silero_vad, get_speech_timestamps
            self.vad_model = load_silero_vad()
            self.get_speech_timestamps = get_speech_timestamps

            self.get_logger().info("Neural VAD loaded")

        except Exception as e:
            self.get_logger().warning(f"Neural VAD not available: {e}")

        #  BACKGROUND WORKER
        self.worker = threading.Thread(target=self.process_loop)
        self.worker.daemon = True
        self.worker.start()

        #  INITIAL CALIBRATION
        time.sleep(0.5)
        self.auto_calib.run()
        self.last_calib_time = time.time()

        # STARTUP INFO
        self.get_logger().info("Google STT Node Ready")
        self._print_startup_banner()
    def _print_startup_banner(self):
        GREEN = "\033[92m"
        CYAN = "\033[96m"
        RESET = "\033[0m"

        supported = [
            "Multi VAD Modes (FIXED / ADAPTIVE / NEURAL / HYBRID / OFF)",
            "Hybrid VAD (RMS pre-filter + Neural confirm)",
            "Dynamic Threshold (Noise-floor based)",
            "Environment-aware multiplier (QUIET / NORMAL / NOISY)",
            "Auto Noise Calibration (OFF / MANUAL / AUTO)",
            "Wake-word detection (สวัสดี / หวัดดี)",
            "Duplicate speech filter",
            "Google timeout protection",
            "Quick RMS reject filter",
            "Auto re-calibration loop (interval based)",
            "Dialog listen timeout",
            "One-shot mode (optional)",
        ]

        banner = (
            "\n"
            "══════════════════════════════════════════════════════════════\n"
            "            Google STT Node — Operational\n"
            "            State Machine  : READY\n"
            f"            Debug Mode     : {'ENABLED' if self.debug else 'DISABLED'}\n"
            "\n"
            "  Audio Config:\n"
            f"      • Sample Rate        : {RATE} Hz\n"
            f"      • Start RMS          : {self.start_rms}\n"
            f"      • Continue RMS       : {self.continue_rms}\n"
            f"      • Noise Floor        : {self.noise_floor:.1f}\n"
            "\n"
            "  Active Modes:\n"
            f"      • VAD Mode           : {self.vad_mode.name}\n"
            f"      • Dynamic Threshold  : {self.dynamic.mode.name}\n"
            f"      • Environment Mode   : {self.env_classifier.mode.name}\n"
            f"      • Auto Calibration   : {self.auto_calib.mode.name}\n"
            f"      • Wake Mode Enabled  : {self.use_wake_mode}\n"
            "\n"
            "  Subscribed Topics:\n"
            "      • /audio/mono\n"
            "      • voice/armed\n"
            "      • voice/stt_control\n"
            "      • voice/dialog_event\n"
            "\n"
            "  Published Topics:\n"
            "      • voice/heard_text\n"
            "      • voice/stt_debug\n"
            "      • voice/wake_detected\n"
            "\n"
            "  Supported Functions:\n"
            "      - " + "\n      - ".join(supported) + "\n"
            "══════════════════════════════════════════════════════════════\n"
        )

        self.get_logger().info(GREEN + banner + RESET)

    def debug_pub(self, text: str):
        msg = String()
        msg.data = text
        self.pub_debug.publish(msg)

    def publish_wake(self, value: bool):
        msg = Bool()
        msg.data = value
        self.pub_wake.publish(msg)

    # Debug Logger
    def logd(self, text: str):
        if self.debug:
            self.get_logger().info(text)

    def armed_callback(self, msg: Bool):
        if msg.data:
            if self._in_recognize:
                self.get_logger().info("Ignore ARM because recognizing")
                return
            self.debug_pub("STATE -> LISTEN (ARMED)")
            self.use_wake_mode = False
            self.woke_up = True
            self.armed = True
            self.state = STTState.LISTEN
            self.listen_window_start = time.time()
            self._need_flush = True
            self.start_time = time.time()
            self.last_voice_time = time.time()
            with self.audio_queue.mutex:
                self.audio_queue.queue.clear()
            self.get_logger().info("STT ARMED = True")

        else:
            self.use_wake_mode = True
            self.woke_up = False
            self.armed = False
            self.state = STTState.IDLE
            self.debug_pub("STATE -> IDLE")
            self._need_flush = True
            self.get_logger().info("STT DISARM = False")

    def audio_callback(self, msg):
        try:
            mono = np.array(msg.data, dtype=np.int16)
            self.audio_queue.put(mono.tobytes())

        except Exception as e:
            self.get_logger().error(f"convert error: {e}")
    
    def quick_reject(self, pcm: bytes) -> bool:
        # chack list
        duration = (len(pcm) / 2) / RATE

        if self.use_wake_mode and not self.woke_up:
            return False
    
        if duration < self.min_audio_sec:
            self.logd(f"Reject: too short {duration:.2f}s")
            return True

        # RMS 
        arr = np.frombuffer(pcm, dtype=np.int16)
        rms = float(np.sqrt(np.mean(arr.astype(np.float32)**2)))

        # threshold to hybrid 
        et = float(self.recognizer.energy_threshold or 300)
        pass_th = max(self.min_rms, et * self.rms_ratio)
        self.logd(f"QR: rms={rms:.0f} pass_th={pass_th:.0f}")

        if rms < pass_th:
            self.logd("Reject: RMS too low")
            return True
        return False

    def process_loop(self):
        buffer = b""
        while rclpy.ok():
            
            if self.paused and self.state != STTState.WAIT_TTS_CLEAR:
                time.sleep(0.2)
                continue

            # STATE: IDLE
            if self.state == STTState.IDLE:
                if self.use_wake_mode:
                    self.state = STTState.LISTEN
                    self.debug_pub("STATE -> LISTEN")
                    self._need_flush = True

                elif self.armed:
                    self.state = STTState.LISTEN
                    self.debug_pub("STATE -> LISTEN")
                    self._need_flush = True

                time.sleep(0.05)
                continue
            # รับเสียง
            try:
                data = self.audio_queue.get(timeout=0.1)
                buffer += data

                MAX_BUF = RATE * 2 * 8
                if len(buffer) > MAX_BUF:
                    buffer = buffer[-MAX_BUF:]
            except queue.Empty:
                pass

            # STATE: WAIT_TTS_CLEAR
            if self.state == STTState.WAIT_TTS_CLEAR:
                if time.time() - self.tts_clear_start > 0.7:
                    with self.audio_queue.mutex:
                        self.audio_queue.queue.clear()
                    self.paused = False
                    self._need_flush = True
                    self.use_wake_mode = False
                    self.woke_up = True
                    self.armed = True
                    self.listen_window_start = time.time()
                    self.state = STTState.LISTEN
                    self.debug_pub("STATE -> LISTEN (AFTER TTS)")
                continue
            
            # AUTO RECALIBRATION LOOP
            if self.auto_calib.mode == AutoCalibrationMode.AUTO:
                now = time.time()

                safe_to_calib = (
                    not self.armed and
                    not self._in_recognize and
                    self.state in [STTState.IDLE, STTState.LISTEN]
                )

                if (now - self.last_calib_time > self.calib_interval and
                    safe_to_calib):
                    self.get_logger().info("AUTO RECALIBRATION")
                    self.paused = True
                    time.sleep(0.2)
                    with self.audio_queue.mutex:
                        self.audio_queue.queue.clear()
                    self.auto_calib.run()
                    self.paused = False
                    self.last_calib_time = time.time()

            # STATE: LISTEN
            if self.state == STTState.LISTEN:
                if not self.use_wake_mode and self.armed:
                    if self.listen_window_start > 0 and \
                        time.time() - self.listen_window_start > self.listen_timeout:
                        self.get_logger().info("Dialog listen timeout")
                        self.armed = False
                        self.woke_up = False
                        self.use_wake_mode = True
                        self.state = STTState.IDLE
                        continue

                if self._need_flush:
                    buffer = b""
                    self._need_flush = False

                if self.vad_mode == VADMode.ADAPTIVE and \
                    len(buffer) > RATE * 2 * 0.1:
                    tail = buffer[-int(RATE * 2 * 0.2):]  
                    rms = self.compute_rms(tail)
                    adaptive_start = self.noise_floor * 1.5
                    if rms < self.noise_floor * 1.3:
                        self.noise_floor = (
                            0.995 * self.noise_floor
                            + 0.005 * rms
                        )
                        self.noise_floor = max(50.0, min(self.noise_floor, 2000.0))
                    self.logd(
                        f"RMS={rms:.0f} "
                        f"NF={self.noise_floor:.0f} "
                        f"TH={adaptive_start:.0f}"
                    )

                if self.is_start_voice(buffer):
                    now = time.time()
                    self.state = STTState.DETECT
                    self.debug_pub("STATE -> DETECT")
                    self.start_time = now
                    self.last_voice_time = now
                    self.get_logger().info("Voice detected")
                continue

            # STATE: DETECT
            if self.state == STTState.DETECT:
                self.state = STTState.COLLECT
                self.debug_pub("STATE -> COLLECT")
                last_time = time.time()
                continue

            if self.state == STTState.COLLECT:
                pass

            # STATE: COLLECT
            if self.state == STTState.COLLECT:
                now = time.time()
                tail = buffer[-int(RATE * 2 * 0.3):]
                rms = self.compute_rms(tail)
                self.rms_history.append(rms)
                speech = False
                if len(self.rms_history) > 50:
                    self.rms_history.pop(0)
                env = self.env_classifier.update()
                # -------- VAD MODE SWITCH --------
                if self.vad_mode == VADMode.NEURAL:
                    speech = self.neural_vad_detect(tail)
                    self.logd(f"[COLLECT][NEURAL] speech={speech}")

                elif self.vad_mode == VADMode.HYBRID:
                    speech = self.hybrid_vad_detect(tail)
                    if self.debug:
                        if speech != self._last_collect_speech:
                            self.get_logger().info(f"[COLLECT][HYBRID] speech={speech}")
                            self._last_collect_speech = speech

                elif self.vad_mode == VADMode.FIXED:
                    th = self.continue_rms
                    speech = rms > th
                    self.logd(f"[COLLECT][FIXED] RMS={rms:.0f} TH={th:.0f}")

                elif self.vad_mode == VADMode.ADAPTIVE:
                    th = self.noise_floor * 1.15
                    speech = rms > th
                    self.logd(f"[COLLECT][ADAPTIVE] RMS={rms:.0f} TH={th:.0f}")

                elif self.vad_mode == VADMode.OFF:
                    speech = True
                    self.logd("[COLLECT][OFF] always speech")

                else:
                    speech = False
                # -------- UPDATE LAST VOICE --------
                if speech:
                    self.last_voice_time = now
                # -------- SILENCE END --------
                if now - self.last_voice_time > 0.6:
                    self.get_logger().info("COLLECT: silence end")
                    self.state = STTState.RECOGNIZE
                    self.debug_pub("STATE -> RECOGNIZE")
                    continue
                # -------- MAX LENGTH PROTECTION --------
                if now - self.start_time > 3.0:
                    self.get_logger().info("COLLECT: force max 3s")
                    self.state = STTState.RECOGNIZE
                    self.debug_pub("STATE -> RECOGNIZE")
                    continue
                continue

            # STATE: RECOGNIZE
            if self.state == STTState.RECOGNIZE:
                self._in_recognize = True
                self.get_logger().info("---- ENTER RECOGNIZE ----")
                try:
                    self.do_stt(buffer)
                finally:
                    self._in_recognize = False
                    buffer = b""
                    self._need_flush = True
                    self.state = STTState.IDLE
                    self.debug_pub("STATE -> IDLE")

    def do_stt(self, pcm_data: bytes):
        try:
            pcm_data = self.noise_gate(pcm_data)
            if not self.is_speech_dominant(pcm_data):
                self.get_logger().info("Drop: no speech dominant")
                return
            audio = sr.AudioData(pcm_data, RATE, 2)
            text = self.recognize_with_timeout(
                audio,
                timeout=self.google_timeout
            )

            if text is None:
                self.get_logger().warning("STT timeout")
                return
            text = text.strip()
            if self.debug_stt:
                self.get_logger().info(f"[RAW GOOGLE TEXT] >>{text}<<")

            if text == "":
                return
            
            if self.armed and not self.use_wake_mode:
                pass

            elif self.use_wake_mode:
                if not self.woke_up:
                    ok, cmd = self.check_wake_word(text)
                    if ok:
                        self.get_logger().info("WAKE WORD DETECTED → ARMED")
                        self.publish_wake(True)
                        self.woke_up = True
                        self.armed = True

                        if cmd.strip() != "":
                            text = cmd
                        else:
                            return
                    else:
                        if self.debug_stt:
                            self.get_logger().info("Ignore (no wake word)")
                        return
                ok, cmd = self.check_wake_word(text)
                text = cmd if ok else text
            # duplicate 
            now = time.time()
            key = text.lower().replace(" ", "")
            if key == self.last_dup and \
               (now - self.last_dup_ts) < self.duplicate_sec:

                self.get_logger().info("Drop duplicate")
                return
            self.last_dup = key
            self.last_dup_ts = now
            # publish 
            msg = String()
            msg.data = text
            self.pub.publish(msg)
            self.get_logger().info(f"Recognized: {text}")
            self.debug_pub(f"TEXT -> {text}")

        except sr.UnknownValueError:
            self.get_logger().info("not understand")
            return 
        except sr.RequestError as e:
            now = time.time()
            if now - self.last_err_ts > self.err_interval:
                self.get_logger().error(f"Google STT error: {e}")
                self.last_err_ts = now
            return
        except Exception as e:
            self.get_logger().error(f"STT exception: {e}")
        if self.one_shot:
            self.armed = False
            self.woke_up = False  
            self.use_wake_mode = True
            self._need_flush = True 
            self.state = STTState.IDLE
            self.debug_pub("STATE -> IDLE (ONE_SHOT)")

            time.sleep(0.3)                  
            with self.audio_queue.mutex:
                self.audio_queue.queue.clear()
            self.get_logger().info("STT DISARM (one shot)")

        if self._tts_timer:
            self._tts_timer.cancel()
            self._tts_timer = None


    def recognize_with_timeout(self, audio, timeout=6.0):
        result = {"text": None, "err": None}
        def _do():
            try:
                result["text"] = self.recognizer.recognize_google(
                    audio, language="th-TH"
                )
            except Exception as e:
                result["err"] = e
        th = threading.Thread(target=_do)
        th.daemon = True
        th.start()
        th.join(timeout)
        if th.is_alive():
            return None
        if result["err"]:
            raise result["err"]
        return result["text"]
    
    def is_start_voice(self, pcm: bytes):
        arr = np.frombuffer(pcm, dtype=np.int16)
        if len(arr) < self.min_samples:
            return False
        rms = np.sqrt(np.mean(arr.astype(np.float32)**2))

        if self.vad_mode == VADMode.NEURAL:
            return self.neural_vad_detect(pcm)

        if self.vad_mode == VADMode.OFF:
            return True
        
        elif self.vad_mode == VADMode.HYBRID:
            return self.hybrid_vad_detect(pcm)
        
        elif self.vad_mode == VADMode.FIXED:
            threshold = self.start_rms

        elif self.vad_mode == VADMode.ADAPTIVE:
            threshold = self.noise_floor * 1.4
        self.logd(
            f"[START] RMS={rms:.0f} TH={threshold:.0f} MODE={self.vad_mode.name}"
        )
        return rms > threshold

    def is_continue_voice(self, pcm: bytes):
        arr = np.frombuffer(pcm, dtype=np.int16)
        if len(arr) < 800:   # ~25ms
            return False

        rms = np.sqrt(np.mean(arr.astype(np.float32)**2))
        self.logd(f"[CONT] RMS={rms:.1f}")
        adaptive_continue = self.noise_floor * 1.8
        return rms > adaptive_continue

    def noise_gate(self, pcm: bytes) -> bytes:
        arr = np.frombuffer(pcm, dtype=np.int16).astype(np.float32)
        rms = float(np.sqrt(np.mean(arr**2)))
        # adaptive gate
        if rms < 500:
            return (arr * 0.05).astype(np.int16).tobytes()
        if rms < 900:
            return (arr * 0.3).astype(np.int16).tobytes()
        return pcm

    def normalize(self, pcm: bytes) -> bytes:
        arr = np.frombuffer(pcm, dtype=np.int16).astype(np.float32)

        peak = np.max(np.abs(arr))
        if peak < 10:
            return pcm
        gain = 3000.0 / peak
        arr = arr * gain
        arr = np.clip(arr, -32768, 32767)
        return arr.astype(np.int16).tobytes()
    
    def preprocess(self, pcm: bytes) -> bytes:
        arr = np.frombuffer(pcm, dtype=np.int16).astype(np.float32)
        arr = arr - np.mean(arr)# DC Removal 
        alpha = 0.95  # High-pass ~100Hz (simple IIR) 
        prev_x = 0.0
        prev_y = 0.0
        for i in range(len(arr)):
            x = arr[i]
            y = alpha * (prev_y + x - prev_x)
            arr[i] = y
            prev_x = x
            prev_y = y
        rms = float(np.sqrt(np.mean(arr**2) + 1e-6))# AGC (RMS based) 
        target_rms = 2200.0
        gain = target_rms / max(rms, 1.0)
        gain = max(0.6, min(gain, 4.0)) # gain
        arr *= gain
        arr = np.clip(arr, -12000, 12000)# Limiter 
        return arr.astype(np.int16).tobytes()
    
    def is_speech_dominant(self, pcm: bytes) -> bool:
        arr = np.frombuffer(pcm, dtype=np.int16).astype(np.float32)
        if len(arr) < 400:
            return False
        rms = float(np.sqrt(np.mean(arr**2)))
      
        if self.use_wake_mode and not self.woke_up:
            return rms > 200   # 👈 สำคัญ
        return rms > 800

    def check_wake_word(self, text: str):
        text = text.strip().lower()
        wake_words = ["สวัสดี", "หวัดดี"]

        for w in wake_words:
            if w in text:
                cmd = text.replace(w, "").strip()
                return True, cmd

        return False, text

    
    def control_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"STT CONTROL CMD: {cmd}")

        if cmd == "MANUAL_ARM":
            self.paused = False
            self.armed = True
            self.woke_up = True
            self.state = STTState.LISTEN
            self.debug_pub("STATE -> LISTEN (MANUAL_ARM)")
            self._need_flush = True
            with self.audio_queue.mutex:
                self.audio_queue.queue.clear()
            self.get_logger().info("MANUAL_ARM -> force RESUME + ARMED")
            return

        if cmd in ["RESET", "CANCEL", "MANUAL_CANCEL"]:
            self.full_reset()
            return
        
        elif cmd == "PAUSE":
            if self.armed:
                self.get_logger().info("Ignore PAUSE because ARMED")
                return
            self.paused = True
            self.get_logger().info("STT PAUSED")

        elif cmd == "RESUME":
            self.paused = False
            self.get_logger().info("STT RESUMED")

        elif cmd == "VAD_FIXED":
            self.vad_mode = VADMode.FIXED
            self.get_logger().info("VAD MODE → FIXED")

        elif cmd == "VAD_ADAPTIVE":
            self.vad_mode = VADMode.ADAPTIVE
            self.get_logger().info("VAD MODE → ADAPTIVE")

        elif cmd == "VAD_OFF":
            self.vad_mode = VADMode.OFF
            self.get_logger().info("VAD MODE → OFF")

        elif cmd == "VAD_NEURAL":
            if self.vad_model is None:
                self.get_logger().warning("Neural VAD not loaded")
                return
            self.vad_mode = VADMode.NEURAL
            self.get_logger().info("VAD MODE → NEURAL")
        
        elif cmd == "VAD_HYBRID":
            if self.vad_model is None:
                self.get_logger().warning("Neural VAD not loaded")
                return
            self.vad_mode = VADMode.HYBRID
            self.get_logger().info("VAD MODE → HYBRID")

        elif cmd == "CALIB_OFF":
            self.auto_calib.set_mode(AutoCalibrationMode.OFF)

        elif cmd == "CALIB_MANUAL":
            self.auto_calib.set_mode(AutoCalibrationMode.MANUAL)

        elif cmd == "CALIB_AUTO":
            self.auto_calib.set_mode(AutoCalibrationMode.AUTO)

        elif cmd == "CALIB_RUN":
            self.paused = True
            time.sleep(0.3)
            with self.audio_queue.mutex:
                self.audio_queue.queue.clear()
            self.auto_calib.run()
            self.paused = False
        
        elif cmd == "DYNAMIC_ON":
            self.dynamic.set_mode(DynamicMode.ON)

        elif cmd == "DYNAMIC_OFF":
            self.dynamic.set_mode(DynamicMode.OFF)
        
        elif cmd == "ENV_ON":
            self.env_classifier.set_mode(EnvironmentMode.ON)

        elif cmd == "ENV_OFF":
            self.env_classifier.set_mode(EnvironmentMode.OFF)
        
        elif cmd == "ENV_DEBUG_ON":
            self.debug_env = True
            self.get_logger().info("ENV DEBUG → ON")

        elif cmd == "ENV_DEBUG_OFF":
            self.debug_env = False
            self.get_logger().info("ENV DEBUG → OFF")
        
        elif cmd == "STT_DEBUG_ON":
            self.debug_stt = True
            self.get_logger().info("STT DEBUG → ON")

        elif cmd == "STT_DEBUG_OFF":
            self.debug_stt = False
            self.get_logger().info("STT DEBUG → OFF")

    def full_reset(self):
        self.get_logger().info("FULL RESET")
        self.debug_pub("FULL RESET")
        self.state = STTState.IDLE
        self.armed = False
        self.woke_up = False
        self.paused = False
        self._need_flush = True
        self.publish_wake(False)
        with self.audio_queue.mutex:
            self.audio_queue.queue.clear()
        time.sleep(0.05)

    def dialog_event_callback(self, msg: String):
        event = msg.data.strip().upper()
        self.get_logger().info(f"DIALOG EVENT: {event}")

        if event == "TTS:DONE":
            if self._in_recognize:
                return
            self.state = STTState.WAIT_TTS_CLEAR
            self.paused = True
            self.tts_clear_start = time.time()

        elif event == "MANUAL:TIMEOUT":
            self.get_logger().info("Received MANUAL:TIMEOUT → Force stop listening")
            self.armed = False
            self.woke_up = False
            self.use_wake_mode = True
            self.paused = False
            self.state = STTState.IDLE
            self._need_flush = True
            with self.audio_queue.mutex:
                self.audio_queue.queue.clear()

    def open_after_tts(self):
        self.get_logger().info("Opening mic after delay")
        with self.audio_queue.mutex:
            self.audio_queue.queue.clear()
        self.paused = False
        self._need_flush = True
        self.use_wake_mode = False
        self.woke_up = True
        self.armed = True
        self.state = STTState.LISTEN
        self._tts_timer = threading.Timer(6.0, self.auto_disarm_after_tts)
        self._tts_timer.start()
        self.debug_pub("STATE -> LISTEN (AFTER TTS)")

    def auto_disarm_after_tts(self):
        if not self._in_recognize and self.armed:
            self.get_logger().info("After TTS → Timeout listening window")

            self.armed = False
            self.woke_up = False
            self.use_wake_mode = True
            self.state = STTState.IDLE
            self.publish_wake(False)

    def compute_rms(self, pcm: bytes) -> float:
        arr = np.frombuffer(pcm, dtype=np.int16).astype(np.float32)
        if len(arr) == 0:
            return 0.0
        return float(np.sqrt(np.mean(arr**2)))
    
    def neural_vad_detect(self, pcm: bytes) -> bool:
        if self.vad_model is None:
            return False

        if len(pcm) < int(RATE * 2 * 0.1):
            return False

        rms = self.compute_rms(pcm)
        if rms < 200:
            return False

        audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
        audio_tensor = self.torch.from_numpy(audio)

        speech = self.get_speech_timestamps(
            audio_tensor,
            self.vad_model,
            sampling_rate=RATE
        )
        return len(speech) > 0
    
    def hybrid_vad_detect(self, pcm: bytes) -> bool:
        if self.vad_model is None:
            return False
        rms = self.compute_rms(pcm)
        base_th = self.dynamic.get_threshold()

        if self.env_classifier.mode == EnvironmentMode.ON:
            env = self.env_classifier.current_env

            if env == "QUIET":
                multiplier = 0.8
            elif env == "NOISY":
                multiplier = 1.4
            else:
                multiplier = 1.0
            dynamic_th = base_th * multiplier
            if self.debug_env:
                self.get_logger().info(
                    f"[ENV:{env}] base={base_th:.0f} "
                    f"mult={multiplier:.2f} "
                    f"final_th={dynamic_th:.0f}"
                )

        else:
            dynamic_th = base_th

        if self.dynamic.mode == DynamicMode.ON:
            if rms < dynamic_th * 0.5:
                return False
        else:
            if rms < 250:
                return False
        audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
        audio_tensor = self.torch.from_numpy(audio)
        speech = self.get_speech_timestamps(
            audio_tensor,
            self.vad_model,
            sampling_rate=RATE
        )
        return len(speech) > 0

def main():
    rclpy.init()
    node = GoogleSTTNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
