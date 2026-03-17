#!/usr/bin/env python3
import re
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# Helpers
def normalize_thai(text: str) -> str:
    t = (text or "").strip()
    t = t.replace("ๆ", "")
    t = re.sub(r"\s+", " ", t)
    return t

def parse_degrees_loose(text: str) -> Optional[float]:
    t = normalize_thai(text).lower()

    m = re.search(r"(\d+(?:\.\d+)?)\s*(องศา|deg|degree|degrees)?(?=\D|$)", t)
    if m:
        try:
            return float(m.group(1))
        except Exception:
            return None

    thai_map = {
        "สิบ": 10, "ยี่สิบ": 20, "สามสิบ": 30, "สี่สิบ": 40, "ห้าสิบ": 50,
        "หกสิบ": 60, "เจ็ดสิบ": 70, "แปดสิบ": 80, "เก้าสิบ": 90,
        "หนึ่ง": 1, "สอง": 2, "สาม": 3, "สี่": 4, "ห้า": 5, "หก": 6,
        "เจ็ด": 7, "แปด": 8, "เก้า": 9,
        "สิบห้า": 15, "ยี่สิบห้า": 25, "สามสิบห้า": 35, "สี่สิบห้า": 45,
    }

    if ("องศา" in t) or ("degree" in t) or ("deg" in t):
        for w, v in sorted(thai_map.items(), key=lambda kv: len(kv[0]), reverse=True):
            if w in t:
                return float(v)

    return None


def parse_position_reply_loose(text: str) -> Optional[int]:
    t = normalize_thai(text).lower()

    m = re.search(r"(^|\D)([1-5])(\D|$)", t)
    if m:
        return int(m.group(2))

    mapping = {
        "หนึ่ง": 1, "one": 1,
        "สอง": 2, "two": 2, "to": 2,
        "สาม": 3, "three": 3,
        "สี่": 4, "four": 4,
        "ห้า": 5, "five": 5,
    }
    for w, v in mapping.items():
        if w in t:
            return v
    return None


def parse_direction_reply_loose(text: str) -> Optional[str]:
    """Return canonical direction: left/right/forward/back/up/down"""
    t = normalize_thai(text).lower()

    if re.search(r"(ซ้าย|left|เลฟ|เล็ฟ|ทวน)", t):
        return "left"
    if re.search(r"(ขวา|right|ไรท์|ตาม)", t):
        return "right"

    if re.search(r"(หน้า|forward|ฟอร์เวิร์ด)", t):
        return "forward"
    if re.search(r"(หลัง|back|แบ็ค)", t):
        return "back"

    if re.search(r"(ขึ้น|บน|up)", t):
        return "up"
    if re.search(r"(ลง|ล่าง|down)", t):
        return "down"

    return None


def parse_distance_loose(text: str) -> Optional[float]:
    """
    Return distance in meters.
    - '20' (no unit) => assume cm (0.20 m)
    - '20 เซน' / '20 cm' => 0.20
    - '200 มม' / '200 mm' => 0.20
    - '0.1 เมตร' / '0.1 m' => 0.1
    """
    t = normalize_thai(text).lower()

    m = re.search(r"(\d+(?:\.\d+)?)\s*(cm|เซน|เซนติ|มม|mm|เมตร|m)?(?=\D|$)", t)
    if not m:
        return None
    try:
        val = float(m.group(1))
    except Exception:
        return None

    unit = (m.group(2) or "").strip()

    if unit in ("มม", "mm"):
        return val / 1000.0
    if unit in ("cm", "เซน", "เซนติ"):
        return val / 100.0
    if unit in ("เมตร", "m"):
        return val

    return val / 100.0


def _dir_th(d: str) -> str:
    return {
        "left": "ซ้าย",
        "right": "ขวา",
        "forward": "หน้า",
        "back": "หลัง",
        "up": "ขึ้น",
        "down": "ลง",
    }.get(d or "", d or "")

class DialogFSMNode(Node):
    def __init__(self):
        super().__init__("dialog_fsm_node")

        # ---------------- topics ----------------
        self.declare_parameter("topic_heard_text", "voice/heard_text")
        self.declare_parameter("topic_dialog_request", "Neural_parser/dialog_request")

        # GUI roles
        self.declare_parameter("topic_gui_event", "gui_control/gui_event")  # GUI -> Dialog
        self.declare_parameter("topic_gui_cmd", "control/gui_cmd")      # Dialog -> GUI

        # busy from TTS manager
        self.declare_parameter("topic_tts_busy", "text_to_speech/tts_busy")

        # derived done -> send to SpeechCore
        self.declare_parameter("topic_tts_done", "control/tts_done")

        self.declare_parameter("topic_feedback_request", "Neural_parser/feedback_request")
        self.declare_parameter("topic_text_raw", "control/text_raw")
        self.declare_parameter("topic_feedback", "control/feedback")
        self.declare_parameter("topic_beep", "control/beep")
        self.declare_parameter("topic_tts_request", "control/tts_request")

        # control SpeechCore
        self.declare_parameter("topic_voice_armed", "control/armed")
        self.declare_parameter("topic_stt_control", "control/stt_control")

        self.declare_parameter("debug", True)
        self._wake_auto_disarm_deadline = None
        self.declare_parameter("topic_dialog_event", "control/dialog_event")
        self.declare_parameter("topic_dialog_debug", "control/dialog_debug")
        self.declare_parameter("topic_wake_detected", "voice/wake_detected") # wake_speech_to_text

        # ---- params ----
        self.declare_parameter("wake_timeout_sec", 6.0)
        self.declare_parameter("manual_timeout_sec", 6.0)
        self.declare_parameter("wake_cooldown_ms", 1200)

        self.declare_parameter("post_command_ignore_sec", 1.0)
        self.declare_parameter("post_tts_ignore_sec", 0.8)
        self.declare_parameter("dialog_max_retry", 2)

        self.declare_parameter("debug_heard_when_active", True)
        self.declare_parameter("beep_enabled", True)
        self.declare_parameter("listen_beep_delay_sec", 0.05)

        self.declare_parameter("tts_enabled", True)
        self.declare_parameter("tts_dedup_window_sec", 1.2)
        self.declare_parameter("tts_dedup_normalize", True)

        self.declare_parameter("dialog_timeout_sec", 10.0)

        # debounce tts_done publish
        self.declare_parameter("tts_done_debounce_sec", 0.12)

        # Dialog สั่ง SpeechCore PAUSE ระหว่าง TTS busy
        self.declare_parameter("stt_pause_while_tts", False)

        self.declare_parameter(
            "ignore_phrases_soft",
            [
                "กำลังดำเนินการ",
                "รับคำสั่งแล้ว",
                "ขอโทษค่ะ",
                "ยังไม่ชัดค่ะ",
                "ต้องการกี่องศา",
                "เลือก 1 2 3 4 หรือ 5",
                "คุณหมายถึงตำแหน่งไหน",
                "ต้องการเลื่อนขึ้นหรือลง",
                "ต้องการหมุนกี่องศา",
                "ต้องการขยับไปทางไหน",
                "ต้องการระยะเท่าไร",
                "ต้องการหมุนซ้ายหรือขวา",
                "ต้องการหมุนข้อมือสามซ้ายหรือขวา",
                "ต้องการมุมมองที่ตำแหน่งไหน",
            ],
        )

        self.declare_parameter(
            "global_cancel_words",
            ["ยกเลิก", "cancel", "แคนเซิล", "ไม่เอา", "พอ", "หยุด", "stop", "สต็อป"],
        )

        self.declare_parameter("ignore_phrases_hard", ["เอ่อ", "อืม", "ครับ", "ค่ะ"])

        # beep options
        self.declare_parameter("beep_on_manual_arm", True)
        self.beep_on_manual_arm = bool(self.get_parameter("beep_on_manual_arm").value)

        # RESET STT หลัง TTS เสร็จ (busy falling edge)  (แนะนำ: ปิดไว้ ถ้าเจอ -1000)
        self.declare_parameter("stt_reset_after_tts_done", False)
        self.stt_reset_after_tts_done = bool(self.get_parameter("stt_reset_after_tts_done").value)

        # Reset dialog
        self.declare_parameter("stt_reset_after_tts_dialog_only", False)
        self.stt_reset_after_tts_dialog_only = bool(self.get_parameter("stt_reset_after_tts_dialog_only").value)

        # Delay affter reset
        self.declare_parameter("stt_reset_settle_sec", 0.05)
        self.stt_reset_settle_sec = float(self.get_parameter("stt_reset_settle_sec").value)

        # Delay TTS ( tail / echo)
        self.declare_parameter("post_tts_listen_delay_sec", 0.3)
        self.post_tts_listen_delay_sec = float(self.get_parameter("post_tts_listen_delay_sec").value)

        # beep aligned with RESUME (manual arm + dialog)
        self.declare_parameter("beep_on_resume_enable", True)
        self.beep_on_resume_enable = bool(self.get_parameter("beep_on_resume_enable").value)

        self.declare_parameter("beep_on_resume_kind", "WAKE")
        self.beep_on_resume_kind = str(self.get_parameter("beep_on_resume_kind").value)

        self._beep_on_next_resume = False
        self._beep_on_next_resume_kind = "WAKE"

        # guards
        self._resume_guard_until_ts = 0.0
        self._tts_guard_until_ts = 0.0

        # confirm busy falling edge
        self.declare_parameter("tts_busy_fall_confirm_sec", 0.25)
        self.tts_busy_fall_confirm_sec = float(self.get_parameter("tts_busy_fall_confirm_sec").value)
        self._tts_fall_deadline_ts = None

        # ---------------- read params ----------------
        self.topic_heard_text = str(self.get_parameter("topic_heard_text").value)
        self.topic_dialog_request = str(self.get_parameter("topic_dialog_request").value)

        self.topic_gui_event = str(self.get_parameter("topic_gui_event").value)
        self.topic_gui_cmd = str(self.get_parameter("topic_gui_cmd").value)

        self.topic_tts_busy = str(self.get_parameter("topic_tts_busy").value)
        self.topic_tts_done = str(self.get_parameter("topic_tts_done").value)

        self.topic_feedback_request = str(self.get_parameter("topic_feedback_request").value)

        self.topic_text_raw = str(self.get_parameter("topic_text_raw").value)
        self.topic_feedback = str(self.get_parameter("topic_feedback").value)
        self.topic_beep = str(self.get_parameter("topic_beep").value)
        self.topic_tts_request = str(self.get_parameter("topic_tts_request").value)

        self.topic_voice_armed = str(self.get_parameter("topic_voice_armed").value)
        self.topic_stt_control = str(self.get_parameter("topic_stt_control").value)

        self.debug = bool(self.get_parameter("debug").value)
        self.topic_dialog_event = str(self.get_parameter("topic_dialog_event").value)
        self.topic_dialog_debug = str(self.get_parameter("topic_dialog_debug").value)

        self.wake_timeout_sec = float(self.get_parameter("wake_timeout_sec").value)
        self.manual_timeout_sec = float(self.get_parameter("manual_timeout_sec").value)
        self.wake_cooldown_ms = int(self.get_parameter("wake_cooldown_ms").value)

        self.post_command_ignore_sec = float(self.get_parameter("post_command_ignore_sec").value)
        self.post_tts_ignore_sec = float(self.get_parameter("post_tts_ignore_sec").value)
        self.dialog_max_retry = int(self.get_parameter("dialog_max_retry").value)

        self.ignore_phrases_soft = [normalize_thai(p).lower() for p in self.get_parameter("ignore_phrases_soft").value]
        self.global_cancel_words = [normalize_thai(w).lower() for w in self.get_parameter("global_cancel_words").value]
        self.ignore_phrases_hard = [normalize_thai(p).lower() for p in self.get_parameter("ignore_phrases_hard").value]

        self.debug_heard_when_active = bool(self.get_parameter("debug_heard_when_active").value)
        self.beep_enabled = bool(self.get_parameter("beep_enabled").value)
        self.listen_beep_delay_sec = float(self.get_parameter("listen_beep_delay_sec").value)

        self.tts_enabled = bool(self.get_parameter("tts_enabled").value)
        self.tts_dedup_window_sec = float(self.get_parameter("tts_dedup_window_sec").value)
        self.tts_dedup_normalize = bool(self.get_parameter("tts_dedup_normalize").value)

        self.dialog_timeout_sec = float(self.get_parameter("dialog_timeout_sec").value)
        self.tts_done_debounce_sec = float(self.get_parameter("tts_done_debounce_sec").value)
        self.stt_pause_while_tts = bool(self.get_parameter("stt_pause_while_tts").value)

        self.topic_wake_detected = str(self.get_parameter("topic_wake_detected").value)

        # ---- pubs ----
        self.text_raw_pub = self.create_publisher(String, self.topic_text_raw, 10)
        self.feedback_pub = self.create_publisher(String, self.topic_feedback, 10)
        self.gui_cmd_pub = self.create_publisher(String, self.topic_gui_cmd, 10)
        self.beep_pub = self.create_publisher(String, self.topic_beep, 10)
        self.tts_req_pub = self.create_publisher(String, self.topic_tts_request, 10)

        self.event_pub = self.create_publisher(String, self.topic_dialog_event, 10)
        self.debug_pub = self.create_publisher(String, self.topic_dialog_debug, 10)

        self.speech_armed_pub = self.create_publisher(Bool, self.topic_voice_armed, 10)
        self.stt_ctl_pub = self.create_publisher(String, self.topic_stt_control, 10)

        self.tts_done_pub = self.create_publisher(Bool, self.topic_tts_done, 10)

        # ---- subs ----
        self.heard_sub = self.create_subscription(String, self.topic_heard_text, self.on_heard_text, 10)
        self.gui_event_sub = self.create_subscription(String, self.topic_gui_event, self.on_gui_event, 10)
        self.dialog_sub = self.create_subscription(String, self.topic_dialog_request, self.on_dialog_request, 10)
        self.tts_busy_sub = self.create_subscription(Bool, self.topic_tts_busy, self.on_tts_busy, 10)
        self.fb_req_sub = self.create_subscription(String, self.topic_feedback_request, self.on_feedback_request, 10)
        self.wake_sub = self.create_subscription(Bool, self.topic_wake_detected, self.on_wake_detected, 10)

        # ---- internal state ----
        self._lock = threading.Lock()
        self._pending_manual_arm = False

        self._mode = "idle"  # idle | await_command | dialog
        self._armed = False
        self._wake_recent = False

        self._last_speech_armed_sent: Optional[bool] = None

        self._dialog_mode: Optional[str] = None
        self._dialog_dir: Optional[str] = None
        self._dialog_retry = 0

        self._ignore_until_ts = 0.0
        self._pending_listen_beep = False

        self._last_tts_text = ""
        self._last_tts_ts = 0.0

        # track busy state
        self._tts_busy_last: Optional[bool] = None

        # prevent publish done spam
        self._last_tts_done_pub_ts = 0.0

        # dialog timeout internals
        self._dialog_timeout_token = 0
        self._dialog_deadline_ts = None
        self._dialog_deadline_token = 0
        self._dialog_deadline_reason = "dialog_wait"

        # beep scheduling
        self._beep_deadline_ts = None
        self._beep_kind = "LISTEN"

        # wake scheduling
        self._wake_timeout_reason = "wake"
        self._wake_cooldown_until_ts = None
        self._wake_timeout_deadline_ts = None

        # schedule resume after TTS done
        self._resume_after_tts_deadline_ts = None
        self._resume_after_tts_reason = "tts_done"

        # scheduler
        self._tick_period_sec = 0.05
        self._tick_timer = self.create_timer(self._tick_period_sec, self._on_tick)

        # init GUI
        self.gui_cmd("SET_STATUS: Idle: รอคำว่า 'สวัสดี'")
        self.gui_cmd("SET_RESULT:Text: (none yet)")
        self.gui_cmd("HIDE_POS")
        self.gui_cmd("HIDE_SCROLL")
        self.gui_cmd("HIDE_ROTATE")

        self._set_speech_armed(False)
        self._print_startup_banner()
        self._dbg("ready")

    # ---------------- banner/debug ----------------
    def _print_startup_banner(self):
        GREEN = "\033[92m"
        RESET = "\033[0m"

        supported = [
            "Wake-word arming -> await_command",
            "Dialog modes: ASK_POS / ASK_VIEW_POS / ASK_SCROLL_DIR->DIST / ASK_ROTATE_DIR->DEG / W3 / MOVE",
            "TTS request de-dup (window + normalize)",
            "TTS busy->done confirm (publish /voice/tts_done=True after stable false)",
            "Optional STT pause while TTS busy (stt_pause_while_tts)",
            "Global cancel words across states",
            "Dialog timeout + retry system",
            "Beep scheduling + GUI cmd panels",
            "GUI topic roles: gui_event(GUI->FSM) / gui_cmd(FSM->GUI)",
        ]

        banner = (
            "\n"
            "──────────────────────────────────────────────────────────────\n"
            "        Dialog FSM Node — Operational\n"
            "        Node State      : READY\n"
            f"        Debug Mode      : {'ENABLED' if self.debug else 'DISABLED'}\n"
            f"        Debug Topic     : {self.topic_dialog_debug if self.debug else '(disabled)'}\n"
            f"        Event Topic     : {self.topic_dialog_event}\n"
            "\n"
            "        Subscribed Topics:\n"
            f"            • {self.topic_heard_text}      (from SpeechCore)\n"
            f"            • {self.topic_gui_event}      (from GUI events)\n"
            f"            • {self.topic_dialog_request}  (from NLU)\n"
            f"            • {self.topic_tts_busy}        (from TTS Manager)\n"
            f"            • {self.topic_feedback_request}\n"
            "\n"
            "        Published Topics:\n"
            f"            • {self.topic_text_raw}        (to NLU)\n"
            f"            • {self.topic_feedback}        (user feedback)\n"
            f"            • {self.topic_gui_cmd}        (GUI cmds)\n"
            f"            • {self.topic_beep}            (beep kind)\n"
            f"            • {self.topic_tts_request}     (Dialog -> TTS)\n"
            f"            • {self.topic_tts_done}        (derived)\n"
            f"            • {self.topic_voice_armed}     (to SpeechCore)\n"
            f"            • {self.topic_stt_control}     (RESET/PAUSE/RESUME)\n"
            f"            • {self.topic_dialog_event}\n"
            f"            • {self.topic_dialog_debug}\n"
            "\n"
            "        Runtime Config:\n"
            f"            • wake_timeout_sec     = {self.wake_timeout_sec}\n"
            f"            • manual_timeout_sec   = {self.manual_timeout_sec}\n"
            f"            • wake_cooldown_ms     = {self.wake_cooldown_ms}\n"
            f"            • dialog_timeout_sec   = {self.dialog_timeout_sec}\n"
            f"            • dialog_max_retry     = {self.dialog_max_retry}\n"
            f"            • post_cmd_ignore_sec  = {self.post_command_ignore_sec}\n"
            f"            • post_tts_ignore_sec  = {self.post_tts_ignore_sec}\n"
            f"            • post_tts_listen_delay_sec = {self.post_tts_listen_delay_sec}\n"
            f"            • tts_busy_fall_confirm_sec = {self.tts_busy_fall_confirm_sec}\n"
            f"            • tts_enabled         = {self.tts_enabled}\n"
            f"            • tts_dedup_window_sec = {self.tts_dedup_window_sec}\n"
            f"            • tts_dedup_normalize  = {self.tts_dedup_normalize}\n"
            f"            • tts_done_debounce_sec= {self.tts_done_debounce_sec}\n"
            f"            • stt_pause_while_tts  = {self.stt_pause_while_tts}\n"
            "\n"
            "        Supported Functions:\n"
            "            - " + "\n            - ".join(supported) + "\n"
            "──────────────────────────────────────────────────────────────\n"
        )

        self.get_logger().info(GREEN + banner + RESET)

        if self.debug:
            try:
                self.debug_pub.publish(String(data="[DEBUG][dialog_fsm] banner:\n" + banner))
            except Exception:
                pass

    # ---------------- Speech control helpers ----------------
    def _set_speech_armed(self, armed: bool):
        armed = bool(armed)
        with self._lock:
            if self._last_speech_armed_sent is not None and self._last_speech_armed_sent == armed:
                return
            self._last_speech_armed_sent = armed
        try:
            self.speech_armed_pub.publish(Bool(data=armed))
        except Exception:
            pass
        self._dbg(f"publish /voice/armed={armed}")

    def _send_stt_control(self, cmd: str):
        cmd = normalize_thai(cmd).upper()
        if not cmd:
            return
        try:
            self.stt_ctl_pub.publish(String(data=cmd))
        except Exception:
            pass
        self._dbg(f"publish /voice/stt_control='{cmd}'")

    def _schedule_resume_after_tts(self, reason: str = "tts_done"):
        d = float(getattr(self, "post_tts_listen_delay_sec", 0.0) or 0.0)
        new_deadline = time.time() + max(0.0, d)

        with self._lock:
            self._resume_after_tts_reason = reason
            if self._resume_after_tts_deadline_ts is None:
                self._resume_after_tts_deadline_ts = new_deadline
            else:
                self._resume_after_tts_deadline_ts = max(self._resume_after_tts_deadline_ts, new_deadline)

            self._ignore_until_ts = max(self._ignore_until_ts, self._resume_after_tts_deadline_ts)

        self._dbg(f"schedule RESUME after TTS in {d:.3f}s reason={reason}")

    def _reset_stt_after_tts_done(self):
        if not getattr(self, "stt_reset_after_tts_done", False):
            return

        if getattr(self, "stt_reset_after_tts_dialog_only", True):
            with self._lock:
                in_dialog_now = (self._dialog_mode is not None)
            if not in_dialog_now:
                return

        self._send_stt_control("RESET")
        s = float(getattr(self, "stt_reset_settle_sec", 0.0) or 0.0)
        if s > 0:
            time.sleep(min(0.2, max(0.0, s)))

    def _publish_tts_done(self):
        now = time.time()
        with self._lock:
            if self.tts_done_debounce_sec > 0 and (now - self._last_tts_done_pub_ts) < self.tts_done_debounce_sec:
                return
            self._last_tts_done_pub_ts = now
        try:
            self.tts_done_pub.publish(Bool(data=True))
        except Exception:
            pass
        self._dbg("publish /voice/tts_done=True (derived)")

    # ---------------- NLU-style helpers ----------------
    def _dbg(self, msg: str) -> None:
        if self.debug and hasattr(self, "debug_pub"):
            try:
                self.debug_pub.publish(String(data=f"[DEBUG][dialog_fsm] {msg}"))
            except Exception:
                pass

    def _event(self, msg: str) -> None:
        try:
            self.event_pub.publish(String(data=msg))
        except Exception:
            pass

    # ---------------- output helpers ----------------
    def publish_text_raw(self, text: str):
        try:
            self.text_raw_pub.publish(String(data=text))
        except Exception:
            pass

    def publish_feedback(self, text: str):
        try:
            self.feedback_pub.publish(String(data=text))
        except Exception:
            pass

    def gui_cmd(self, text: str):
        try:
            self.gui_cmd_pub.publish(String(data=text))
        except Exception:
            pass

    def beep(self, kind: str = "BEEP"):
        if not self.beep_enabled:
            return
        try:
            self.beep_pub.publish(String(data=str(kind)))
        except Exception:
            pass

    def _schedule_beep(self, kind: str, delay_sec: float):
        if not self.beep_enabled:
            return
        with self._lock:
            self._beep_kind = str(kind or "LISTEN")
            if self._beep_deadline_ts is None:
                self._beep_deadline_ts = time.time() + max(0.0, float(delay_sec))
            else:
                self._beep_deadline_ts = min(self._beep_deadline_ts, time.time() + max(0.0, float(delay_sec)))

    def _schedule_beep_on_resume(self, kind: str = "WAKE"):
        if not getattr(self, "beep_on_resume_enable", True):
            return
        with self._lock:
            self._beep_on_next_resume = True
            self._beep_on_next_resume_kind = str(kind or "WAKE")

    def tts_request(self, text: str):
        if not self.tts_enabled:
            return

        text = normalize_thai(text)
        if not text:
            return

        # ถ้าเปิด pause ระหว่าง tts: สั่ง pause ไว้ก่อน
        if self.stt_pause_while_tts:
            self._send_stt_control("PAUSE")
            with self._lock:
                self._resume_after_tts_deadline_ts = None

        now = time.time()
        key = normalize_thai(text).lower() if self.tts_dedup_normalize else text
        with self._lock:
            last_key = self._last_tts_text
            last_ts = self._last_tts_ts
            if last_key and (key == last_key) and ((now - last_ts) <= self.tts_dedup_window_sec):
                self._dbg(f"TTS de-dup drop text='{text}'")
                return
            self._last_tts_text = key
            self._last_tts_ts = now

        try:
            self.tts_req_pub.publish(String(data=text))
        except Exception:
            pass
        self._dbg(f"TTS request text='{text}'")

    # ---------------- dialog timeout helpers ----------------
    def _cancel_dialog_timeout(self):
        with self._lock:
            self._dialog_timeout_token += 1
            self._dialog_deadline_ts = None
            self._dialog_deadline_token = self._dialog_timeout_token

    def _start_dialog_timeout(self, reason="dialog_wait"):
        with self._lock:
            self._dialog_timeout_token += 1
            token = self._dialog_timeout_token
            self._dialog_deadline_ts = time.time() + self.dialog_timeout_sec
            self._dialog_deadline_token = token
            self._dialog_deadline_reason = reason

    # ---------------- state transitions ----------------
    def _enter_dialog(
        self,
        dialog_mode: str,
        dialog_dir: Optional[str],
        *,
        show_pos: bool = False,
        show_scroll: bool = False,
        show_rotate: bool = False,
        status: str = " Listening...",
        timeout_reason: str = "enter_dialog",
        beep_listen: bool = True,
        reset_retry: bool = True,
    ):
        with self._lock:
            self._mode = "dialog"
            self._armed = True
            self._dialog_mode = dialog_mode
            self._dialog_dir = dialog_dir
            if reset_retry:
                self._dialog_retry = 0
            self._pending_listen_beep = bool(beep_listen)
            self._wake_timeout_deadline_ts = None
            self._wake_timeout_reason = "wake"
            tts_busy_now = (self._tts_busy_last is True)

        self._set_speech_armed(True)

        #  beep LISTEN: ถ้า TTS ไม่ busy → beep ทันที / ถ้า busy → ไป beep ตอน RESUME
        if beep_listen and self.beep_enabled:
            if tts_busy_now:
                self._schedule_beep_on_resume("LISTEN")
                self._dbg("enter_dialog: TTS busy -> beep LISTEN on next RESUME")
            else:
                self._schedule_beep("LISTEN", self.listen_beep_delay_sec)
                self._dbg("enter_dialog: beep LISTEN scheduled immediately")

        if show_pos:
            self.gui_cmd("SHOW_POS")
        if show_scroll:
            self.gui_cmd("SHOW_SCROLL")
        if show_rotate:
            self.gui_cmd("SHOW_ROTATE")

        self.gui_cmd(f"SET_STATUS:{status}")
        self._start_dialog_timeout(reason=timeout_reason)
        self._dbg(
            f"enter_dialog mode={dialog_mode} dir={dialog_dir} "
            f"pos={show_pos} scroll={show_scroll} rotate={show_rotate} "
            f"beep={beep_listen} reason={timeout_reason}"
        )

    def _exit_dialog_to_idle(
        self,
        *,
        status: str = " Idle: รอคำว่า 'สวัสดี'",
        hide_pos: bool = True,
        hide_scroll: bool = True,
        hide_rotate: bool = True,
        ignore_sec: Optional[float] = None,
        event_tag: Optional[str] = None,
        stt_cmd_on_exit: Optional[str] = None,
    ):
        self._cancel_dialog_timeout()

        with self._lock:
            self._dialog_mode = None
            self._dialog_dir = None
            self._dialog_retry = 0
            self._mode = "idle"
            self._armed = False
            self._pending_listen_beep = False

            if ignore_sec is None:
                ignore_sec = self.post_command_ignore_sec
            self._ignore_until_ts = time.time() + float(ignore_sec)

            self._wake_timeout_deadline_ts = None
            self._wake_timeout_reason = "wake"

        self._set_speech_armed(False)

        if stt_cmd_on_exit:
            self._send_stt_control(stt_cmd_on_exit)

        if hide_pos:
            self.gui_cmd("HIDE_POS")
        if hide_scroll:
            self.gui_cmd("HIDE_SCROLL")
        if hide_rotate:
            self.gui_cmd("HIDE_ROTATE")

        self.gui_cmd(f"SET_STATUS:{status}")

        if event_tag:
            self._event(event_tag)

        self._dbg(f"exit_dialog_to_idle status='{status}' ignore={ignore_sec}")

    def _enter_await_command(
        self,
        *,
        status: str = " Listening... (พูดคำสั่งได้เลย)",
        beep_wake: bool = False,
        beep_listen: bool = False,
        start_wake_timeout: bool = True,
        wake_timeout_override: Optional[float] = None
    ):
        with self._lock:
            self._mode = "await_command"
            self._armed = True
            self._dialog_mode = None
            self._dialog_dir = None
            self._dialog_retry = 0
            self._pending_listen_beep = bool(beep_listen)

            tts_busy_now = (self._tts_busy_last is True)

            if start_wake_timeout:
                tout = float(wake_timeout_override) if wake_timeout_override is not None else self.wake_timeout_sec
                self._wake_timeout_deadline_ts = time.time() + tout
                self._wake_timeout_reason = "manual" if wake_timeout_override is not None else "wake"
            else:
                self._wake_timeout_deadline_ts = None
                self._wake_timeout_reason = "wake"

        self._set_speech_armed(True)

        if beep_wake:
            self.beep("WAKE")

        if beep_listen and self.beep_enabled:
            with self._lock:
                from_wake = self._wake_recent  
            if not from_wake:                   
                if tts_busy_now:
                    self._schedule_beep_on_resume("LISTEN")
                else:
                    self._schedule_beep("LISTEN", self.listen_beep_delay_sec)
        self.gui_cmd(f"SET_STATUS:{status}")
        self._dbg(
            f"enter_await_command beep_wake={beep_wake} beep_listen={beep_listen} start_wake_timeout={start_wake_timeout}"
        )

    def _finish_await_command_to_idle(self, *, status: str = " Idle: รอคำว่า 'สวัสดี'"):
        with self._lock:
            self._mode = "idle"
            self._armed = False
            self._pending_listen_beep = False
            self._ignore_until_ts = time.time() + self.post_command_ignore_sec
            self._wake_timeout_deadline_ts = None
            self._wake_timeout_reason = "wake"

        self._set_speech_armed(False)

        self.gui_cmd(f"SET_STATUS:{status}")
        self._dbg("await_command -> idle")

    def _reset_to_idle(self, status: str = " IDLE"):
        with self._lock:
            self._resume_after_tts_deadline_ts = None
            self._beep_deadline_ts = None
            self._wake_timeout_deadline_ts = None
            self._wake_timeout_reason = "wake"

        self._send_stt_control("RESET")

        self._exit_dialog_to_idle(
            status=status,
            hide_pos=True,
            hide_scroll=True,
            hide_rotate=True,
            ignore_sec=self.post_command_ignore_sec,
            event_tag="STATE:RESET_TO_IDLE",
        )
        self.gui_cmd("SET_RESULT:Text: (cancelled)")
        self._dbg(f"reset_to_idle status='{status}'")

    def _switch_dialog(self, dialog_mode: str, dialog_dir: Optional[str], *, timeout_reason: str, reset_retry: bool = True):
        with self._lock:
            self._dialog_mode = dialog_mode
            self._dialog_dir = dialog_dir
            if reset_retry:
                self._dialog_retry = 0
        self._start_dialog_timeout(reason=timeout_reason)
        self._dbg(f"switch_dialog mode={dialog_mode} dir={dialog_dir} reason={timeout_reason}")

    # ---------------- ignore / wake helpers ----------------
    def _should_soft_ignore(self, text: str) -> bool:
        # contains (กัน STT แปลงไม่ตรงเป๊ะ)
        t = normalize_thai(text).lower()
        for p in self.ignore_phrases_soft:
            pp = normalize_thai(p).lower()
            if pp and (pp in t):
                return True
        return False

    def _is_global_cancel(self, text: str) -> bool:
        t = normalize_thai(text).lower()
        words = sorted(self.global_cancel_words, key=len, reverse=True)
        return any(w in t for w in words)


    def _should_hard_ignore(self, text: str) -> bool:
        t = normalize_thai(text).lower()
        return any(p == t for p in self.ignore_phrases_hard)

    # ---------------- dialog finalize/cancel helpers ----------------
    def _finalize_phrase(
        self,
        phrase: str,
        *,
        hide_pos: bool = False,
        hide_scroll: bool = False,
        hide_rotate: bool = False,
        status: str = " Idle: รอคำว่า 'สวัสดี'",
        event_tag: str = "DIALOG:FINALIZE",
    ):
        phrase = normalize_thai(phrase)
        if not phrase:
            return

        self.publish_text_raw(phrase)
        self.publish_feedback(f"ส่งให้ NLU แล้ว: {phrase}")
        self.gui_cmd(f"SET_RESULT:Text: {phrase}")
        self._event(event_tag)
        self._dbg(f"finalize phrase='{phrase}' tag='{event_tag}'")

        self._exit_dialog_to_idle(
            status=status,
            hide_pos=hide_pos,
            hide_scroll=hide_scroll,
            hide_rotate=hide_rotate,
            ignore_sec=self.post_command_ignore_sec,
        )

    def _cancel_dialog(
        self,
        *,
        reason: str = "cancel",
        hide_pos: bool = True,
        hide_scroll: bool = True,
        hide_rotate: bool = True,
        tts: str = "ยกเลิกค่ะ",
        status: str = " Idle: ยกเลิก",
        event_tag: str = "DIALOG:CANCEL",
        stt_cmd: str = "RESET",
    ):
        if stt_cmd:
            self._send_stt_control(stt_cmd)

        self.tts_request(tts)
        self._event(event_tag)
        self._dbg(f"cancel dialog reason='{reason}' tag='{event_tag}' stt_cmd='{stt_cmd}'")

        self._exit_dialog_to_idle(
            status=status,
            hide_pos=hide_pos,
            hide_scroll=hide_scroll,
            hide_rotate=hide_rotate,
            ignore_sec=self.post_command_ignore_sec,
        )

    def _retry_or_cancel(
        self,
        *,
        prompt_tts: str,
        retry_event: str,
        cancel_event: str,
        hide_pos_on_cancel: bool = True,
        hide_scroll_on_cancel: bool = True,
        hide_rotate_on_cancel: bool = True,
    ) -> bool:
        with self._lock:
            cur = self._dialog_retry

        if cur < self.dialog_max_retry:
            with self._lock:
                self._dialog_retry += 1
                cur2 = self._dialog_retry

            self._event(retry_event)
            self._dbg(f"dialog retry -> {cur2}")

            self.tts_request(prompt_tts)
            return True

        self._cancel_dialog(
            reason="max_retry",
            hide_pos=hide_pos_on_cancel,
            hide_scroll=hide_scroll_on_cancel,
            hide_rotate=hide_rotate_on_cancel,
            tts="ยกเลิกค่ะ",
            status=" Idle: ยกเลิก",
            event_tag=cancel_event,
            stt_cmd="RESET",
        )
        return False

    # ---------------- tick scheduler ----------------
    def _on_tick(self):
        now = time.time()

        # --- RESUME allowed? (do not return whole tick) ---
        resume_blocked = False
        with self._lock:
            if now < self._tts_guard_until_ts:
                resume_blocked = True
            elif self._tts_busy_last is True:
                resume_blocked = True
            elif now < getattr(self, "_resume_guard_until_ts", 0.0):
                resume_blocked = True

        # ---- dialog timeout (ทำจริง) ----
        dialog_fire = False
        dialog_reason = "dialog_wait"
        with self._lock:
            dl = self._dialog_deadline_ts
            tok = self._dialog_deadline_token
            cur_tok = self._dialog_timeout_token
            in_dialog = (self._dialog_mode is not None)

            if in_dialog and dl is not None and now >= dl and tok == cur_tok:
                dialog_fire = True
                dialog_reason = str(self._dialog_deadline_reason or "dialog_wait")
                # กันยิงซ้ำ
                self._dialog_deadline_ts = None

        if dialog_fire:
            self._event(f"DIALOG:TIMEOUT:{dialog_reason}")
            self._cancel_dialog(
                reason="timeout",
                hide_pos=True,
                hide_scroll=True,
                hide_rotate=True,
                tts="หมดเวลาค่ะ",
                status=" Idle: Timeout",
                event_tag=f"DIALOG:CANCEL:TIMEOUT:{dialog_reason}",
                stt_cmd="RESET",
            )
            return

        # ---- beep schedule ----
        do_beep = False
        beep_kind = "LISTEN"
        with self._lock:
            if self._beep_deadline_ts is not None and now >= self._beep_deadline_ts:
                do_beep = True
                beep_kind = self._beep_kind
                self._beep_deadline_ts = None

        if do_beep:
            self.beep(beep_kind)
            self._dbg(f"beep {beep_kind} (tick)")

        # ---- wake cooldown end ----
        end_cooldown = False
        with self._lock:
            if self._wake_cooldown_until_ts is not None and now >= self._wake_cooldown_until_ts:
                self._wake_cooldown_until_ts = None
                self._wake_recent = False
                end_cooldown = True

        if end_cooldown:
            self._dbg("wake cooldown ended (tick)")

        # ---- wake timeout ----
        fire_wake_timeout = False
        reason = "wake"
        with self._lock:
            if self._wake_timeout_deadline_ts is not None and now >= self._wake_timeout_deadline_ts:
                reason = getattr(self, "_wake_timeout_reason", "wake")
                if self._dialog_mode is None and self._mode == "await_command":
                    fire_wake_timeout = True
                    self._mode = "idle"
                    self._armed = False
                    self._pending_listen_beep = False
                self._wake_timeout_deadline_ts = None
                self._wake_timeout_reason = "wake"

        # ---- auto disarm after wake 6s ----
        if self._wake_auto_disarm_deadline is not None:
            if time.time() >= self._wake_auto_disarm_deadline:
                self._set_speech_armed(False)
                self._wake_auto_disarm_deadline = None
                self._event("WAKE:AUTO_DISARM")
                self._dbg("auto disarm after 6s")


        if fire_wake_timeout:
            self._set_speech_armed(False)

            if reason == "manual":
                self.gui_cmd("SET_STATUS: Idle: Manual timeout")
                self._event("MANUAL:TIMEOUT")
                self._dbg("manual timeout -> back to IDLE (tick)")
            else:
                self.gui_cmd("SET_STATUS: Idle: Wake timeout")
                self._event("WAKE:TIMEOUT")
                self._dbg("wake timeout -> back to IDLE (tick)")

            self.gui_cmd("SET_RESULT:Text: (timeout)")
            self.gui_cmd("HIDE_POS"); self.gui_cmd("HIDE_SCROLL"); self.gui_cmd("HIDE_ROTATE")

        # ---- resume after tts delay ----
        do_resume = False
        resume_reason = "tts_done"
        with self._lock:
            if self._resume_after_tts_deadline_ts is not None and now >= self._resume_after_tts_deadline_ts:
                do_resume = True
                resume_reason = self._resume_after_tts_reason
                self._resume_after_tts_deadline_ts = None

        if do_resume:
            if resume_blocked:
                with self._lock:
                    self._resume_after_tts_deadline_ts = time.time() + 0.10
                self._dbg("do_resume deferred: resume_blocked -> +0.10s")
            else:
                with self._lock:
                    mode = self._mode
                    in_dialog = (self._dialog_mode is not None)

                if (mode == "await_command") or in_dialog:
                    self._send_stt_control("RESUME")

                    do_beep2 = False
                    beep_kind2 = "WAKE"
                    with self._lock:
                        if self._beep_on_next_resume:
                            do_beep2 = True
                            beep_kind2 = self._beep_on_next_resume_kind
                            self._beep_on_next_resume = False

                    if do_beep2 and self.beep_enabled:
                        self.beep(beep_kind2)

                    self._dbg(f"RESUME after delay reason={resume_reason} beep={do_beep2} kind={beep_kind2}")
                else:
                    self._dbg(f"skip RESUME (not listening) mode={mode} in_dialog={in_dialog} reason={resume_reason}")

        # ---- confirm TTS done (busy false stable) ----
        fire_done = False
        with self._lock:
            dl = self._tts_fall_deadline_ts
            if dl is not None and now >= dl:
                if self._tts_busy_last is False:
                    fire_done = True
                self._tts_fall_deadline_ts = None

        if fire_done:
            with self._lock:
                tail = float(self.post_tts_ignore_sec)
                self._resume_guard_until_ts = time.time() + tail
                self._tts_guard_until_ts = max(self._tts_guard_until_ts, time.time() + tail)

            self._publish_tts_done()
            self._reset_stt_after_tts_done()
            self._handle_tts_finish()

            if self.stt_pause_while_tts:
                self._schedule_resume_after_tts(reason="tts_done")

    # ---------------- feedback request ----------------
    def on_feedback_request(self, msg: String):
        text = normalize_thai(msg.data or "")
        if not text:
            return
        self.publish_feedback(text)
        self._dbg(f"feedback_request passthrough text='{text}'")

    # ---------------- TTS finish handler ----------------
    def _handle_tts_finish(self):
        with self._lock:
            self._ignore_until_ts = max(self._ignore_until_ts, time.time() + self.post_tts_ignore_sec)
            do_beep = self._pending_listen_beep
            self._pending_listen_beep = False
            in_dialog = (self._dialog_mode is not None)

        self._event("TTS:DONE")
        self._dbg(f"tts_finish -> pending_beep={do_beep} in_dialog={in_dialog}")

        if in_dialog:
            self._start_dialog_timeout(reason="after_tts_done")

        if do_beep and self.beep_enabled:
            self._schedule_beep("LISTEN", self.listen_beep_delay_sec)
            self._dbg("beep LISTEN scheduled (after TTS done)")

    # ---------------- TTS busy -> confirm done in tick (single source) ----------------
    def on_tts_busy(self, msg: Bool):
        now = time.time()
        busy = bool(msg.data)

        with self._lock:
            last = self._tts_busy_last
            self._tts_busy_last = busy

            if busy:
                self._tts_fall_deadline_ts = None
                self._tts_guard_until_ts = max(self._tts_guard_until_ts, now + 3.0)

                if self.stt_pause_while_tts and last is not True:
                    self._send_stt_control("PAUSE")
                    self._resume_after_tts_deadline_ts = None
            else:
                self._tts_fall_deadline_ts = now + float(self.tts_busy_fall_confirm_sec)

        if self.debug:
            self._dbg(f"tts_busy={busy} last={last} fall_deadline={self._tts_fall_deadline_ts}")

    # ---------------- dialog request from NLU ----------------
    def on_dialog_request(self, msg: String):
        data = (msg.data or "").strip()
        if not data:
            return

        self._event(f"DIALOG:REQUEST:{data}")
        self._dbg(f"dialog_request='{data}'")

        if data == "ASK_POS":
            self._enter_dialog(
                "await_pos", None,
                show_pos=True,
                status=" Listening... (พูดเลข 1-5 ได้เลย)",
                timeout_reason="ask_pos",
            )
            return

        if data.startswith("ASK_VIEW_POS"):
            view_kind = data.split(":", 1)[1].strip().lower() if ":" in data else None
            self._enter_dialog(
                "await_view_pos", view_kind,
                show_pos=True,
                status=" Listening... (พูดเลข 1-5 ได้เลย)",
                timeout_reason="ask_view_pos",
            )
            return

        if data == "ASK_SCROLL_DIR":
            self._enter_dialog(
                "await_scroll_dir", None,
                show_scroll=True,
                status=" Listening... (บอกขึ้นหรือลง)",
                timeout_reason="ask_scroll_dir",
            )
            return

        if data.startswith("ASK_SCROLL_DIST:"):
            direction = data.split(":", 1)[1].strip()
            self._enter_dialog(
                "await_scroll_dist", direction,
                show_scroll=True,
                status=" Listening... (พูดระยะ เช่น 10 เซน หรือ 0.1 เมตร)",
                timeout_reason="ask_scroll_dist",
            )
            return

        if data == "ASK_ROTATE_DIR":
            self._enter_dialog(
                "await_rotate_dir", None,
                show_rotate=True,
                status=" Listening... (บอกซ้ายหรือขวา)",
                timeout_reason="ask_rotate_dir",
            )
            return

        if data.startswith("ASK_ROTATE_DEG:"):
            direction = data.split(":", 1)[1].strip()
            self._enter_dialog(
                "await_rotate_deg", direction,
                show_rotate=True,
                status=" Listening... (พูดจำนวนองศาหมุนได้เลย)",
                timeout_reason="ask_rotate_deg",
            )
            return

        if data == "ASK_W3_DIR":
            self._enter_dialog(
                "await_w3_dir", None,
                show_rotate=True,
                status=" Listening... (บอกซ้ายหรือขวา สำหรับข้อมือสาม)",
                timeout_reason="ask_w3_dir",
            )
            return

        if data.startswith("ASK_W3_ROTATE_DEG:"):
            direction = data.split(":", 1)[1].strip()
            self._enter_dialog(
                "await_w3_deg", direction,
                show_rotate=True,
                status=" Listening... (พูดจำนวนองศาข้อมือสาม)",
                timeout_reason="ask_w3_deg",
            )
            return

        if data == "ASK_MOVE_DIR":
            self._enter_dialog(
                "await_move_dir", None,
                status=" Listening... (บอกทิศ: ซ้าย/ขวา/หน้า/หลัง/ขึ้น/ลง)",
                timeout_reason="ask_move_dir",
            )
            return

        if data.startswith("ASK_MOVE_DIST:"):
            direction = data.split(":", 1)[1].strip()
            self._enter_dialog(
                "await_move_dist", direction,
                status=" Listening... (บอกระยะ เช่น 10 เซน หรือ 0.1 เมตร)",
                timeout_reason="ask_move_dist",
            )
            return

        self._dbg(f"unknown dialog_request '{data}' (ignored)")

    # ---------------- GUI event (from GUI) ----------------
    def on_gui_event(self, msg: String):
        data = (msg.data or "").strip()
        if not data:
            return

        self._dbg(f"gui_event='{data}'")

        if data == "MANUAL_ARM":
            with self._lock:
                tts_busy_now = (self._tts_busy_last is True)

            if tts_busy_now:
                with self._lock:
                    self._beep_on_next_resume = False
                    self._pending_manual_arm = True
                    self._ignore_until_ts = max(self._ignore_until_ts, time.time() + self.post_tts_ignore_sec)
                    self._wake_timeout_deadline_ts = None
                    self._beep_deadline_ts = None
                    self._pending_listen_beep = False
                self._schedule_beep_on_resume(self.beep_on_resume_kind)
                self._send_stt_control("PAUSE")
                self._set_speech_armed(False)

                self.gui_cmd("SET_STATUS: Waiting... (TTS speaking)")
                self._event("GUI:MANUAL_ARM:PENDING_TTS")
                self._dbg("manual arm requested while TTS busy -> pending until TTS done")
                return

            with self._lock:
                self._beep_on_next_resume = False
                self._ignore_until_ts = 0.0
                self._wake_timeout_deadline_ts = None
                self._beep_deadline_ts = None
                self._pending_listen_beep = False

            self._send_stt_control("PAUSE")
            self._schedule_beep_on_resume(self.beep_on_resume_kind)
            self._schedule_resume_after_tts(reason="manual_arm")

            self._enter_await_command(
                status=" Listening... (Manual)",
                beep_wake=False,
                beep_listen=False,
                start_wake_timeout=True,
                wake_timeout_override=self.manual_timeout_sec,
            )
            self._event("GUI:MANUAL_ARM")
            return

        if data == "MANUAL_CANCEL":
            with self._lock:
                self._pending_manual_arm = False
                self._resume_after_tts_deadline_ts = None
            self._event("GUI:MANUAL_CANCEL")
            self._send_stt_control("MANUAL_CANCEL")
            self._set_speech_armed(False)
            self._send_stt_control("PAUSE")
            self._reset_to_idle(status=" IDLE (Manual cancelled)")
            return

        if data.startswith("TEXT:"):
            cmd_text = data.split(":", 1)[1].strip()
            if cmd_text:
                t = normalize_thai(cmd_text)
                self.gui_cmd(f"SET_RESULT:Text: {t}")
                self.publish_text_raw(t)
                self.publish_feedback(f"ส่งให้ NLU แล้ว: {t}")
                self._event("GUI:SEND_TEXT")
                self._dbg(f"send_to_nlu (gui text) text='{t}'")
            return

        if data.startswith("POS:"):
            try:
                k = int(data.split(":", 1)[1])
            except Exception:
                return
            if k not in (1, 2, 3, 4, 5):
                return
            self._event(f"GUI:POS:{k}")

            with self._lock:
                dm = self._dialog_mode
                vk = self._dialog_dir

            if dm == "await_view_pos":
                phrase = (
                    f"มุมมองบน ตำแหน่งที่ {k}" if vk == "top" else
                    f"มุมมองข้าง ตำแหน่งที่ {k}" if vk == "side" else
                    f"ตำแหน่งที่ {k}"
                )
                self._finalize_phrase(
                    phrase,
                    hide_pos=True,
                    status=" Idle: รอคำว่า 'สวัสดี'",
                    event_tag=f"DIALOG:FINALIZE:VIEW_POS:{vk or 'unknown'}:{k}",
                )
            else:
                self._finalize_phrase(
                    f"ตำแหน่งที่ {k}",
                    hide_pos=True,
                    status=" Idle: รอคำว่า 'สวัสดี'",
                    event_tag=f"DIALOG:FINALIZE:POS:{k}",
                )
            return

        if data.startswith("SCROLL:"):
            direction = data.split(":", 1)[1].strip()
            if direction in ("up", "down"):
                self._event(f"GUI:SCROLL:{direction}:ASK_DIST")
                self._enter_dialog(
                    "await_scroll_dist", direction,
                    show_scroll=True,
                    status="Listening... (พูดระยะ เช่น 10 เซน หรือ 0.1 เมตร)",
                    timeout_reason="gui_scroll",
                )
            return

        if data.startswith("ROTATE:"):
            direction = data.split(":", 1)[1].strip()
            if direction in ("left", "right"):
                self._event(f"GUI:ROTATE:{direction}:ASK_DEG")
                self._enter_dialog(
                    "await_rotate_deg", direction,
                    show_rotate=True,
                    status=" Listening... (พูดจำนวนองศาหมุนได้เลย)",
                    timeout_reason="gui_rotate",
                )
            return

    # ---------------- main input: recognized text ----------------
    def on_heard_text(self, msg: String):
        text = msg.data or ""
        if not text.strip():
            return

        now = time.time()
        t_norm = normalize_thai(text)

        with self._lock:
            if now < getattr(self, "_tts_guard_until_ts", 0.0):
                self._dbg(f"drop: tts_guard text='{t_norm}'")
                return
            if self._tts_busy_last is True:
                self._dbg(f"drop: tts_busy text='{t_norm}'")
                return
            if now < getattr(self, "_resume_guard_until_ts", 0.0):
                self._dbg(f"drop: resume_guard text='{t_norm}'")
                return

            mode = self._mode
            armed = self._armed
            dialog_mode = self._dialog_mode
            dialog_dir = self._dialog_dir
            ignore_until = self._ignore_until_ts

        if self._is_global_cancel(t_norm):
            if dialog_mode is not None:
                self._cancel_dialog_timeout()
                self._event("GLOBAL:CANCEL:DIALOG")
                self._cancel_dialog(
                    reason="global_cancel",
                    hide_pos=True,
                    hide_scroll=True,
                    hide_rotate=True,
                    tts="ยกเลิกค่ะ",
                    status=" Idle: ยกเลิก",
                    event_tag="DIALOG:CANCEL:GLOBAL",
                    stt_cmd="CANCEL",
                )
                return

            if mode == "await_command":
                self._event("GLOBAL:CANCEL:AWAIT_COMMAND")
                self._send_stt_control("CANCEL")
                self.tts_request("ยกเลิกค่ะ")
                self._finish_await_command_to_idle(status=" Idle: ยกเลิก")
                self.gui_cmd("HIDE_POS")
                self.gui_cmd("HIDE_SCROLL")
                self.gui_cmd("HIDE_ROTATE")
                self.gui_cmd("SET_RESULT:Text: (cancelled)")
                return

            self._event("GLOBAL:CANCEL:IDLE")
            self.publish_feedback("อยู่ในโหมด Idle อยู่แล้วค่ะ")
            self.gui_cmd("SET_STATUS: Idle: รอคำว่า 'สวัสดี' หรือ 'Hello'")
            self.gui_cmd("SET_RESULT:Text: (idle)")
            self.gui_cmd("HIDE_POS"); self.gui_cmd("HIDE_SCROLL"); self.gui_cmd("HIDE_ROTATE")
            return

        if self._should_hard_ignore(t_norm):
            self._dbg(f"drop: hard_ignore text='{t_norm}'")
            return

        if now < ignore_until:
            self._dbg(f"drop: ignore_window dt={ignore_until - now:.2f}s text='{t_norm}'")
            return

        if self._should_soft_ignore(t_norm):
            self._dbg(f"drop: soft_ignore_phrase text='{t_norm}'")
            return

        if dialog_mode is not None:
            self._cancel_dialog_timeout()

        if self.debug_heard_when_active and (mode != "idle" or dialog_mode is not None):
            self.get_logger().info(f"[HEARD] {t_norm}")

        self._dbg(f"heard norm='{t_norm}' mode={mode} dialog={dialog_mode} armed={armed}")

        # ---- dialog: scroll dist ----
        if dialog_mode == "await_scroll_dist":
            dist_m = parse_distance_loose(t_norm)
            if dist_m is not None:
                d = dialog_dir if dialog_dir in ("up", "down") else "up"
                if dist_m < 1.0:
                    phrase = f"เลื่อน{'ขึ้น' if d=='up' else 'ลง'} {dist_m*100:g} เซน"
                else:
                    phrase = f"เลื่อน{'ขึ้น' if d=='up' else 'ลง'} {dist_m:g} เมตร"

                self._finalize_phrase(
                    phrase,
                    hide_scroll=True,
                    status=" Idle: รอคำว่า 'สวัสดี'",
                    event_tag=f"DIALOG:FINALIZE:SCROLL_DIST:{d}:{dist_m:g}",
                )
                return

            self._retry_or_cancel(
                prompt_tts="ยังไม่ชัดค่ะ กรุณาพูดระยะ เช่น 10 เซน หรือ 0.1 เมตร",
                retry_event="DIALOG:RETRY:SCROLL_DIST",
                cancel_event="DIALOG:CANCEL:SCROLL_DIST",
                hide_scroll_on_cancel=True,
            )
            self._start_dialog_timeout(reason="retry_scroll_dist")
            return

        # ---- dialog: pos ----
        if dialog_mode == "await_pos":
            k = parse_position_reply_loose(t_norm)
            if k is not None:
                self._finalize_phrase(
                    f"ตำแหน่งที่ {k}",
                    hide_pos=True,
                    status=" Idle: รอคำว่า 'สวัสดี'",
                    event_tag=f"DIALOG:FINALIZE:POS:{k}",
                )
                return

            if self._retry_or_cancel(
                prompt_tts="ยังไม่ชัดค่ะ กรุณาพูด 1 2 3 4 หรือ 5",
                retry_event="DIALOG:RETRY:POS",
                cancel_event="DIALOG:CANCEL:POS",
                hide_pos_on_cancel=True,
                hide_scroll_on_cancel=False,
                hide_rotate_on_cancel=False,
            ):
                self._start_dialog_timeout(reason="retry_pos")
                return
            return

        # ---- dialog: view pos ----
        if dialog_mode == "await_view_pos":
            k = parse_position_reply_loose(t_norm)
            if k is not None:
                vk = (dialog_dir or "").lower()
                if vk == "top":
                    phrase = f"มุมมองบน ตำแหน่งที่ {k}"
                elif vk == "side":
                    phrase = f"มุมมองข้าง ตำแหน่งที่ {k}"
                else:
                    phrase = f"ตำแหน่งที่ {k}"

                self._finalize_phrase(
                    phrase,
                    hide_pos=True,
                    status=" Idle: รอคำว่า 'สวัสดี'",
                    event_tag=f"DIALOG:FINALIZE:VIEW_POS:{vk or 'unknown'}:{k}",
                )
                return

            if self._retry_or_cancel(
                prompt_tts="ยังไม่ชัดค่ะ กรุณาพูด 1 2 3 4 หรือ 5",
                retry_event="DIALOG:RETRY:VIEW_POS",
                cancel_event="DIALOG:CANCEL:VIEW_POS",
                hide_pos_on_cancel=True,
                hide_scroll_on_cancel=False,
                hide_rotate_on_cancel=False,
            ):
                self._start_dialog_timeout(reason="retry_view_pos")
                return
            return

        # ---- dialog: rotate dir ----
        if dialog_mode == "await_rotate_dir":
            d = parse_direction_reply_loose(t_norm)
            if d in ("left", "right"):
                phrase = "หมุนซ้าย" if d == "left" else "หมุนขวา"
                self._finalize_phrase(
                    phrase,
                    hide_rotate=True,
                    status=" Idle: รอคำว่า 'สวัสดี'",
                    event_tag=f"DIALOG:FINALIZE:ROTATE_DIR:{d}",
                )
                return

            if self._retry_or_cancel(
                prompt_tts="ยังไม่ชัดค่ะ กรุณาพูด ซ้าย หรือ ขวา",
                retry_event="DIALOG:RETRY:ROTATE_DIR",
                cancel_event="DIALOG:CANCEL:ROTATE_DIR",
                hide_pos_on_cancel=False,
                hide_scroll_on_cancel=False,
                hide_rotate_on_cancel=True,
            ):
                self._start_dialog_timeout(reason="retry_rotate_dir")
                return
            return

        # ---- dialog: rotate deg ----
        if dialog_mode == "await_rotate_deg":
            deg = parse_degrees_loose(t_norm)
            if deg is not None:
                d = (dialog_dir or "left").strip().lower()
                d = d if d in ("left", "right") else "left"
                phrase = f"หมุน{'ซ้าย' if d=='left' else 'ขวา'} {deg:g} องศา"
                self._finalize_phrase(
                    phrase,
                    hide_rotate=True,
                    status=" Idle: รอคำว่า 'สวัสดี'",
                    event_tag=f"DIALOG:FINALIZE:ROTATE_DEG:{d}:{deg:g}",
                )
                return

            if self._retry_or_cancel(
                prompt_tts="ยังไม่ชัดค่ะ กรุณาพูดจำนวนองศา เช่น 15 หรือ 20 องศา",
                retry_event="DIALOG:RETRY:DEG",
                cancel_event="DIALOG:CANCEL:DEG",
                hide_pos_on_cancel=False,
                hide_scroll_on_cancel=False,
                hide_rotate_on_cancel=True,
            ):
                self._start_dialog_timeout(reason="retry_rotate_deg")
                return
            return

        # ---- dialog: scroll dir -> chain to dist ----
        if dialog_mode == "await_scroll_dir":
            d = parse_direction_reply_loose(t_norm)
            if d in ("up", "down"):
                self.gui_cmd("SHOW_SCROLL")
                self.gui_cmd("SET_STATUS: Listening... (พูดระยะ เช่น 10 เซน หรือ 0.1 เมตร)")
                self.tts_request("ต้องการระยะเท่าไร เช่น 10 เซน หรือ 0.1 เมตร")
                self._event(f"DIALOG:CHAIN:SCROLL_DIR->{d}:ASK_DIST")
                self._switch_dialog("await_scroll_dist", d, timeout_reason="chain_scroll_dir_to_dist", reset_retry=True)
                return

            if self._retry_or_cancel(
                prompt_tts="ยังไม่ชัดค่ะ กรุณาพูด ขึ้น หรือ ลง",
                retry_event="DIALOG:RETRY:SCROLL_DIR",
                cancel_event="DIALOG:CANCEL:SCROLL_DIR",
                hide_pos_on_cancel=False,
                hide_scroll_on_cancel=True,
                hide_rotate_on_cancel=False,
            ):
                self._start_dialog_timeout(reason="retry_scroll_dir")
                return
            return

        # ---- dialog: w3 dir ----
        if dialog_mode == "await_w3_dir":
            d = parse_direction_reply_loose(t_norm)
            if d in ("left", "right"):
                phrase = "ข้อมือสาม ซ้าย" if d == "left" else "ข้อมือสาม ขวา"
                self._finalize_phrase(
                    phrase,
                    hide_rotate=True,
                    status=" Idle: รอคำว่า 'สวัสดี'",
                    event_tag=f"DIALOG:FINALIZE:W3_DIR:{d}",
                )
                return

            if self._retry_or_cancel(
                prompt_tts="ยังไม่ชัดค่ะ กรุณาพูด ซ้าย หรือ ขวา",
                retry_event="DIALOG:RETRY:W3_DIR",
                cancel_event="DIALOG:CANCEL:W3_DIR",
                hide_pos_on_cancel=False,
                hide_scroll_on_cancel=False,
                hide_rotate_on_cancel=True,
            ):
                self._start_dialog_timeout(reason="retry_w3_dir")
                return
            return

        # ---- dialog: w3 deg ----
        if dialog_mode == "await_w3_deg":
            deg = parse_degrees_loose(t_norm)
            if deg is not None:
                d = (dialog_dir or "left").strip().lower()
                d = d if d in ("left", "right") else "left"
                phrase = f"ข้อมือสาม {'ซ้าย' if d=='left' else 'ขวา'} {deg:g} องศา"
                self._finalize_phrase(
                    phrase,
                    hide_rotate=True,
                    status=" Idle: รอคำว่า 'สวัสดี'",
                    event_tag=f"DIALOG:FINALIZE:W3_DEG:{d}:{deg:g}",
                )
                return

            if self._retry_or_cancel(
                prompt_tts="ยังไม่ชัดค่ะ กรุณาพูดจำนวนองศา เช่น 5 หรือ 20 องศา",
                retry_event="DIALOG:RETRY:W3_DEG",
                cancel_event="DIALOG:CANCEL:W3_DEG",
                hide_pos_on_cancel=False,
                hide_scroll_on_cancel=False,
                hide_rotate_on_cancel=True,
            ):
                self._start_dialog_timeout(reason="retry_w3_deg")
                return
            return

        # ---- dialog: move dir ----
        if dialog_mode == "await_move_dir":
            d = parse_direction_reply_loose(t_norm)
            if d in ("left", "right", "forward", "back", "up", "down"):
                phrase = f"ขยับ{_dir_th(d)}"
                self._finalize_phrase(
                    phrase,
                    status=" Idle: รอคำว่า 'สวัสดี'",
                    event_tag=f"DIALOG:FINALIZE:MOVE_DIR:{d}",
                )
                return

            if self._retry_or_cancel(
                prompt_tts="ยังไม่ชัดค่ะ กรุณาพูด ซ้าย ขวา หน้า หลัง ขึ้น หรือ ลง",
                retry_event="DIALOG:RETRY:MOVE_DIR",
                cancel_event="DIALOG:CANCEL:MOVE_DIR",
                hide_pos_on_cancel=False,
                hide_scroll_on_cancel=False,
                hide_rotate_on_cancel=False,
            ):
                self._start_dialog_timeout(reason="retry_move_dir")
                return
            return

        # ---- dialog: move dist ----
        if dialog_mode == "await_move_dist":
            dist_m = parse_distance_loose(t_norm)
            if dist_m is not None:
                d = (dialog_dir or "up").strip().lower()
                d = d if d in ("left", "right", "forward", "back", "up", "down") else "up"
                if dist_m < 1.0:
                    phrase = f"ขยับ{_dir_th(d)} {dist_m*100:g} เซน"
                else:
                    phrase = f"ขยับ{_dir_th(d)} {dist_m:g} เมตร"

                self._finalize_phrase(
                    phrase,
                    status=" Idle: รอคำว่า 'สวัสดี'",
                    event_tag=f"DIALOG:FINALIZE:MOVE_DIST:{d}:{dist_m:g}",
                )
                return

            if self._retry_or_cancel(
                prompt_tts="ยังไม่ชัดค่ะ กรุณาพูดระยะ เช่น 10 เซน หรือ 0.1 เมตร",
                retry_event="DIALOG:RETRY:MOVE_DIST",
                cancel_event="DIALOG:CANCEL:MOVE_DIST",
                hide_pos_on_cancel=False,
                hide_scroll_on_cancel=False,
                hide_rotate_on_cancel=False,
            ):
                self._start_dialog_timeout(reason="retry_move_dist")
                return
            return

        # ---------------- await command (armed) ----------------
        if mode == "await_command":
            if not armed:
                self._dbg("await_command but not armed -> drop")
                return

            self.gui_cmd(f"SET_RESULT:Text: {t_norm}")
            self.publish_text_raw(t_norm)
            self.publish_feedback(f"ส่งให้ NLU แล้ว: {t_norm}")

            self._event("NLU:SEND_TEXT_RAW")
            self._dbg(f"send_to_nlu text='{t_norm}' (await_command)")

            self._finish_await_command_to_idle(status=" Idle: รอคำว่า 'สวัสดี'")
            return
       
    def on_wake_detected(self, msg: Bool):
        if not msg.data:
            return  

        now = time.time()
        self._event("WAKE:DETECTED_FROM_STT")
        self._enter_await_command(
            status=" Listening... (พูดคำสั่งได้เลย)",
            beep_wake=True,
            beep_listen=False,
            start_wake_timeout=False
        )

        self._wake_auto_disarm_deadline = now + 6.0

def main(args=None):
    rclpy.init(args=args)
    node = DialogFSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
