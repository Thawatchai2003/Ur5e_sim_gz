#!/usr/bin/env python3
import threading
import math
import time
import queue
from array import array
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import simpleaudio as sa
except Exception:
    sa = None


class BeepNode(Node):
    """
    Beep Node (with debug + event style like Dialog)

    Sub:
      - /voice/beep (String)          : request beep pattern

    Pub:
      - /voice/beep_event (String)    : event back to FSM/Dialog
      - /voice/beep_debug (String)    : optional debug topic

    Payload examples (incoming /voice/beep):
      - "BEEP" / "WAKE" / "LISTEN" / "OK"        -> double beep
      - "ERROR" / "FAIL"                          -> error beep
      - "DOUBLE"                                  -> double beep
      - "ERROR:mic_fail"                          -> error beep + tag
      - "CUSTOM:freq=900,dur=0.07,vol=0.25"       -> single tone
      - "SEQ:DOUBLE,ERROR,DOUBLE"                 -> play sequence
      - "STOP"                                    -> clear queue (soft)
    """

    def __init__(self):
        super().__init__("beep_node")

        # ---------------- parameters (DEBUG first) ----------------
        self.declare_parameter("debug", True)
        self.declare_parameter("debug_rate_hz", 12.0)     # max debug lines per sec
        self.declare_parameter("debug_to_topic", True)    # publish to topic_beep_debug
        self.declare_parameter("debug_verbose", False)    # allow PUB/SUB spam logs

        # ---------------- topics ----------------
        self.declare_parameter("topic_beep_sub", "control/beep")
        self.declare_parameter("topic_beep_event", "sound/beep_event")
        self.declare_parameter("topic_beep_debug", "sound/beep_debug")

        # read debug params
        self.debug = bool(self.get_parameter("debug").value)
        self.debug_rate_hz = float(self.get_parameter("debug_rate_hz").value)
        self.debug_to_topic = bool(self.get_parameter("debug_to_topic").value)
        self.debug_verbose = bool(self.get_parameter("debug_verbose").value)

        # read topics
        self.topic_beep_sub = str(self.get_parameter("topic_beep_sub").value)
        self.topic_beep_event = str(self.get_parameter("topic_beep_event").value)
        self.topic_beep_debug = str(self.get_parameter("topic_beep_debug").value)

        # -----------------------------
        # Sound params (Google-ish presets)
        # -----------------------------
        self.declare_parameter("enabled", True)
        self.declare_parameter("sr", 44100)

        self.declare_parameter("freq1", 880.0)   # A5
        self.declare_parameter("freq2", 1320.0)  # E6

        self.declare_parameter("dur1", 0.060)
        self.declare_parameter("gap", 0.025)
        self.declare_parameter("dur2", 0.085)

        self.declare_parameter("volume", 0.22)
        self.declare_parameter("fade_sec", 0.010)

        self.declare_parameter("sweep2_pct", 0.04)
        self.declare_parameter("sweep2_enable", True)

        self.declare_parameter("error_freq", 440.0)
        self.declare_parameter("error_dur", 0.18)
        self.declare_parameter("error_volume_scale", 0.80)

        # NEW: queue + concurrency
        self.declare_parameter("queue_size", 8)
        self.declare_parameter("drop_if_busy", False)  # if True: ignore when playing
        self.declare_parameter("event_emit_done", True)

        # Read sound params
        self.enabled = bool(self.get_parameter("enabled").value)
        self.sr = int(self.get_parameter("sr").value)

        self.freq1 = float(self.get_parameter("freq1").value)
        self.freq2 = float(self.get_parameter("freq2").value)

        self.dur1 = float(self.get_parameter("dur1").value)
        self.gap = float(self.get_parameter("gap").value)
        self.dur2 = float(self.get_parameter("dur2").value)

        self.volume = float(self.get_parameter("volume").value)
        self.fade_sec = float(self.get_parameter("fade_sec").value)

        self.sweep2_enable = bool(self.get_parameter("sweep2_enable").value)
        self.sweep2_pct = float(self.get_parameter("sweep2_pct").value)

        self.error_freq = float(self.get_parameter("error_freq").value)
        self.error_dur = float(self.get_parameter("error_dur").value)
        self.error_volume_scale = float(self.get_parameter("error_volume_scale").value)

        self.queue_size = int(self.get_parameter("queue_size").value)
        self.drop_if_busy = bool(self.get_parameter("drop_if_busy").value)
        self.event_emit_done = bool(self.get_parameter("event_emit_done").value)

        # ---------------- pubs/subs ----------------
        self.sub = self.create_subscription(String, self.topic_beep_sub, self.on_beep, 10)
        self.event_pub = self.create_publisher(String, self.topic_beep_event, 10)
        self.debug_pub = self.create_publisher(String, self.topic_beep_debug, 10)

        # ---------------- debug internals ----------------
        self._dbg_last_ts = 0.0
        self._dbg_min_dt = (1.0 / max(self.debug_rate_hz, 1e-6)) if self.debug_rate_hz > 0 else 0.0
        self._dbg_lock = threading.Lock()

        # ---------------- player worker ----------------
        self._q: "queue.Queue[Dict]" = queue.Queue(maxsize=max(1, self.queue_size))
        self._stop_flag = threading.Event()
        self._playing_lock = threading.Lock()
        self._is_playing = False
        self._player_thread = threading.Thread(target=self._player_loop, daemon=True)
        self._player_thread.start()

        if sa is None:
            self.get_logger().warning(
                "simpleaudio not available -> terminal bell fallback (\\a). "
                "Install: python3 -m pip install --user simpleaudio"
            )

        self._print_startup_banner()
        self._dbg("BeepNode ready")

    # ---------------- Debug helpers ----------------
    def _dbg(self, msg: str):
        if not self.debug:
            return
        now = time.time()
        with self._dbg_lock:
            if self._dbg_min_dt > 0 and (now - self._dbg_last_ts) < self._dbg_min_dt:
                return
            self._dbg_last_ts = now

        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"

        try:
            self.get_logger().info(line)
        except Exception:
            pass

        if self.debug_to_topic:
            try:
                self.debug_pub.publish(String(data=f"[DEBUG][beep_node] {line}"))
            except Exception:
                pass

    def _event(self, s: str):
        if not rclpy.ok():
            return
        try:
            self.event_pub.publish(String(data=s))
        except Exception:
            pass
        if self.debug_verbose:
            self._dbg(f"PUB -> {self.topic_beep_event}: '{s}'")

    def _print_startup_banner(self):
        GREEN = "\033[92m"
        RESET = "\033[0m"

        supported = [
            "Double beep + error beep presets",
            "Queue worker (avoid overlapping sounds)",
            "Debug: rate-limited + optional topic publish",
            "Event publish: PLAY_START / PLAY_DONE / PLAY_FAIL",
            "Payload parser: BEEP/ERROR/DOUBLE/CUSTOM/SEQ/STOP",
        ]

        banner = (
            "\n"
            "──────────────────────────────────────────────────────────────\n"
            "        Beep Node — Operational\n"
            "        Node State      : READY\n"
            f"        Debug Mode      : {'ENABLED' if self.debug else 'DISABLED'}\n"
            f"        Debug Topic     : {self.topic_beep_debug if (self.debug and self.debug_to_topic) else '(disabled)'}\n"
            "\n"
            "        Subscribed Topics:\n"
            f"            • {self.topic_beep_sub}   (/voice/beep requests)\n"
            "\n"
            "        Published Topics:\n"
            f"            • {self.topic_beep_event} (events)\n"
            f"            • {self.topic_beep_debug if (self.debug and self.debug_to_topic) else '(disabled)'}\n"
            "\n"
            "        Runtime Config:\n"
            f"            • enabled      = {self.enabled}\n"
            f"            • sr           = {self.sr}\n"
            f"            • volume       = {self.volume}\n"
            f"            • fade_sec     = {self.fade_sec}\n"
            f"            • queue_size   = {self.queue_size}\n"
            f"            • drop_if_busy = {self.drop_if_busy}\n"
            "\n"
            "        Supported Functions:\n"
            "            - " + "\n            - ".join(supported) + "\n"
            "──────────────────────────────────────────────────────────────\n"
        )

        try:
            self.get_logger().info(GREEN + banner + RESET)
        except Exception:
            pass

        if self.debug and self.debug_to_topic:
            try:
                self.debug_pub.publish(String(data="[DEBUG][beep_node] banner:\n" + banner))
            except Exception:
                pass

    # ---------------- sound helpers ----------------
    def _tone_pcm16(
        self,
        freq_hz: float,
        dur_sec: float,
        sr: int,
        volume: float,
        fade_sec: float,
        sweep_pct: float = 0.0,
        enable_sweep: bool = False,
    ) -> bytes:
        n = max(1, int(sr * max(0.0, float(dur_sec))))
        amp = int(32767 * max(0.0, min(float(volume), 1.0)))
        buf = array("h")

        fade_n = max(1, int(max(0.0, float(fade_sec)) * sr))

        # precompute for speed
        two_pi = 2.0 * math.pi

        for i in range(n):
            # envelope
            if i < fade_n:
                env = i / fade_n
            elif i > n - fade_n:
                env = max(0.0, (n - i) / fade_n)
            else:
                env = 1.0

            # sweep
            if enable_sweep and sweep_pct != 0.0:
                k = i / max(1, (n - 1))
                f = float(freq_hz) * (1.0 + float(sweep_pct) * k)
            else:
                f = float(freq_hz)

            t = i / sr
            s = math.sin(two_pi * f * t)
            buf.append(int(amp * env * s))

        return buf.tobytes()

    def _silence_pcm16(self, dur_sec: float, sr: int) -> bytes:
        n = max(1, int(sr * max(0.0, float(dur_sec))))
        return array("h", [0] * n).tobytes()

    def _play_pcm_blocking(self, pcm: bytes, sr: int) -> bool:
        """Blocking play. Return True if played OK, else False."""
        if not self.enabled:
            return True
        # preferred: simpleaudio
        try:
            if sa is not None:
                play_obj = sa.play_buffer(pcm, 1, 2, sr)  # mono, int16
                play_obj.wait_done()
                return True
        except Exception as e:
            self._dbg(f"beep play failed: {e}")

        # fallback: terminal bell (often silent)
        try:
            print("\a", end="", flush=True)
            time.sleep(0.05)
            return True
        except Exception:
            return False

    # ---------------- patterns (return pcm bytes) ----------------
    def _pcm_double(self) -> bytes:
        b1 = self._tone_pcm16(
            self.freq1, self.dur1, self.sr, self.volume,
            fade_sec=self.fade_sec,
            enable_sweep=False
        )
        b2 = self._tone_pcm16(
            self.freq2, self.dur2, self.sr, self.volume,
            fade_sec=self.fade_sec,
            sweep_pct=self.sweep2_pct,
            enable_sweep=self.sweep2_enable,
        )
        return b"".join([b1, self._silence_pcm16(self.gap, self.sr), b2])

    def _pcm_error(self) -> bytes:
        return self._tone_pcm16(
            self.error_freq,
            self.error_dur,
            self.sr,
            self.volume * self.error_volume_scale,
            fade_sec=self.fade_sec,
            enable_sweep=False,
        )

    def _pcm_custom(self, freq: float, dur: float, vol: float) -> bytes:
        return self._tone_pcm16(
            float(freq), float(dur), self.sr, float(vol),
            fade_sec=self.fade_sec,
            enable_sweep=False,
        )

    # ---------------- parser ----------------
    def _parse_kv(self, s: str) -> Dict[str, str]:
        out: Dict[str, str] = {}
        parts = [p.strip() for p in s.split(",") if p.strip()]
        for p in parts:
            if "=" in p:
                k, v = p.split("=", 1)
                out[k.strip().lower()] = v.strip()
        return out

    def _enqueue_job(self, kind: str, meta: Optional[Dict] = None):
        meta = meta or {}

        # drop_if_busy option
        if self.drop_if_busy:
            with self._playing_lock:
                if self._is_playing:
                    if self.debug_verbose:
                        self._dbg(f"drop_if_busy=True -> drop kind={kind}")
                    return

        job = {"kind": kind, "meta": meta, "ts": time.time()}

        # bounded queue: drop oldest
        if self._q.full():
            try:
                _ = self._q.get_nowait()
            except Exception:
                pass

        try:
            self._q.put_nowait(job)
        except Exception:
            pass

        if self.debug_verbose:
            self._dbg(f"ENQ kind={kind} meta={meta}")

    # ---------------- worker loop ----------------
    def _player_loop(self):
        while not self._stop_flag.is_set():
            try:
                job = self._q.get(timeout=0.2)
            except queue.Empty:
                continue

            kind = str(job.get("kind", "DOUBLE"))
            meta = job.get("meta", {}) or {}
            tag = str(meta.get("tag", ""))

            with self._playing_lock:
                self._is_playing = True

            self._event(f"BEEP_PLAY_START:{kind}{(':'+tag) if tag else ''}")

            ok = True
            try:
                if kind == "DOUBLE":
                    pcm = self._pcm_double()
                    ok = self._play_pcm_blocking(pcm, self.sr)

                elif kind == "ERROR":
                    pcm = self._pcm_error()
                    ok = self._play_pcm_blocking(pcm, self.sr)

                elif kind == "CUSTOM":
                    freq = float(meta.get("freq", 880.0))
                    dur = float(meta.get("dur", 0.06))
                    vol = float(meta.get("vol", self.volume))
                    pcm = self._pcm_custom(freq, dur, vol)
                    ok = self._play_pcm_blocking(pcm, self.sr)

                else:
                    # fallback
                    pcm = self._pcm_double()
                    ok = self._play_pcm_blocking(pcm, self.sr)

            except Exception as e:
                ok = False
                self._dbg(f"PLAY exception: {e}")

            if ok:
                if self.event_emit_done:
                    self._event(f"BEEP_PLAY_DONE:{kind}{(':'+tag) if tag else ''}")
            else:
                self._event(f"BEEP_PLAY_FAIL:{kind}{(':'+tag) if tag else ''}")

            with self._playing_lock:
                self._is_playing = False

    # ---------------- ROS callback ----------------
    def on_beep(self, msg: String):
        raw = (msg.data or "").strip()
        if not raw:
            return

        cmd = raw.strip()
        up = cmd.upper()

        if self.debug_verbose:
            self._dbg(f"SUB <- {self.topic_beep_sub}: '{cmd}'")

        # stop/clear queue
        if up == "STOP":
            # soft stop: clear queue; cannot interrupt simpleaudio already playing reliably
            n = 0
            try:
                while True:
                    self._q.get_nowait()
                    n += 1
            except Exception:
                pass
            self._event(f"BEEP_QUEUE_CLEARED:{n}")
            self._dbg(f"queue cleared {n}")
            return

        # sequence
        if up.startswith("SEQ:"):
            seq = cmd.split(":", 1)[1].strip()
            items = [x.strip() for x in seq.split(",") if x.strip()]
            if not items:
                return
            self._dbg(f"SEQ items={items}")
            for it in items:
                it_up = it.upper()
                if it_up in ("BEEP", "WAKE", "LISTEN", "OK", "DOUBLE"):
                    self._enqueue_job("DOUBLE")
                elif it_up in ("ERROR", "FAIL"):
                    self._enqueue_job("ERROR")
                else:
                    self._enqueue_job("DOUBLE")
            return

        # custom
        if up.startswith("CUSTOM:"):
            kv = self._parse_kv(cmd.split(":", 1)[1])
            meta = {
                "freq": float(kv.get("freq", 880.0)),
                "dur": float(kv.get("dur", 0.06)),
                "vol": float(kv.get("vol", self.volume)),
                "tag": kv.get("tag", ""),
            }
            self._enqueue_job("CUSTOM", meta=meta)
            return

        # error with tag
        if up.startswith("ERROR:") or up.startswith("FAIL:"):
            tag = cmd.split(":", 1)[1].strip()
            self._enqueue_job("ERROR", meta={"tag": tag})
            return

        # normal presets
        if up in ("BEEP", "WAKE", "LISTEN", "OK", "DOUBLE"):
            self._enqueue_job("DOUBLE")
        elif up in ("ERROR", "FAIL"):
            self._enqueue_job("ERROR")
        else:
            # default
            self._enqueue_job("DOUBLE")

    # ---------------- cleanup ----------------
    def destroy_node(self):
        try:
            self._stop_flag.set()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BeepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
