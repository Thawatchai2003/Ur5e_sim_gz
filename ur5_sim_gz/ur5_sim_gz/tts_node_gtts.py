#!/usr/bin/env python3
import os
import hashlib
import threading
import subprocess
import shutil
import time
import queue
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from gtts import gTTS


def cache_name(text: str, lang: str, slow: bool) -> str:
    key = f"{lang}|{int(slow)}|{text}"
    h = hashlib.sha256(key.encode("utf-8")).hexdigest()[:16]
    return f"tts_{h}.mp3"


class TTSManagerGTTSNode(Node):
    def __init__(self):
        super().__init__("tts_manager_gtts_node")

        # ---------------- params ----------------
        self.declare_parameter("lang", "th")
        self.declare_parameter("slow", False)
        self.declare_parameter("player", "mpg123")  # mpg123 recommended
        self.declare_parameter("cache_dir", os.path.expanduser("~/.cache/ros_tts"))
        self.declare_parameter("queue_size", 10)

        # debug pattern like DialogFSMNode
        self.declare_parameter("debug", True)
        self.declare_parameter("topic_tts_debug", "text_to_speech/tts_debug")
        self.declare_parameter("topic_tts_event", "text_to_speech/tts_event")

        # topics
        self.declare_parameter("topic_tts_request", "control/tts_request")
        self.declare_parameter("topic_tts_busy", "text_to_speech/tts_busy")

        # ---------------- read params ----------------
        self.lang = str(self.get_parameter("lang").value)
        self.slow = bool(self.get_parameter("slow").value)
        self.player = str(self.get_parameter("player").value)
        self.cache_dir = str(self.get_parameter("cache_dir").value)
        self.queue_size = int(self.get_parameter("queue_size").value)

        self.debug = bool(self.get_parameter("debug").value)
        self.topic_tts_debug = str(self.get_parameter("topic_tts_debug").value)
        self.topic_tts_event = str(self.get_parameter("topic_tts_event").value)

        self.topic_tts_request = str(self.get_parameter("topic_tts_request").value)
        self.topic_tts_busy = str(self.get_parameter("topic_tts_busy").value)

        os.makedirs(self.cache_dir, exist_ok=True)

        # ---------------- pubs/subs ----------------
        self.sub = self.create_subscription(String, self.topic_tts_request, self.cb, 10)
        self.busy_pub = self.create_publisher(Bool, self.topic_tts_busy, 10)

        # debug/event pubs (เหมือน dialog)
        self.debug_pub = self.create_publisher(String, self.topic_tts_debug, 10)
        self.event_pub = self.create_publisher(String, self.topic_tts_event, 10)

        # ---------------- internals ----------------
        self._proc_lock = threading.Lock()
        self._player_proc: Optional[subprocess.Popen] = None
        self._destroyed = False
        self._busy_last: Optional[bool] = None

        # single-worker queue
        self._q: "queue.Queue[str]" = queue.Queue(maxsize=max(1, self.queue_size))
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

                # ---------------- banner (NLU-style) ----------------
        GREEN = "\033[92m"
        RESET = "\033[0m"

        player_ok = (shutil.which(self.player) is not None)

        supported_players = ["mpg123", "ffplay", "<custom_player_binary>"]

        banner = (
            "\n"
            "──────────────────────────────────────────────────────────────\n"
            "        TTS Manager gTTS Node — Operational\n"
            "        Node State      : READY\n"
            f"        Debug Mode      : {'ENABLED' if self.debug else 'DISABLED'}\n"
            f"        Debug Topic     : {self.topic_tts_debug if self.debug else '(disabled)'}\n"
            f"        Event Topic     : {self.topic_tts_event}\n"
            "\n"
            "        Subscribed Topics:\n"
            f"            • {self.topic_tts_request}\n"
            "\n"
            "        Published Topics:\n"
            f"            • {self.topic_tts_busy}   (True/False)\n"
            f"            • {self.topic_tts_event}\n"
            f"            • {self.topic_tts_debug if self.debug else '(disabled)'}\n"
            "\n"
            "        Runtime Config:\n"
            f"            • lang      = {self.lang}\n"
            f"            • slow      = {self.slow}\n"
            f"            • player    = {self.player}  (ok={player_ok})\n"
            f"            • cache_dir = {self.cache_dir}\n"
            f"            • queue_size= {self.queue_size}\n"
            "\n"
            "        Supported Players:\n"
            "            - " + "\n            - ".join(supported_players) + "\n"
            "──────────────────────────────────────────────────────────────\n"
        )
        self.get_logger().info(GREEN + banner + RESET)
        self._dbg("banner printed")

        # banner (debug stream)
        self._dbg(
            f"ready lang={self.lang} slow={self.slow} player={self.player} "
            f"cache_dir={self.cache_dir} qsize={self.queue_size}"
        )

        if not player_ok:
            self.get_logger().error(
                f"❌ player '{self.player}' not found in PATH. "
                f"Install: sudo apt install mpg123  (or set player:=ffplay)"
            )
            self._event(f"TTS:PLAYER_MISSING:{self.player}")
        else:
            self.get_logger().info(
                f"✅ TTS Manager gTTS ready: lang={self.lang}, slow={self.slow}, player={self.player}, cache_dir={self.cache_dir}"
            )
            self._event("TTS:READY")

    # ---------------- debug/event helpers ----------------
    def _dbg(self, msg: str) -> None:
        if not self.debug:
            return
        try:
            self.debug_pub.publish(String(data=f"[DEBUG][tts_gtts] {msg}"))
        except Exception:
            pass

    def _event(self, msg: str) -> None:
        try:
            self.event_pub.publish(String(data=msg))
        except Exception:
            pass

    # ---------------- lifecycle ----------------
    def destroy_node(self):
        if self._destroyed:
            return
        self._destroyed = True

        self._event("TTS:SHUTDOWN")
        self._dbg("destroy_node start")

        self._stop.set()
        self._stop_playback()

        # unblock worker
        try:
            self._q.put_nowait("")
        except Exception:
            pass

        try:
            if self._worker.is_alive():
                self._worker.join(timeout=2.0)
        except Exception:
            pass

        self._set_busy(False)
        self._dbg("destroy_node done")
        super().destroy_node()

    # ---------------- ROS callback ----------------
    def cb(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return

        is_latest = False
        if text.upper().startswith("LATEST:"):
            is_latest = True
            text = text.split(":", 1)[1].strip()
            if not text:
                return

            self._event("TTS:LATEST")
            self._dbg("LATEST received -> clear_queue + stop_playback")
            self._clear_queue()
            self._stop_playback()
            self._set_busy(False)

        # enqueue
        try:
            self._q.put_nowait(text)
            self._event("TTS:ENQUEUE")
            self._dbg(f"enqueue ok latest={is_latest} qsize~{self._q.qsize()} text='{text[:60]}'")
        except Exception:
            if is_latest:
                self._dbg("queue full but latest=True -> force clear then enqueue")
                self._clear_queue()
                try:
                    self._q.put_nowait(text)
                    self._event("TTS:ENQUEUE_FORCE")
                except Exception:
                    self._event("TTS:DROP_LATEST")
                    self._dbg("still cannot enqueue latest -> drop")
            else:
                self._event("TTS:DROP_NEWEST")
                self._dbg("queue full -> drop newest")

    # ---------------- worker loop ----------------
    def _worker_loop(self):
        self._dbg("worker started")
        while not self._stop.is_set():
            try:
                text = self._q.get(timeout=0.2)
            except queue.Empty:
                continue

            if not text:
                continue

            self._set_busy(True)
            self._event("TTS:START")
            ok = False

            try:
                mp3_path = self._ensure_mp3(text)
                if self._stop.is_set():
                    break

                self._event("TTS:PLAY")
                self._play_mp3(mp3_path)
                ok = True
                self._event("TTS:OK")

            except Exception as e:
                self._event("TTS:ERR")
                self.get_logger().error(f"❌ TTS error: {e}")
                self._dbg(f"exception: {repr(e)}")

            finally:
                self._set_busy(False)
                self._dbg(f"done ok={ok}")

        self._dbg("worker exit")

    # ---------------- core ops ----------------
    def _ensure_mp3(self, text: str) -> str:
        path = os.path.join(self.cache_dir, cache_name(text, self.lang, self.slow))

        if os.path.exists(path) and os.path.getsize(path) > 0:
            self._event("TTS:CACHE_HIT")
            self._dbg(f"cache hit -> {os.path.basename(path)}")
            return path

        tmp = path + ".tmp"
        t0 = time.time()

        self._event("TTS:GEN")
        self._dbg(f"gTTS generating... len={len(text)}")
        tts = gTTS(text=text, lang=self.lang, slow=self.slow)
        tts.save(tmp)
        os.replace(tmp, path)

        dt = time.time() - t0
        self._dbg(f"gTTS generated in {dt:.2f}s -> {os.path.basename(path)}")
        return path

    def _play_mp3(self, path: str):
        if shutil.which(self.player) is None:
            raise RuntimeError(f"player '{self.player}' not found")

        if self.player == "mpg123":
            cmd = ["mpg123", "-q", path]
        elif self.player == "ffplay":
            cmd = ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", path]
        else:
            cmd = [self.player, path]

        self._dbg(f"spawn player cmd={' '.join(cmd[:3])} ...")
        p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        with self._proc_lock:
            self._player_proc = p

        try:
            rc = p.wait()
        finally:
            with self._proc_lock:
                if self._player_proc is p:
                    self._player_proc = None

        if rc != 0:
            raise RuntimeError(f"player failed rc={rc}")

    def _stop_playback(self):
        with self._proc_lock:
            p = self._player_proc
            self._player_proc = None

        if p is None:
            return

        self._event("TTS:STOP")
        self._dbg("stop_playback terminate()")

        try:
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self._dbg("terminate timeout -> kill()")
                    p.kill()
                    p.wait(timeout=1.0)
        except Exception as e:
            self._dbg(f"stop_playback warning: {repr(e)}")

    def _clear_queue(self):
        n = 0
        try:
            while True:
                _ = self._q.get_nowait()
                n += 1
        except queue.Empty:
            pass
        except Exception:
            pass
        self._dbg(f"clear_queue removed={n}")

    def _set_busy(self, v: bool):
        v = bool(v)
        if self._busy_last is not None and self._busy_last == v:
            return
        self._busy_last = v

        try:
            self.busy_pub.publish(Bool(data=v))
        except Exception:
            pass

        self._dbg(f"publish {self.topic_tts_busy}={v}")


def main(args=None):
    rclpy.init(args=args)
    node = TTSManagerGTTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
