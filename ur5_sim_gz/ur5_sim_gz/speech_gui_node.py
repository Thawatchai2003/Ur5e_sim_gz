#!/usr/bin/env python3
import threading
import time
import tkinter as tk
from tkinter import ttk
import tkinter.font as tkfont
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# macOS-ish color palette (refined)
BG0 = "#0b0f14"
CARD = "#121a26"
CARD2 = "#0e1520"
BORDER = "#1a2433"
TEXT = "#e8eef6"
MUTED = "#8b96a8"
ACCENT = "#4ea1ff"
ACCENT2 = "#6ee7ff"
BTN = "#1b2a3f"
BTN_H = "#233756"
BTN2 = "#1a2332"
BTN2_H = "#223047"

# Font helper
def mac_font(root: tk.Tk, size: int, weight: str = "normal"):
    preferred = [
        "SF Pro Text", "SF Pro Display", "Helvetica Neue", "Helvetica",
        "Segoe UI", "DejaVu Sans", "Liberation Sans", "TkDefaultFont",
    ]
    available = set(tkfont.families(root))
    family = next((f for f in preferred if f in available), "TkDefaultFont")
    return (family, size, weight)


def _rounded_frame(parent, bg=CARD, pad=14):
    outer = tk.Frame(parent, bg=bg, highlightthickness=1, highlightbackground=BORDER)
    inner = tk.Frame(outer, bg=bg)
    inner.pack(fill="both", expand=True, padx=pad, pady=pad)
    return outer, inner


def mac_dot(parent: tk.Widget, color: str, size: int = 12):
    bg = parent.cget("bg") if parent else BG0
    c = tk.Canvas(parent, width=size, height=size, bg=bg, highlightthickness=0, bd=0)
    c.pack(side="left", padx=4)
    c.create_oval(2, 2, size - 2, size - 2, fill=color, outline=color)
    return c

class SpeechGUINode(Node):
    """
    GUI-only node.

    Naming convention (recommended):
      - /voice/gui_cmd   : Dialog/FSM -> GUI (commands to show/hide/set status/result)
      - /voice/gui_event : GUI -> Dialog/FSM (user actions: MANUAL_ARM, TEXT:..., POS:...)

    Debug system:
      - Param: debug (bool)
      - Pub:   <topic_gui_debug> (String)  (optional)
      - Rate-limited _dbg() to avoid GUI lag
      - Debug panel in GUI (scrolling log)
      - debug_verbose to allow PUB/SUB spam logs
    """

    def __init__(self):
        super().__init__("speech_gui_node")

        # ---------------- parameters (DEBUG first) ----------------
        self.declare_parameter("debug", True)
        self.declare_parameter("debug_rate_hz", 12.0)    # max debug lines per sec
        self.declare_parameter("debug_to_topic", True)   # publish to topic_gui_debug
        self.declare_parameter("debug_to_gui", True)     # show in GUI panel
        self.declare_parameter("debug_verbose", False)   # allow PUB/SUB log spam

        # ---------------- topic params (FIX: cmd/event roles) ----------------
        # GUI -> FSM : event
        self.declare_parameter("topic_gui_event", "gui_control/gui_event")
        # FSM -> GUI : cmd
        self.declare_parameter("topic_gui_cmd", "control/gui_cmd")
        # debug topic
        self.declare_parameter("topic_gui_debug", "gui_control/gui_debug")
        self.declare_parameter("topic_audio_gui_enable", "/gui_control/gui_enable")
        self.declare_parameter("topic_voice_gui_enable", "/gui_control/gui_enable_firmware")
        self.declare_parameter("topic_logger_gui_enable", "/gui_control/logger_enable")

        # read debug params
        self.debug = bool(self.get_parameter("debug").value)
        self.debug_rate_hz = float(self.get_parameter("debug_rate_hz").value)
        self.debug_to_topic = bool(self.get_parameter("debug_to_topic").value)
        self.debug_to_gui = bool(self.get_parameter("debug_to_gui").value)
        self.debug_verbose = bool(self.get_parameter("debug_verbose").value)
        self.topic_voice_gui_enable = str(self.get_parameter("topic_voice_gui_enable").value)
        self.topic_logger_gui_enable = str(self.get_parameter("topic_logger_gui_enable").value)
        
        

        # read topics
        self.topic_gui_event = str(self.get_parameter("topic_gui_event").value)
        self.topic_gui_cmd = str(self.get_parameter("topic_gui_cmd").value)
        self.topic_gui_debug = str(self.get_parameter("topic_gui_debug").value)
        self.topic_audio_gui_enable = str(self.get_parameter("topic_audio_gui_enable").value)

        # ---------------- pubs/subs ----------------
        # Debug pub (always create; publish only when allowed)
        self.gui_event_pub = self.create_publisher(String, self.topic_gui_event, 10)
        self.gui_debug_pub = self.create_publisher(String, self.topic_gui_debug, 10)
        self.audio_gui_pub = self.create_publisher(Bool, self.topic_audio_gui_enable, 10)
        self.voice_gui_pub = self.create_publisher(Bool,self.topic_voice_gui_enable ,10)
        self.logger_gui_pub = self.create_publisher(Bool, self.topic_logger_gui_enable, 10)
        
        # Sub
        self.gui_cmd_sub = self.create_subscription(String, self.topic_gui_cmd, self.on_gui_cmd, 10)

        # ---------------- debug internals ----------------
        self._dbg_last_ts = 0.0
        self._dbg_min_dt = (1.0 / max(self.debug_rate_hz, 1e-6)) if self.debug_rate_hz > 0 else 0.0
        self._dbg_lock = threading.Lock()

        # UI visibility guards
        self._ui_pos_visible = False
        self._ui_scroll_visible = False
        self._ui_rotate_visible = False

        # ---------- root ----------
        self.root = tk.Tk()
        self.root.title("Voice Command System")

        self._window_geometry = "980x760"
        self.root.geometry(self._window_geometry)

        self.root.configure(bg=BG0)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # ---------- ttk style ----------
        style = ttk.Style()
        style.theme_use("clam")

        style.configure("Title.TLabel",
                        font=mac_font(self.root, 22, "bold"),
                        foreground=TEXT,
                        background=BG0)

        style.configure("SubTitle.TLabel",
                        font=mac_font(self.root, 11),
                        foreground=MUTED,
                        background=BG0)

        style.configure("Badge.TLabel",
                        font=mac_font(self.root, 11, "bold"),
                        foreground=TEXT,
                        background=CARD2,
                        padding=(10, 6))

        style.configure("Status.TLabel",
                        font=mac_font(self.root, 12),
                        foreground=MUTED,
                        background=CARD2)

        style.configure("Result.TLabel",
                        font=mac_font(self.root, 14, "bold"),
                        foreground=ACCENT2,
                        background=CARD2)

        style.configure("Hint.TLabel",
                        font=mac_font(self.root, 11),
                        foreground=MUTED,
                        background=CARD)

        style.configure("Primary.TButton",
                        font=mac_font(self.root, 12, "bold"),
                        foreground=TEXT,
                        background=ACCENT,
                        padding=(14, 10),
                        borderwidth=0)
        style.map("Primary.TButton", background=[("active", "#5ab0ff")])

        style.configure("Action.TButton",
                        font=mac_font(self.root, 11, "bold"),
                        foreground=TEXT,
                        background=BTN,
                        padding=(12, 9),
                        borderwidth=0)
        style.map("Action.TButton", background=[("active", BTN_H)])

        style.configure("Small.TButton",
                        font=mac_font(self.root, 10, "bold"),
                        foreground=TEXT,
                        background=BTN2,
                        padding=(10, 7),
                        borderwidth=0)
        style.map("Small.TButton", background=[("active", BTN2_H)])

        # ---------- header ----------
        header = tk.Frame(self.root, bg=BG0)
        header.pack(fill="x", padx=22, pady=(14, 6))

        dots = tk.Frame(header, bg=BG0)
        dots.pack(side="left", padx=(0, 12))
        mac_dot(dots, "#ff5f57")
        mac_dot(dots, "#febc2e")
        mac_dot(dots, "#28c840")

        title_box = tk.Frame(header, bg=BG0)
        title_box.pack(side="left", fill="x", expand=True)

        ttk.Label(title_box, text="Voice Command System", style="Title.TLabel").pack(anchor="w")
        ttk.Label(title_box, text="GUI Node • Push-to-talk + Quick actions + Debug",
                  style="SubTitle.TLabel").pack(anchor="w", pady=(2, 0))

        # ---------- body ----------
        body = tk.Frame(self.root, bg=BG0)
        body.pack(fill="both", expand=True, padx=22, pady=(0, 18))

        left = tk.Frame(body, bg=BG0)
        left.pack(side="left", fill="both", expand=True, padx=(0, 10))

        right = tk.Frame(body, bg=BG0)
        right.pack(side="right", fill="both", expand=True, padx=(10, 0))

        # ---------- status card ----------
        status_card, status_inner = _rounded_frame(left, bg=CARD, pad=14)
        status_card.pack(fill="x", pady=(0, 12))

        top_row = tk.Frame(status_inner, bg=CARD)
        top_row.pack(fill="x")

        self.status_badge = ttk.Label(top_row, text="🟦 Idle", style="Badge.TLabel")
        self.status_badge.pack(side="left")

        self.last_badge = ttk.Label(top_row, text="Last: (none)", style="Badge.TLabel")
        self.last_badge.pack(side="right")

        # --- Push-to-talk toggle state ---
        self._ptt_active = False
        self._ptt_default_text = "🎤  Push to Talk (ไม่ต้องพูด 'สวัสดี')"
        self._ptt_cancel_text = "⏹  ยกเลิก (หยุดรับคำสั่ง)"

        self.manual_button = ttk.Button(
            status_inner,
            text=self._ptt_default_text,
            style="Primary.TButton",
            command=self.on_manual_toggle
        )
        self.manual_button.pack(fill="x", pady=(12, 10))

        result_card, result_inner = _rounded_frame(status_inner, bg=CARD2, pad=12)
        result_card.pack(fill="x")

        self.status_label = ttk.Label(result_inner, text="Status: waiting…", style="Status.TLabel")
        self.status_label.pack(anchor="w")

        self.result_label = ttk.Label(
            result_inner,
            text="Text: (none yet)",
            style="Result.TLabel",
            wraplength=520,
            anchor="w"
        )
        self.result_label.pack(anchor="w", pady=(8, 2))

        # ---------- quick actions ----------
        quick_card, quick_inner = _rounded_frame(left, bg=CARD, pad=14)
        quick_card.pack(fill="x", pady=(0, 12))

        ttk.Label(
            quick_inner,
            text="Quick Actions",
            style="Hint.TLabel"
        ).pack(anchor="w", pady=(0, 10))

        grid = tk.Frame(quick_inner, bg=CARD)
        grid.pack(fill="x")

        # BTN
        ttk.Button(grid, text="⬆️ เลื่อนขึ้น (ทันที)",
                style="Action.TButton",
                command=lambda: self.on_send_text("เลื่อนขึ้น"))\
            .grid(row=0, column=0, sticky="ew", padx=(0, 8), pady=(0, 8))

        ttk.Button(grid, text="⬇️ เลื่อนลง (ทันที)",
                style="Action.TButton",
                command=lambda: self.on_send_text("เลื่อนลง"))\
            .grid(row=0, column=1, sticky="ew", padx=(8, 0), pady=(0, 8))

        ttk.Button(grid, text="↩️ หมุนซ้าย (ระบุมุม)",
                style="Action.TButton",
                command=lambda: self.on_choose_rotate("left"))\
            .grid(row=1, column=0, sticky="ew", padx=(0, 8))

        ttk.Button(grid, text="↪️ หมุนขวา (ระบุมุม)",
                style="Action.TButton",
                command=lambda: self.on_choose_rotate("right"))\
            .grid(row=1, column=1, sticky="ew", padx=(8, 0))

        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        # =========================
        # GUI Control Panel
        # =========================

        ttk.Label(
            quick_inner,
            text="GUI Control",
            style="Hint.TLabel"
        ).pack(anchor="w", pady=(18, 8))

        # ----- Voice GUI Toggle -----
        self._voice_gui_enabled = False

        self.voice_gui_button = ttk.Button(
            quick_inner,
            text="🟢 เปิด Voice GUI",
            style="Action.TButton",
            command=self.toggle_voice_gui
        )
        self.voice_gui_button.pack(fill="x", pady=(4, 6))

        # ----- Audio Monitor Toggle -----
        self._audio_gui_enabled = False
        self._logger_gui_enabled = False

        self.audio_gui_button = ttk.Button(
            quick_inner,
            text="🟢 เปิด Audio Monitor GUI",
            style="Action.TButton",
            command=self.toggle_audio_gui
        )
        self.audio_gui_button.pack(fill="x")

        self.logger_gui_button = ttk.Button(
            quick_inner,
            text="🟢 เปิด Logger GUI",
            style="Action.TButton",
            command=self.toggle_logger_gui
        )
        self.logger_gui_button.pack(fill="x", pady=(6, 0))

        # ---------- debug card ----------
        debug_card, debug_inner = _rounded_frame(left, bg=CARD, pad=14)
        debug_card.pack(fill="both", expand=True, pady=(0, 12))

        top_dbg = tk.Frame(debug_inner, bg=CARD)
        top_dbg.pack(fill="x")

        ttk.Label(top_dbg, text="Debug Log", style="Hint.TLabel").pack(side="left")
        self._dbg_status = ttk.Label(top_dbg, text=("ON" if self.debug else "OFF"), style="Badge.TLabel")
        self._dbg_status.pack(side="right")

        dbg_box = tk.Frame(debug_inner, bg=CARD)
        dbg_box.pack(fill="both", expand=True, pady=(10, 10))

        self.debug_text = tk.Text(
            dbg_box,
            height=10,
            bg=CARD2,
            fg=TEXT,
            insertbackground=TEXT,
            relief="flat",
            wrap="word",
            font=mac_font(self.root, 10),
        )
        self.debug_text.pack(side="left", fill="both", expand=True)

        dbg_scroll = ttk.Scrollbar(dbg_box, orient="vertical", command=self.debug_text.yview)
        dbg_scroll.pack(side="right", fill="y")
        self.debug_text.configure(yscrollcommand=dbg_scroll.set)

        # make read-only by default
        self.debug_text.configure(state="disabled")

        btnrow = tk.Frame(debug_inner, bg=CARD)
        btnrow.pack(fill="x")

        ttk.Button(btnrow, text="🧹 Clear", style="Small.TButton",
                   command=self._dbg_clear).pack(side="left")

        ttk.Button(btnrow, text="📌 Print Config", style="Small.TButton",
                   command=self._dbg_print_config).pack(side="left", padx=(8, 0))

        # ---------- interactive panels ----------
        panel_card, panel_inner = _rounded_frame(right, bg=CARD, pad=14)
        panel_card.pack(fill="both", expand=True)

        ttk.Label(panel_inner, text="Interactive Panels", style="Hint.TLabel")\
            .pack(anchor="w", pady=(0, 10))

        panel_inner.columnconfigure(0, weight=1)

        # ---------- Position panel ----------
        self.pos_frame = tk.Frame(panel_inner, bg=CARD)
        self.pos_hint = ttk.Label(self.pos_frame, text="", style="Hint.TLabel")
        self.pos_hint.pack(anchor="w", pady=(0, 8))

        pos_grid = tk.Frame(self.pos_frame, bg=CARD)
        pos_grid.pack(fill="x")
        pos_grid.columnconfigure(0, weight=1)
        pos_grid.columnconfigure(1, weight=1)

        btns = [1, 2, 3, 4, 5]
        for idx, k in enumerate(btns):
            r = idx // 2
            c = idx % 2
            b = ttk.Button(
                pos_grid, text=f"ตำแหน่ง {k}", style="Small.TButton",
                command=lambda kk=k: self.on_choose_pos(kk)
            )
            b.grid(row=r, column=c, sticky="ew", padx=6, pady=6)

        # ---------- Scroll panel ----------
        self.scroll_frame = tk.Frame(panel_inner, bg=CARD)
        self.scroll_hint = ttk.Label(self.scroll_frame, text="", style="Hint.TLabel")
        self.scroll_hint.pack(anchor="w", pady=(0, 8))

        srow = tk.Frame(self.scroll_frame, bg=CARD)
        srow.pack(fill="x")
        srow.columnconfigure(0, weight=1)
        srow.columnconfigure(1, weight=1)

        ttk.Button(srow, text="⬆️ ขึ้น (ระบุมุม)", style="Small.TButton",
                   command=lambda: self.on_choose_scroll("up"))\
            .grid(row=0, column=0, sticky="ew", padx=6, pady=6)

        ttk.Button(srow, text="⬇️ ลง (ระบุมุม)", style="Small.TButton",
                   command=lambda: self.on_choose_scroll("down"))\
            .grid(row=0, column=1, sticky="ew", padx=6, pady=6)

        # ---------- Rotate panel ----------
        self.rotate_frame = tk.Frame(panel_inner, bg=CARD)
        self.rotate_hint = ttk.Label(self.rotate_frame, text="", style="Hint.TLabel")
        self.rotate_hint.pack(anchor="w", pady=(0, 8))

        rrow = tk.Frame(self.rotate_frame, bg=CARD)
        rrow.pack(fill="x")
        rrow.columnconfigure(0, weight=1)
        rrow.columnconfigure(1, weight=1)

        ttk.Button(rrow, text="↩️ ซ้าย (ระบุมุม)", style="Small.TButton",
                   command=lambda: self.on_choose_rotate("left"))\
            .grid(row=0, column=0, sticky="ew", padx=6, pady=6)

        ttk.Button(rrow, text="↪️ ขวา (ระบุมุม)", style="Small.TButton",
                   command=lambda: self.on_choose_rotate("right"))\
            .grid(row=0, column=1, sticky="ew", padx=6, pady=6)

        # hide panels initially
        self.hide_pos()
        self.hide_scroll()
        self.hide_rotate()

        # start ROS spin thread + UI tick
        self._spin_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self._spin_thread.start()
        self.root.after(150, self._tick)

        # initial debug banner
        self._print_startup_banner()
        self._dbg_print_config()
        self._dbg("GUI ready")

    # ---------------- Debug helpers ----------------

    def _safe(self, s: str) -> str:
        try:
            return str(s)
        except Exception:
            return "(?)"

    def _print_startup_banner(self):
        GREEN = "\033[92m"
        RESET = "\033[0m"

        supported = [
            "GUI cmd renderer (SHOW/HIDE panels, SET_STATUS, SET_RESULT)",
            "Push-to-talk toggle (MANUAL_ARM / MANUAL_CANCEL)",
            "Quick actions (send TEXT / choose POS / choose SCROLL / choose ROTATE)",
            "Auto-sync PTT button when status returns to IDLE",
            "Threaded spin_once + Tk mainloop scheduling (root.after)",
            "Debug panel (rate-limited) + optional topic publish",
            "Verbose debug mode (PUB/SUB logs) controlled by debug_verbose",
            "Topic roles: cmd(FSM->GUI) / event(GUI->FSM)",
        ]

        banner = (
            "\n"
            "──────────────────────────────────────────────────────────────\n"
            "        Speech GUI Node — Operational\n"
            "        Node State      : READY\n"
            f"        Debug Mode      : {'ENABLED' if self.debug else 'DISABLED'}\n"
            f"        Debug Topic     : {self._safe(self.topic_gui_debug) if (self.debug and self.debug_to_topic) else '(disabled)'}\n"
            "\n"
            "        Subscribed Topics (FSM -> GUI):\n"
            f"            • {self._safe(self.topic_gui_cmd)}    (GUI commands)\n"
            "\n"
            "        Published Topics (GUI -> FSM):\n"
            f"            • {self._safe(self.topic_gui_event)}  (GUI events)\n"
            f"            • {self._safe(self.topic_gui_debug) if (self.debug and self.debug_to_topic) else '(disabled)'}\n"
            "\n"
            "        Runtime Config:\n"
            f"            • window_geometry   = {self._window_geometry}\n"
            f"            • debug_rate_hz     = {self.debug_rate_hz}\n"
            f"            • debug_to_topic    = {self.debug_to_topic}\n"
            f"            • debug_to_gui      = {self.debug_to_gui}\n"
            f"            • debug_verbose     = {self.debug_verbose}\n"
            f"            • ptt_toggle_text   = '{self._ptt_default_text}' / '{self._ptt_cancel_text}'\n"
            f"            • theme             = clam\n"
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
                self.gui_debug_pub.publish(String(data="[DEBUG][speech_gui] banner:\n" + banner))
            except Exception:
                pass

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
                self.gui_debug_pub.publish(String(data=line))
            except Exception:
                pass

        if self.debug_to_gui:
            try:
                self.root.after(0, lambda l=line: self._dbg_append_ui(l))
            except Exception:
                pass

    def _dbg_append_ui(self, line: str):
        if not hasattr(self, "debug_text"):
            return
        try:
            self.debug_text.configure(state="normal")
            self.debug_text.insert("end", line + "\n")
            self.debug_text.see("end")
        finally:
            try:
                self.debug_text.configure(state="disabled")
            except Exception:
                pass

    def _dbg_clear(self):
        try:
            self.debug_text.configure(state="normal")
            self.debug_text.delete("1.0", "end")
        except Exception:
            pass
        finally:
            try:
                self.debug_text.configure(state="disabled")
            except Exception:
                pass
        self._dbg("debug cleared")

    def _dbg_print_config(self):
        cfg = (
            f"debug={self.debug} rate_hz={self.debug_rate_hz} "
            f"to_topic={self.debug_to_topic} to_gui={self.debug_to_gui} verbose={self.debug_verbose} | "
            f"topics: gui_cmd='{self.topic_gui_cmd}' gui_event='{self.topic_gui_event}' gui_debug='{self.topic_gui_debug}'"
        )
        self._dbg(cfg)

    # ---------- ROS ----------
    def _spin_ros(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _tick(self):
        self.root.after(150, self._tick)

    # GUI -> FSM event publisher
    def _pub_event(self, s: str):
        self.gui_event_pub.publish(String(data=s))
        if self.debug_verbose:
            self._dbg(f"PUB -> {self.topic_gui_event}: '{s}'")

    # Toggle: ARM / CANCEL inside one button
    def on_manual_toggle(self):
        self._ptt_active = not self._ptt_active

        if self._ptt_active:
            self.manual_button.config(text=self._ptt_cancel_text)
            self.last_badge.config(text="Last: MANUAL_ARM")
            self.result_label.config(text="Text: (Push-to-talk) พูดคำสั่งได้เลย…")
            self._pub_event("MANUAL_ARM")
            self._dbg("PTT -> ACTIVE (MANUAL_ARM)")
        else:
            self.manual_button.config(text=self._ptt_default_text)
            self.last_badge.config(text="Last: MANUAL_CANCEL")
            self.result_label.config(text="Text: (cancelled) ยกเลิกการพูดแล้ว")
            self._pub_event("MANUAL_CANCEL")
            self._dbg("PTT -> INACTIVE (MANUAL_CANCEL)")

    def on_send_text(self, text: str):
        self.last_badge.config(text=f"Last: TEXT:{text}")
        self.result_label.config(text=f"Text: {text}")
        self._pub_event(f"TEXT:{text}")

    def on_choose_pos(self, k: int):
        self.last_badge.config(text=f"Last: POS:{k}")
        self._pub_event(f"POS:{k}")

    def on_choose_scroll(self, direction: str):
        self.last_badge.config(text=f"Last: SCROLL:{direction}")
        self._pub_event(f"SCROLL:{direction}")

    def on_choose_rotate(self, direction: str):
        self.last_badge.config(text=f"Last: ROTATE:{direction}")
        self._pub_event(f"ROTATE:{direction}")

    def show_pos(self):
        if self._ui_pos_visible:
            return
        self._ui_pos_visible = True
        self.pos_hint.config(text="ตำแหน่ง: เลือก 1–5 หรือพูดเลข 1–5")
        self.pos_frame.pack(fill="x", pady=(0, 14))
        self._dbg("UI: SHOW_POS")

    def hide_pos(self):
        if not self._ui_pos_visible:
            return
        self._ui_pos_visible = False
        self.pos_hint.config(text="")
        self.pos_frame.pack_forget()
        self._dbg("UI: HIDE_POS")

    def show_scroll(self):
        if self._ui_scroll_visible:
            return
        self._ui_scroll_visible = True
        self.scroll_hint.config(text="เลื่อน (ระบุมุม): เลือกขึ้น/ลง แล้วพูดจำนวนองศา")
        self.scroll_frame.pack(fill="x", pady=(0, 14))
        self._dbg("UI: SHOW_SCROLL")

    def hide_scroll(self):
        if not self._ui_scroll_visible:
            return
        self._ui_scroll_visible = False
        self.scroll_hint.config(text="")
        self.scroll_frame.pack_forget()
        self._dbg("UI: HIDE_SCROLL")

    def show_rotate(self):
        if self._ui_rotate_visible:
            return
        self._ui_rotate_visible = True
        self.rotate_hint.config(text="หมุน (ระบุมุม): เลือกซ้าย/ขวา แล้วพูดจำนวนองศา")
        self.rotate_frame.pack(fill="x", pady=(0, 14))
        self._dbg("UI: SHOW_ROTATE")

    def hide_rotate(self):
        if not self._ui_rotate_visible:
            return
        self._ui_rotate_visible = False
        self.rotate_hint.config(text="")
        self.rotate_frame.pack_forget()
        self._dbg("UI: HIDE_ROTATE")

    # FSM -> GUI command subscriber
    def on_gui_cmd(self, msg: String):
        data = (msg.data or "").strip()
        if self.debug_verbose:
            self._dbg(f"SUB <- {self.topic_gui_cmd}: '{data}'")
        self.root.after(0, lambda d=data: self._handle_gui_cmd(d))

    def _handle_gui_cmd(self, data: str):
        if data.startswith("SET_STATUS:"):
            st = data.split(":", 1)[1]
            self.status_badge.config(text=st if st else "🟦 Idle")
            self.status_label.config(text=f"Status: {st}")

            st_upper = (st or "").upper()
            if ("IDLE" in st_upper) or ("🕒" in st):
                if self._ptt_active:
                    self._ptt_active = False
                    self.manual_button.config(text=self._ptt_default_text)
                    self._dbg("PTT auto-reset -> INACTIVE (status idle)")
            return

        if data.startswith("SET_RESULT:"):
            res = data.split(":", 1)[1]
            self.result_label.config(text=res)
            self.last_badge.config(text=f"Last: {res[:36]}{'…' if len(res) > 36 else ''}")
            return

        if data == "SHOW_POS":
            self.show_pos()
            return
        if data == "HIDE_POS":
            self.hide_pos()
            return
        if data == "SHOW_SCROLL":
            self.show_scroll()
            return
        if data == "HIDE_SCROLL":
            self.hide_scroll()
            return
        if data == "SHOW_ROTATE":
            self.show_rotate()
            return
        if data == "HIDE_ROTATE":
            self.hide_rotate()
            return

        self._dbg(f"UI: unknown gui_cmd '{data}'")

    def on_close(self):
        self._dbg("GUI closing…")
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.root.destroy()

    def run(self):
        self.root.mainloop()

    def send_audio_gui(self, state: bool):
        msg = Bool()
        msg.data = state
        self.audio_gui_pub.publish(msg)
        if self.debug:
            self._dbg(f"PUB -> {self.topic_audio_gui_enable}: {state}")

    def toggle_voice_gui(self):
        self._voice_gui_enabled = not self._voice_gui_enabled

        msg = Bool()
        msg.data = self._voice_gui_enabled
        self.voice_gui_pub.publish(msg)

        if self._voice_gui_enabled:
            self.voice_gui_button.config(text="🔴 ปิด Voice GUI")
            self._dbg("Voice GUI -> ENABLED")
        else:
            self.voice_gui_button.config(text="🟢 เปิด Voice GUI")
            self._dbg("Voice GUI -> DISABLED")
    
    def toggle_audio_gui(self):
        self._audio_gui_enabled = not self._audio_gui_enabled

        msg = Bool()
        msg.data = self._audio_gui_enabled
        self.audio_gui_pub.publish(msg)

        if self._audio_gui_enabled:
            self.audio_gui_button.config(text="🔴 ปิด Audio Monitor GUI")
            self._dbg("Audio Monitor GUI -> ENABLED")
        else:
            self.audio_gui_button.config(text="🟢 เปิด Audio Monitor GUI")
            self._dbg("Audio Monitor GUI -> DISABLED")

    def toggle_logger_gui(self):
        self._logger_gui_enabled = not self._logger_gui_enabled

        msg = Bool()
        msg.data = self._logger_gui_enabled
        self.logger_gui_pub.publish(msg)

        if self._logger_gui_enabled:
            self.logger_gui_button.config(text="🔴 ปิด Logger GUI")
            self._dbg("Logger GUI -> ENABLED")
        else:
            self.logger_gui_button.config(text="🟢 เปิด Logger GUI")
            self._dbg("Logger GUI -> DISABLED")


def main(args=None):
    rclpy.init(args=args)
    node = SpeechGUINode()
    try:
        node.run()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
