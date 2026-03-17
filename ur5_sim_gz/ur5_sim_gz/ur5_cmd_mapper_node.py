# ============================================================
#  File:        ur5_cmd_mapper_node.py
#  Node Name:   UR5CmdMapperNode
#
#  Description:
#  ------------------------------------------------------------
#  UR5CmdMapperNode is the command normalization and routing
#  layer for the UR5/UR5e voice control system. It receives
#  grouped text commands from the voice/NLU pipeline and
#  converts them into standardized high-level commands for
#  the motion executor, with safety gating, deduplication,
#  and incremental command debouncing.
#
#  Core Responsibilities:
#    ✓ Parse grouped commands from /voice/cmd_group
#    ✓ Normalize commands into executor-ready formats
#    ✓ Publish high-level commands to /ur5/high_level_cmd
#    ✓ Provide dedicated CANCEL pulse to abort motion instantly
#    ✓ Enforce LOCK / UNLOCK safety gating
#    ✓ Debounce incremental commands (JOG / ROTATE / ROTATE_W3)
#    ✓ Dedupe non-incremental commands (POS / HOME / VIEW / SPEED)
#    ✓ Emit mapper events for UI, logging, and diagnostics
#
#  Not responsible for:
#    ✗ Speech-to-text (STT) processing
#    ✗ Natural language understanding (NLU)
#    ✗ Dialog or state machine logic
#    ✗ Motion planning or trajectory execution
#    ✗ Hardware-level UR driver or RTDE communication
#
#  ------------------------------------------------------------
#  Communication Overview:
#
#  Subscribed Topics:
#    • /voice/cmd_group        (std_msgs/String)
#
#  Published Topics:
#    • /ur5/high_level_cmd     (std_msgs/String)
#    • /ur5/cmd_cancel         (std_msgs/Bool)
#    • /voice/mapper_event    (std_msgs/String)
#
#  Debug Topics:
#    • /voice/mapper_debug    (std_msgs/String)
#
#  ------------------------------------------------------------
#  Command Output Conventions:
#
#    High-Level Commands:
#      • STOP / HOME / BACK / PICK / PLACE
#      • LOCK / UNLOCK
#      • POS:<n>
#      • TOP_VIEW:<n>
#      • SIDE_VIEW:<n>
#      • SPEED:<slow|normal|fast>
#
#    Incremental Commands:
#      • JOG:<dir>:<meters>        dir = left/right/forward/back/up/down
#      • ROTATE:<dir>:<deg>        dir = left/right
#      • ROTATE_W3:<dir>:<deg>     wrist3 rotation
#
#    Cancel Behavior:
#      • /ur5/cmd_cancel publishes True (edge-trigger)
#      • Automatically resets to False after cancel_pulse_sec
#
#  ------------------------------------------------------------
#  Safety & Robustness Features:
#    • LOCK gating blocks all motion-related commands
#    • CANCEL always allowed (even when LOCKED)
#    • Incremental debounce prevents command flooding
#    • Non-incremental dedupe prevents repeated execution
#    • Parameterized limits for jog distance and rotation angle
#
#  ------------------------------------------------------------
#  Author:       Mr. Thawatchai Thongbai, Miss Ruksina Janthawong
#  Affiliation:  Prince of Songkla University (PSU)
#  Project:      UR5e Voice-Control / ROS2 Command Mapping Layer
#  Version:      v1.0.0
#  Last Update:  2026-01-03
# ============================================================

#!/usr/bin/env python3
import re
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import String, Bool


def _norm(s: str) -> str:
    return (s or "").strip()


def parse_group_cmd(s: str) -> Tuple[str, Optional[str], Optional[float], Optional[int]]:
    """
    Parse text command from /voice/cmd_group into:
      (kind, direction, value, pos_index)

    Kinds:
      STOP, CANCEL, HOME, BACK, PICK, PLACE, UNLOCK, LOCK,
      POS, TOP_VIEW, SIDE_VIEW,
      MOVE, ROTATE, ROTATE_W3,
      SPEED,
      UNKNOWN
    """
    t = _norm(s).upper()
    if not t:
        return ("UNKNOWN", None, None, None)

    # CANCEL keywords
    if t in ("CANCEL", "ABORT", "HALT_MOTION"):
        return ("CANCEL", None, None, None)

    # simple keywords
    if t == "STOP":
        return ("STOP", None, None, None)
    if t in ("HOME", "BACK_HOME"):
        return ("HOME", None, None, None)
    if t in ("BACK", "RETURN"):
        return ("BACK", None, None, None)

    if t == "PICK":
        return ("PICK", None, None, None)
    if t == "PLACE":
        return ("PLACE", None, None, None)

    if t == "UNLOCK":
        return ("UNLOCK", None, None, None)
    if t == "LOCK":
        return ("LOCK", None, None, None)

    # POS_n (support multi-digit)
    mpos = re.fullmatch(r"POS_(\d+)", t)
    if mpos:
        return ("POS", None, None, int(mpos.group(1)))

    # TOP_VIEW_n (support multi-digit)
    mtv = re.fullmatch(r"TOP_VIEW_(\d+)", t)
    if mtv:
        return ("TOP_VIEW", None, None, int(mtv.group(1)))

    # SIDE_VIEW_n (support multi-digit)
    msv = re.fullmatch(r"SIDE_VIEW_(\d+)", t)
    if msv:
        return ("SIDE_VIEW", None, None, int(msv.group(1)))

    # ROTATE_LEFT[:deg] / ROTATE_RIGHT[:deg]
    mrot = re.fullmatch(r"ROTATE_(LEFT|RIGHT)(?::([0-9]+(?:\.[0-9]+)?))?", t)
    if mrot:
        return (
            "ROTATE",
            mrot.group(1).lower(),
            float(mrot.group(2)) if mrot.group(2) else None,
            None,
        )

    # wrist_3 rotation (aliases)
    mw3 = re.fullmatch(r"(?:ROTATE_)?W3_(LEFT|RIGHT)(?::([0-9]+(?:\.[0-9]+)?))?", t)
    if mw3:
        return (
            "ROTATE_W3",
            mw3.group(1).lower(),
            float(mw3.group(2)) if mw3.group(2) else None,
            None,
        )

    mw3b = re.fullmatch(r"WRIST_?3_(LEFT|RIGHT)(?::([0-9]+(?:\.[0-9]+)?))?", t)
    if mw3b:
        return (
            "ROTATE_W3",
            mw3b.group(1).lower(),
            float(mw3b.group(2)) if mw3b.group(2) else None,
            None,
        )

    # SPEED_SLOW / SPEED_NORMAL / SPEED_FAST
    mspeed = re.fullmatch(r"SPEED_(SLOW|NORMAL|FAST)", t)
    if mspeed:
        return ("SPEED", mspeed.group(1).lower(), None, None)

    # MOVE_LEFT[:m], MOVE_FORWARD[:m], ...
    mmove = re.fullmatch(
        r"MOVE_(LEFT|RIGHT|FORWARD|BACK|UP|DOWN)(?::([0-9]+(?:\.[0-9]+)?))?",
        t,
    )
    if mmove:
        return (
            "MOVE",
            mmove.group(1).lower(),
            float(mmove.group(2)) if mmove.group(2) else None,
            None,
        )

    return ("UNKNOWN", None, None, None)


class UR5CmdMapperNode(Node):
    """
    UR5 Cmd Mapper Node

    Output command conventions:
      - POS:<n>
      - TOP_VIEW:<n>
      - SIDE_VIEW:<n>
      - JOG:<dir>:<meters>
      - ROTATE:<dir>:<deg>
      - ROTATE_W3:<dir>:<deg>
      - SPEED:<slow|normal|fast>
      - STOP / CANCEL / HOME / BACK / PICK / PLACE / UNLOCK / LOCK
    """

    def __init__(self):
        super().__init__("ur5_cmd_mapper_node")

        GREEN = "\033[92m"
        RESET = "\033[0m"

        # ---------------- Parameters ----------------
        self.declare_parameter("debug", False)
        self.debug = bool(self.get_parameter("debug").value)

        # default: ROTATE and ROTATE_W3 
        self.declare_parameter("default_rotate_deg", 5.0)        
        self.declare_parameter("default_w3_rotate_deg", 15.0)     

        self.declare_parameter("default_jog_m", 0.03)
        self.declare_parameter("max_rotate_deg", 180.0)
        self.declare_parameter("max_jog_m", 0.20)

        # optional: convert direction into signed degrees (executor dependent)
        self.declare_parameter("rotate_signed", False)
        self.rotate_signed = bool(self.get_parameter("rotate_signed").value)

        # ---------------- Debounce / min interval for incremental commands ----------------
        self.declare_parameter("inc_debounce_enable", True)
        self.declare_parameter("inc_min_interval_sec", 0.25)
        self.declare_parameter("inc_debounce_same_only", True)
        self.declare_parameter("inc_report_drop_event", True)

        self.inc_debounce_enable = bool(self.get_parameter("inc_debounce_enable").value)
        self.inc_min_interval_sec = float(self.get_parameter("inc_min_interval_sec").value)
        self.inc_debounce_same_only = bool(self.get_parameter("inc_debounce_same_only").value)
        self.inc_report_drop_event = bool(self.get_parameter("inc_report_drop_event").value)

        # ---------------- Dedupe / min interval for non-incremental commands ----------------
        self.declare_parameter("noninc_dedupe_enable", True)
        self.declare_parameter("noninc_min_interval_sec", 0.40)
        self.declare_parameter("noninc_dedupe_same_only", True)
        self.declare_parameter("noninc_report_drop_event", True)

        self.noninc_dedupe_enable = bool(self.get_parameter("noninc_dedupe_enable").value)
        self.noninc_min_interval_sec = float(self.get_parameter("noninc_min_interval_sec").value)
        self.noninc_dedupe_same_only = bool(self.get_parameter("noninc_dedupe_same_only").value)
        self.noninc_report_drop_event = bool(self.get_parameter("noninc_report_drop_event").value)

        # ---------------- CANCEL pulse (NEW) ----------------
        self.declare_parameter("cancel_pulse_enable", True)
        self.declare_parameter("cancel_pulse_sec", 0.05)
        self.cancel_pulse_enable = bool(self.get_parameter("cancel_pulse_enable").value)
        self.cancel_pulse_sec = float(self.get_parameter("cancel_pulse_sec").value)

        # read params
        self.default_rotate_deg = float(self.get_parameter("default_rotate_deg").value)
        self.default_w3_rotate_deg = float(self.get_parameter("default_w3_rotate_deg").value)

        self.default_jog_m = float(self.get_parameter("default_jog_m").value)
        self.max_rotate_deg = float(self.get_parameter("max_rotate_deg").value)
        self.max_jog_m = float(self.get_parameter("max_jog_m").value)

        self.declare_parameter("topic_cmd_group", "/control_position/cmd_mapper")
        self.declare_parameter("topic_high_level_cmd", "/mapper/high_level_cmd")
        self.declare_parameter("topic_cancel", "/mapper/cmd_cancel")
        self.declare_parameter("topic_mapper_event", "/mapper/mapper_event")
        self.declare_parameter("topic_debug", "/mapper/mapper_debug")

        self.topic_cmd_group = str(self.get_parameter("topic_cmd_group").value)
        self.topic_high_level_cmd = str(self.get_parameter("topic_high_level_cmd").value)
        self.topic_cancel = str(self.get_parameter("topic_cancel").value)
        self.topic_mapper_event = str(self.get_parameter("topic_mapper_event").value)
        self.topic_debug = str(self.get_parameter("topic_debug").value)

        # ---------------- ROS I/O ----------------
        self.sub = self.create_subscription(String, self.topic_cmd_group, self.on_group, 10)
        self.hl_pub = self.create_publisher(String, self.topic_high_level_cmd, 10)
        self.cancel_pub = self.create_publisher(Bool, self.topic_cancel, 10)
        self.event_pub = self.create_publisher(String, self.topic_mapper_event, 10)
        self.debug_pub = self.create_publisher(String, self.topic_debug, 10)

        # ---------------- State ----------------
        self._clock: Clock = self.get_clock()

        self._locked: bool = False

        self._last_inc_ts: Optional[float] = None
        self._last_inc_sig: Optional[str] = None

        self._last_noninc_ts: Optional[float] = None
        self._last_noninc_sig: Optional[str] = None

        self._cancel_reset_timer = None

        # ---------------- Banner ----------------
        self.supported_kinds: List[str] = [
            "STOP / CANCEL / UNLOCK / LOCK",
            "HOME / BACK",
            "POS_<n>",
            "TOP_VIEW_<n>",
            "SIDE_VIEW_<n>",
            "MOVE_<dir>[:m] (left/right/forward/back/up/down)",
            "ROTATE_<dir>[:deg] (left/right)",
            "ROTATE_W3_<dir>[:deg] or W3_<dir>[:deg] or WRIST3_<dir>[:deg]",
            "PICK / PLACE",
            "SPEED_<slow|normal|fast>",
        ]

        banner = (
            "\n"
            "──────────────────────────────────────────────────────────────\n"
            "        UR5 Cmd Mapper Node — Operational\n"
            "        Node State      : READY\n"
            f"        Debug Mode      : {'ENABLED' if self.debug else 'DISABLED'}\n"
            f"        Debug Channel   : {self.topic_debug if self.debug else '(disabled)'}\n"
            f"        Default ROTATE  : {self.default_rotate_deg:g} deg (general)\n"
            f"        Default ROTATE_W3: {self.default_w3_rotate_deg:g} deg (wrist3)\n"
            f"        Rotate Signed   : {'YES' if self.rotate_signed else 'NO'}\n"
            f"        Inc Debounce    : {'ON' if self.inc_debounce_enable else 'OFF'} "
            f"(min={self.inc_min_interval_sec:.3f}s, same_only={self.inc_debounce_same_only})\n"
            f"        NonInc Dedupe   : {'ON' if self.noninc_dedupe_enable else 'OFF'} "
            f"(min={self.noninc_min_interval_sec:.3f}s, same_only={self.noninc_dedupe_same_only})\n"
            f"        Cancel Pulse    : {'ON' if self.cancel_pulse_enable else 'OFF'} "
            f"(sec={self.cancel_pulse_sec:.3f}s)\n"
            f"        Lock State      : {'LOCKED' if self._locked else 'UNLOCKED'}\n"
            "\n"
            "        Subscribed Topics:\n"
            f"            • {self.topic_cmd_group}\n"
            "\n"
            "        Published Topics:\n"
            f"            • {self.topic_high_level_cmd}\n"
            f"            • {self.topic_cancel}\n"
            f"            • {self.topic_mapper_event}\n"
            f"            • {self.topic_debug}\n"
            "\n"
            "        Supported Commands:\n"
            "            - " + "\n            - ".join(self.supported_kinds) + "\n"
            "──────────────────────────────────────────────────────────────\n"
        )
        self.get_logger().info(GREEN + banner + RESET)

    # ---------------- Helpers ----------------
    def _dbg(self, message: str) -> None:
        if self.debug:
            self.debug_pub.publish(String(data=f"[DEBUG][mapper] {message}"))

    def _event(self, payload: str) -> None:
        self.event_pub.publish(String(data=payload))

    def _pub_hl(self, cmd: str) -> None:
        self.hl_pub.publish(String(data=cmd))
        self._dbg(f"HL_CMD -> {cmd}")

    # ---------------- CANCEL pulse (NEW) ----------------
    def _cancel_pulse(self) -> None:
        self.cancel_pub.publish(Bool(data=True))
        self._dbg("CANCEL pulse: publish True")

        if not self.cancel_pulse_enable:
            return

        if self._cancel_reset_timer is not None:
            try:
                self._cancel_reset_timer.cancel()
            except Exception:
                pass
            self._cancel_reset_timer = None

        def _reset():
            self.cancel_pub.publish(Bool(data=False))
            self._dbg("CANCEL pulse: publish False")
            if self._cancel_reset_timer is not None:
                try:
                    self._cancel_reset_timer.cancel()
                except Exception:
                    pass
                self._cancel_reset_timer = None

        self._cancel_reset_timer = self.create_timer(self.cancel_pulse_sec, _reset)

    def _clamp_deg(self, deg: float) -> float:
        deg = float(deg)
        if deg > self.max_rotate_deg:
            deg = self.max_rotate_deg
        if deg < 0.0:
            deg = 0.0
        return deg

    def _clamp_jog(self, jog: float) -> float:
        jog = float(jog)
        if jog < 0.0:
            jog = 0.0
        if jog > self.max_jog_m:
            jog = self.max_jog_m
        return jog

    def _now_sec(self) -> float:
        return float(self._clock.now().nanoseconds) * 1e-9

    # -------- incremental debounce --------
    def _mk_inc_sig(self, kind: str, direction: str, value: float) -> str:
        if kind == "MOVE":
            q = round(float(value), 3)
            return f"INC:JOG:{direction}:{q:.3f}"
        if kind == "ROTATE":
            q = round(float(value), 1)
            return f"INC:ROTATE:{direction}:{q:.1f}"
        if kind == "ROTATE_W3":
            q = round(float(value), 1)
            return f"INC:ROTATE_W3:{direction}:{q:.1f}"
        return f"INC:{kind}:{direction}:{value}"

    def _should_drop_incremental(self, sig: str) -> bool:
        if not self.inc_debounce_enable:
            return False

        now = self._now_sec()
        if self._last_inc_ts is None:
            self._last_inc_ts = now
            self._last_inc_sig = sig
            return False

        dt = now - self._last_inc_ts
        if dt >= self.inc_min_interval_sec:
            self._last_inc_ts = now
            self._last_inc_sig = sig
            return False

        if self.inc_debounce_same_only:
            return (self._last_inc_sig == sig)

        return True

    # -------- non-incremental dedupe --------
    def _mk_noninc_sig(
        self,
        kind: str,
        pos_index: Optional[int],
        direction: Optional[str],
        value: Optional[float],
    ) -> str:
        if kind in ("POS", "TOP_VIEW", "SIDE_VIEW") and pos_index is not None:
            return f"NONINC:{kind}:{pos_index}"
        if kind == "SPEED" and direction:
            return f"NONINC:SPEED:{direction}"
        return f"NONINC:{kind}"

    def _should_drop_nonincremental(self, sig: str) -> bool:
        if not self.noninc_dedupe_enable:
            return False

        now = self._now_sec()
        if self._last_noninc_ts is None:
            self._last_noninc_ts = now
            self._last_noninc_sig = sig
            return False

        dt = now - self._last_noninc_ts
        if dt >= self.noninc_min_interval_sec:
            self._last_noninc_ts = now
            self._last_noninc_sig = sig
            return False

        if self.noninc_dedupe_same_only:
            return (self._last_noninc_sig == sig)

        return True

    def _drop_noninc(self, sig: str, raw: str) -> bool:
        if self._should_drop_nonincremental(sig):
            self._dbg(f"DROP non-inc (dedupe): {sig} raw='{raw}'")
            if self.noninc_report_drop_event:
                self._event(f"DROP:NONINC_DEDUPE:{sig}")
            return True
        return False

    # ---------------- Main callback ----------------
    def on_group(self, msg: String) -> None:
        raw = _norm(msg.data)
        kind, direction, value, pos_index = parse_group_cmd(raw)

        self._dbg(f"IN='{raw}' -> {kind}, {direction}, {value}, {pos_index}")

        if kind == "UNKNOWN":
            self._event(f"ERR:UNKNOWN:{raw}")
            return

        if kind == "CANCEL":
            self._cancel_pulse()
            self._event("ACK:CANCEL")
            self._dbg("CANCEL -> /ur5/cmd_cancel pulse (no HL STOP)")
            return

        if kind == "STOP":
            self._cancel_pulse()
            self._event("ACK:STOP")
            self._pub_hl("STOP")
            return

        if kind == "UNLOCK":
            self._locked = False
            sig = self._mk_noninc_sig(kind, pos_index, direction, value)
            if self._drop_noninc(sig, raw):
                return
            self._event("ACK:UNLOCK")
            self._pub_hl("UNLOCK")
            return

        if kind == "LOCK":
            self._locked = True
            sig = self._mk_noninc_sig(kind, pos_index, direction, value)
            if self._drop_noninc(sig, raw):
                return
            self._event("ACK:LOCK")
            self._pub_hl("LOCK")
            return

        if self._locked:
            self._event(f"ERR:LOCKED:{raw}")
            self._dbg(f"LOCK-GATE drop: kind={kind}, raw='{raw}'")
            return

        if kind == "HOME":
            sig = self._mk_noninc_sig(kind, pos_index, direction, value)
            if self._drop_noninc(sig, raw):
                return
            self._event("ACK:HOME")
            self._pub_hl("HOME")
            return

        if kind == "BACK":
            sig = self._mk_noninc_sig(kind, pos_index, direction, value)
            if self._drop_noninc(sig, raw):
                return
            self._event("ACK:BACK")
            self._pub_hl("BACK")
            return

        if kind == "SPEED" and direction:
            sig = self._mk_noninc_sig(kind, pos_index, direction, value)
            if self._drop_noninc(sig, raw):
                return
            self._event(f"ACK:SPEED:{direction}")
            self._pub_hl(f"SPEED:{direction}")
            return

        if kind == "POS" and pos_index is not None:
            sig = self._mk_noninc_sig(kind, pos_index, direction, value)
            if self._drop_noninc(sig, raw):
                return
            self._event(f"ACK:POS:{pos_index}")
            self._pub_hl(f"POS:{pos_index}")
            return

        if kind == "TOP_VIEW" and pos_index is not None:
            sig = self._mk_noninc_sig(kind, pos_index, direction, value)
            if self._drop_noninc(sig, raw):
                return
            self._event(f"ACK:TOP_VIEW:{pos_index}")
            self._pub_hl(f"TOP_VIEW:{pos_index}")
            return

        if kind == "SIDE_VIEW" and pos_index is not None:
            sig = self._mk_noninc_sig(kind, pos_index, direction, value)
            if self._drop_noninc(sig, raw):
                return
            self._event(f"ACK:SIDE_VIEW:{pos_index}")
            self._pub_hl(f"SIDE_VIEW:{pos_index}")
            return

        if kind == "PICK":
            sig = self._mk_noninc_sig(kind, pos_index, direction, value)
            if self._drop_noninc(sig, raw):
                return
            self._event("ACK:PICK")
            self._pub_hl("PICK")
            return

        if kind == "PLACE":
            sig = self._mk_noninc_sig(kind, pos_index, direction, value)
            if self._drop_noninc(sig, raw):
                return
            self._event("ACK:PLACE")
            self._pub_hl("PLACE")
            return

        if kind == "MOVE" and direction:
            jog = value if value is not None else self.default_jog_m
            jog = self._clamp_jog(jog)

            sig = self._mk_inc_sig("MOVE", direction, jog)
            if self._should_drop_incremental(sig):
                self._dbg(f"DROP inc (debounce): {sig} raw='{raw}'")
                if self.inc_report_drop_event:
                    self._event(f"DROP:DEBOUNCE:{sig}")
                return

            self._event(f"ACK:MOVE:{direction}:{jog:g}")
            self._pub_hl(f"JOG:{direction}:{jog:g}")
            return

        # ROTATE (ทั่วไป) default = 5
        if kind == "ROTATE" and direction:
            deg = value if value is not None else self.default_rotate_deg
            deg = self._clamp_deg(deg)

            out_deg = deg
            if self.rotate_signed:
                out_deg = -abs(deg) if direction == "right" else abs(deg)

            sig = self._mk_inc_sig("ROTATE", direction, out_deg)
            if self._should_drop_incremental(sig):
                self._dbg(f"DROP inc (debounce): {sig} raw='{raw}'")
                if self.inc_report_drop_event:
                    self._event(f"DROP:DEBOUNCE:{sig}")
                return

            self._event(f"ACK:ROTATE:{direction}:{out_deg:g}")
            self._pub_hl(f"ROTATE:{direction}:{out_deg:g}")
            return

        # ROTATE_W3 (wrist3) default = 15 (แยกจาก rotate ปกติ)
        if kind == "ROTATE_W3" and direction:
            deg = value if value is not None else self.default_w3_rotate_deg
            deg = self._clamp_deg(deg)

            out_deg = deg
            if self.rotate_signed:
                out_deg = -abs(deg) if direction == "right" else abs(deg)

            sig = self._mk_inc_sig("ROTATE_W3", direction, out_deg)
            if self._should_drop_incremental(sig):
                self._dbg(f"DROP inc (debounce): {sig} raw='{raw}'")
                if self.inc_report_drop_event:
                    self._event(f"DROP:DEBOUNCE:{sig}")
                return

            self._event(f"ACK:ROTATE_W3:{direction}:{out_deg:g}")
            self._pub_hl(f"ROTATE_W3:{direction}:{out_deg:g}")
            return

        self._event(f"ERR:UNHANDLED:{raw}")


def main(args=None):
    rclpy.init(args=args)
    node = UR5CmdMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
