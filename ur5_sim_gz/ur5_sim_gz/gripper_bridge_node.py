import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState

try:
    from ur_msgs.srv import SetIO
except Exception:
    SetIO = None


class GripperBridgeNode(Node):
    def __init__(self):
        super().__init__("gripper_bridge_node")

        # Common params
        self.declare_parameter("mode", "real")  # sim | real
        self.declare_parameter("cmd_topic", "/Neural_parser/cmd_group")

        # SIM params
        self.declare_parameter("sim_topic", "/gripper_controller/commands")
        self.declare_parameter("sim_open", 0.01)
        self.declare_parameter("sim_close", 0.0)
        self.declare_parameter("sim_publish_also_in_real", True)

        # REAL (Digital IO) params
        self.declare_parameter("io_service", "/io_and_status_controller/set_io")
        self.declare_parameter("fun", 1)

        # single-pin mode
        self.declare_parameter("use_two_pins", True)
        self.declare_parameter("do_pin", 17)
        self.declare_parameter("open_state", 0.0)
        self.declare_parameter("close_state", 1.0)

        # two-pin mode (pulse)
        self.declare_parameter("open_pin", 16)
        self.declare_parameter("close_pin", 17)
        self.declare_parameter("pulse_ms", 200)

        # safety/robustness
        self.declare_parameter("interlock", True)
        self.declare_parameter("ensure_low_ms", 30)

        # Joint state params (NEW)
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_state_rate", 165.0)  # Hz

        # Load params
        self.mode = str(self.get_parameter("mode").value).strip().lower()
        self.cmd_topic = self.get_parameter("cmd_topic").value

        self.sim_topic = self.get_parameter("sim_topic").value
        self.sim_open = float(self.get_parameter("sim_open").value)
        self.sim_close = float(self.get_parameter("sim_close").value)
        self.sim_publish_also_in_real = bool(self.get_parameter("sim_publish_also_in_real").value)

        self.io_service = self.get_parameter("io_service").value
        self.fun = int(self.get_parameter("fun").value)

        self.use_two_pins = bool(self.get_parameter("use_two_pins").value)
        self.do_pin = int(self.get_parameter("do_pin").value)
        self.open_state = float(self.get_parameter("open_state").value)
        self.close_state = float(self.get_parameter("close_state").value)

        self.open_pin = int(self.get_parameter("open_pin").value)
        self.close_pin = int(self.get_parameter("close_pin").value)
        self.pulse_ms = int(self.get_parameter("pulse_ms").value)

        self.interlock = bool(self.get_parameter("interlock").value)
        self.ensure_low_ms = int(self.get_parameter("ensure_low_ms").value)

        self.publish_joint_states = bool(self.get_parameter("publish_joint_states").value)
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.joint_state_rate = float(self.get_parameter("joint_state_rate").value)

        # กัน rate ผิดพลาด
        if self.joint_state_rate <= 0.0:
            self.get_logger().warn("joint_state_rate <= 0.0, fallback to 10.0 Hz")
            self.joint_state_rate = 10.0

        # ROS interfaces
        self.pub_sim = self.create_publisher(Float64MultiArray, self.sim_topic, 10)
        self.sub = self.create_subscription(String, self.cmd_topic, self.on_cmd, 10)

        # Joint state publisher
        self.joint_pub = None
        if self.publish_joint_states:
            self.joint_pub = self.create_publisher(JointState, self.joint_state_topic, 10)

        # current finger state (for RViz)
        self.current_position = self.sim_open

        # Publish once at startup so RViz can see gripper immediately
        if self.publish_joint_states:
            self._publish_joint_state()

        # Publish continuously for RViz / robot_state_publisher
        self.js_timer = None
        if self.publish_joint_states:
            period = 1.0 / self.joint_state_rate
            self.js_timer = self.create_timer(period, self._publish_joint_state)

        # Real IO client
        self.io_cli = None
        if self.mode == "real":
            if SetIO is None:
                self.get_logger().error(
                    "ur_msgs.srv.SetIO not found. Install/overlay ur_msgs (UR ROS2 Driver)."
                )
            else:
                self.io_cli = self.create_client(SetIO, self.io_service)
                self.get_logger().info(f"[REAL] waiting for IO service: {self.io_service} ...")
                self.io_cli.wait_for_service(timeout_sec=2.0)
                if not self.io_cli.service_is_ready():
                    self.get_logger().warn("[REAL] IO service not ready yet. Will retry on demand.")

        # Log summary
        self.get_logger().info(f"mode={self.mode} cmd_topic={self.cmd_topic}")
        self.get_logger().info(
            f"[SIM] topic={self.sim_topic} open={self.sim_open} close={self.sim_close} "
            f"publish_also_in_real={self.sim_publish_also_in_real}"
        )

        if self.publish_joint_states:
            self.get_logger().info(
                f"[JOINT_STATE] topic={self.joint_state_topic} rate={self.joint_state_rate} Hz"
            )
        else:
            self.get_logger().warn("[JOINT_STATE] publishing disabled")

        if self.mode == "real":
            if self.use_two_pins:
                self.get_logger().info(
                    f"[REAL] two-pin TOOL DO: open_pin={self.open_pin}, "
                    f"close_pin={self.close_pin}, pulse_ms={self.pulse_ms}, "
                    f"interlock={self.interlock}"
                )
            else:
                self.get_logger().info(
                    f"[REAL] single-pin: do_pin={self.do_pin}, "
                    f"open_state={self.open_state}, close_state={self.close_state}"
                )
            self.get_logger().info(f"[REAL] io_service={self.io_service} fun={self.fun}")
    # SIM publis
    def _pub_sim(self, value: float):
        msg = Float64MultiArray()
        msg.data = [float(value), float(value)]
        self.pub_sim.publish(msg)
    # REAL SetIO cal
    def _set_do(self, pin: int, state: float) -> bool:
        if self.io_cli is None or SetIO is None:
            self.get_logger().warn("[REAL] IO client not available.")
            return False

        if not self.io_cli.service_is_ready():
            self.io_cli.wait_for_service(timeout_sec=1.0)
            if not self.io_cli.service_is_ready():
                self.get_logger().warn(f"[REAL] IO service not ready: {self.io_service}")
                return False

        req = SetIO.Request()
        req.fun = int(self.fun)
        req.pin = int(pin)
        req.state = float(state)

        future = self.io_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done() or future.result() is None:
            self.get_logger().error(f"[REAL] SetIO timeout/no response pin={pin} state={state}")
            return False

        ok = bool(future.result().success)
        if ok:
            self.get_logger().info(f"[REAL] SetIO OK pin={pin} state={state}")
        else:
            self.get_logger().warn(f"[REAL] SetIO FAILED pin={pin} state={state}")
        return ok

    def _pulse_do(self, pin: int, pulse_ms: int):
        if self._set_do(pin, 1.0):
            time.sleep(max(0.0, pulse_ms / 1000.0))
            self._set_do(pin, 0.0)

    def _interlock_low(self, pin: int):
        """Force pin low and wait a tiny bit so external controller sees stable low."""
        self._set_do(pin, 0.0)
        if self.ensure_low_ms > 0:
            time.sleep(self.ensure_low_ms / 1000.0)
    # Command handlin
    def do_open(self):
        # update state for RViz
        self.current_position = self.sim_open

        # publish immediately once (in addition to timer)
        self._publish_joint_state()

        # SIM visualization
        if self.mode == "sim" or self.sim_publish_also_in_real:
            self._pub_sim(self.sim_open)

        if self.mode != "real":
            return

        if self.use_two_pins:
            if self.interlock:
                self._interlock_low(self.close_pin)
            self._pulse_do(self.open_pin, self.pulse_ms)
        else:
            self._set_do(self.do_pin, self.open_state)

    def do_close(self):
        # update state for RViz
        self.current_position = self.sim_close

        # publish immediately once (in addition to timer)
        self._publish_joint_state()

        # SIM visualization
        if self.mode == "sim" or self.sim_publish_also_in_real:
            self._pub_sim(self.sim_close)

        if self.mode != "real":
            return

        if self.use_two_pins:
            if self.interlock:
                self._interlock_low(self.open_pin)
            self._pulse_do(self.close_pin, self.pulse_ms)
        else:
            self._set_do(self.do_pin, self.close_state)

    def on_cmd(self, msg: String):
        cmd = (msg.data or "").strip().upper()

        if cmd == "PICK":
            self.get_logger().info("[GRIPPER] PICK -> CLOSE")
            self.do_close()

        elif cmd == "PLACE":
            self.get_logger().info("[GRIPPER] PLACE -> OPEN")
            self.do_open()

        else:
            self.get_logger().warn(f"[GRIPPER] Unknown command: '{cmd}'")
    # JointState publis
    def _publish_joint_state(self):
        if not self.publish_joint_states or self.joint_pub is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = [
            "zimmer_left_finger_joint",
            "zimmer_right_finger_joint",
        ]

        msg.position = [
            float(self.current_position),
            float(self.current_position),
        ]

        self.joint_pub.publish(msg)


def main():
    rclpy.init()
    node = GripperBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()