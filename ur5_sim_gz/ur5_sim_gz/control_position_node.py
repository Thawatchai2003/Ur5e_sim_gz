#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ControlPositionNode(Node):

    def __init__(self):
        super().__init__('control_position_node')
        self.subscription = self.create_subscription(String,'/Neural_parser/cmd_group',self.cmd_callback,10)
        self.view_subscription = self.create_subscription(String,'/ur5/pos_random_view',self.view_callback,10)
        self.publisher = self.create_publisher(String,'/control_position/cmd_mapper',10)
        self.current_view = None
        self.locked_view = None
        self.get_logger().info("ControlPositionNode started.\n""Sub: /voice/cmd_group\n""Pub: /voice/cmd_mapper")

    def cmd_callback(self, msg: String):
        cmd = msg.data.strip()
        if cmd == "HOME":
            self.current_view = None
            self.locked_view = None
            self.get_logger().info("HOME detected → View state reset")

        # Detect view change
        if cmd.startswith("SIDE_VIEW_"):
            self.current_view = "side"
            self.get_logger().info("Entered SIDE mode")

        elif cmd.startswith("TOP_VIEW_"):
            self.current_view = "top"
            self.get_logger().info("Entered TOP mode")

        # PICK → lock current view
        if cmd == "PICK":
            if self.current_view is None:
                self.get_logger().warn("PICK received but current_view is None (random view not received yet)")
            else:
                self.locked_view = self.current_view
                self.get_logger().info(f"{self.locked_view.upper()} locked (object picked)")
                
        # PLACE → clear lock
        if cmd == "PLACE":
            self.locked_view = None
            self.get_logger().info("View lock cleared (object placed)")

        # Force POS when locked
        if self.locked_view and cmd.startswith("POS_"):
            pos_number = cmd.split("_")[1]

            if self.locked_view == "side":
                forced_cmd = f"SIDE_VIEW_{pos_number}"
            else:
                forced_cmd = f"TOP_VIEW_{pos_number}"
            self.get_logger().info(f"{self.locked_view.upper()} lock active → forcing {cmd} → {forced_cmd}")
            out = String()
            out.data = forced_cmd
            self.publisher.publish(out)
            return

        # default forward
        out = String()
        out.data = cmd
        self.publisher.publish(out)

    def view_callback(self, msg: String):
        view = msg.data.strip().upper()
        if view == "TOP":
            self.current_view = "top"
            self.get_logger().info("Current view set from random topic → TOP")

        elif view == "SIDE":
            self.current_view = "side"
            self.get_logger().info("Current view set from random topic → SIDE")

        else:
            self.get_logger().warn(f"Unknown view from topic: {view}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()