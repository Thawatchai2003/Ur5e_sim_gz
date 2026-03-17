# ============================================================
#  File:        ur5_executor_node.py
#  Node Name:   UR5ExecutorNode
#
#  Description:
#  ------------------------------------------------------------
#  UR5ExecutorNode is the high-level motion executor for the
#  UR5/UR5e robotic arm. It handles behavior logic for motion
#  commands such as POSITION presets, HOME/BACK routines,
#  MoveJ/MoveL execution, workspace-restricted jogging, safety
#  lifting, and trajectory delivery to the controller.
#
#  Core Responsibilities:
#    ✓ Parse high-level commands from /ur5/high_level_cmd
#    ✓ Execute MoveJ and MoveL with braking / soft-stop curves
#    ✓ Safe-lift system (MoveL up before MoveJ transition)
#    ✓ Center transit poses for safer motion at specific POS
#    ✓ Per-POS workspace limit setup (Z/R/Theta constraints)
#    ✓ Jog controls with limit enforcement (fwd/back/left/right/up/down)
#    ✓ Joint-limit protection + automatic STOP + LOCK
#    ✓ Fallback: MoveL → motion planning when Cartesian fails
#
#  Not responsible for:
#    ✗ Speech or NLU interpretation
#    ✗ Dialog logic (handled externally)
#    ✗ Hardware-level URCaps configuration
#    ✗ Servo driver / RTDE low-layer actuation
#
#  ------------------------------------------------------------
#  Communication Overview:
#
#  Subscribed Topics:
#    • /joint_states           (sensor_msgs/JointState)
#    • /ur5/high_level_cmd     (std_msgs/String)
#    • /ur5/cmd_cancel         (std_msgs/Bool)
#
#  Published Topics:
#    • /ur5/command_trajectory (trajectory_msgs/JointTrajectory)
#ข้อ
#  Debug Topics :
#    • /ur5/executor_dbg       (std_msgs/String)  - all _dbg() messages
#
#  Services Used:
#    • /compute_cartesian_path (GetCartesianPath)  - MoveL generation
#    • /plan_kinematic_path    (GetMotionPlan)     - Fallback planner
#
#  TF Frames:
#    • base_frame → ee_link lookup for workspace & MoveL reference
#
#  ------------------------------------------------------------
#  Safety Guarantees:
#    • Joint-limit detection with LOCK + soft-stop trajectory
#    • Automatic lift when transitioning between POS targets
#    • Workspace boundaries per POS (Z / R / Theta locked region)
#    • Blocking prevention for MoveL/TF/Service timeouts
#
#  ------------------------------------------------------------
#  Author:       Mr. Thawatchai Thongbai
#  Affiliation:  Prince of Songkla University (PSU)
#  Project:      UR5e Voice-Control / ROS2 Motion Execution Layer
#  Version:      v1.0.0
#  Last Update:  2026-01-01
# ============================================================

import math
import random
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter

from std_msgs.msg import String, Bool, Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from geometry_msgs.msg import Pose

from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from moveit_msgs.msg import (
    RobotState,
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive

import tf2_ros


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _parse_float(x: str) -> Optional[float]:
    try:
        return float(x)
    except Exception:
        return None


class UR5ExecutorNode(Node):
    """
    UR5 Executor Node
    High-level command topic: /ur5/high_level_cmd (std_msgs/String)

      - HOME / BACK_HOME          -> SAFE MoveL-lift (Z+) then MoveJ to HOME (smooth + braking)
      - BACK / RETURN             -> SAFE MoveL-lift (Z+) then MoveJ back to prev (if transit_on_back=True)

      - POS:1..5                  -> (random) choose top view or side view
      - top_viewN                 -> force preset joint of top view (POS:N)
      - side_viewN                -> force preset joint of side view (side_pos_N)
                                     separated from POS:1..5
                                     can optionally pass center pose (side_use_center_transit)

      pattern POS / side view:
         current POS (might be low)
           -> MoveL up in Z (lift_dz per preset)
           -> (if center transit enabled) MoveJ to high center pose
           -> MoveJ to target (with braking)
           -> at target -> MoveL down in Z by down_dz
           -> set workspace limit (Z/R/T) for JOG around that POS

      - JOG:dir:meter             -> Jog via joint / MoveL
           forward/back = radial from base (out/in) + R-LIMIT
           left/right   = tangential (left/right wrt world)   + T-LIMIT
           up/down      = MoveL in Z (world)                  + Z-LIMIT

      - ROTATE:left/right:deg     -> rotate base joint
      - ROTATE_W3:left/right:deg  -> rotate wrist_3_joint
      - MOVEL:dx:dy:dz            -> MoveL Cartesian (ComputeCartesianPath + fallback)
      - STOP                      -> SOFT-STOP + LOCK (ignore new commands until UNLOCK)
      - UNLOCK                    -> clear LOCK

    Joint limit:
      - joint_limits_min / joint_limits_max (rad)
      - joint_limit_margin (rad) is subtracted from both sides
      - if any trajectory point violates limits -> immediately stop + LOCK (require UNLOCK)
    """

    def __init__(self):
        super().__init__("ur5_executor_node")
        self.ws_y_ref: float = 0.0
        self.ws_y_min: float = 0.0
        self.ws_y_max: float = 0.0
        self.ws_x_ref: float = 0.0
        self.ws_x_min: float = 0.0
        self.ws_x_max: float = 0.0

        self.active_pos_idx: Optional[int] = None
        self.active_pos_is_side: bool = False
        

        self.declare_parameter("post_drop_wait", 5.0)         # wait after MoveL down
        self.declare_parameter("post_drop_auto_rise", True)   # enable auto rise after drop
        

        self.declare_parameter("dbg_enable", True)
        self.declare_parameter("dbg_to_topic", True)
        self.declare_parameter("dbg_topic", "/ur5/executor_dbg")
        self.declare_parameter("dbg_also_console", False)  

        # ---------------- params ----------------
        self.declare_parameter("debug", True) 

        self.declare_parameter("topic_hl", "/mapper/high_level_cmd")
        self.declare_parameter("topic_cancel", "/mapper/cmd_cancel")
        self.declare_parameter("topic_joint_states", "/joint_states")
        self.declare_parameter("topic_traj_out", "/ur5/command_trajectory")
        self.declare_parameter("topic_pos_random_view", "/ur5/pos_random_view")
        

        # Move_up_Gripper
        self.post_drop_wait = float(self.get_parameter("post_drop_wait").value)
        self.post_drop_auto_rise = bool(self.get_parameter("post_drop_auto_rise").value)

        self.declare_parameter(
            "joint_names",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

        self.declare_parameter("base_time", 1.0)
        self.declare_parameter("min_time", 0.4)
        self.declare_parameter("max_joint_vel", 0.8)  # rad/s

        self.declare_parameter("meter_to_rad_gain", 6.0)
        self.declare_parameter("max_step_rad", 1.5)

        # smooth factor
        self.declare_parameter("traj_global_scale", 1.3)
        self.declare_parameter("traj_pos_scale", 1.6)

        # POS:1-5 random topview/sideview
        self.declare_parameter("pos_random_view", True)

        # HOME + POS1-POS5 (rad) – top view
        self.declare_parameter("home", [0.0, -1.57, 0.0, -1.57, 0.0, 0.0])

        # TOP VIEW POS1-5
        self.declare_parameter(
            "pos_1",
            [-0.4948, -1.2495, 1.0119, -1.3327, -1.5656, -0.9222],
        )
        self.declare_parameter(
            "pos_2",
            [1.3598, -1.0987, 0.7056, -1.1811, -1.5664, -0.6381],
        )
        self.declare_parameter(
            "pos_3",
            [3.2699, -1.3008, 1.2572, -1.5266, -1.5650, -0.2688],
        )
        self.declare_parameter(
            "pos_4",
            [2.5157, -1.2085, 1.1669, -1.5174, -1.5403, -0.9936],
        )
        self.declare_parameter(
            "pos_5",
            [0.2648, -1.1983, 1.1554, -1.5275, -1.5403, -0.1721],
        )

        # Move up (Z-axis) Top_view
        self.declare_parameter("pos_1_lift_dz", 0.20)
        self.declare_parameter("pos_2_lift_dz", 0.20)
        self.declare_parameter("pos_3_lift_dz", 0.20)
        self.declare_parameter("pos_4_lift_dz", 0.25)
        self.declare_parameter("pos_5_lift_dz", 0.25)

        # Move down (Z-axis) Top_view
        self.declare_parameter("pos_1_down_dz", 0.05)
        self.declare_parameter("pos_2_down_dz", 0.05)
        self.declare_parameter("pos_3_down_dz", 0.05)
        self.declare_parameter("pos_4_down_dz", 0.25)
        self.declare_parameter("pos_5_down_dz", 0.25)

        # Side view (Pos1-Pos5)

        # 2 [2.2143, -1.1994, 2.0600, -0.9231, 0.6468, -1.9441]
        # 5 [0.3573, -1.4128, 1.8181, -0.4026, 1.9289, -1.9823]
        self.declare_parameter(
            "side_pos_1",
            [-0.8397, -1.4689, 2.0684, -3.7523, -0.7353, 1.1947],
        )
        self.declare_parameter(
            "side_pos_2",
            [1.0854, -1.7270, 2.0602, -3.5095, -1.0448, 1.1947],
        )
        self.declare_parameter(
            "side_pos_3",
            [3.4325, -1.5837, 2.0312, -3.5816, -1.8188, 1.1366],
        )
        self.declare_parameter(
            "side_pos_4",
            [2.1927, -1.5258, 1.4591, 0.0202, 0.6657, -1.9361],   
        )
        self.declare_parameter(
            "side_pos_5",
            [0.3786, -1.4774, 1.3781, 0.0970, 1.8808, -1.9668],
        )

        # Move up (Z-axis) Side_view
        self.declare_parameter("side_pos_1_lift_dz", 0.30)
        self.declare_parameter("side_pos_2_lift_dz", 0.30)
        self.declare_parameter("side_pos_3_lift_dz", 0.30)
        self.declare_parameter("side_pos_4_lift_dz", 0.35) #30
        self.declare_parameter("side_pos_5_lift_dz", 0.35) #30

        # Move down (Z-axis) Side_view
        self.declare_parameter("side_pos_1_down_dz", 0.17)
        self.declare_parameter("side_pos_2_down_dz", 0.30)
        self.declare_parameter("side_pos_3_down_dz", 0.19) 
        self.declare_parameter("side_pos_4_down_dz", 0.40) #10
        self.declare_parameter("side_pos_5_down_dz", 0.40) #21

        # Define tolerance (rad)
        self.declare_parameter("pos_reached_tol_rad", 0.02)

        # ---------------- Safe transit / joint mid lift (fallback joint) ----------------
        self.declare_parameter("use_transit", True)
        self.declare_parameter("use_dynamic_lift", True)

        # Center Pose dynamic
        self.declare_parameter("transit", [0.0, -1.20, 1.20, -1.57, -1.57, 0.0])

        # Pos_center_pose
        self.declare_parameter(
            "pos_center_pose",
            [0.7679, -1.0472, -0.5236, -1.5708, 0.0, 0.0],
        )

        # Pos_center_pose 1-5
        self.declare_parameter(
            "pos_center_pose_15",
            [0.767945, -1.902409, 1.27409, -2.141593, -1.53589, -0.418879],
        )
        self.declare_parameter(
            "pos_center_pose_2",
            [1.570796, -1.117011, 0.680678, -1.186824, -1.570796, -0.401426],
        )
        self.declare_parameter(
            "pos_center_pose_34",
            [2.5307, -2.0769, 1.4312, -2.4260, -1.5184, -0.38397],
        )

        # High-Pass Mode (Top_view ,Side_view)
        self.declare_parameter("pos_use_center_transit", True)
        self.declare_parameter("side_use_center_transit", True)

        # Dynamic Lift Height
        self.declare_parameter("lift_shoulder2", -1.20)  # shoulder_lift_joint
        self.declare_parameter("lift_elbow", 1.20)       # elbow_joint
        self.declare_parameter("lift_wrist1", -1.57)     # wrist_1_joint

        self.declare_parameter("transit_time", 1.2)
        self.declare_parameter("target_extra_time", 1.2)

        self.declare_parameter("transit_on_back", True)

        # ---------------- MoveL (Cartesian) params ----------------
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("base_frame", "base_link_inertia")
        self.declare_parameter("ee_link", "tool0")

        self.declare_parameter("movel_max_step", 0.001)
        self.declare_parameter("movel_jump_threshold", 0.0)
        self.declare_parameter("movel_avoid_collisions", False)
        self.declare_parameter("movel_num_waypoints", 10)

        self.declare_parameter("fallback_enable", True)
        self.declare_parameter("fallback_pos_tol", 0.003)   # meters
        self.declare_parameter("fallback_ori_tol", 0.20)    # radians
        self.declare_parameter("fallback_planning_time", 2.0)
        self.declare_parameter("fallback_num_attempts", 5)

        # Smooth MoveL
        self.declare_parameter("movel_traj_global_scale", 1.4)

        # MoveL-lift params (generic lift)
        self.declare_parameter("movel_lift_dz", 0.09)             # default 9cm
        self.declare_parameter("movel_lift_min_fraction", 0.70)   # fraction < 0.7 -> fallback joint mid
        self.declare_parameter("movel_lift_wait_after", 3.0)      # MoveL-to-MoveJ dwell 

        # Jog via MoveL
        self.declare_parameter("jog_updown_use_movel", True)
        self.declare_parameter("jog_updown_max_dz", 0.10)  # 10cm

        self.declare_parameter("jog_lr_use_movel", True)
        self.declare_parameter("jog_lr_max_dy", 0.10)      # 10cm

        self.declare_parameter("jog_fb_use_movel", True)
        self.declare_parameter("jog_fb_max_dx", 0.10)      # 10cm

        # Workspace limit params (Z / R / LEFT-RIGHT)
        self.declare_parameter("ws_enable", True)

        # global default workspace limits
        self.declare_parameter("ws_z_down_limit", 0.03)
        self.declare_parameter("ws_z_up_limit", 0.10)
        self.declare_parameter("ws_r_back_limit", 0.05)
        self.declare_parameter("ws_r_forward_limit", 0.05)
        self.declare_parameter("ws_lr_left_limit", 0.05)
        self.declare_parameter("ws_lr_right_limit", 0.05)

        # Per-POS top view workspace limits
        self.declare_parameter("ws_top_pos_1_z_down_limit", 0.18)
        self.declare_parameter("ws_top_pos_1_z_up_limit", 0.10)
        self.declare_parameter("ws_top_pos_1_r_back_limit", 0.15)
        self.declare_parameter("ws_top_pos_1_r_forward_limit", 0.15)
        self.declare_parameter("ws_top_pos_1_lr_left_limit", 0.15)
        self.declare_parameter("ws_top_pos_1_lr_right_limit", 0.15)

        self.declare_parameter("ws_top_pos_2_z_down_limit", 0.24)
        self.declare_parameter("ws_top_pos_2_z_up_limit", 0.10)
        self.declare_parameter("ws_top_pos_2_r_back_limit", 0.15)
        self.declare_parameter("ws_top_pos_2_r_forward_limit", 0.15)
        self.declare_parameter("ws_top_pos_2_lr_left_limit", 0.15)
        self.declare_parameter("ws_top_pos_2_lr_right_limit", 0.15)

        self.declare_parameter("ws_top_pos_3_z_down_limit", 0.15)
        self.declare_parameter("ws_top_pos_3_z_up_limit", 0.10)
        self.declare_parameter("ws_top_pos_3_r_back_limit", 0.15)
        self.declare_parameter("ws_top_pos_3_r_forward_limit", 0.30)
        self.declare_parameter("ws_top_pos_3_lr_left_limit", 0.15)
        self.declare_parameter("ws_top_pos_3_lr_right_limit", 0.15)

        self.declare_parameter("ws_top_pos_4_z_down_limit", 0.34) 
        self.declare_parameter("ws_top_pos_4_z_up_limit", 0.10)
        self.declare_parameter("ws_top_pos_4_r_back_limit", 0.15)
        self.declare_parameter("ws_top_pos_4_r_forward_limit", 0.15)
        self.declare_parameter("ws_top_pos_4_lr_left_limit", 0.15)
        self.declare_parameter("ws_top_pos_4_lr_right_limit", 0.15)

        self.declare_parameter("ws_top_pos_5_z_down_limit", 0.35) 
        self.declare_parameter("ws_top_pos_5_z_up_limit", 0.10)
        self.declare_parameter("ws_top_pos_5_r_back_limit", 0.15)
        self.declare_parameter("ws_top_pos_5_r_forward_limit", 0.15)
        self.declare_parameter("ws_top_pos_5_lr_left_limit", 0.15)
        self.declare_parameter("ws_top_pos_5_lr_right_limit", 0.15)

        # Per-POS side view workspace limits
        self.declare_parameter("ws_side_pos_1_z_down_limit", 0.26)
        self.declare_parameter("ws_side_pos_1_z_up_limit", 0.10)
        self.declare_parameter("ws_side_pos_1_r_back_limit", 0.15)
        self.declare_parameter("ws_side_pos_1_r_forward_limit", 0.15)
        self.declare_parameter("ws_side_pos_1_lr_left_limit", 0.15)
        self.declare_parameter("ws_side_pos_1_lr_right_limit", 0.15)

        self.declare_parameter("ws_side_pos_2_z_down_limit", 0.37)
        self.declare_parameter("ws_side_pos_2_z_up_limit", 0.10)
        self.declare_parameter("ws_side_pos_2_r_back_limit", 0.15)
        self.declare_parameter("ws_side_pos_2_r_forward_limit", 0.15)
        self.declare_parameter("ws_side_pos_2_lr_left_limit", 0.15)
        self.declare_parameter("ws_side_pos_2_lr_right_limit", 0.15)

        self.declare_parameter("ws_side_pos_3_z_down_limit", 0.33)
        self.declare_parameter("ws_side_pos_3_z_up_limit", 0.10)
        self.declare_parameter("ws_side_pos_3_r_back_limit", 0.15)
        self.declare_parameter("ws_side_pos_3_r_forward_limit", 0.15)
        self.declare_parameter("ws_side_pos_3_lr_left_limit", 0.15)
        self.declare_parameter("ws_side_pos_3_lr_right_limit", 0.15)

        self.declare_parameter("ws_side_pos_4_z_down_limit", 0.49) #15
        self.declare_parameter("ws_side_pos_4_z_up_limit", 0.10)
        self.declare_parameter("ws_side_pos_4_r_back_limit", 0.15)
        self.declare_parameter("ws_side_pos_4_r_forward_limit", 0.15)
        self.declare_parameter("ws_side_pos_4_lr_left_limit", 0.15)
        self.declare_parameter("ws_side_pos_4_lr_right_limit", 0.15)

        self.declare_parameter("ws_side_pos_5_z_down_limit", 0.49) #25
        self.declare_parameter("ws_side_pos_5_z_up_limit", 0.10)
        self.declare_parameter("ws_side_pos_5_r_back_limit", 0.15)
        self.declare_parameter("ws_side_pos_5_r_forward_limit", 0.15)
        self.declare_parameter("ws_side_pos_5_lr_left_limit", 0.15)
        self.declare_parameter("ws_side_pos_5_lr_right_limit", 0.15)

        # Speed mode params
        self.declare_parameter("speed_mode", "normal")  # "slow" / "normal" / "fast"
        self.declare_parameter("speed_scale_slow", 1.8)
        self.declare_parameter("speed_scale_normal", 1.0)
        self.declare_parameter("speed_scale_fast", 0.6)

        # Joint limit params
        self.declare_parameter(
            "joint_limits_min",
            [-6.283, -6.283, -6.283, -6.283, -6.283, -6.283],
        )
        self.declare_parameter(
            "joint_limits_max",
            [6.283, 6.283, 6.283, 6.283, 6.283, 6.283],
        )
        self.declare_parameter("joint_limit_margin", 0.05)

        # MAP_Position
        self.declare_parameter("topic_tcp_z", "/ur5/tcp_z_position")

        # TCP Calibrate
        self.surface_height = {
            ("TOP", 5): 0.007365375374325578,
            ("TOP", 4): 0.01724184377549795,
            ("TOP", 3): 0.23901945352820272,
            ("TOP", 2): 0.22048952284580753, #0.34226560532972894
            ("TOP", 1): 0.2568775230167647,

            # ----- SIDE ----
            ("SIDE", 5): 0.030494,
            ("SIDE", 4): 0.018637,
            ("SIDE", 3): 0.106773,
            ("SIDE", 2): 0.066907,    #0.0962,0.09690073687260636
            ("SIDE", 1): 0.125301,
        }
        self.pick_offset_memory: Optional[float] = None
        self.last_pick_surface_z: Optional[float] = None
        self.current_pos_id: Optional[int] = None
        self.current_view: Optional[str] = None

        # Center Mode 
        self.declare_parameter("center_mode", "DIRECT")   # "CENTER" or "DIRECT"
        self.center_mode = str(
            self.get_parameter("center_mode").value
        ).upper()

        # Central Lift Mode 
        self.declare_parameter("use_central_lift", True)
        self.declare_parameter("central_lift_dz", 0.35)
        self.declare_parameter("topic_movel_stop", "/ur5/force_condition_result")

        # ---------------- load params ----------------
        self.debug = bool(self.get_parameter("debug").value)

        # DBG load
        self.dbg_enable = bool(self.get_parameter("dbg_enable").value)
        self.dbg_to_topic = bool(self.get_parameter("dbg_to_topic").value)
        self.dbg_topic = str(self.get_parameter("dbg_topic").value)
        self.dbg_also_console = bool(self.get_parameter("dbg_also_console").value)

        # DBG publisher
        self.dbg_pub = self.create_publisher(String, self.dbg_topic, 50)

        self.topic_hl = str(self.get_parameter("topic_hl").value)
        self.topic_cancel = str(self.get_parameter("topic_cancel").value)
        self.topic_joint_states = str(self.get_parameter("topic_joint_states").value)
        self.topic_traj_out = str(self.get_parameter("topic_traj_out").value)
        self.topic_movel_stop = str(self.get_parameter("topic_movel_stop").value)

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.base_time = float(self.get_parameter("base_time").value)
        self.min_time = float(self.get_parameter("min_time").value)
        self.max_joint_vel = float(self.get_parameter("max_joint_vel").value)

        self.meter_to_rad_gain = float(self.get_parameter("meter_to_rad_gain").value)
        self.max_step_rad = float(self.get_parameter("max_step_rad").value)

        # smooth factor
        self.traj_global_scale = float(self.get_parameter("traj_global_scale").value)
        self.traj_pos_scale = float(self.get_parameter("traj_pos_scale").value)

        # random POS top/side
        self.pos_random_view = bool(self.get_parameter("pos_random_view").value)

        self.home_joints: List[float] = list(self.get_parameter("home").value)

        # POS map (top view)
        self.pos_map: Dict[int, List[float]] = {
            1: list(self.get_parameter("pos_1").value),
            2: list(self.get_parameter("pos_2").value),
            3: list(self.get_parameter("pos_3").value),
            4: list(self.get_parameter("pos_4").value),
            5: list(self.get_parameter("pos_5").value),
        }

        # POS lift map (top view)
        self.pos_lift_map: Dict[int, float] = {
            1: float(self.get_parameter("pos_1_lift_dz").value),
            2: float(self.get_parameter("pos_2_lift_dz").value),
            3: float(self.get_parameter("pos_3_lift_dz").value),
            4: float(self.get_parameter("pos_4_lift_dz").value),
            5: float(self.get_parameter("pos_5_lift_dz").value),
        }

        # POS post-down (top view)
        self.pos_down_map: Dict[int, float] = {
            1: float(self.get_parameter("pos_1_down_dz").value),
            2: float(self.get_parameter("pos_2_down_dz").value),
            3: float(self.get_parameter("pos_3_down_dz").value),
            4: float(self.get_parameter("pos_4_down_dz").value),
            5: float(self.get_parameter("pos_5_down_dz").value),
        }

        # side view map
        self.side_pos_map: Dict[int, List[float]] = {
            1: list(self.get_parameter("side_pos_1").value),
            2: list(self.get_parameter("side_pos_2").value),
            3: list(self.get_parameter("side_pos_3").value),
            4: list(self.get_parameter("side_pos_4").value),
            5: list(self.get_parameter("side_pos_5").value),
        }

        self.side_lift_map: Dict[int, float] = {
            1: float(self.get_parameter("side_pos_1_lift_dz").value),
            2: float(self.get_parameter("side_pos_2_lift_dz").value),
            3: float(self.get_parameter("side_pos_3_lift_dz").value),
            4: float(self.get_parameter("side_pos_4_lift_dz").value),
            5: float(self.get_parameter("side_pos_5_lift_dz").value),
        }

        self.side_down_map: Dict[int, float] = {
            1: float(self.get_parameter("side_pos_1_down_dz").value),
            2: float(self.get_parameter("side_pos_2_down_dz").value),
            3: float(self.get_parameter("side_pos_3_down_dz").value),
            4: float(self.get_parameter("side_pos_4_down_dz").value),
            5: float(self.get_parameter("side_pos_5_down_dz").value),
        }

        self.pos_reached_tol = float(self.get_parameter("pos_reached_tol_rad").value)

        self.use_transit = bool(self.get_parameter("use_transit").value)
        self.use_dynamic_lift = bool(self.get_parameter("use_dynamic_lift").value)
        self.transit_pose: List[float] = list(self.get_parameter("transit").value)

        self.lift_shoulder2 = float(self.get_parameter("lift_shoulder2").value)
        self.lift_elbow = float(self.get_parameter("lift_elbow").value)
        self.lift_wrist1 = float(self.get_parameter("lift_wrist1").value)

        self.transit_time = float(self.get_parameter("transit_time").value)
        self.target_extra_time = float(self.get_parameter("target_extra_time").value)
        self.transit_on_back = bool(self.get_parameter("transit_on_back").value)

        # pos_center_pose
        self.pos_center_pose: List[float] = list(self.get_parameter("pos_center_pose").value)

        # pos_center_pose 1-5
        self.pos_center_pose_15: List[float] = list(self.get_parameter("pos_center_pose_15").value)
        self.pos_center_pose_2: List[float] = list(self.get_parameter("pos_center_pose_2").value)
        self.pos_center_pose_34: List[float] = list(self.get_parameter("pos_center_pose_34").value)
        self.pos_use_center_transit = bool(self.get_parameter("pos_use_center_transit").value)
        self.side_use_center_transit = bool(self.get_parameter("side_use_center_transit").value)

        # MoveL specific
        self.group_name = str(self.get_parameter("group_name").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_link = str(self.get_parameter("ee_link").value)

        self.movel_max_step = float(self.get_parameter("movel_max_step").value)
        self.movel_jump_threshold = float(self.get_parameter("movel_jump_threshold").value)
        self.movel_avoid_collisions = bool(self.get_parameter("movel_avoid_collisions").value)
        self.movel_num_waypoints = int(self.get_parameter("movel_num_waypoints").value)

        self.fallback_enable = bool(self.get_parameter("fallback_enable").value)
        self.fallback_pos_tol = float(self.get_parameter("fallback_pos_tol").value)
        self.fallback_ori_tol = float(self.get_parameter("fallback_ori_tol").value)
        self.fallback_planning_time = float(self.get_parameter("fallback_planning_time").value)
        self.fallback_num_attempts = int(self.get_parameter("fallback_num_attempts").value)

        # MoveL time scale
        self.movel_traj_global_scale = float(self.get_parameter("movel_traj_global_scale").value)

        # MoveL-lift params
        self.movel_lift_dz = float(self.get_parameter("movel_lift_dz").value)
        self.movel_lift_min_fraction = float(self.get_parameter("movel_lift_min_fraction").value)
        self.movel_lift_wait_after = float(self.get_parameter("movel_lift_wait_after").value)

        # Jog via MoveL
        self.jog_updown_use_movel = bool(self.get_parameter("jog_updown_use_movel").value)
        self.jog_updown_max_dz = float(self.get_parameter("jog_updown_max_dz").value)

        self.jog_lr_use_movel = bool(self.get_parameter("jog_lr_use_movel").value)
        self.jog_lr_max_dy = float(self.get_parameter("jog_lr_max_dy").value)

        self.jog_fb_use_movel = bool(self.get_parameter("jog_fb_use_movel").value)
        self.jog_fb_max_dx = float(self.get_parameter("jog_fb_max_dx").value)

        # Workspace limits (global)
        self.ws_enable = bool(self.get_parameter("ws_enable").value)
        self.ws_z_down_limit = float(self.get_parameter("ws_z_down_limit").value)
        self.ws_z_up_limit = float(self.get_parameter("ws_z_up_limit").value)
        self.ws_r_back_limit = float(self.get_parameter("ws_r_back_limit").value)
        self.ws_r_forward_limit = float(self.get_parameter("ws_r_forward_limit").value)
        self.ws_lr_left_limit = float(self.get_parameter("ws_lr_left_limit").value)
        self.ws_lr_right_limit = float(self.get_parameter("ws_lr_right_limit").value)

        # Per-POS top view workspace maps
        self.ws_top_z_down_map: Dict[int, float] = {
            i: float(self.get_parameter(f"ws_top_pos_{i}_z_down_limit").value)
            for i in range(1, 6)
        }
        self.ws_top_z_up_map: Dict[int, float] = {
            i: float(self.get_parameter(f"ws_top_pos_{i}_z_up_limit").value)
            for i in range(1, 6)
        }
        self.ws_top_r_back_map: Dict[int, float] = {
            i: float(self.get_parameter(f"ws_top_pos_{i}_r_back_limit").value)
            for i in range(1, 6)
        }
        self.ws_top_r_fwd_map: Dict[int, float] = {
            i: float(self.get_parameter(f"ws_top_pos_{i}_r_forward_limit").value)
            for i in range(1, 6)
        }
        self.ws_top_lr_left_map: Dict[int, float] = {
            i: float(self.get_parameter(f"ws_top_pos_{i}_lr_left_limit").value)
            for i in range(1, 6)
        }
        self.ws_top_lr_right_map: Dict[int, float] = {
            i: float(self.get_parameter(f"ws_top_pos_{i}_lr_right_limit").value)
            for i in range(1, 6)
        }

        # Per-POS side view workspace maps
        self.ws_side_z_down_map: Dict[int, float] = {
            i: float(self.get_parameter(f"ws_side_pos_{i}_z_down_limit").value)
            for i in range(1, 6)
        }
        self.ws_side_z_up_map: Dict[int, float] = {
            i: float(self.get_parameter(f"ws_side_pos_{i}_z_up_limit").value)
            for i in range(1, 6)
        }
        self.ws_side_r_back_map: Dict[int, float] = {
            i: float(self.get_parameter(f"ws_side_pos_{i}_r_back_limit").value)
            for i in range(1, 6)
        }
        self.ws_side_r_fwd_map: Dict[int, float] = {
            i: float(self.get_parameter(f"ws_side_pos_{i}_r_forward_limit").value)
            for i in range(1, 6)
        }
        self.ws_side_lr_left_map: Dict[int, float] = {
            i: float(self.get_parameter(f"ws_side_pos_{i}_lr_left_limit").value)
            for i in range(1, 6)
        }
        self.ws_side_lr_right_map: Dict[int, float] = {
            i: float(self.get_parameter(f"ws_side_pos_{i}_lr_right_limit").value)
            for i in range(1, 6)
        }

        # ---------------- load speed mode ----------------
        self.speed_mode = str(self.get_parameter("speed_mode").value).lower()
        self.speed_scale_slow = float(self.get_parameter("speed_scale_slow").value)
        self.speed_scale_normal = float(self.get_parameter("speed_scale_normal").value)
        self.speed_scale_fast = float(self.get_parameter("speed_scale_fast").value)

        # ---------------- load joint limits ----------------
        self.joint_limits_min: List[float] = list(self.get_parameter("joint_limits_min").value)
        self.joint_limits_max: List[float] = list(self.get_parameter("joint_limits_max").value)
        self.joint_limit_margin: float = float(self.get_parameter("joint_limit_margin").value)

        self.use_central_lift = bool(self.get_parameter("use_central_lift").value)
        self.central_lift_dz = float(self.get_parameter("central_lift_dz").value)

        # load topic_pos_random_view
        self.topic_pos_random_view = str(self.get_parameter("topic_pos_random_view").value)

        # ---------------- state ----------------
        self.current_joints: Optional[List[float]] = None
        self.prev_joints: Optional[List[float]] = None
        self.locked = False
        self.last_commanded_joints: Optional[List[float]] = None
        self.last_command_was_movel: bool = False

        # Last high-level command received from /mapper/high_level_cmd
        self.last_hl_cmd_raw: str = ""
        self.last_hl_cmd_name: str = ""
        self.place_rise_permitted: bool = False

        self.last_js: Optional[JointState] = None
        self.movel_busy: bool = False
        self.movel_goal_pose: Optional[Pose] = None

        self.movel_executing: bool = False
        self.movel_exec_timer = None 

        # Safe MoveL-lift
        self.lift_busy: bool = False
        self.lift_target: Optional[List[float]] = None

        # Pattern POS / side view -> MoveL
        self.pending_pos_down_idx: Optional[int] = None
        self.pending_pos_down_dz: float = 0.0
        self.pending_pos_down_target: Optional[List[float]] = None
        self.pending_pos_down_is_side: bool = False  # distinguish POS/top and side_view

        # POS/side center transit:
        self.pending_center_idx: Optional[int] = None
        self.pending_center_pose: Optional[List[float]] = None
        self.pending_center_final_target: Optional[List[float]] = None
        self.pending_center_is_side: bool = False

        # Workspace limit state
        self.ws_active: bool = False
        self.ws_idx: Optional[int] = None
        self.ws_is_side: bool = False

        self.ws_z_ref: float = 0.0
        self.ws_z_min: float = 0.0
        self.ws_z_max: float = 0.0

        self.ws_r_ref: float = 0.0
        self.ws_r_min: float = 0.0
        self.ws_r_max: float = 0.0

        self.ws_theta_ref: float = 0.0
        self.ws_theta_min: float = 0.0
        self.ws_theta_max: float = 0.0

        # MAP Positon
        self.topic_tcp_z = str(self.get_parameter("topic_tcp_z").value)

        # ---------------- ros io ----------------
        self.js_sub = self.create_subscription(JointState, self.topic_joint_states, self._on_joint_states, 50)
        self.hl_sub = self.create_subscription(String, self.topic_hl, self._on_hl_cmd, 10)
        self.cancel_sub = self.create_subscription(Bool, self.topic_cancel, self._on_cancel, 10)
        self.movel_stop_sub = self.create_subscription(
            Bool, self.topic_movel_stop, self._on_movel_stop, 10
        )

        self.traj_pub = self.create_publisher(JointTrajectory, self.topic_traj_out, 10)
        self.tcp_z_pub = self.create_publisher(Float64, self.topic_tcp_z, 10)
        self.pos_random_pub = self.create_publisher(String,self.topic_pos_random_view,10)

        # TF + MoveIt services
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cart_srv = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.lift_cart_srv = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.plan_srv = self.create_client(GetMotionPlan, "/plan_kinematic_path")

        # MAIN SCREEN
        self.get_logger().info("\033[92m[✓] ur5_executor_node started\033[0m")
        # READY summary 
        self._ready_timer = self.create_timer(0.5, self._startup_ready_summary)
        

    def _banner_lines(self, title: str, lines: List[str]) -> List[str]:
        sep = "──────────────────────────────────────────────────────────────"
        out: List[str] = []
        out.append(sep)
        out.append(f"   {title}")
        out.extend(lines)
        out.append(sep)
        return out

    def _print_banner(self, title: str, lines: List[str], also_dbg: bool = False):
        for s in self._banner_lines(title, lines):  
            self.get_logger().info(s)
            if also_dbg:
                self._dbg(s)

    # ---------------- READY summary (main screen) ----------------
    def _startup_ready_summary(self):
        Green = "\033[92m"
        Reset = "\033[0m"
        ok_cart = self.cart_srv.wait_for_service(timeout_sec=0.01)
        ok_plan = self.plan_srv.wait_for_service(timeout_sec=0.01)

        banner = f"""
    ───────────────────────────────────────────────────────────────
        UR5 Executor Node — Operational
        Motion State    : READY
        Debug Mode      : {'ENABLED' if self.debug else 'DISABLED'}
        Debug Channel   : {self.dbg_topic if self.dbg_enable else '(disabled)'}

        Subscribed Topics:
            • {self.topic_joint_states}
            • {self.topic_hl}
            • {self.topic_cancel}

        Published Topics:
            • {self.topic_traj_out}

        Services Used:
            • /compute_cartesian_path  : {'OK' if ok_cart else 'NO'}
            • /plan_kinematic_path     : {'OK' if ok_plan else 'NO'}

        Supported Commands:
            - STOP / UNLOCK
            - HOME / BACK
            - POS 1-5 (Top / Side / Random)
            - TOP_VIEWn / SIDE_VIEWn
            - JOG (left/right/forward/back/up/down)
            - MOVEL (dx dy dz)
            - ROTATE / ROTATE_W3
            - PICK / PLACE
    ───────────────────────────────────────────────────────────────
    """.strip("\n")

        print(Green + banner + Reset, flush=True)

        try:
            self._ready_timer.cancel()
        except Exception:
            pass

    # ---------------- debug helper ----------------
    def _dbg(self, s: str):
        if not self.debug:
            return
        if not self.dbg_enable:
            return
        if self.dbg_to_topic:
            try:
                msg = String()
                msg.data = str(s)
                self.dbg_pub.publish(msg)
            except Exception:
                pass
        if self.dbg_also_console:
            self.get_logger().info(s)

    # ---------------- Speed  ----------------
    def _get_speed_scale(self) -> float:
        mode = (self.speed_mode or "normal").lower()
        if mode == "slow":
            return self.speed_scale_slow
        if mode == "fast":
            return self.speed_scale_fast
        return self.speed_scale_normal

    # ---------------- Joint limit helpers ----------------
    def _is_within_joint_limits(self, joints: List[float]) -> Tuple[bool, str]:
        # Always read from the parameter server (supports runtime ros2 param set)
        jl_min = list(self.get_parameter("joint_limits_min").value)
        jl_max = list(self.get_parameter("joint_limits_max").value)
        margin = float(self.get_parameter("joint_limit_margin").value)

        if len(joints) != len(jl_min) or len(joints) != len(jl_max):
            return False, (
                f"len(joints)={len(joints)} "
                f"!= len(joint_limits_min)={len(jl_min)} "
                f"or len(joint_limits_max)={len(jl_max)}"
            )

        for i, q in enumerate(joints):
            qmin = float(jl_min[i])
            qmax = float(jl_max[i])
            qmin_eff = qmin + margin
            qmax_eff = qmax - margin

            if q < qmin_eff or q > qmax_eff:
                reason = (
                    f"joint[{i}]={q:.3f} rad not in "
                    f"[{qmin_eff:.3f}, {qmax_eff:.3f}] "
                    f"(raw=[{qmin:.3f},{qmax:.3f}], margin={margin:.3f})"
                )
                return False, reason
        return True, "OK"
    
    def _jog_lr_clamp_dxdy(self, dx_req: float, dy_req: float) -> Tuple[float, float, str]:
        if not self.ws_active:
            return dx_req, dy_req, "WS not active"
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.ee_link, Time())
        except Exception as e:
            return dx_req, dy_req, f"TF error: {e}"

        x_cur = float(tf.transform.translation.x)
        y_cur = float(tf.transform.translation.y)
        x_raw = x_cur + float(dx_req)
        y_raw = y_cur + float(dy_req)
        x_clamped = _clamp(x_raw, self.ws_x_min, self.ws_x_max)
        y_clamped = _clamp(y_raw, self.ws_y_min, self.ws_y_max)
        dx_eff = x_clamped - x_cur
        dy_eff = y_clamped - y_cur

        return dx_eff, dy_eff, (
            f"x {x_cur:.3f}->{x_raw:.3f}->{x_clamped:.3f} "
            f"(min={self.ws_x_min:.3f},max={self.ws_x_max:.3f}), "
            f"y {y_cur:.3f}->{y_raw:.3f}->{y_clamped:.3f} "
            f"(min={self.ws_y_min:.3f},max={self.ws_y_max:.3f})"
        )


    def _reset_motion_state(self, clear_ws: bool = True):
        # Basic motion flags
        self.movel_busy = False
        self.lift_busy = False

        # NEW: clear MoveL execution window
        self.movel_executing = False
        if self.movel_exec_timer is not None:
            try:
                self.movel_exec_timer.cancel()
            except Exception:
                pass
            self.movel_exec_timer = None

        # pending center transit
        self.pending_center_idx = None
        self.pending_center_pose = None
        self.pending_center_final_target = None
        self.pending_center_is_side = False

        # pending pos-down
        self.pending_pos_down_idx = None
        self.pending_pos_down_target = None
        self.pending_pos_down_dz = 0.0
        self.pending_pos_down_is_side = False

        # workspace limit
        if clear_ws:
            self._clear_workspace_limits()
            
    def _handle_joint_limit_violation(self, tag: str, reason: str):
        RED = "\033[91m"
        Reset = "\033[0m"
        self.get_logger().error(
            f"{RED}🛑 JOINT LIMIT VIOLATION ({tag}): {reason} -> SOFT-STOP + LOCK{Reset}"
        )

        # Clear all motion state + workspace and lock motion
        self.locked = True
        self._reset_motion_state(clear_ws=True)

        if self.current_joints is not None:
            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            pt = JointTrajectoryPoint()
            pt.positions = list(self.current_joints)
            pt.time_from_start.sec = 0
            pt.time_from_start.nanosec = int(0.2 * 1e9)  # 0.2s soft-stop
            traj.points.append(pt)
            self.traj_pub.publish(traj)
            self.get_logger().warn(
                f"{RED}🛑 JOINT LIMIT: soft-stop trajectory to current_joints{Reset}"
            )
        else:
            self.get_logger().warn(
                f"{RED}🛑 JOINT LIMIT: current_joints not available -> lock only{Reset}"
            )

    # ---------------- Workspace helper ----------------
    def _clear_workspace_limits(self):
        Blue = "\033[94m"
        Reset = "\033[0m"
        if self.ws_active:
            self._dbg(
                f"{Blue}WS: clear workspace limits (idx={self.ws_idx}, side={self.ws_is_side}){Reset}"
            )

        self.ws_active = False
        self.ws_idx = None
        self.ws_is_side = False
        self.ws_z_ref = 0.0
        self.ws_z_min = 0.0
        self.ws_z_max = 0.0
        self.ws_r_ref = 0.0
        self.ws_r_min = 0.0
        self.ws_r_max = 0.0
        self.ws_theta_ref = 0.0
        self.ws_theta_min = 0.0
        self.ws_theta_max = 0.0
        self.ws_y_ref = 0.0
        self.ws_y_min = 0.0
        self.ws_y_max = 0.0
        self.ws_x_ref = 0.0
        self.ws_x_min = 0.0
        self.ws_x_max = 0.0

    def _set_workspace_limits_from_tf(self, idx: int, is_side: bool):
        Yellow = "\033[93m"
        Green = "\033[92m"
        Reset = "\033[0m"
        if not self.ws_enable:
            self._dbg("\033[94mWS: ws_enable=False -> skip workspace limit setup\033[0m")
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                Time(),
            )
        except Exception as e:
            self.get_logger().warn(
                f"{Yellow}WS: cannot get TF {self.base_frame}->{self.ee_link}{Reset} "
                f"{Yellow}while setting workspace (idx={idx}, side={is_side}): {e}{Reset}"
            )
            return

        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        z = float(tf.transform.translation.z)
        r = math.hypot(x, y)

        # Limit view (top / side) + POS index
        if is_side:
            z_down_lim = self.ws_side_z_down_map.get(idx, self.ws_z_down_limit)
            z_up_lim = self.ws_side_z_up_map.get(idx, self.ws_z_up_limit)
            r_back_lim = self.ws_side_r_back_map.get(idx, self.ws_r_back_limit)
            r_fwd_lim = self.ws_side_r_fwd_map.get(idx, self.ws_r_forward_limit)
            lr_left_lim = self.ws_side_lr_left_map.get(idx, self.ws_lr_left_limit)
            lr_right_lim = self.ws_side_lr_right_map.get(idx, self.ws_lr_right_limit)
        else:
            z_down_lim = self.ws_top_z_down_map.get(idx, self.ws_z_down_limit)
            z_up_lim = self.ws_top_z_up_map.get(idx, self.ws_z_up_limit)
            r_back_lim = self.ws_top_r_back_map.get(idx, self.ws_r_back_limit)
            r_fwd_lim = self.ws_top_r_fwd_map.get(idx, self.ws_r_forward_limit)
            lr_left_lim = self.ws_top_lr_left_map.get(idx, self.ws_lr_left_limit)
            lr_right_lim = self.ws_top_lr_right_map.get(idx, self.ws_lr_right_limit)

        # Z-LIMIT
        z_ref = z
        z_min = z_ref - float(z_down_lim)
        z_max = z_ref + float(z_up_lim)

        # R-LIMIT
        r_ref = r
        r_min = max(0.0, r_ref - float(r_back_lim))
        r_max = max(r_min, r_ref + float(r_fwd_lim))

        # Y-LIMIT (axis-aligned left/right)
        # left  = +Y, right = -Y in base_frame
        y_ref = y
        y_min = y_ref - float(lr_right_lim)  # right side
        y_max = y_ref + float(lr_left_lim)   # left side

        # X-LIMIT (axis-aligned front/back)
        x_ref = x
        x_min = x_ref - float(r_back_lim)     
        x_max = x_ref + float(r_fwd_lim)      

        # T-LIMIT (keep for logs / old behavior)
        theta_ref = math.atan2(y, x)
        if r_ref < 1e-4:
            theta_min = -math.pi
            theta_max = +math.pi
            self.get_logger().warn(
                f"{Yellow}WS: r_ref too small ({r_ref:.6f}) -> disable strict T-LIMIT{Reset} "
                f"{Yellow}(idx={idx}, side={is_side}){Reset}"
            )
        else:
            left_lim = float(lr_left_lim)
            right_lim = float(lr_right_lim)

            dtheta_left = left_lim / r_ref
            dtheta_right = right_lim / r_ref

            theta_min = theta_ref - dtheta_right
            theta_max = theta_ref + dtheta_left

        # -------------------- Commit workspace state --------------------
        self.ws_active = True
        self.ws_idx = idx
        self.ws_is_side = is_side
        self.ws_z_ref = z_ref
        self.ws_z_min = z_min
        self.ws_z_max = z_max
        self.ws_r_ref = r_ref
        self.ws_r_min = r_min
        self.ws_r_max = r_max
        self.ws_theta_ref = theta_ref
        self.ws_theta_min = theta_min
        self.ws_theta_max = theta_max
        self.ws_y_ref = y_ref
        self.ws_y_min = y_min
        self.ws_y_max = y_max
        self.ws_x_ref = x_ref
        self.ws_x_min = x_min
        self.ws_x_max = x_max

        # -------------------- Debug prints --------------------
        self._dbg(
            f"{Green}Z-LIMIT: set for idx={idx} side={is_side} "
            f"z_ref={z_ref:.3f} z_min={z_min:.3f} z_max={z_max:.3f} "
            f"(down_lim={z_down_lim:.3f}, up_lim={z_up_lim:.3f}){Reset}"
        )
        self._dbg(
            f"{Green}R-LIMIT: set for idx={idx} side={is_side} "
            f"r_ref={r_ref:.3f} r_min={r_min:.3f} r_max={r_max:.3f} "
            f"(back_lim={r_back_lim:.3f}, fwd_lim={r_fwd_lim:.3f}){Reset}"
        )
        self._dbg(
            f"{Green}Y-LIMIT: set for idx={idx} side={is_side} "
            f"y_ref={y_ref:.3f} y_min={y_min:.3f} y_max={y_max:.3f} "
            f"(left_lim={lr_left_lim:.3f}m, right_lim={lr_right_lim:.3f}m){Reset}"
        )

        self._dbg(
            f"{Green}X-LIMIT: set for idx={idx} side={is_side} "
            f"x_ref={x_ref:.3f} x_min={x_min:.3f} x_max={x_max:.3f}{Reset}"
        )

        self._dbg(
            (
                Green
                + "T-LIMIT: set for idx={} side={} theta_ref={:.1f}° "
                "theta_min={:.1f}° theta_max={:.1f}° (left_lim={:.3f}m, right_lim={:.3f}m)"
                + Reset
            ).format(
                idx,
                is_side,
                math.degrees(theta_ref),
                math.degrees(theta_min),
                math.degrees(theta_max),
                lr_left_lim,
                lr_right_lim,
            )
        )

    # ---------------- JointStates ----------------
    def _on_joint_states(self, msg: JointState):
        if not msg.name or not msg.position:
            return
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.current_joints = [float(name_to_pos[jn]) for jn in self.joint_names]
        except KeyError:
            return
        self.last_js = msg
        #self._publish_tcp_z()

        # If locked -> skip all center/pos-down checks
        if self.locked:
            return

        # Check center first (when current is close to center -> send MoveJ to the final target)
        self._check_center_reached()
        # Then check whether final POS/side is reached -> MoveL down Z + set workspace limits
        self._check_pos_reached_for_down()

    def _check_center_reached(self):
        Green = "\033[92m"
        Reset = "\033[0m"
        if self.pending_center_idx is None:
            return
        if self.pending_center_pose is None:
            return
        if self.pending_center_final_target is None:
            return
        if self.current_joints is None:
            return

        diffs = [abs(a - b) for a, b in zip(self.current_joints, self.pending_center_pose)]
        max_diff = max(diffs) if diffs else 0.0

        if max_diff > self.pos_reached_tol:
            return

        idx = self.pending_center_idx
        kind = "SIDE_VIEW" if self.pending_center_is_side else "POS"
        self._dbg(
            f"{Green}Center pose for {kind}:{idx} reached (max joint diff={max_diff:.3f} rad) "
            f"-> MoveJ to final target (braking){Reset}"
        )

        target = list(self.pending_center_final_target)

        # Clear center state
        self.pending_center_idx = None
        self.pending_center_pose = None
        self.pending_center_final_target = None
        self.pending_center_is_side = False

        # MoveJ to final target (with braking + traj_pos_scale)
        self._publish_traj_1pt(target, time_scale=self.traj_pos_scale)

    def _check_pos_reached_for_down(self):
        Magenta = "\033[95m"
        Reset = "\033[0m"
        if self.pending_pos_down_idx is None:
            return
        if self.pending_pos_down_target is None:
            return
        if self.current_joints is None:
            return

        diffs = [abs(a - b) for a, b in zip(self.current_joints, self.pending_pos_down_target)]
        max_diff = max(diffs) if diffs else 0.0

        if max_diff > self.pos_reached_tol:
            return

        idx = self.pending_pos_down_idx
        dz = self.pending_pos_down_dz
        is_side = self.pending_pos_down_is_side

        self._dbg(
            f"{Magenta}POS/VIEW:{idx} reached (max joint diff={max_diff:.3f} rad) "
            f"-> MoveL down dz={dz:.3f} m{Reset}"
        )

        # Clear pending state first
        self.pending_pos_down_idx = None
        self.pending_pos_down_target = None
        self.pending_pos_down_dz = 0.0
        self.pending_pos_down_is_side = False

        # Set workspace limits at this point
        if self.ws_enable:
            self._set_workspace_limits_from_tf(idx, is_side)

        if dz > 0.0:
            rise_dz = self._get_lift_height(idx, is_side)
            allow_rise = self._should_auto_rise_after_drop()

            self._dbg(
                f"\033[96mPOST-DROP condition for idx={idx} side={is_side}: "
                f"last_cmd={self.last_hl_cmd_name} -> auto_rise={'YES' if allow_rise else 'NO'}\033[0m"
            )

            self._do_movel_smooth_drop(dz, rise_after=allow_rise, rise_dz=rise_dz)

    # ---------------- Cancel / lock ----------------
    def _on_cancel(self, msg: Bool):
        Red = "\033[91m"
        Reset = "\033[0m"
        if msg.data:
            self.locked = True
            self._reset_motion_state(clear_ws=True)
            self.get_logger().warn(
                Red + "🛑 /ur5/cmd_cancel -> LOCK motion + reset motion state" + Reset
            )

    # ---------------- High-level cmd parsing ----------------
    def _parse_hl(self, s: str) -> Tuple[str, List[str]]:
        t = (s or "").strip()
        if not t:
            return ("", [])
        parts = t.split(":")
        return (parts[0].upper(), parts[1:])

    # ---------------- Basic joint traj helper ----------------
    def _compute_duration_from(self, cur: List[float], target: List[float]) -> float:
        duration = max(self.min_time, self.base_time)
        deltas = [abs(t - c) for t, c in zip(target, cur)]
        max_delta = max(deltas) if deltas else 0.0
        if self.max_joint_vel > 0.0 and max_delta > 0.0:
            duration = max(duration, max_delta / self.max_joint_vel)
        return duration

    def _publish_traj_1pt(self, target: List[float], time_scale: float = 1.0) -> float:
        assert self.current_joints is not None

        ok, reason = self._is_within_joint_limits(target)
        if not ok:
            self._handle_joint_limit_violation("TRAJ_1PT", reason)
            return 0.0

        cur = self.current_joints
        duration = self._compute_duration_from(cur, target)
        speed_scale = self._get_speed_scale()
        scale = max(0.1, self.traj_global_scale * max(0.1, time_scale) * max(0.1, speed_scale))
        duration *= scale

        # SMOOTH SETTINGS 
        n_points = 25   # 20–30 UR5e
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        for i in range(1, n_points + 1):
            s = i / float(n_points)

            # 5th order minimum-jerk polynomial
            smooth = 6*s**5 - 15*s**4 + 10*s**3

            pos = [
                float(c + smooth * (t - c))
                for c, t in zip(cur, target)
            ]

            pt = JointTrajectoryPoint()
            pt.positions = pos
            t_scaled = duration * s
            pt.time_from_start.sec = int(t_scaled)
            pt.time_from_start.nanosec = int((t_scaled - int(t_scaled)) * 1e9)
            traj.points.append(pt)

        self.last_commanded_joints = list(target)
        self.last_command_was_movel = False
        self.traj_pub.publish(traj)
        self._dbg(
            f"\033[92mTRAJ(SMOOTH-SCURVE) duration={duration:.2f}s "
            f"points={n_points} speed_mode={self.speed_mode}\033[0m"
        )
        return duration
    
    def _publish_traj_2pt(self, mid: List[float], target: List[float], time_scale: float = 1.0, ) -> float:
        assert self.current_joints is not None
        ok_mid, reason_mid = self._is_within_joint_limits(mid)
        if not ok_mid:
            self._handle_joint_limit_violation("TRAJ_2PT_MID", reason_mid)
            return 0.0

        ok_tg, reason_tg = self._is_within_joint_limits(target)
        if not ok_tg:
            self._handle_joint_limit_violation("TRAJ_2PT_TARGET", reason_tg)
            return 0.0

        cur = self.current_joints

        # ----- duration segment 1 (cur -> mid) -----
        t1 = max(self.min_time, float(self.transit_time))
        t1 = max(t1, self._compute_duration_from(cur, mid))

        # ----- duration segment 2 (mid -> target) -----
        t2_step = max(self.min_time, float(self.target_extra_time))
        t2_step = max(t2_step, self._compute_duration_from(mid, target))
        speed_scale = self._get_speed_scale()
        scale = max(0.1, self.traj_global_scale * max(0.1, time_scale) * max(0.1, speed_scale))
        t1 *= scale
        t2_step *= scale
        total_time = t1 + t2_step

        # Smooth resolution
        n1 = 15   # points for first segment
        n2 = 15   # points for second segment
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        # Segment 1 (cur -> mid)
        for i in range(1, n1 + 1):
            s = i / float(n1)
            smooth = 6*s**5 - 15*s**4 + 10*s**3
            pos = [float(c + smooth * (m - c)) for c, m in zip(cur, mid)]
            pt = JointTrajectoryPoint()
            pt.positions = pos
            t_scaled = t1 * s
            pt.time_from_start.sec = int(t_scaled)
            pt.time_from_start.nanosec = int((t_scaled - int(t_scaled)) * 1e9)
            traj.points.append(pt)

        # ---------- Segment 2 (mid -> target) ----------
        for i in range(1, n2 + 1):
            s = i / float(n2)
            smooth = 6*s**5 - 15*s**4 + 10*s**3
            pos = [float(m + smooth * (tg - m)) for m, tg in zip(mid, target)]
            pt = JointTrajectoryPoint()
            pt.positions = pos
            t_scaled = t1 + t2_step * s
            pt.time_from_start.sec = int(t_scaled)
            pt.time_from_start.nanosec = int((t_scaled - int(t_scaled)) * 1e9)
            traj.points.append(pt)

        self.last_commanded_joints = list(target)
        self.last_command_was_movel = False
        self.traj_pub.publish(traj)
        Cyan = "\033[96m"
        Reset = "\033[0m"
        self._dbg(
            f"{Cyan}TRAJ(2pt-SMOOTH-SCURVE) total={total_time:.2f}s "
            f"points={n1+n2} scale={scale:.2f}{Reset}"
        )
        return total_time

    # ---------------- Joint mid lift helpers (fallback joint) ----------------
    def _make_lift_mid(self, target: List[float]) -> List[float]:
        assert self.current_joints is not None
        mid = list(self.current_joints)
        mid[0] = float(target[0])
        mid[1] = float(self.lift_shoulder2)
        mid[2] = float(self.lift_elbow)
        mid[3] = float(self.lift_wrist1)
        return mid

    def _go_safe_joint_mid(self, target: List[float], use_transit: bool = True, time_scale: float = 1.0,) -> float:
        if use_transit and self.use_transit:
            if self.use_dynamic_lift:
                mid = self._make_lift_mid(target)
            else:
                mid = self.transit_pose
            t = self._publish_traj_2pt(mid, target, time_scale=time_scale)
        else:
            t = self._publish_traj_1pt(target, time_scale=time_scale)
        return t

    def _remember_prev(self):
        if self.current_joints is not None:
            self.prev_joints = list(self.current_joints)

    # Helper: check whether we are already close to HOME
    def _is_at_home(self) -> bool:
        if self.current_joints is None:
            return False
        if len(self.current_joints) != len(self.home_joints):
            return False

        diffs = [abs(a - b) for a, b in zip(self.current_joints, self.home_joints)]
        max_diff = max(diffs) if diffs else 0.0
        return max_diff <= self.pos_reached_tol

    # Select center pose based on POS index (top view only)
    def _get_center_pose_for_pos(self, idx: int) -> Optional[List[float]]:
        if idx == 2 and len(self.pos_center_pose_2) == len(self.joint_names):
            return list(self.pos_center_pose_2)

        if idx in (1, 5) and len(self.pos_center_pose_15) == len(self.joint_names):
            return list(self.pos_center_pose_15)

        if idx in (3, 4) and len(self.pos_center_pose_34) == len(self.joint_names):
            return list(self.pos_center_pose_34)

        return None

    # ---------------- MoveL handler for MOVEL:dx:dy:dz ----------------
    def _do_movel(self, dx: float, dy: float, dz: float):
        Yellow = "\033[93m"
        Cyan = "\033[96m"
        Reset = "\033[0m"
        if self.locked:
            self.get_logger().warn(
                f"{Yellow}MoveL ignored because motion is LOCKED (dx={dx}, dy={dy}, dz={dz}){Reset}"
            )
            return

        if self.movel_busy:
            self.get_logger().warn(
                Yellow + "MoveL is busy, ignore new MOVEL cmd." + Reset
            )
            return

        if self.last_js is None or self.current_joints is None:
            self.get_logger().warn(
                Yellow + "MoveL: No /joint_states available (last_js is None)" + Reset
            )
            return

        if not self.cart_srv.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(
                Yellow + "MoveL: /compute_cartesian_path not available." + Reset
            )
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                Time()
            )
        except Exception as e:
            self.get_logger().warn(
                f"{Yellow}MoveL: Waiting TF {self.base_frame}->{self.ee_link} ... ({e}){Reset}"
            )
            return

        n = max(2, int(self.movel_num_waypoints))
        cur = tf.transform.translation
        rot = tf.transform.rotation
        waypoints: List[Pose] = []
        for i in range(1, n + 1):
            s = i / float(n)
            wp = Pose()
            wp.position.x = cur.x + dx * s
            wp.position.y = cur.y + dy * s
            wp.position.z = cur.z + dz * s
            wp.orientation = rot
            waypoints.append(wp)
        goal_pose = Pose()
        goal_pose.position.x = cur.x + dx
        goal_pose.position.y = cur.y + dy
        goal_pose.position.z = cur.z + dz
        goal_pose.orientation = rot
        self.movel_goal_pose = goal_pose
        rs = RobotState()
        rs.joint_state.name = list(self.last_js.name)
        rs.joint_state.position = list(self.last_js.position)
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.start_state = rs
        req.group_name = self.group_name
        req.link_name = self.ee_link
        req.waypoints = waypoints
        req.max_step = self.movel_max_step
        req.jump_threshold = self.movel_jump_threshold
        req.avoid_collisions = self.movel_avoid_collisions
        self.get_logger().info(
            f"{Cyan}MoveL: ComputeCartesianPath frame={self.base_frame} ee={self.ee_link} "
            f"d=({dx:.3f},{dy:.3f},{dz:.3f}) group={self.group_name} "
            f"waypoints={len(waypoints)}{Reset}"
        )
        self.movel_busy = True
        fut = self.cart_srv.call_async(req)
        fut.add_done_callback(self._on_movel_cart_done)

    def _on_movel_cart_done(self, fut):
        Red = "\033[91m"
        Yellow = "\033[93m"
        Blue = "\033[94m"
        Reset = "\033[0m"
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f"{Red}MoveL: GetCartesianPath failed: {e}{Reset}")
            self.movel_busy = False
            return

        frac = float(res.fraction)
        self.get_logger().info(f"{Blue}MoveL: Cartesian fraction = {frac:.3f}{Reset}")

        if frac >= 0.95:
            jt = res.solution.joint_trajectory

            # Check joint limits for every point before sending
            for p in jt.points:
                ok, reason = self._is_within_joint_limits(list(p.positions))
                if not ok:
                    self._handle_joint_limit_violation("MOVEL_CART", reason)
                    self.movel_busy = False
                    return
            self._send_joint_trajectory(jt, tag="movel_cartesian")
            self.movel_busy = False
            return

        if not self.fallback_enable:
            self.get_logger().error(
                Red + "MoveL: Cartesian failed and fallback disabled." + Reset
            )
            self.movel_busy = False
            return

        if self.movel_goal_pose is None:
            self.get_logger().error(
                Red + "MoveL: Fallback requested but goal pose is missing." + Reset
            )
            self.movel_busy = False
            return

        if not self.plan_srv.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(
                Yellow + "MoveL: /plan_kinematic_path not available." + Reset
            )
            self.movel_busy = False
            return

        self.get_logger().warn(
            Yellow + "MoveL: Cartesian failed -> fallback to /plan_kinematic_path ..." + Reset
        )
        plan_req = GetMotionPlan.Request()
        plan_req.motion_plan_request = self._build_motion_plan_request(self.movel_goal_pose)
        fut2 = self.plan_srv.call_async(plan_req)
        fut2.add_done_callback(self._on_movel_plan_done)

    def _build_motion_plan_request(self, goal_pose: Pose) -> MotionPlanRequest:
        rs = RobotState()
        rs.joint_state.name = list(self.last_js.name)
        rs.joint_state.position = list(self.last_js.position)
        pos_tol = self.fallback_pos_tol
        ori_tol = self.fallback_ori_tol
        pc = PositionConstraint()
        pc.link_name = self.ee_link
        pc.header.frame_id = self.base_frame
        pc.weight = 1.0
        bv = BoundingVolume()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [pos_tol * 2.0, pos_tol * 2.0, pos_tol * 2.0]
        bv.primitives.append(prim)
        bv.primitive_poses.append(goal_pose)
        pc.constraint_region = bv
        oc = OrientationConstraint()
        oc.link_name = self.ee_link
        oc.header.frame_id = self.base_frame
        oc.orientation = goal_pose.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0
        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        mpr = MotionPlanRequest()
        mpr.group_name = self.group_name
        mpr.start_state = rs
        mpr.goal_constraints.append(c)
        mpr.allowed_planning_time = self.fallback_planning_time
        mpr.num_planning_attempts = self.fallback_num_attempts
        return mpr

    def _on_movel_plan_done(self, fut):
        Red = "\033[91m"
        Reset = "\033[0m"
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f"{Red}MoveL: Fallback GetMotionPlan failed: {e}{Reset}")
            self.movel_busy = False
            return

        err = int(res.motion_plan_response.error_code.val)
        if err != 1:
            self.get_logger().error(f"{Red}MoveL: fallback planning failed, error_code={err}{Reset}")
            self.movel_busy = False
            return
        jt = res.motion_plan_response.trajectory.joint_trajectory

        # Check joint limits for every point before sending
        for p in jt.points:
            ok, reason = self._is_within_joint_limits(list(p.positions))
            if not ok:
                self._handle_joint_limit_violation("MOVEL_FALLBACK", reason)
                self.movel_busy = False
                return
        self._send_joint_trajectory(jt, tag="movel_fallback")
        self.movel_busy = False

    # ---------------- Send joint trajectory (with MoveL time scaling) ----------------
    def _send_joint_trajectory(self, jt: JointTrajectory, tag: str = "traj"):
        Red = "\033[91m"
        Green = "\033[92m"
        Yellow = "\033[93m"
        Reset = "\033[0m"
        if self.locked:
            self.get_logger().warn(
                f"{Yellow}{tag}: motion is LOCKED -> ignore trajectory publish{Reset}"
            )
            return

        if not jt.joint_names or not jt.points:
            self.get_logger().error(f"{Red}{tag}: empty trajectory, ignore.{Reset}")
            return

        out = JointTrajectory()
        out.joint_names = list(jt.joint_names)

        # --- HEAVY BRAKE for MoveL ---
        if tag in ("movel_cartesian", "movel_fallback", "safe_movel_lift", "safe_movel_drop", "safe_movel_rise"):
            pts = list(jt.points)
            total = len(pts)

            if total <= 1:
                self.get_logger().warn(
                    f"{tag}: trajectory has only {total} point(s), skip S-curve scaling"
                )
                out.points = pts
                self.traj_pub.publish(out)

                if out.points:
                    self.last_commanded_joints = list(out.points[-1].positions)
                    self.last_command_was_movel = tag in (
                        "movel_cartesian",
                        "movel_fallback",
                        "safe_movel_lift",
                        "safe_movel_drop",
                        "safe_movel_rise",
                    )

                if tag in ("movel_cartesian", "movel_fallback", "safe_movel_lift", "safe_movel_drop", "safe_movel_rise"):
                    self._mark_movel_executing(out)

                self.get_logger().info(
                    f"{Green}{tag}: publish joint trajectory (single-point passthrough), "
                    f"points={len(out.points)}{Reset}"
                )
                return

            new_pts = []
            for i, p in enumerate(pts):
                t = float(p.time_from_start.sec) + \
                    float(p.time_from_start.nanosec) * 1e-9
                s = i / float(total - 1)  # 0 → 1
                smooth = 6*s**5 - 15*s**4 + 10*s**3
                t_scaled = t * (1.0 + 0.15*smooth)
                q = JointTrajectoryPoint()
                q.positions = list(p.positions)
                q.time_from_start.sec = int(t_scaled)
                q.time_from_start.nanosec = int((t_scaled - int(t_scaled)) * 1e9)
                new_pts.append(q)
            jt.points = new_pts

        # Use a different time scale for MoveL / safe_movel_lift
        scale = 1.0
        if tag == "safe_movel_lift":
            scale = max(0.1, float(self.movel_traj_global_scale) * 2.0)

        elif tag == "safe_movel_drop":
            scale = max(0.1, float(self.movel_traj_global_scale) * 2.0)

        elif tag == "safe_movel_rise":
            scale = max(0.1, float(self.movel_traj_global_scale) * 2.0)

        elif tag in ("movel_cartesian", "movel_fallback"):
            scale = max(0.1, float(self.movel_traj_global_scale))
            
        # Apply speed mode scaling as well
        speed_scale = self._get_speed_scale()
        scale = max(0.1, scale * max(0.1, speed_scale))
        for p in jt.points:
            ok, reason = self._is_within_joint_limits(list(p.positions))
            if not ok:
                self._handle_joint_limit_violation(tag, reason)
                return
            q = JointTrajectoryPoint()
            q.positions = list(p.positions)
            q.velocities = list(p.velocities)
            q.accelerations = list(p.accelerations)
            q.effort = list(p.effort)
            t = float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9
            t *= scale
            q.time_from_start.sec = int(t)
            q.time_from_start.nanosec = int((t - int(t)) * 1e9)
            out.points.append(q)
        self.traj_pub.publish(out)
        if out.points:
            self.last_commanded_joints = list(out.points[-1].positions)
            self.last_command_was_movel = tag in (
                "movel_cartesian",
                "movel_fallback",
                "safe_movel_lift",
                "safe_movel_drop",
                "safe_movel_rise",
            )
        if tag in ("movel_cartesian", "movel_fallback", "safe_movel_lift", "safe_movel_drop", "safe_movel_rise"):
            self._mark_movel_executing(out)
        self.get_logger().info(
            f"{Green}{tag}: publish joint trajectory (time_scale={scale:.2f}), "
            f"points={len(out.points)}{Reset}"
        )

    # ---------------- SAFE MoveL-lift in Z before MoveJ -> target ----------------
    def _safe_lift_then_movej(
        self,
        target: List[float],
        use_lift: bool = True,
        lift_dz: Optional[float] = None,
        extra_time_scale: float = 1.0,
    ):
        # If already locked, do not start any new motion
        if self.locked:
            self.get_logger().warn(
                "\033[93mSAFE: motion is LOCKED -> ignore safe_lift_then_movej\033[0m"
            )
            return

        # For large MoveJ motions (HOME, BACK, change POS) clear old workspace first
        self._clear_workspace_limits()

        # If the target violates joint limits -> do not execute any MoveL/MoveJ, stop first
        ok, reason = self._is_within_joint_limits(target)
        if not ok:
            self._handle_joint_limit_violation("SAFE_LIFT_TARGET", reason)
            return

        if not use_lift:
            self._dbg("SAFE: use_lift=False -> joint mid transit")
            self._go_safe_joint_mid(target, use_transit=True, time_scale=extra_time_scale)
            return

        if self.lift_busy:
            self.get_logger().warn(
                "\033[93mSAFE: lift is busy -> fallback joint mid transit\033[0m"
            )
            self._go_safe_joint_mid(target, use_transit=True, time_scale=extra_time_scale)
            return

        if self.last_js is None or self.current_joints is None:
            self.get_logger().warn(
                "\033[93mSAFE: no joint_states / last_js -> use joint mid transit\033[0m"
            )
            self._go_safe_joint_mid(target, use_transit=True, time_scale=extra_time_scale)
            return

        if not self.lift_cart_srv.wait_for_service(timeout_sec=0.3):
            self.get_logger().warn(
                "\033[93mSAFE: /compute_cartesian_path not ready -> joint mid transit\033[0m"
            )
            self._go_safe_joint_mid(target, use_transit=True, time_scale=extra_time_scale)
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                Time()
            )
        except Exception as e:
            self.get_logger().warn(
                f"\033[93mSAFE: TF {self.base_frame}->{self.ee_link} missing ({e}) "
                "-> joint mid transit\033[0m"
            )
            self._go_safe_joint_mid(target, use_transit=True, time_scale=extra_time_scale)
            return

        dz = float(lift_dz) if lift_dz is not None else float(self.movel_lift_dz)
        dz = _clamp(dz, 0.0, 0.50) # 0.25 side move up
        n = max(2, int(self.movel_num_waypoints))
        cur = tf.transform.translation
        rot = tf.transform.rotation
        waypoints: List[Pose] = []
        for i in range(1, n + 1):
            s = i / float(n)
            wp = Pose()
            wp.position.x = cur.x
            wp.position.y = cur.y
            wp.position.z = cur.z + dz * s
            wp.orientation = rot
            waypoints.append(wp)
        rs = RobotState()
        rs.joint_state.name = list(self.last_js.name)
        rs.joint_state.position = list(self.last_js.position)
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.start_state = rs
        req.group_name = self.group_name
        req.link_name = self.ee_link
        req.waypoints = waypoints
        req.max_step = self.movel_max_step
        req.jump_threshold = self.movel_jump_threshold
        req.avoid_collisions = self.movel_avoid_collisions
        Blue = "\033[94m"
        Reset = "\033[0m"
        self._dbg(
            f"{Blue}SAFE via MoveL-lift: dz={dz:.3f} "
            f"frame={self.base_frame} ee={self.ee_link} waypoints={len(waypoints)}{Reset}"
        )

        self.lift_busy = True
        self.lift_target = list(target)
        self._lift_extra_time_scale = extra_time_scale
        fut = self.lift_cart_srv.call_async(req)
        fut.add_done_callback(self._on_lift_cart_done)

    def _on_lift_cart_done(self, fut):
        Red = "\033[91m"
        Yellow = "\033[93m"
        Blue = "\033[94m"
        Green = "\033[92m"
        Reset = "\033[0m"
        target = self.lift_target
        extra_time_scale = getattr(self, "_lift_extra_time_scale", 1.0)
        self.lift_busy = False

        if target is None:
            self.get_logger().error("SAFE: lift_target is None in callback")
            return

        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f"{Red}SAFE: MoveL-lift GetCartesianPath failed: {e}{Reset}")
            self._dbg("\033[93mSAFE: MoveL-lift error -> use joint mid transit\033[0m")
            self._go_safe_joint_mid(target, use_transit=True, time_scale=extra_time_scale)
            return

        frac = float(res.fraction)
        self._dbg(f"{Blue}SAFE MoveL-lift fraction = {frac:.3f}{Reset}")

        if frac < self.movel_lift_min_fraction or not res.solution.joint_trajectory.points:
            self.get_logger().warn(
                f"{Yellow}MoveL-lift fraction too low ({frac:.3f} < {self.movel_lift_min_fraction:.3f}) "
                f"-> fallback joint mid transit{Reset}"
            )
            self._dbg("\033[93mSAFE: MoveL-lift failed -> use joint mid transit instead\033[0m")
            self._go_safe_joint_mid(target, use_transit=True, time_scale=extra_time_scale)
            return

        jt = res.solution.joint_trajectory

        # Check joint limits for every lift point first
        for p in jt.points:
            ok, reason = self._is_within_joint_limits(list(p.positions))
            if not ok:
                self._handle_joint_limit_violation("SAFE_LIFT_TRAJ", reason)
                return

        self._send_joint_trajectory(jt, tag="safe_movel_lift")
        self._dbg(
            f"{Green}SAFE: MoveL-lift done, schedule MoveJ to target after "
            f"{self.movel_lift_wait_after:.2f}s{Reset}"
        )

        delay = max(self.min_time, self.movel_lift_wait_after)
        node = self

        def _cb():
            nonlocal target, timer
            node._dbg("\033[92mSAFE: MoveL-lift done -> MoveJ to target (smooth braking)\033[0m")
            node._publish_traj_1pt(target, time_scale=extra_time_scale)
            timer.cancel()
        timer = self.create_timer(delay, _cb)

    # ---------------- POS handler (top view) ----------------
    def _handle_pos_idx(self, idx: int):
        self.current_pos_id = idx
        self.active_pos_idx = idx
        self.current_view = "TOP"
        self.active_pos_is_side = False

        Yellow  = "\033[93m"
        Green   = "\033[92m"
        Magenta = "\033[95m"
        Cyan    = "\033[96m"
        Reset   = "\033[0m"
        if self.current_joints is None:
            self.get_logger().warn("\033[93mNo /joint_states yet -> waiting for robot state before POS command\033[0m")
            return

        # Clear previous motion state (workspace will be cleared below)
        self._reset_motion_state(clear_ws=False)

        # Moving to a new POS → clear old workspace first
        self._clear_workspace_limits()

        target = self.pos_map.get(idx)
        if not target or len(target) != 6:
            self.get_logger().warn(f"\033[93mPOS:{idx} preset not found\033[0m")
            return

        # If this POS target violates joint limits -> stop first
        ok, reason = self._is_within_joint_limits(target)
        if not ok:
            self._handle_joint_limit_violation(f"POS_{idx}", reason)
            return

        dz_down = _clamp(float(self.pos_down_map.get(idx, 0.0)), 0.0, 0.50) #0.25
        # lift_dz_up = _clamp(float(self.pos_lift_map.get(idx, self.movel_lift_dz)), 0.0, 0.50) #0.25
        lift_dz_up = self._get_lift_height(idx, is_side=False)
        self._remember_prev()

        if dz_down > 0.0:
            self.pending_pos_down_idx = idx
            self.pending_pos_down_target = list(target)
            self.pending_pos_down_dz = dz_down
            self.pending_pos_down_is_side = False
            self._dbg(
                f"{Magenta}POS:{idx} -> will MoveL-down dz={dz_down:.3f} m "
                f"when joint diff < {self.pos_reached_tol:.3f} rad{Reset}"
            )
        else:
            self.pending_pos_down_idx = None
            self.pending_pos_down_target = None
            self.pending_pos_down_dz = 0.0
            self.pending_pos_down_is_side = False

        at_home = self._is_at_home()
        use_center_this_cmd = (
            self.center_mode == "CENTER"
            and self.pos_use_center_transit
            and (not at_home)
        )

        if at_home:
            self._dbg(
                f"{Cyan}POS:{idx} command while at HOME -> skip CENTER pose, "
                f"use direct SAFE MoveL-lift to POS{Reset}"
            )

        if use_center_this_cmd:
            center_pose = self._get_center_pose_for_pos(idx)
            if center_pose is None:
                self.get_logger().warn(
                    f"\033[93mPOS:{idx} has no center pose -> fallback to normal SAFE MoveL-lift to target\033[0m"
                )
                self._dbg(
                    f"{Yellow}POS:{idx} -> fallback: MoveL-up dz={lift_dz_up:.3f} m + "
                    f"MoveJ(scale={self.traj_pos_scale:.2f}){Reset}"
                )
                self._safe_lift_then_movej(
                    target,
                    use_lift=True,
                    lift_dz=lift_dz_up,
                    extra_time_scale=self.traj_pos_scale,
                )
                return
            
            self._dbg(
                f"{Green}POS:{idx} -> MoveL-up dz={lift_dz_up:.3f} m + MoveJ to CENTER pose "
                f"+ then MoveJ to POS with braking, "
                f"then MoveL-down dz={dz_down:.3f} m{Reset}"
            )

            self._safe_lift_then_movej(
                center_pose,
                use_lift=True,
                lift_dz=lift_dz_up,
                extra_time_scale=self.traj_pos_scale,
            )

            self.pending_center_idx = idx
            self.pending_center_pose = list(center_pose)
            self.pending_center_final_target = list(target)
            self.pending_center_is_side = False
        else:
            self._dbg(
                f"{Green}POS:{idx} (no center transit) -> MoveL-up dz={lift_dz_up:.3f} m + "
                f"MoveJ(scale={self.traj_pos_scale:.2f}, braking) "
                f"then auto MoveL-down dz={dz_down:.3f} m{Reset}"
            )
            self._safe_lift_then_movej(
                target,
                use_lift=True,
                lift_dz=lift_dz_up,
                extra_time_scale=self.traj_pos_scale,
            )

    # ---------------- side view handler (using side_pos_* presets) ----------------
    def _handle_side_idx(self, idx: int):
        self.current_pos_id = idx
        self.active_pos_idx = idx
        self.current_view = "SIDE"
        self.active_pos_is_side = True

        Yellow  = "\033[93m"
        Green   = "\033[92m"
        Magenta = "\033[95m"
        Cyan    = "\033[96m"
        Reset   = "\033[0m"
        if self.current_joints is None:
            self.get_logger().warn(
                "\033[93mNo /joint_states yet -> waiting for robot state before SIDE_VIEW command\033[0m"
            )
            return
        
        self.get_logger().error(
            f"DEBUG LIFT MAP idx={idx} value={self.side_lift_map.get(idx)}"
        )
        
        # Clear previous motion state (workspace will be cleared below)
        self._reset_motion_state(clear_ws=False)

        # Moving to a new SIDE view → clear old workspace first
        self._clear_workspace_limits()
        target = self.side_pos_map.get(idx)
        if not target or len(target) != 6:
            self.get_logger().warn(
                f"\033[93mSIDE_VIEW:{idx} preset not found\033[0m"
            )
            return

        # If this SIDE target violates joint limits -> stop first
        ok, reason = self._is_within_joint_limits(target)
        if not ok:
            self._handle_joint_limit_violation(f"SIDE_VIEW_{idx}", reason)
            return

        dz_down = _clamp(float(self.side_down_map.get(idx, 0.0)), 0.0, 0.50) #0.25
        # lift_dz_up = _clamp(float(self.side_lift_map.get(idx, self.movel_lift_dz)), 0.0, 0.50)
        lift_dz_up = self._get_lift_height(idx, is_side=True)
        self._remember_prev()
        if dz_down > 0.0:
            self.pending_pos_down_idx = idx
            self.pending_pos_down_target = list(target)
            self.pending_pos_down_dz = dz_down
            self.pending_pos_down_is_side = True
            self._dbg(
                f"{Magenta}SIDE_VIEW:{idx} -> will MoveL-down dz={dz_down:.3f} m "
                f"when joint diff < {self.pos_reached_tol:.3f} rad{Reset}"
            )
        else:
            self.pending_pos_down_idx = None
            self.pending_pos_down_target = None
            self.pending_pos_down_dz = 0.0
            self.pending_pos_down_is_side = False

        at_home = self._is_at_home()
        use_center_this_cmd = (
            self.center_mode == "CENTER"
            and self.side_use_center_transit
            and (not at_home)
        )

        if at_home:
            self._dbg(
                f"{Cyan}SIDE_VIEW:{idx} command while at HOME -> "
                f"skip CENTER pose and use SAFE MoveL-lift directly to side_pos_{idx}{Reset}"
            )

        if use_center_this_cmd:
            center_pose = self._get_center_pose_for_pos(idx)
            if center_pose is None:
                self.get_logger().warn(
                    f"\033[93mSIDE_VIEW:{idx} has no center pose (from top view) -> "
                    f"fallback to direct SAFE MoveL-lift\033[0m"
                )
                self._dbg(
                    f"{Yellow}SIDE_VIEW:{idx} -> fallback: MoveL-up dz={lift_dz_up:.3f} m + "
                    f"MoveJ(scale={self.traj_pos_scale:.2f}){Reset}"
                )
                self._safe_lift_then_movej(
                    target,
                    use_lift=True,
                    lift_dz=lift_dz_up,
                    extra_time_scale=self.traj_pos_scale,
                )
                return

            self._dbg(
                f"{Green}SIDE_VIEW:{idx} -> MoveL-up dz={lift_dz_up:.3f} m + "
                f"MoveJ to CENTER pose (from top view) + then MoveJ to side_pos_{idx} "
                f"with braking, then MoveL-down dz={dz_down:.3f} m{Reset}"
            )

            self._safe_lift_then_movej(
                center_pose,
                use_lift=True,
                lift_dz=lift_dz_up,
                extra_time_scale=self.traj_pos_scale,
            )

            self.pending_center_idx = idx
            self.pending_center_pose = list(center_pose)
            self.pending_center_final_target = list(target)
            self.pending_center_is_side = True
        else:
            self._dbg(
                f"{Green}SIDE_VIEW:{idx} (no center transit) -> MoveL-up dz={lift_dz_up:.3f} m + "
                f"MoveJ(scale={self.traj_pos_scale:.2f}, braking) "
                f"then auto MoveL-down dz={dz_down:.3f} m{Reset}"
            )
            self._safe_lift_then_movej(
                target,
                use_lift=True,
                lift_dz=lift_dz_up,
                extra_time_scale=self.traj_pos_scale,
            )

    # ---------------- High-level command handler ----------------
    def _on_hl_cmd(self, msg: String):
        raw = (msg.data or "").strip()
        raw_nospace = raw.replace(" ", "")
        raw_lower = raw_nospace.lower()
        cmd, args = self._parse_hl(raw)
        # Remember latest command seen on /mapper/high_level_cmd
        self.last_hl_cmd_raw = raw
        self.last_hl_cmd_name = cmd
        self._update_place_rise_permission(cmd)

        #  MODE SWITCH 
        if cmd == "MODE" and len(args) >= 1:
            mode_req = (args[0] or "").upper()
            if mode_req not in ("CENTER", "DIRECT"):
                self.get_logger().warn(f"\033[93mUnknown MODE: {mode_req} (use CENTER or DIRECT)\033[0m")
                return

            self.center_mode = mode_req
            self.set_parameters([
                Parameter("center_mode", value=mode_req)
            ])

            if mode_req == "CENTER":
                self.get_logger().info("\033[92mMODE set to CENTER (use center transit)\033[0m")
            else:
                self.get_logger().info("\033[94mMODE set to DIRECT (skip center transit)\033[0m")
            return

        # ----- UNLOCK / STOP are handled first -----
        if cmd == "UNLOCK":
            self.locked = False
            self._reset_motion_state(clear_ws=True)
            self.get_logger().info("\033[92mUNLOCK -> motion enabled\033[0m")
            return

            
        if cmd == "STOP":
            # lock first to block new commands
            self.locked = True

            # clear motion state
            self._reset_motion_state(clear_ws=True)

            # try smooth stop first
            ok = self._publish_smooth_stop_traj(
                lock_after=True,
                stop_tag="STOP"
            )

            if not ok:
                if self.current_joints is not None:
                    traj = JointTrajectory()
                    traj.joint_names = self.joint_names

                    pt = JointTrajectoryPoint()
                    pt.positions = list(self.current_joints)
                    pt.time_from_start.sec = 0
                    pt.time_from_start.nanosec = int(0.2 * 1e9)

                    traj.points.append(pt)
                    self.traj_pub.publish(traj)

                    self.get_logger().warn(
                        "\033[93m🛑 STOP fallback: hold current pose (freeze robot)\033[0m"
                    )
                else:
                    self.get_logger().warn(
                        "\033[93m🛑 STOP: no joint_states available -> lock motion only\033[0m"
                    )

            return
                # CALIBRATE_SURFACE
        if cmd == "CALIBRATE_SURFACE":

            if self.current_pos_id is None:
                self.get_logger().warn("No current POS to calibrate")
                return

            z = self._get_tcp_z()
            if z is None:
                return
            
            key = (self.current_view, self.current_pos_id)
            if key in self.surface_height:
                old_val = self.surface_height[key]
                self.get_logger().info(
                    f"\033[93mUpdating surface {key}: "
                    f"{old_val:.6f} -> {z:.6f}\033[0m"
                )
            else:
                self.get_logger().info(
                    f"\033[92mNew surface {key}: "
                    f"{z:.6f}\033[0m"
                )

            self.surface_height[key] = z
            self.get_logger().info(f"CALIBRATED {key} surface_z = {z:.6f}")
            return
        
        if cmd == "SHOW_SURFACE":
            if not self.surface_height:
                self.get_logger().info("Surface map is empty")
                return

            self.get_logger().info(f"\033[96mCurrent Surface Map: {self.surface_height}\033[0m")
            return
        
        # ----- SPEED mode (SLOW / NORMAL / FAST) -----
        if cmd == "SPEED" and len(args) >= 1:
            mode = (args[0] or "").lower()
            if mode in ("slow", "normal", "fast"):
                self.speed_mode = mode
                results = self.set_parameters([Parameter("speed_mode", value=mode)])
                r = results[0]
                self.get_logger().warn(
                    f"[SPEED_SET] try='{mode}' success={r.successful} reason='{r.reason}' "
                    f"param_now='{self.get_parameter('speed_mode').value}' self.speed_mode='{self.speed_mode}'"
                )

                self.get_logger().info(
                    f"\033[94m⚙️ SPEED mode set to {mode.upper()} "
                    f"(scale slow={self.speed_scale_slow:.2f}, "
                    f"normal={self.speed_scale_normal:.2f}, "
                    f"fast={self.speed_scale_fast:.2f})\033[0m"
                )
            else:
                self.get_logger().warn(f"\033[93mUnknown SPEED mode: {mode}\033[0m")
            return

        # If still locked at this point -> block all other commands
        if self.locked:
            self.get_logger().warn(
                f"\033[93mMotion locked (STOP/JOINT_LIMIT). Ignoring cmd: {raw}\033[0m")
            return

        if self.current_joints is None:
            self.get_logger().warn(
                "\033[93mNo /joint_states yet -> waiting for robot state before executing command\033[0m")
            return

        # ----- top_viewN / side_viewN -----
        if raw_lower.startswith("top_view"):
            num_part = raw_lower[len("top_view"):]
            num_str = "".join(ch for ch in num_part if (ch.isdigit() or ch == "-"))
            if not num_str:
                self.get_logger().warn(f"\033[93mBad TOP_VIEW command: {raw}\033[0m")
                return
            try:
                idx = int(num_str)
            except Exception:
                self.get_logger().warn(f"\033[93mBad TOP_VIEW index: {raw}\033[0m")
                return
            self.get_logger().info(f"\033[94mTOP_VIEW -> POS:{idx}\033[0m")
            self._handle_pos_idx(idx)
            return

        if raw_lower.startswith("side_view"):
            num_part = raw_lower[len("side_view"):]
            num_str = "".join(ch for ch in num_part if (ch.isdigit() or ch == "-"))
            if not num_str:
                self.get_logger().warn(f"\033[93mBad SIDE_VIEW command: {raw}\033[0m")
                return
            try:
                idx = int(num_str)
            except Exception:
                self.get_logger().warn(f"\033[93mBad SIDE_VIEW index: {raw}\033[0m")
                return
            self.get_logger().info(f"\033[94mSIDE_VIEW -> side_pos_{idx}\033[0m")
            self._handle_side_idx(idx)
            return

        # HOME (Safe MoveL-lift in Z and then MoveJ to HOME – smooth + brake)
        if cmd in ("HOME", "BACK_HOME"):
            self._remember_prev()
            # Clear previous motion state (workspace will be cleared in safe_lift)
            self._reset_motion_state(clear_ws=False)
            self._safe_lift_then_movej(
                self.home_joints,
                use_lift=True,
                extra_time_scale=self.traj_pos_scale,
            )
            return

        # BACK (Return to previous pose – smooth + brake)
        if cmd in ("BACK", "RETURN"):
            if self.prev_joints is None:
                self.get_logger().warn(
                    "\033[93mNo previous pose stored (prev_joints is None) -> cannot BACK/RETURN\033[0m"
                )
                return
            # Clear previous motion state (workspace will be cleared in safe_lift)
            self._reset_motion_state(clear_ws=False)
            self._safe_lift_then_movej(
                self.prev_joints,
                use_lift=self.transit_on_back,
                extra_time_scale=self.traj_pos_scale,
            )
            return

        # POS:x → random top/side (if pos_random_view=True)
        if cmd == "POS" and len(args) >= 1:
            try:
                idx = int(args[0])
            except Exception:
                self.get_logger().warn(f"\033[93mBad POS: {msg.data}\033[0m")
                return

            if not (1 <= idx <= 5):
                self.get_logger().warn(
                    f"\033[93mPOS index out of range: {idx}\033[0m"
                )
                return

            if self.pos_random_view:
                choice = random.randint(0, 1)
                if choice == 0:
                    view = "TOP"
                    self._publish_pos_random_view(view)
                    self.get_logger().info(
                        f"\033[94mPOS:{idx} (random) -> TOP_VIEW (pos_{idx})\033[0m"
                    )
                    self._handle_pos_idx(idx)

                else:
                    view = "SIDE"
                    self._publish_pos_random_view(view)
                    self.get_logger().info(
                        f"\033[94mPOS:{idx} (random) -> SIDE_VIEW (side_pos_{idx})\033[0m"
                    )
                    self._handle_side_idx(idx)
                    
            else:
                self.get_logger().info(
                    f"\033[94mPOS:{idx} -> TOP_VIEW only (pos_random_view=False)\033[0m"
                )
                self._handle_pos_idx(idx)
            return

        # ROTATE – base rotation
        if cmd == "ROTATE" and len(args) >= 2:
            direction = (args[0] or "").lower()
            deg = _parse_float(args[1])
            if deg is None:
                self.get_logger().warn(f"\033[93mBad ROTATE: {msg.data}\033[0m")
                return

            rad = _clamp(abs(deg) * math.pi / 180.0, 0.0, self.max_step_rad)
            target = list(self.current_joints)
            sign = +1.0 if direction == "left" else -1.0
            target[0] += sign * rad

            # If joint limit violation -> stop immediately
            ok, reason = self._is_within_joint_limits(target)
            if not ok:
                self._handle_joint_limit_violation("ROTATE", reason)
                return
            self._remember_prev()
            self._publish_traj_1pt(target)
            return

        # ROTATE_W3 – wrist_3 rotation
        if cmd == "ROTATE_W3" and len(args) >= 2:
            direction = (args[0] or "").lower()
            deg = _parse_float(args[1])
            if deg is None:
                self.get_logger().warn(f"\033[93mBad ROTATE_W3: {msg.data}\033[0m")
                return
            rad = _clamp(abs(deg) * math.pi / 180.0, 0.0, self.max_step_rad)
            target = list(self.current_joints)
            sign = +1.0 if direction == "left" else -1.0
            target[5] += sign * rad
            ok, reason = self._is_within_joint_limits(target)
            if not ok:
                self._handle_joint_limit_violation("ROTATE_W3", reason)
                return
            self._remember_prev()
            self._publish_traj_1pt(target)
            return

        # JOG
        if cmd == "JOG" and len(args) >= 2:
            direction = (args[0] or "").lower()
            dist_m = _parse_float(args[1])
            if dist_m is None:
                self.get_logger().warn(f"\033[93mBad JOG: {msg.data}\033[0m")
                return

            # ---------- forward/back using radial MoveL from base ----------
            if direction in ("forward", "back") and self.jog_fb_use_movel:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        self.ee_link,
                        Time(),
                    )
                except Exception as e:
                    self.get_logger().warn(
                        f"\033[93mJOG:{direction} TF error ({e}) -> fallback joint jog\033[0m"
                    )
                else:
                    x = float(tf.transform.translation.x)
                    y = float(tf.transform.translation.y)
                    r = math.hypot(x, y)
                    if r < 1e-4:
                        self.get_logger().warn(
                            f"\033[93mJOG:{direction} radial length too small (r={r:.6f}) "
                            "-> fallback joint jog\033[0m"
                        )
                    else:
                        ux = x / r
                        uy = y / r
                        step = _clamp(abs(dist_m), 0.0, self.jog_fb_max_dx)
                        sign = +1.0 if direction == "forward" else -1.0

                        if self.ws_active:
                            r_min = self.ws_r_min
                            r_max = self.ws_r_max
                            r_target_raw = r + sign * step
                            r_clamped = _clamp(r_target_raw, r_min, r_max)
                            if abs(r_clamped - r) < 1e-5:
                                self.get_logger().warn(
                                    "\033[93mR-LIMIT: target r out of range -> no MoveL executed. "
                                    f"(r={r:.3f}, target={r_target_raw:.3f}, "
                                    f"r_min={r_min:.3f}, r_max={r_max:.3f})\033[0m"
                                )
                                return
                            eff_step = r_clamped - r
                            dx_signed = eff_step * ux
                            dy_signed = eff_step * uy
                            self._dbg(
                                f"\033[96mJOG:{direction} radial (limited) -> MoveL "
                                f"dx={dx_signed:.3f} dy={dy_signed:.3f} "
                                f"(r={r:.3f} -> r_clamped={r_clamped:.3f}, "
                                f"r_min={r_min:.3f}, r_max={r_max:.3f})\033[0m"
                            )
                        else:
                            dx_signed = sign * step * ux
                            dy_signed = sign * step * uy
                            self._dbg(
                                f"\033[96mJOG:{direction} radial -> MoveL "
                                f"dx={dx_signed:.3f} dy={dy_signed:.3f} "
                                f"(step={step:.3f}, r={r:.3f})\033[0m"
                            )
                        self._remember_prev()
                        self._do_movel(dx_signed, dy_signed, 0.0)
                        return

            # ---------- left/right straight MoveL (axis depends on POS idx) ----------
            if direction in ("left", "right") and self.jog_lr_use_movel:
                step = _clamp(abs(dist_m), 0.0, self.jog_lr_max_dy)
                sign = +1.0 if direction == "left" else -1.0
                idx = self.ws_idx if self.ws_active else self.active_pos_idx
                use_x = (idx == 2)
                dx_signed = 0.0
                dy_signed = 0.0

                if use_x:
                    dx_signed = sign * step
                    axis = "X"
                else:
                    dy_signed = sign * step
                    axis = "Y"

                dx_eff, dy_eff = dx_signed, dy_signed
                if self.ws_active:
                    dx_eff, dy_eff, info = self._jog_lr_clamp_dxdy(dx_signed, dy_signed)

                    if abs(dx_eff) < 1e-5 and abs(dy_eff) < 1e-5:
                        self.get_logger().warn(
                            "\033[93mX/Y-LIMIT: target out of range -> no MoveL executed. "
                            f"(req dx={dx_signed:+.3f}, dy={dy_signed:+.3f})\033[0m"
                        )
                        self._dbg(f"\033[93mX/Y-LIMIT clamp detail: {info}\033[0m")
                        return

                    self._dbg(f"\033[96mX/Y-LIMIT clamp detail: {info}\033[0m")

                self._dbg(
                    f"\033[96mJOG:{direction} axis-{axis} (idx={idx}) -> MoveL "
                    f"req(dx={dx_signed:+.3f},dy={dy_signed:+.3f}) "
                    f"eff(dx={dx_eff:+.3f},dy={dy_eff:+.3f}) (step={step:.3f})\033[0m"
                )
                self._remember_prev()
                self._do_movel(dx_eff, dy_eff, 0.0)
                return

            # ---------- up/down using MoveL in Z axis + Z-LIMIT ----------
            if direction in ("up", "down") and self.jog_updown_use_movel:
                dz = _clamp(abs(dist_m), 0.0, self.jog_updown_max_dz)
                sign = +1.0 if direction == "up" else -1.0
                dz_signed = sign * dz

                if self.ws_active:
                    try:
                        tf = self.tf_buffer.lookup_transform(
                            self.base_frame,
                            self.ee_link,
                            Time(),
                        )
                    except Exception as e:
                        self.get_logger().warn(
                            f"\033[93mJOG:{direction} Z TF error ({e}) -> skip Z-LIMIT clamp\033[0m"
                        )
                    else:
                        z_cur = float(tf.transform.translation.z)
                        z_min = self.ws_z_min
                        z_max = self.ws_z_max
                        z_target_raw = z_cur + dz_signed
                        z_clamped = _clamp(z_target_raw, z_min, z_max)
                        if abs(z_clamped - z_cur) < 1e-5:
                            self.get_logger().warn(
                                "\033[93mZ-LIMIT: target Z out of range -> no MoveL executed. "
                                f"(z={z_cur:.3f}, target={z_target_raw:.3f}, "
                                f"z_min={z_min:.3f}, z_max={z_max:.3f})\033[0m"
                            )
                            return
                        dz_effective = z_clamped - z_cur
                        self.get_logger().info(
                            f"\033[94mZ-LIMIT: clamp dz from {dz_signed:+.3f} to {dz_effective:+.3f} "
                            f"(z_min={z_min:.3f}, z_max={z_max:.3f})\033[0m"
                        )
                        dz_signed = dz_effective
                self._remember_prev()
                self.get_logger().info(
                    f"\033[94mJOG:{direction} -> MoveL dz={dz_signed:.3f} m "
                    f"(limit={self.jog_updown_max_dz:.3f})\033[0m"
                )
                self._do_movel(0.0, 0.0, dz_signed)
                return

            # ---------- fallback: joint jog with basic braking ----------
            jog_map: Dict[str, Tuple[int, float]] = {
                "left": (0, +1.0),
                "right": (0, -1.0),
                "forward": (2, -1.0),
                "back": (2, +1.0),
                "up": (1, -1.0),
                "down": (1, +1.0),
            }

            if direction not in jog_map:
                self.get_logger().warn(
                    f"\033[93mUnknown jog dir: {direction}\033[0m"
                )
                return

            joint_idx, joint_sign = jog_map[direction]
            step = _clamp(abs(dist_m) * self.meter_to_rad_gain, 0.0, self.max_step_rad)
            target = list(self.current_joints)
            target[joint_idx] += joint_sign * step
            ok, reason = self._is_within_joint_limits(target)
            if not ok:
                self._handle_joint_limit_violation(f"JOG_{direction.upper()}", reason)
                return

            self._remember_prev()
            self.get_logger().info(
                f"\033[94mJOG:{direction} fallback joint -> joint[{joint_idx}] += "
                f"{joint_sign * step:.3f} rad\033[0m"
            )
            self._publish_traj_1pt(target)
            return

        # MOVEL:dx:dy:dz
        if cmd == "MOVEL" and len(args) >= 3:
            dx = _parse_float(args[0])
            dy = _parse_float(args[1])
            dz = _parse_float(args[2])
            if dx is None or dy is None or dz is None:
                self.get_logger().warn(f"\033[93mBad MOVEL: {msg.data}\033[0m")
                return
            self._remember_prev()
            self._do_movel(dx, dy, dz)
            return

        # PICK
        if cmd == "PICK":
            if self.current_view is None or self.current_pos_id is None:
                self.get_logger().warn("PICK: view or pos not set")
                return
            key = (self.current_view, self.current_pos_id)
            if key not in self.surface_height:
                self.get_logger().warn(f"PICK: surface not calibrated for {key}")
                return
            z_tcp = self._get_tcp_z()
            if z_tcp is None:
                self.get_logger().warn("PICK: failed to read TCP Z")
                return
            surface_z = self.surface_height[key]
            offset = z_tcp - surface_z

            # offset ติดลบจาก TF noise
            if offset < 0:
                self.get_logger().warn(
                    f"PICK: offset negative ({offset:.6f}) → clamp to 0"
                )
                offset = 0.0

            self.pick_offset_memory = offset
            self.last_pick_surface_z = surface_z
            self.get_logger().info(
                f"\033[92mPICK OK | view={self.current_view} "
                f"pos={self.current_pos_id} "
                f"surface={surface_z:.6f} "
                f"tcp={z_tcp:.6f} "
                f"offset={offset:.6f}\033[0m"
            )
            return
        
        # PLACE
        if cmd == "PLACE":

            if self.pick_offset_memory is None:
                self.get_logger().warn(
                    "\033[93mPLACE: no pick memory -> skip drop, but allow upward retreat\033[0m"
                )
                is_side = (self.current_view == "SIDE")
                rise_dz = self._get_lift_height(self.current_pos_id, is_side)

                if self._should_auto_rise_after_drop():
                    self._dbg(
                        f"\033[94mPLACE(no pick memory) -> rise only dz={rise_dz:.3f}\033[0m"
                    )
                    self._do_movel(0.0, 0.0, rise_dz)
                else:
                    self.get_logger().info(
                        "\033[93mPLACE(no pick memory): no PLACE permission -> no rise\033[0m"
                    )
                return

            if self.current_view is None or self.current_pos_id is None:
                self.get_logger().warn("PLACE: view or pos not set")
                return
            key = (self.current_view, self.current_pos_id)
            if key not in self.surface_height:
                self.get_logger().warn(
                    f"PLACE: surface not calibrated for {key}"
                )
                return
            z_current = self._get_tcp_z()
            if z_current is None:
                self.get_logger().warn("PLACE: failed to read TCP Z")
                return

            surface_z_new = self.surface_height[key]
            safety_clearance = 0.003  # 3 mm
            target_z = (
                surface_z_new
                + self.pick_offset_memory
                + safety_clearance
            )
            dz = target_z - z_current
            dz = _clamp(dz, -0.20, 0.20)

            self.get_logger().info(
                f"\033[96mPLACE | view={self.current_view} "
                f"pos={self.current_pos_id} "
                f"surface={surface_z_new:.6f} "
                f"offset={self.pick_offset_memory:.6f} "
                f"target_z={target_z:.6f} "
                f"dz={dz:.6f}\033[0m"
            )

            # smooth drop + optional auto rise
            drop_deadband = 0.005  
            if dz < -drop_deadband:
                # move down
                is_side = (self.current_view == "SIDE")
                rise_dz = self._get_lift_height(self.current_pos_id, is_side)
                allow_rise = self._should_auto_rise_after_drop()
                self._dbg(
                    f"\033[96mPLACE -> smooth_drop dz={abs(dz):.3f} "
                    f"rise_dz={rise_dz:.3f} auto_rise={'YES' if allow_rise else 'NO'} "
                    f"(last_cmd={self.last_hl_cmd_name})\033[0m"
                )
                self._do_movel_smooth_drop(
                    abs(dz),
                    rise_after=allow_rise,
                    rise_dz=rise_dz,
                )
            else:
                self.get_logger().info(
                    f"\033[94mPLACE: dz >= 0 ({dz:.6f}) -> no drop needed, wait then rise\033[0m"
                )
                is_side = (self.current_view == "SIDE")
                rise_dz = self._get_lift_height(self.current_pos_id, is_side)

                if self._should_auto_rise_after_drop() and self.post_drop_auto_rise:
                    wait_sec = max(0.0, float(self.post_drop_wait))
                    self._dbg(
                        f"\033[96mPLACE(no-drop): wait {wait_sec:.2f}s then rise dz={rise_dz:.3f} m\033[0m"
                    )
                    node = self
                    def _rise_cb():
                        try:
                            node.place_rise_permitted = False
                            node._do_movel(0.0, 0.0, rise_dz)
                        finally:
                            timer.cancel()
                    timer = self.create_timer(wait_sec, _rise_cb)
                return
        self.get_logger().warn(f"\033[93mUnhandled HL cmd: {msg.data}\033[0m")

    def _publish_tcp_z(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                Time()
            )
        except Exception as e:
            self.get_logger().warn(
                f"\033[93mTCP Z: TF lookup failed ({e})\033[0m"
            )
            return
        z = float(tf.transform.translation.z)
        msg = Float64()
        msg.data = z
        self.tcp_z_pub.publish(msg)
        self.get_logger().info(
            f"\033[94mPICK: Published TCP Z = {z:.4f} m "
            f"to {self.topic_tcp_z}\033[0m"
        )

    def _get_tcp_z(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                Time()
            )
        except Exception as e:
            self.get_logger().warn(
                f"\033[93mTCP Z lookup failed: {e}\033[0m"
            )
            return None
        return float(tf.transform.translation.z)
    
    def _get_lift_height(self, idx: int, is_side: bool) -> float:
        if self.use_central_lift:
            return _clamp(self.central_lift_dz, 0.0, 0.50)
        
        if is_side:
            return _clamp(
                float(self.side_lift_map.get(idx, self.movel_lift_dz)),
                0.0,
                0.50,
            )
        else:
            return _clamp(
                float(self.pos_lift_map.get(idx, self.movel_lift_dz)),
                0.0,
                0.50,
            )
        
    def _do_movel_smooth_rise(self, dz: float):
        if self.locked:
            return
        if self.last_js is None or self.current_joints is None:
            return
        if not self.cart_srv.wait_for_service(timeout_sec=0.3):
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                Time()
            )
        except Exception:
            return

        cur = tf.transform.translation
        rot = tf.transform.rotation

        dz = abs(float(dz))
        n = 6  
        waypoints = []
        for i in range(1, n + 1):
            s = i / float(n)
            ease = 6*s**5 - 15*s**4 + 10*s**3
            wp = Pose()
            wp.position.x = cur.x
            wp.position.y = cur.y
            wp.position.z = cur.z + dz * ease   
            wp.orientation = rot
            waypoints.append(wp)
        rs = RobotState()
        rs.joint_state.name = list(self.last_js.name)
        rs.joint_state.position = list(self.last_js.position)
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.start_state = rs
        req.group_name = self.group_name
        req.link_name = self.ee_link
        req.waypoints = waypoints
        req.max_step = self.movel_max_step
        req.jump_threshold = self.movel_jump_threshold
        req.avoid_collisions = self.movel_avoid_collisions
        self.movel_busy = True
        fut = self.cart_srv.call_async(req)

        def _cb(fut):
            self.movel_busy = False
            try:
                res = fut.result()
            except Exception:
                return
            if res.fraction < 0.7:
                self.get_logger().warn(
                    f"\033[93mRISE failed: fraction={res.fraction:.3f} < 0.8\033[0m"
                )
                return
            self._send_joint_trajectory(
                res.solution.joint_trajectory,
                tag="safe_movel_rise"
            )
        fut.add_done_callback(_cb)

    def _do_movel_smooth_drop(
            self,
            dz: float,
            rise_after: bool = False,
            rise_dz: Optional[float] = None,
            place_seq_id: Optional[int] = None,
        ):
        if self.locked:
            return
        if self.last_js is None or self.current_joints is None:
            self.get_logger().warn("\033[93mDROP: no joint state -> abort\033[0m")
            return
        if not self.cart_srv.wait_for_service(timeout_sec=0.3):
            self.get_logger().warn("\033[93mDROP: /compute_cartesian_path unavailable\033[0m")
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                Time()
            )
        except Exception as e:
            self.get_logger().warn(f"\033[93mDROP: TF lookup failed ({e})\033[0m")
            return

        cur = tf.transform.translation
        rot = tf.transform.rotation

        dz = abs(float(dz))
        n = 6  

        waypoints: List[Pose] = []
        for i in range(1, n + 1):
            s = i / float(n)
            ease = 6*s**5 - 15*s**4 + 10*s**3
            wp = Pose()
            wp.position.x = cur.x
            wp.position.y = cur.y
            wp.position.z = cur.z - dz * ease   
            wp.orientation = rot
            waypoints.append(wp)

        rs = RobotState()
        rs.joint_state.name = list(self.last_js.name)
        rs.joint_state.position = list(self.last_js.position)
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.start_state = rs
        req.group_name = self.group_name
        req.link_name = self.ee_link
        req.waypoints = waypoints
        req.max_step = self.movel_max_step
        req.jump_threshold = self.movel_jump_threshold
        req.avoid_collisions = self.movel_avoid_collisions
        self.movel_busy = True
        fut = self.cart_srv.call_async(req)

        def _cb(fut):
            self.movel_busy = False
            try:
                res = fut.result()
            except Exception as e:
                self.get_logger().warn(f"\033[93mDROP failed: {e}\033[0m")
                return
            if res.fraction < 0.8:
                self.get_logger().warn(
                    f"\033[93mDROP failed: fraction={res.fraction:.3f} < 0.8\033[0m"
                )
                return
            self._send_joint_trajectory(
                res.solution.joint_trajectory,
                tag="safe_movel_drop"
            )
            if not rise_after or not self.post_drop_auto_rise:
                self._dbg("\033[94mDROP done -> auto rise disabled\033[0m")
                return
            dz_up = abs(float(rise_dz if rise_dz is not None else self.movel_lift_dz))
            wait_sec = max(0.0, float(self.post_drop_wait))
            traj_time = self._traj_duration_sec(
                res.solution.joint_trajectory,
                scale=max(0.1, float(self.movel_traj_global_scale) * 2.0 * self._get_speed_scale())
            )
            total_wait = traj_time + wait_sec
            self._dbg(
                f"\033[96mDROP done -> wait traj={traj_time:.2f}s + dwell={wait_sec:.2f}s "
                f"= {total_wait:.2f}s, then smooth rise dz={dz_up:.3f} m\033[0m"
            )
            node = self

            def _rise_cb():
                try:
                    node.place_rise_permitted = False
                    node._do_movel_smooth_rise(dz_up)
                finally:
                    timer.cancel()
            timer = self.create_timer(total_wait, _rise_cb)
        fut.add_done_callback(_cb)

    def _publish_pos_random_view(self, view: str):
        msg = String()
        msg.data = view  
        self.pos_random_pub.publish(msg)
        self._dbg(f"\033[96mPOS_RANDOM_VIEW published: {view}\033[0m")

    def _should_auto_rise_after_drop(self) -> bool:
        ok = bool(self.place_rise_permitted)

        self._dbg(
            f"\033[95mAUTO_RISE CHECK: "
            f"place_rise_permitted={self.place_rise_permitted} "
            f"last_hl_cmd_raw='{self.last_hl_cmd_raw}' "
            f"last_hl_cmd_name='{self.last_hl_cmd_name}' "
            f"-> {'ALLOW' if ok else 'BLOCK'}\033[0m"
        )
        return ok
        
    def _update_place_rise_permission(self, cmd: str):
        cmd_u = (cmd or "").strip().upper()

        if cmd_u == "PLACE":
            self.place_rise_permitted = True
            self._dbg("\033[92mPLACE_RISE_PERMISSION -> ENABLED\033[0m")
            return

        should_clear = (
            cmd_u in ("STOP", "HOME", "BACK_HOME", "BACK", "RETURN", "POS")
            or cmd_u.startswith("TOP_VIEW")
            or cmd_u.startswith("SIDE_VIEW")
        )

        if should_clear:
            if self.place_rise_permitted:
                self._dbg(f"\033[93mPLACE_RISE_PERMISSION -> CLEARED by cmd={cmd_u}\033[0m")
            self.place_rise_permitted = False

    def _traj_duration_sec(self, jt: JointTrajectory, scale: float = 1.0) -> float:
        if not jt.points:
            return 0.0
        last = jt.points[-1].time_from_start
        t = float(last.sec) + float(last.nanosec) * 1e-9
        return t * scale
    
    def _mark_movel_executing(self, jt: JointTrajectory):
     
        # cancel previous timer if any
        if self.movel_exec_timer is not None:
            try:
                self.movel_exec_timer.cancel()
            except Exception:
                pass
            self.movel_exec_timer = None

        duration = self._traj_duration_sec(jt, scale=1.0)
        duration = max(0.1, float(duration))  # at least 0.1 sec

        self.movel_executing = True
        self._dbg(f"\033[96mMOVEL_EXEC start for {duration:.2f}s\033[0m")

        node = self

        def _done_cb():
            try:
                node.movel_executing = False
                node._dbg("\033[92mMOVEL_EXEC finished\033[0m")
            finally:
                try:
                    timer.cancel()
                except Exception:
                    pass
                node.movel_exec_timer = None

        timer = self.create_timer(duration, _done_cb)
        self.movel_exec_timer = timer

    def _on_movel_stop(self, msg: Bool):
        """
        Emergency stop for MoveL only:
        If Bool(True) is received while a MoveL-related trajectory is executing,
        publish a smooth-stop trajectory (or fallback hold) and allow next command.
        """
        Red = "\033[91m"
        Yellow = "\033[93m"
        Reset = "\033[0m"

        if not msg.data:
            return

        if not self.movel_executing:
            self._dbg(
                f"{Yellow}MOVEL_STOP received=True but movel_executing=False -> ignore{Reset}"
            )
            return

        self.get_logger().warn(
            f"{Red}🛑 MOVEL_STOP=True received during MoveL execution -> SMOOTH STOP{Reset}"
        )

        # stop current motion flags
        self.movel_busy = False
        self.movel_executing = False

        # cancel executing timer
        if self.movel_exec_timer is not None:
            try:
                self.movel_exec_timer.cancel()
            except Exception:
                pass
            self.movel_exec_timer = None

        # clear pending states / workspace (design choice)
        self._reset_motion_state(clear_ws=True)

        # smooth stop first
        ok = self._publish_smooth_stop_traj(
            lock_after=False,
            stop_tag="MOVEL_STOP"
        )

        # fallback = hold current pose
        if not ok:
            if self.current_joints is not None:
                traj = JointTrajectory()
                traj.joint_names = self.joint_names

                pt = JointTrajectoryPoint()
                pt.positions = list(self.current_joints)
                pt.time_from_start.sec = 0
                pt.time_from_start.nanosec = int(0.1 * 1e9)

                traj.points.append(pt)
                self.traj_pub.publish(traj)

                self.get_logger().warn(
                    f"{Red}🛑 MOVEL_STOP fallback: hold current pose trajectory published{Reset}"
                )
            else:
                self.get_logger().warn(
                    f"{Yellow}MOVEL_STOP: current_joints unavailable -> cannot publish hold trajectory{Reset}"
                )

    def _publish_smooth_stop_traj(
        self,
        lock_after: bool = True,
        stop_tag: str = "SMOOTH_STOP",
        max_continue_ratio: float = 0.12,   # move only 12% of remaining distance
        min_step_rad: float = 0.0001,        # ignore tiny residuals
        max_step_rad_per_joint: float = 0.05,  # cap overshoot per joint
    ) -> bool:
        
        Yellow = "\033[93m"
        Green = "\033[92m"
        Cyan = "\033[96m"
        Reset = "\033[0m"

        if self.current_joints is None:
            self.get_logger().warn(f"{Yellow}{stop_tag}: current_joints unavailable{Reset}")
            return False

        cur = list(self.current_joints)

        # No previous target -> cannot infer direction, fallback to hard hold
        if self.last_commanded_joints is None or len(self.last_commanded_joints) != len(cur):
            self.get_logger().warn(
                f"{Yellow}{stop_tag}: no last_commanded_joints -> fallback to hold current pose{Reset}"
            )
            return False

        rem = [t - c for c, t in zip(cur, self.last_commanded_joints)]

        # Build a tiny forward continuation in the same direction
        # Decreasing fractions for smooth brake
        fractions = [0.03, 0.08, 0.14, 0.22, 0.31, 0.42, 0.54, 0.66, 0.77, 0.86, 0.92, 0.96, 0.99, 1.00]
        time_marks = [0.14, 0.24, 0.34, 0.46, 0.60, 0.76, 0.94, 1.14, 1.34, 1.54, 1.70, 1.84, 1.96, 2.04]
    
        # Compute first continuation vector (small)
        cont = []
        for d in rem:
            if abs(d) < min_step_rad:
                cont.append(0.0)
                continue

            step = d * max_continue_ratio
            step = _clamp(step, -max_step_rad_per_joint, max_step_rad_per_joint)
            cont.append(step)

        # If all continuation is ~0 -> fallback
        if max(abs(x) for x in cont) < min_step_rad:
            self.get_logger().warn(
                f"{Yellow}{stop_tag}: residual motion too small -> fallback to hold current pose{Reset}"
            )
            return False

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        last_pos = list(cur)

        for frac, tmark in zip(fractions, time_marks):
            pt = JointTrajectoryPoint()

            pos = []
            for c, step in zip(cur, cont):
                q = c + step * frac
                pos.append(float(q))

            ok, reason = self._is_within_joint_limits(pos)
            if not ok:
                self.get_logger().warn(
                    f"{Yellow}{stop_tag}: soft-stop point violates joint limit ({reason}) "
                    f"-> fallback to hold current pose{Reset}"
                )
                return False

            pt.positions = pos
            pt.time_from_start.sec = int(tmark)
            pt.time_from_start.nanosec = int((tmark - int(tmark)) * 1e9)
            traj.points.append(pt)
            last_pos = pos

        # Final hold point
        final_t = 2.18
        hold = JointTrajectoryPoint()
        hold.positions = list(last_pos)
        hold.time_from_start.sec = int(final_t)
        hold.time_from_start.nanosec = int((final_t - int(final_t)) * 1e9)
        traj.points.append(hold)

        self.traj_pub.publish(traj)

        # update last commanded pose to stop point
        self.last_commanded_joints = list(last_pos)

        self.get_logger().info(
            f"{Green}{stop_tag}: smooth stop trajectory published "
            f"(points={len(traj.points)}, total={final_t:.2f}s){Reset}"
        )

        if lock_after:
            self.locked = True

        return True

def main(args=None):
    rclpy.init(args=args)
    node = UR5ExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
