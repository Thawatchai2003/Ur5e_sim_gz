# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     # -----------------------------
#     # Launch args
#     # -----------------------------
#     ur_type = LaunchConfiguration("ur_type")
#     robot_ip = LaunchConfiguration("robot_ip")
#     use_fake_hardware = LaunchConfiguration("use_fake_hardware")
#     launch_rviz = LaunchConfiguration("launch_rviz")
#     gripper_mode = LaunchConfiguration("gripper_mode")  # sim | real

#     declare_ur_type = DeclareLaunchArgument("ur_type", default_value="ur5e")
#     declare_robot_ip = DeclareLaunchArgument("robot_ip", default_value="192.168.1.200")
#     declare_use_fake_hw = DeclareLaunchArgument("use_fake_hardware", default_value="false")
#     declare_launch_rviz = DeclareLaunchArgument("launch_rviz", default_value="true")
#     declare_gripper_mode = DeclareLaunchArgument("gripper_mode", default_value="real")

#     # ur_robot_driver: initial controller
#     initial_joint_controller = PythonExpression([
#         "'joint_trajectory_controller' if '", use_fake_hardware, "' == 'true' "
#         "else 'scaled_joint_trajectory_controller'"
#     ])

#     # controllers yaml (ของคุณ)
#     controllers_file = PathJoinSubstitution([
#         FindPackageShare("ur5_custom_description"),
#         "config",
#         "ur5e_with_gripper_controllers.yaml",
#     ])

#     # rviz config (ของคุณ)
#     rviz_config = PathJoinSubstitution([
#         FindPackageShare("ur5_sim_gz"),
#         "rviz",
#         "ur5_with_gripper.rviz",
#     ])

#     # -----------------------------
#     # ✅ UR config files (ใช้ของ ur5_custom_description)
#     # - fake: ใช้ default_kinematics.yaml
#     # - real: ใช้ calibration.yaml
#     # -----------------------------
#     ur5e_cfg_dir = PathJoinSubstitution([
#         FindPackageShare("ur5_custom_description"),
#         "config",
#         "ur5e",
#     ])

#     joint_limit_params = PathJoinSubstitution([ur5e_cfg_dir, "joint_limits.yaml"])
#     physical_params = PathJoinSubstitution([ur5e_cfg_dir, "physical_parameters.yaml"])
#     visual_params = PathJoinSubstitution([ur5e_cfg_dir, "visual_parameters.yaml"])

#     default_kinematics_params = PathJoinSubstitution([ur5e_cfg_dir, "default_kinematics.yaml"])
#     calibration_params = PathJoinSubstitution([ur5e_cfg_dir, "calibration.yaml"])

#     # ถ้า fake hardware -> ใช้ default_kinematics
#     # ถ้า real -> ใช้ calibration
#     kinematics_params = PythonExpression([
#         "('", default_kinematics_params, "') if ('", use_fake_hardware, "' == 'true') "
#         "else ('", calibration_params, "')"
#     ])

#     # -----------------------------
#     # ✅ UR driver (ur_robot_driver) ใช้ URDF + config ของคุณ
#     # -----------------------------
#     ur_driver = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare("ur_robot_driver"),
#                 "launch",
#                 "ur_control.launch.py",
#             ])
#         ),
#         launch_arguments={
#             "ur_type": ur_type,
#             "robot_ip": robot_ip,
#             "use_fake_hardware": use_fake_hardware,
#             "launch_rviz": launch_rviz,
#             "initial_joint_controller": initial_joint_controller,

#             # ✅ ใช้ URDF custom ของคุณ
#             "description_package": "ur5_custom_description",
#             "description_file": "ur5e_with_gripper.urdf.xacro",

#             # ✅ controllers ของคุณ
#             "controllers_file": controllers_file,

#             # ✅ rviz config ของคุณ
#             "rviz_config": rviz_config,

#             # ✅ ส่ง config params ให้ ur_control
#             "joint_limit_params": joint_limit_params,
#             "kinematics_params": kinematics_params,
#             "physical_params": physical_params,
#             "visual_params": visual_params,
#         }.items(),
#     )

#     # -----------------------------
#     # Respeaker mic array
#     # -----------------------------
#     respeaker_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare("respeaker_mic_array"),
#                 "launch",
#                 "respeaker.launch.py",
#             ])
#         ),
#         launch_arguments={
#             "dsp_mode": "full"
#         }.items(),
#     )


#     # -----------------------------
#     # ✅ รวม MoveIt (ur_moveit_config) มารันใน launch นี้เลย
#     # -----------------------------
#     moveit_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare("ur_moveit_config"),
#                 "launch",
#                 "ur_moveit.launch.py",
#             ])
#         ),
#         launch_arguments={
#             "ur_type": ur_type,
#             "use_fake_hardware": use_fake_hardware,
#             # ไม่ต้องเปิด RViz ซ้ำ (RViz ใช้ของ ur_driver ก็พอ)
#             "launch_rviz": "false",
#         }.items(),
#     )

#     # -----------------------------
#     # Spawn only GRIPPER controller (SIM only)
#     # แขน: ur_control.launch.py spawn ให้อยู่แล้ว ห้าม spawn ซ้ำ
#     # -----------------------------
#     gripper_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
#         output="screen",
#         condition=IfCondition(PythonExpression(["'", gripper_mode, "' == 'sim'"])),
#     )
#     auto_spawn_gripper = TimerAction(period=3.5, actions=[gripper_spawner])

#     # -----------------------------
#     # Voice stack nodes
#     # -----------------------------
#     audio_receiver = Node(
#         package="ur5_sim_gz",
#         executable="audio_receiver_node",
#         name="audio_receiver_node",
#         output="screen",
#     )

#     audio_input = Node(
#         package="ur5_sim_gz",
#         executable="audio_input",
#         name="audio_input_node",
#         output="screen",
#         parameters=[{
#             "sample_rate": 16000,
#             "chunk_size": 1024,
#             "device_index": -1,
#             "debug": False,
#         }],
#     )

#     beep = Node(
#         package="ur5_sim_gz",
#         executable="beep_node",
#         name="beep_node",
#         output="screen",
#     )

#     tts = Node(
#         package="ur5_sim_gz",
#         executable="tts_node_gtts",
#         name="tts_node_gtts",
#         output="screen",
#     )

#     stt = Node(
#         package="ur5_sim_gz",
#         executable="speech_to_text_node",
#         name="speech_to_text_node",
#         output="screen",
#         parameters=[{
#             "debug_print_heard": False,
#             "idle_recognize_interval_sec": 0.6,
#             "post_tts_ignore_sec": 0.8,
#             "tts_resume_delay_sec": 0.8,
#         }],
#     )

#     fsm = Node(
#         package="ur5_sim_gz",
#         executable="dialog_fsm_node",
#         name="dialog_fsm_node",
#         output="screen",
#         parameters=[{
#             "wake_words": ["สวัสดี", "Hello"],
#             "wake_timeout_sec": 6.0,
#             "wake_cooldown_ms": 1200,
#             "post_command_ignore_sec": 1.0,
#             "tts_resume_delay_sec": 1.0,
#             "dialog_max_retry": 2,
#             "debug_heard_when_active": True,
#             "beep_enabled": True,
#         }],
#     )

#     gui = Node(
#         package="ur5_sim_gz",
#         executable="speech_gui_node",
#         name="speech_gui_node",
#         output="screen",
#     )

#     nlu = Node(
#         package="ur5_sim_gz",
#         executable="nlu_parser_node",
#         name="nlu_parser_node",
#         output="screen",
#     )

#     logger = Node(
#         package="ur5_sim_gz",
#         executable="voice_logger_node",
#         name="voice_logger_node",
#         output="screen",
#     )

#     # -----------------------------
#     # Gripper bridge (sim/real)
#     # -----------------------------
#     gripper_bridge = Node(
#         package="ur5_sim_gz",
#         executable="gripper_bridge_node",
#         name="gripper_bridge_node",
#         output="screen",
#         parameters=[{
#             "mode": gripper_mode,
#             "cmd_topic": "/Neural_parser/cmd_group",

#             "sim_publish_also_in_real": True,
#             "sim_topic": "/gripper_controller/commands",
#             "sim_open": 0.0,
#             "sim_close": 0.01,

#             "io_service": "/io_and_status_controller/set_io",
#             "fun": 1,

#             # REAL: DO16/DO17
#             "use_two_pins": True,
#             "open_pin": 16,
#             "close_pin": 17,
#             "pulse_ms": 200,

#             # fallback (ไม่ใช้ใน two-pin)
#             "do_pin": 0,
#             "open_state": 0.0,
#             "close_state": 1.0,
#         }],
#     )

#     # -----------------------------
#     # Robot command nodes
#     # -----------------------------
#     mapper_sim = Node(
#         package="ur5_sim_gz",
#         executable="ur5_cmd_mapper_node",
#         name="ur5_cmd_mapper_node",
#         output="screen",
#         parameters=[{
#             "mode": "high_level",
#             "debug": True,
#             "traj_topic": "/joint_trajectory_controller/joint_trajectory",
#             "step_xyz_m": 0.03,
#             "rotate_step_deg": 15.0,
#             "traj_time_s": 1.0,
#             "max_deg": 180.0,
#             "max_step_m": 0.20,
#         }],
#         condition=IfCondition(use_fake_hardware),
#     )

#     # -----------------------------
#     # Control Position Node (NEW)
#     # -----------------------------
#     control_position_node = Node(
#         package="ur5_sim_gz",   # <-- แก้ให้ตรงกับ package ที่คุณใส่ไฟล์ไว้
#         executable="control_position_node",
#         name="control_position_node",
#         output="screen",
#     )

#     executor_sim = Node(
#         package="ur5_sim_gz",
#         executable="ur5_executor_node",
#         name="ur5_executor_node",
#         output="screen",
#         remappings=[
#             ("/ur5/command_trajectory", "/joint_trajectory_controller/joint_trajectory"),
#         ],
#         condition=IfCondition(use_fake_hardware),
#     )

#     mapper_real = Node(
#         package="ur5_sim_gz",
#         executable="ur5_cmd_mapper_node",
#         name="ur5_cmd_mapper_node_real",
#         output="screen",
#         parameters=[{
#             "mode": "high_level",
#             "debug": True,
#             "traj_topic": "/scaled_joint_trajectory_controller/joint_trajectory",
#             "step_xyz_m": 0.03,
#             "rotate_step_deg": 15.0,
#             "traj_time_s": 1.0,
#             "max_deg": 180.0,
#             "max_step_m": 0.20,
#         }],
#         condition=IfCondition(PythonExpression(["'", use_fake_hardware, "' == 'false'"])),
#     )

#     executor_real = Node(
#         package="ur5_sim_gz",
#         executable="ur5_executor_node",
#         name="ur5_executor_node_real",
#         output="screen",
#         remappings=[
#             ("/ur5/command_trajectory", "/scaled_joint_trajectory_controller/joint_trajectory"),
#         ],
#         condition=IfCondition(PythonExpression(["'", use_fake_hardware, "' == 'false'"])),
#     )

#     # -----------------------------
#     # Logical pick object (SIM only)
#     # -----------------------------
#     logical_pick_object_node = Node(
#         package="ur5_workcell_scene",
#         executable="logical_pick_object_node",
#         name="logical_pick_object_node",
#         output="screen",
#         parameters=[{
#             "world_frame": "world",
#             "gripper_frame": "tool0",   # ถ้าทีหลังมี frame gripper จริง ค่อยเปลี่ยน
#         }],
#         condition=IfCondition(use_fake_hardware),
#     )

#     delayed_logical_pick_object = TimerAction(
#         period=4.0,
#         actions=[logical_pick_object_node],
#     )

#     # -----------------------------
#     # driver + MoveIt + controllers ก่อนค่อยสตาร์ต command nodes
#     # -----------------------------
#     delayed_robot_nodes = TimerAction(
#         period=6.0,
#         actions=[
#             mapper_sim,
#             executor_sim,
#             mapper_real,
#             executor_real,
#         ],
#     )

#     # -----------------------------
#     # Force monitor nodes (SIM / REAL)
#     # -----------------------------
#     force_monitor_sim = Node(
#         package="ur5_force_tools",
#         executable="force_gui",
#         name="ur5_force_gui_sim",
#         output="screen",
#         condition=IfCondition(use_fake_hardware),
#     )

#     force_monitor_real = Node(
#         package="ur5_force_tools",
#         executable="force_gui",
#         name="ur5_force_gui_real",
#         output="screen",
#         condition=IfCondition(PythonExpression(["'", use_fake_hardware, "' == 'false'"])),
#     )

#     return LaunchDescription([
#         declare_ur_type,
#         declare_robot_ip,
#         declare_use_fake_hw,
#         declare_launch_rviz,
#         declare_gripper_mode,

#         ur_driver,
#         moveit_launch,        

#         # spawn only gripper controller in sim
#         auto_spawn_gripper,
#        # audio_input,

#         #respeaker_launch,#audio_receiver,

#         # voice stack
#         stt, beep, tts, fsm, gui, nlu, logger, gripper_bridge,

#         # force monitor
#         force_monitor_sim,
#         force_monitor_real,

#         #
#         control_position_node, 
#         #
#         delayed_logical_pick_object,
        
        
#         # motion nodes
#         delayed_robot_nodes,
#     ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -----------------------------
    # Launch args
    # -----------------------------
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gripper_mode = LaunchConfiguration("gripper_mode")  # sim | real

    # NEW: ReSpeaker args
    enable_respeaker = LaunchConfiguration("enable_respeaker")
    respeaker_mic_mode = LaunchConfiguration("respeaker_mic_mode")
    respeaker_enable_graph = LaunchConfiguration("respeaker_enable_graph")

    declare_ur_type = DeclareLaunchArgument("ur_type", default_value="ur5e")
    declare_robot_ip = DeclareLaunchArgument("robot_ip", default_value="192.168.1.200")
    declare_use_fake_hw = DeclareLaunchArgument("use_fake_hardware", default_value="false")
    declare_launch_rviz = DeclareLaunchArgument("launch_rviz", default_value="true")
    declare_gripper_mode = DeclareLaunchArgument("gripper_mode", default_value="real")

    # NEW
    declare_enable_respeaker = DeclareLaunchArgument(
        "enable_respeaker",
        default_value="false",
        description="Enable ReSpeaker mic array launch"
    )

    declare_respeaker_mic_mode = DeclareLaunchArgument(
        "respeaker_mic_mode",
        default_value="mode2",
        description="ReSpeaker mic mode: mode1=ReSpeaker, mode2=auto scan"
    )

    declare_respeaker_enable_graph = DeclareLaunchArgument(
        "respeaker_enable_graph",
        default_value="false",
        description="Enable ReSpeaker graph node"
    )

    # ur_robot_driver: initial controller
    initial_joint_controller = PythonExpression([
        "'joint_trajectory_controller' if '", use_fake_hardware, "' == 'true' "
        "else 'scaled_joint_trajectory_controller'"
    ])

    # controllers yaml (ของคุณ)
    controllers_file = PathJoinSubstitution([
        FindPackageShare("ur5_custom_description"),
        "config",
        "ur5e_with_gripper_controllers.yaml",
    ])

    # rviz config (ของคุณ)
    rviz_config = PathJoinSubstitution([
        FindPackageShare("ur5_sim_gz"),
        "rviz",
        "ur5_with_gripper.rviz",
    ])

    # -----------------------------
    # ✅ UR config files (ใช้ของ ur5_custom_description)
    # - fake: ใช้ default_kinematics.yaml
    # - real: ใช้ calibration.yaml
    # -----------------------------
    ur5e_cfg_dir = PathJoinSubstitution([
        FindPackageShare("ur5_custom_description"),
        "config",
        "ur5e",
    ])

    joint_limit_params = PathJoinSubstitution([ur5e_cfg_dir, "joint_limits.yaml"])
    physical_params = PathJoinSubstitution([ur5e_cfg_dir, "physical_parameters.yaml"])
    visual_params = PathJoinSubstitution([ur5e_cfg_dir, "visual_parameters.yaml"])

    default_kinematics_params = PathJoinSubstitution([ur5e_cfg_dir, "default_kinematics.yaml"])
    calibration_params = PathJoinSubstitution([ur5e_cfg_dir, "calibration.yaml"])

    # ถ้า fake hardware -> ใช้ default_kinematics
    # ถ้า real -> ใช้ calibration
    kinematics_params = PythonExpression([
        "('", default_kinematics_params, "') if ('", use_fake_hardware, "' == 'true') "
        "else ('", calibration_params, "')"
    ])

    # -----------------------------
    # ✅ UR driver (ur_robot_driver) ใช้ URDF + config ของคุณ
    # -----------------------------
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz": launch_rviz,
            "initial_joint_controller": initial_joint_controller,

            # ✅ ใช้ URDF custom ของคุณ
            "description_package": "ur5_custom_description",
            "description_file": "ur5e_with_gripper.urdf.xacro",

            # ✅ controllers ของคุณ
            "controllers_file": controllers_file,

            # ✅ rviz config ของคุณ
            "rviz_config": rviz_config,

            # ✅ ส่ง config params ให้ ur_control
            "joint_limit_params": joint_limit_params,
            "kinematics_params": kinematics_params,
            "physical_params": physical_params,
            "visual_params": visual_params,
        }.items(),
    )

    # -----------------------------
    # ReSpeaker mic array (NEW)
    # เปิด/ปิดได้ด้วย launch arg
    # -----------------------------
    respeaker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("respeaker_mic_array"),
                "launch",
                "respeaker.launch.py",
            ])
        ),
        launch_arguments={
            "mic_mode": respeaker_mic_mode,
            "enable_graph": respeaker_enable_graph,
        }.items(),
        condition=IfCondition(enable_respeaker),
    )

    # -----------------------------
    # ✅ รวม MoveIt (ur_moveit_config) มารันใน launch นี้เลย
    # -----------------------------
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_fake_hardware": use_fake_hardware,
            # ไม่ต้องเปิด RViz ซ้ำ (RViz ใช้ของ ur_driver ก็พอ)
            "launch_rviz": "false",
        }.items(),
    )

    # -----------------------------
    # Spawn only GRIPPER controller (SIM only)
    # แขน: ur_control.launch.py spawn ให้อยู่แล้ว ห้าม spawn ซ้ำ
    # -----------------------------
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", gripper_mode, "' == 'sim'"])),
    )
    auto_spawn_gripper = TimerAction(period=3.5, actions=[gripper_spawner])

    # -----------------------------
    # Voice stack nodes
    # -----------------------------
    audio_receiver = Node(
        package="ur5_sim_gz",
        executable="audio_receiver_node",
        name="audio_receiver_node",
        output="screen",
    )

    audio_input = Node(
        package="ur5_sim_gz",
        executable="audio_input",
        name="audio_input_node",
        output="screen",
        parameters=[{
            "sample_rate": 16000,
            "chunk_size": 1024,
            "device_index": -1,
            "debug": False,
        }],
    )

    beep = Node(
        package="ur5_sim_gz",
        executable="beep_node",
        name="beep_node",
        output="screen",
    )

    tts = Node(
        package="ur5_sim_gz",
        executable="tts_node_gtts",
        name="tts_node_gtts",
        output="screen",
    )

    stt = Node(
        package="ur5_sim_gz",
        executable="speech_to_text_node",
        name="speech_to_text_node",
        output="screen",
        parameters=[{
            "debug_print_heard": False,
            "idle_recognize_interval_sec": 0.6,
            "post_tts_ignore_sec": 0.8,
            "tts_resume_delay_sec": 0.8,
        }],
    )

    fsm = Node(
        package="ur5_sim_gz",
        executable="dialog_fsm_node",
        name="dialog_fsm_node",
        output="screen",
        parameters=[{
            "wake_words": ["สวัสดี", "Hello"],
            "wake_timeout_sec": 6.0,
            "wake_cooldown_ms": 1200,
            "post_command_ignore_sec": 1.0,
            "tts_resume_delay_sec": 1.0,
            "dialog_max_retry": 2,
            "debug_heard_when_active": True,
            "beep_enabled": True,
        }],
    )

    gui = Node(
        package="ur5_sim_gz",
        executable="speech_gui_node",
        name="speech_gui_node",
        output="screen",
    )

    nlu = Node(
        package="ur5_sim_gz",
        executable="nlu_parser_node",
        name="nlu_parser_node",
        output="screen",
    )

    logger = Node(
        package="ur5_sim_gz",
        executable="voice_logger_node",
        name="voice_logger_node",
        output="screen",
    )

    # -----------------------------
    # Gripper bridge (sim/real)
    # -----------------------------
    gripper_bridge = Node(
        package="ur5_sim_gz",
        executable="gripper_bridge_node",
        name="gripper_bridge_node",
        output="screen",
        parameters=[{
            "mode": gripper_mode,
            "cmd_topic": "/Neural_parser/cmd_group",

            "sim_publish_also_in_real": True,
            "sim_topic": "/gripper_controller/commands",
            "sim_open": 0.0,
            "sim_close": 0.01,

            "io_service": "/io_and_status_controller/set_io",
            "fun": 1,

            # REAL: DO16/DO17
            "use_two_pins": True,
            "open_pin": 16,
            "close_pin": 17,
            "pulse_ms": 200,

            # fallback (ไม่ใช้ใน two-pin)
            "do_pin": 0,
            "open_state": 0.0,
            "close_state": 1.0,
        }],
    )

    # -----------------------------
    # Robot command nodes
    # -----------------------------
    mapper_sim = Node(
        package="ur5_sim_gz",
        executable="ur5_cmd_mapper_node",
        name="ur5_cmd_mapper_node",
        output="screen",
        parameters=[{
            "mode": "high_level",
            "debug": True,
            "traj_topic": "/joint_trajectory_controller/joint_trajectory",
            "step_xyz_m": 0.03,
            "rotate_step_deg": 15.0,
            "traj_time_s": 1.0,
            "max_deg": 180.0,
            "max_step_m": 0.20,
        }],
        condition=IfCondition(use_fake_hardware),
    )

    # -----------------------------
    # Control Position Node
    # -----------------------------
    control_position_node = Node(
        package="ur5_sim_gz",
        executable="control_position_node",
        name="control_position_node",
        output="screen",
    )

    executor_sim = Node(
        package="ur5_sim_gz",
        executable="ur5_executor_node",
        name="ur5_executor_node",
        output="screen",
        remappings=[
            ("/ur5/command_trajectory", "/joint_trajectory_controller/joint_trajectory"),
        ],
        condition=IfCondition(use_fake_hardware),
    )

    mapper_real = Node(
        package="ur5_sim_gz",
        executable="ur5_cmd_mapper_node",
        name="ur5_cmd_mapper_node_real",
        output="screen",
        parameters=[{
            "mode": "high_level",
            "debug": True,
            "traj_topic": "/scaled_joint_trajectory_controller/joint_trajectory",
            "step_xyz_m": 0.03,
            "rotate_step_deg": 15.0,
            "traj_time_s": 1.0,
            "max_deg": 180.0,
            "max_step_m": 0.20,
        }],
        condition=IfCondition(PythonExpression(["'", use_fake_hardware, "' == 'false'"])),
    )

    executor_real = Node(
        package="ur5_sim_gz",
        executable="ur5_executor_node",
        name="ur5_executor_node_real",
        output="screen",
        remappings=[
            ("/ur5/command_trajectory", "/scaled_joint_trajectory_controller/joint_trajectory"),
        ],
        condition=IfCondition(PythonExpression(["'", use_fake_hardware, "' == 'false'"])),
    )

    # -----------------------------
    # Logical pick object (SIM only)
    # -----------------------------
    logical_pick_object_node = Node(
        package="ur5_workcell_scene",
        executable="logical_pick_object_node",
        name="logical_pick_object_node",
        output="screen",
        parameters=[{
            "world_frame": "world",
            "gripper_frame": "tool0",
        }],
        condition=IfCondition(use_fake_hardware),
    )

    delayed_logical_pick_object = TimerAction(
        period=4.0,
        actions=[logical_pick_object_node],
    )

    # -----------------------------
    # driver + MoveIt + controllers ก่อนค่อยสตาร์ต command nodes
    # -----------------------------
    delayed_robot_nodes = TimerAction(
        period=6.0,
        actions=[
            mapper_sim,
            executor_sim,
            mapper_real,
            executor_real,
        ],
    )

    # -----------------------------
    # Force monitor nodes (SIM / REAL)
    # -----------------------------
    force_monitor_sim = Node(
        package="ur5_force_tools",
        executable="force_gui",
        name="ur5_force_gui_sim",
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    force_monitor_real = Node(
        package="ur5_force_tools",
        executable="force_gui",
        name="ur5_force_gui_real",
        output="screen",
        condition=IfCondition(PythonExpression(["'", use_fake_hardware, "' == 'false'"])),
    )

    return LaunchDescription([
        declare_ur_type,
        declare_robot_ip,
        declare_use_fake_hw,
        declare_launch_rviz,
        declare_gripper_mode,

        # NEW
        declare_enable_respeaker,
        declare_respeaker_mic_mode,
        declare_respeaker_enable_graph,

        ur_driver,
        moveit_launch,

        # NEW: ReSpeaker launch
        respeaker_launch,

        # spawn only gripper controller in sim
        auto_spawn_gripper,

        # ถ้าอยากใช้ node เดิมใน ur5_sim_gz ค่อยเปิด
        # audio_input,
        # audio_receiver,

        # voice stack
        stt, beep, tts, fsm, gui, nlu, logger, gripper_bridge,

        # force monitor
        force_monitor_sim,
        force_monitor_real,

        control_position_node,

        # logical object (sim only)
        delayed_logical_pick_object,

        # motion nodes
        delayed_robot_nodes,
    ])