from glob import glob
from setuptools import setup

package_name = 'ur5_sim_gz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    # Install shared files (ROS2)
    data_files=[
        # Register package in ament index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Install package.xml
        ('share/' + package_name,
         ['package.xml']),

        # Install all launch files
        ('share/' + package_name + '/launch',
         glob('launch/*.launch.py')),

        #Install config
        ('share/' + package_name + '/config',[
            'config/initial_positions.yaml',
            'config/ur5e_executor_params.yaml',
        ]),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    # Package metadata
    maintainer='thawatchai',
    maintainer_email='example@example.com',
    description='UR5 simulation and control tools',
    license='Apache License 2.0',
    tests_require=['pytest'],

    # ROS2 Console Scripts (Nodes)
    entry_points={
        'console_scripts': [
            # Speech & Voice
            'speech_to_text_node = ur5_sim_gz.speech_to_text_node:main',
            'tts_node_gtts = ur5_sim_gz.tts_node_gtts:main',
            'speech_gui_node = ur5_sim_gz.speech_gui_node:main',
            'nlu_parser_node = ur5_sim_gz.nlu_parser_node:main',
            'beep_node = ur5_sim_gz.beep_node:main',
            'voice_logger_node = ur5_sim_gz.voice_logger_node:main',
            'dialog_fsm_node = ur5_sim_gz.dialog_fsm_node:main',
            
            'audio_monitor_gui = ur5_sim_gz.audio_monitor_gui:main',
            

            # UR5 Control
            'ur5_cmd_mapper_node = ur5_sim_gz.ur5_cmd_mapper_node:main',
            'control_position_node = ur5_sim_gz.control_position_node:main',

            'ur5_executor_node = ur5_sim_gz.ur5_executor_node:main',
            'gripper_bridge_node = ur5_sim_gz.gripper_bridge_node:main',
            'audio_receiver_node = ur5_sim_gz.audio_receiver_node:main',

        ],
    },
)
