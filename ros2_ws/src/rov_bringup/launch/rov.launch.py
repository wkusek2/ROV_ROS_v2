"""Caly stos ROV na Jetsonie:
  - micro_ros_agent (USB CDC do Pico 2 "fw")      -> topiki /rov/* z Pico (ESC, IMU, cisnienie)
  - micro_ros_agent_bms (USB CDC do Pico 2 "bms") -> /rov/bms/status, /rov/battery (CAN -> Daly BMS)
  - thruster_mixer (rov_control)                  -> /rov/cmd_vel + /rov/arm + /rov/gain -> /rov/thrusters
  - rosbridge_websocket (port 9090)               -> GUI w przegladarce
  - serwer http GUI (port 8088, katalog web/)     -> http://<ip-jetsona>:8088

Argumenty: dev:=/dev/... (port Pico), gui:=false (bez rosbridge/http), mixer:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    web_dir = os.path.join(get_package_share_directory("rov_bringup"), "web")
    mixer_cfg = os.path.join(get_package_share_directory("rov_control"), "config", "mixer.yaml")
    att_cfg = os.path.join(get_package_share_directory("rov_control"), "config", "attitude.yaml")
    gui = LaunchConfiguration("gui")
    return LaunchDescription([
        DeclareLaunchArgument(
            "dev",
            default_value="/dev/serial/by-id/usb-Raspberry_Pi_Pico_A320C46EADFA28CF-if00",
            description="port szeregowy Pico (USB CDC); stabilna nazwa z /dev/serial/by-id/",
        ),
        DeclareLaunchArgument(
            "bms_dev",
            default_value="/dev/serial/by-id/usb-Raspberry_Pi_Pico_1DBD3353E3ED6781-if00",
            description="port szeregowy Pico BMS (CAN do Daly)",
        ),
        DeclareLaunchArgument("bms", default_value="true"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("record", default_value="true", description="ciagle nagrywanie bagow do ~/nowy_rov/logs/bags"),
        DeclareLaunchArgument("agents", default_value="true", description="agenty micro-ROS (false na laptopie bez Pico)"),
        DeclareLaunchArgument("sim", default_value="false", description="symulator obu Pico (testy bez pojazdu)"),
        DeclareLaunchArgument("verbosity", default_value="4"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("mixer", default_value="true"),
        DeclareLaunchArgument("http_port", default_value="8088"),
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            output="screen",
            arguments=["serial", "--dev", LaunchConfiguration("dev"), "-b", "115200",
                       "-v", LaunchConfiguration("verbosity")],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration("agents")),
        ),
        Node(
            package="rov_control",
            executable="sim_pico",
            name="sim_pico",
            output="screen",
            condition=IfCondition(LaunchConfiguration("sim")),
        ),
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent_bms",
            output="screen",
            arguments=["serial", "--dev", LaunchConfiguration("bms_dev"), "-b", "115200",
                       "-v", LaunchConfiguration("verbosity")],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("bms"), "' == 'true' and '", LaunchConfiguration("agents"), "' == 'true'"])),
        ),
        Node(
            package="rov_control",
            executable="thruster_mixer",
            name="thruster_mixer",
            output="screen",
            parameters=[mixer_cfg],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration("mixer")),
        ),
        Node(
            package="rov_control",
            executable="attitude_controller",
            name="attitude_controller",
            output="screen",
            parameters=[att_cfg],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration("mixer")),
        ),
        Node(
            package="rov_camera",
            executable="camera_node",
            name="camera",
            output="screen",
            parameters=[{"sensor_id": 0, "sensor_mode": 1, "width": 1280, "height": 720, "fps": 30,
                         "quality": 60, "http_port": 8089, "ros_fps": 5.0, "vcm_bus": 10, "flip": 0}],
            respawn=True,
            respawn_delay=3.0,
            condition=IfCondition(LaunchConfiguration("camera")),
        ),
        Node(
            package="rov_control",
            executable="event_logger",
            name="event_logger",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            # serwisy w osobnym watku + timeout: inaczej wywolanie serwisu na odpietym Pico wiesza caly rosbridge
            parameters=[{"port": 9090, "unregister_timeout": 1.0,
                         "call_services_in_new_thread": True, "send_action_goals_in_new_thread": True,
                         "default_call_service_timeout": 5.0}],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(gui),
        ),
        Node(
            package="rosapi",
            executable="rosapi_node",
            name="rosapi",
            condition=IfCondition(gui),
        ),
        ExecuteProcess(
            cmd=["bash", os.path.join(get_package_share_directory("rov_bringup"), "scripts", "record_bags.sh")],
            name="bag_recorder",
            output="screen",
            respawn=True,
            respawn_delay=5.0,
            condition=IfCondition(LaunchConfiguration("record")),
        ),
        ExecuteProcess(
            cmd=["python3", "-m", "http.server", LaunchConfiguration("http_port"), "--bind", "0.0.0.0",
                 "--directory", web_dir],
            name="gui_http",
            output="screen",
            respawn=True,
            condition=IfCondition(gui),
        ),
    ])
