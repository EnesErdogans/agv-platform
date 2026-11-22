"""agv_gateway launch. Örnek: ros2 launch agv_gateway gateway.launch.py token:=gizli"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory("agv_gateway"), "config", "gateway.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("port", default_value="8765"),
        DeclareLaunchArgument("token", default_value=""),
        Node(package="agv_gateway", executable="gateway_node", name="gateway_node", output="screen",
             parameters=[params, {"use_sim_time": LaunchConfiguration("use_sim_time"),
                                  "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                                  # boş string YAML'da None olur; tipi açıkça str yap
                                  "token": ParameterValue(LaunchConfiguration("token"), value_type=str)}]),
    ])
