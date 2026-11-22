"""LiDAR-only Cartographer SLAM + agv_gateway (masaustu uygulamasi) + tarayici izleyici + Foxglove.

Calistirmadan once ortam: source /opt/ros/humble/setup.bash && source /opt/agv_ws/install/setup.bash
  ws://agv.local:8765  -> masaustu kontrol uygulamasi (agv_gateway, INTERFACES.md §7)
  http://agv.local:8080 -> birlesik panel: canli harita + kamera (harita_izle.py)
  ws://agv.local:8766  -> Foxglove/Lichtblick (istege bagli)
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
MAP_DIR = '/maps'   # host'ta ~/maps (docker -v ~/maps:/maps)


def generate_launch_description():
    exe = 'rplidar_node' if os.path.exists('/opt/ros/humble/lib/rplidar_ros/rplidar_node') else 'rplidar_composition'
    return LaunchDescription([
        Node(package='rplidar_ros', executable=exe, name='rplidar', output='screen',
             respawn=True, respawn_delay=3.0,
             parameters=[{'serial_port': '/dev/ttyUSB0', 'serial_baudrate': 115200,
                          'frame_id': 'laser', 'angle_compensate': True,
                          'scan_mode': 'Standard'}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0', '--y', '0', '--z', '0.75', '--yaw', '0',
                        '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser']),
        # respawn: panelden "Haritayi sil" bu iki sureci durdurur, launch yeniden baslatir -> bos harita
        Node(package='cartographer_ros', executable='cartographer_node', output='screen',
             respawn=True, respawn_delay=2.0,
             arguments=['-configuration_directory', HERE,
                        '-configuration_basename', 'agv_lidar.lua']),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             respawn=True, respawn_delay=2.0,
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']),
        # Masaustu kontrol uygulamasi: ws://agv.local:8765
        # Cartographer TF agaci: map -> odom -> base_link -> laser (base_footprint yok -> base_frame=base_link).
        # Odometri/IMU/Nav2/mode_manager henuz yok: gateway bunlarsiz calisir (konum TF'den, mod UNKNOWN,
        # hedef gonderme "Nav2 hazir degil" der). "Haritayi kaydet" -> map_saver_cli -> /maps/<ad>.yaml/.pgm
        Node(package='agv_gateway', executable='gateway_node', name='gateway_node', output='screen',
             parameters=[{'port': 8765,  # host 0.0.0.0, token bos (sifresiz) = varsayilan
                          'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_link',
                          'map_dir': MAP_DIR,
                          'stations_file': os.path.join(MAP_DIR, 'stations.yaml'),
                          'telemetry_hz': 10.0, 'scan_hz': 5.0, 'map_period': 2.0}]),
        # Birlesik panel (harita + kamera): http://<Pi-IP>:8080  (kamera host'ta :8000)
        ExecuteProcess(cmd=['python3', os.path.join(HERE, 'harita_izle.py')], output='screen'),
        # Istege bagli: Foxglove/Lichtblick ile baglanmak icin ws://agv.local:8766
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             parameters=[{'port': 8766}]),
    ])
