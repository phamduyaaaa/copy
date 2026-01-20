# copy
```bash
"""
Copyright (c) 2026 Duy Pham
All rights reserved.

Author: Duy Pham
Contact: duypham.robotics@gmail.com

This software is proprietary and confidential.
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # =========================================================
    # 1. THIẾT LẬP ĐƯỜNG DẪN & CONFIG
    # =========================================================
    option = 'mecanum' # Tên folder config của em
    
    # Gói chứa file config .yaml (vẫn lấy từ gói bringup cũ hoặc gói mới tùy bạn copy chưa)
    # Nếu config vẫn nằm ở gói 'bringup' cũ thì giữ nguyên dòng dưới
    config_pkg_path = get_package_share_directory('bringup') 
    
    # Gói chứa node C++ mới
    cpp_pkg_name = 'bringup_cpp' 

    robot_desc_pkg_path = get_package_share_directory('robot_description')
    rplidar_pkg_path = get_package_share_directory('rplidar_ros')
    
    # Đường dẫn các file config
    ekf_config_path = os.path.join(config_pkg_path, 'config', option, 'ekf.yaml')
    ports_yaml_file = os.path.join(config_pkg_path, 'config', option, 'hardware_ports.yaml')
    odom_yaml_file  = os.path.join(config_pkg_path, 'config', option, 'odom.yaml')
    xacro_file_path = os.path.join(robot_desc_pkg_path, 'urdf', 'my_robot.urdf.xacro')

    # =========================================================
    # 2. ĐỌC DỮ LIỆU TỪ YAML
    # =========================================================
    # Đọc Hardware Ports
    with open(ports_yaml_file, 'r') as f:
        ports = yaml.safe_load(f)
        
    # Đọc Odom Params
    with open(odom_yaml_file, 'r') as f:
        odom_data = yaml.safe_load(f)
        odom_params = odom_data['odom'] # Lấy dictionary bên trong key 'odom'

    # Gán biến cho gọn
    stm32_port = ports['stm32']['port']
    stm32_baud = ports['stm32']['baudrate']
    
    rplidar_port = ports['rplidar']['port']
    rplidar_baud = ports['rplidar']['baudrate']

    # =========================================================
    # 3. NODE: ROBOT STATE PUBLISHER (URDF)
    # =========================================================
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_description_xml = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml}]
    )

    # =========================================================
    # 4. NODE: LIDAR (RPLIDAR)
    # =========================================================
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_pkg_path, 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': rplidar_port,
            'serial_baudrate': str(rplidar_baud),
            'frame_id': 'laser_frame',
            'angle_compensate': 'true',
            'scan_mode': 'Standard'
        }.items()
    )

    # =========================================================
    # 5. NODE: STM32 FULL BRIDGE (C++ Version)
    # =========================================================
    # Node này thay thế hoàn toàn stm32_bridge_full (Python)
    stm32_node_cpp = Node(
        package=cpp_pkg_name,        # Tên gói: bringup_cpp
        executable='stm32_node',     # Tên executable trong CMakeLists.txt
        name='stm32_driver_cpp',     # Tên node hiển thị trong rqt_graph
        output='screen',
        parameters=[
            # Tham số kết nối
            {'port': stm32_port},
            {'baudrate': stm32_baud},
            # Tham số vật lý (wheel_radius, lx, ly...) lấy từ yaml
            odom_params
        ]
    )

    # =========================================================
    # 6. NODE: EKF (Robot Localization)
    # =========================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', 'odom')]
    )

    # =========================================================
    # 7. KHỞI CHẠY
    # =========================================================
    return LaunchDescription([
        robot_state_publisher_node,
        rplidar_launch,
        stm32_node_cpp, # Chạy Node C++ mới
        ekf_node
    ])
```
