# copy
```bash
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Định nghĩa tên package của bạn
    package_name = 'navigation2_config'
    
    # 2. Lấy đường dẫn đến các thư mục cần thiết
    pkg_share = get_package_share_directory(package_name)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 3. Định nghĩa đường dẫn mặc định tới file Map, Param và RViz dựa trên cấu trúc tree của bạn
    # Đường dẫn: src/navigation2_config/map/map.yaml
    default_map_path = os.path.join(pkg_share, 'map', 'map.yaml')
    
    # Đường dẫn: src/navigation2_config/param/nav2_params_custom.yaml
    default_params_file_path = os.path.join(pkg_share, 'param', 'nav2_params_custom.yaml')
    
    # Đường dẫn: src/navigation2_config/rviz/navigation2_config.rviz
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'navigation2_config.rviz')

    # 4. Khởi tạo các LaunchConfiguration
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')

    # 5. Khai báo các tham số launch (Arguments)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup')

    # 6. Gọi launch file chính của Nav2 (bringup_launch.py)
    # File này sẽ tự động chạy amcl, map_server, controller, planner, behavior_server, v.v.
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': 'False'
        }.items()
    )

    # 7. Chạy RViz2 với file config của bạn
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        nav2_bringup_launch,
        rviz_node
    ])
```
