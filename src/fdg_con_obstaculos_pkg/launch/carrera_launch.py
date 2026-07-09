from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Auto 1 (Ego) - 100% de velocidad
        Node(
            package='fdg_con_obstaculos_pkg',
            executable='follow_the_gap',
            name='ftg_ego',
            output='screen',
            parameters=[{'speed_scale': 1.0}]
        ),
        
        # Auto 2 (Opp) - 40% de velocidad
        Node(
            package='fdg_con_obstaculos_pkg',
            executable='follow_the_gap',
            name='ftg_opp',
            output='screen',
            remappings=[
                ('/scan', '/opp_scan'),
                ('/ego_racecar/odom', '/opp_racecar/odom'),
                ('/drive', '/opp_drive')
            ],
            parameters=[{'speed_scale': 0.4}]
        ),
        
        # Auto 3 (Opp2) - 40% de velocidad
        Node(
            package='fdg_con_obstaculos_pkg',
            executable='follow_the_gap',
            name='ftg_opp2',
            output='screen',
            remappings=[
                ('/scan', '/opp2_scan'),
                ('/ego_racecar/odom', '/opp2_racecar/odom'),
                ('/drive', '/opp2_drive')
            ],
            parameters=[{'speed_scale': 0.4}]
        )
    ])