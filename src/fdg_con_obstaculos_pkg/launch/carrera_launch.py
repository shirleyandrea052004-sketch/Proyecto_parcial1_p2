from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    scaleOpp = 0.35   # Escala de velocidad de Opp con respecto al principal
    scaleOpp2 = 0.35  # Escala de velocidad de Opp2 con respecto al principal

    return LaunchDescription([
        # Auto 1 (Ego) - 100% de velocidad
        Node(
            package='fdg_con_obstaculos_pkg',
            executable='follow_the_gap',
            name='ftg_ego',
            output='screen',
            parameters=[{'speed_scale': 1.0}]
        ),

        # Auto 2 (Opp)
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
            parameters=[{'speed_scale': scaleOpp}]
        ),

        # Auto 3 (Opp2)
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
            parameters=[{'speed_scale': scaleOpp2}]
        )
    ])