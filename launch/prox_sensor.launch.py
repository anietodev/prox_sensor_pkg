from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Lanzar CoppeliaSim con la escena del almacén
        ExecuteProcess(
            cmd=[
                '~/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04/coppeliaSim.sh',
                '~/RM_prac/src/prox_sensor_pkg/coppelia/almacen.ttt'
            ],
            shell=True,
            output='screen'
        ),

        # Nodo de control de velocidad con sensores de proximidad
        Node(
            package='prox_sensor_pkg',
            executable='prox_sensor',
            name='prox_sensor_node',
            output='screen'
        )
    ])

