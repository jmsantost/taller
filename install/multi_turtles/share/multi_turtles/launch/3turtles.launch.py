from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

def generate_launch_description():
    # Crear un único nodo de turtlesim
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )
    
    # Comandos para crear las tortugas adicionales
    # turtle2 a la izquierda de turtle1
    spawn_turtle2 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
             '{x: 2.5, y: 5.5, theta: 0.0, name: "turtle2"}'],
        output='screen'
    )
    
    # turtle3 a la derecha de turtle1
    spawn_turtle3 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
             '{x: 8.5, y: 5.5, theta: 0.0, name: "turtle3"}'],
        output='screen'
    )

    # Añadir pequeños retrasos para asegurar que los servicios estén disponibles
    spawn_turtle2_timer = TimerAction(
        period=2.0,
        actions=[spawn_turtle2]
    )

    spawn_turtle3_timer = TimerAction(
        period=3.0,
        actions=[spawn_turtle3]
    )

    return LaunchDescription([
        turtlesim_node,
        spawn_turtle2_timer,
        spawn_turtle3_timer
    ])