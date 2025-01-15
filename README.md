
# Taller: Creación de un Paquete ROS2 con Múltiples Tortugas

Este tutorial te guiará en la creación de un paquete ROS2 que:
1. Lance tres tortugas en una misma ventana de turtlesim
2. Posicione las tortugas una al lado de la otra
3. Haga que cada tortuga dibuje una forma geométrica diferente

## 1. Crear el Workspace

Primero, creamos y configuramos nuestro workspace:

```bash
# Crear el directorio del workspace
mkdir -p ~/taller_multi_turtles/src
cd ~/taller_multi_turtles/src

# Crear el paquete
ros2 pkg create --build-type ament_python multi_turtles --dependencies rclpy launch_ros turtlesim
```

## 2. Configurar el Launch File

Crear el directorio para los archivos launch:
```bash
cd ~/taller_multi_turtles/src/multi_turtles
mkdir launch
```

Crear el archivo `launch/three_turtles.launch.py`:
```python
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
```

## 3. Crear el Nodo de Control de Movimiento

Crear el archivo `multi_turtles/turtle_movements.py`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from functools import partial

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        
        # Publishers para cada tortuga
        self.pub1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.pub3 = self.create_publisher(Twist, '/turtle3/cmd_vel', 10)
        
        # Variables para el control de movimiento
        self.square_side = 2.0  # Longitud del lado del cuadrado
        self.circle_radius = 1.0  # Radio del círculo
        self.triangle_side = 2.0  # Longitud del lado del triángulo
        
        # Timers para cada patrón
        self.square_state = 0
        self.square_timer = self.create_timer(2.0, self.move_square)
        
        self.circle_angle = 0.0
        self.circle_timer = self.create_timer(0.1, self.move_circle)
        
        self.triangle_state = 0
        self.triangle_timer = self.create_timer(2.0, self.move_triangle)

    def move_square(self):
        msg = Twist()
        if self.square_state % 2 == 0:  # Movimiento recto
            msg.linear.x = 1.0
            msg.angular.z = 0.0
        else:  # Giro
            msg.linear.x = 0.0
            msg.angular.z = math.pi/2  # 90 grados
        
        self.pub1.publish(msg)
        self.square_state = (self.square_state + 1) % 8

    def move_circle(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0  # Velocidad angular constante
        self.pub2.publish(msg)

    def move_triangle(self):
        msg = Twist()
        if self.triangle_state % 2 == 0:  # Movimiento recto
            msg.linear.x = 1.0
            msg.angular.z = 0.0
        else:  # Giro
            msg.linear.x = 0.0
            msg.angular.z = 2.0 * math.pi / 3  # 120 grados
        
        self.pub3.publish(msg)
        self.triangle_state = (self.triangle_state + 1) % 6

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. Configurar el setup.py

Modificar `setup.py` para incluir el launch file y el punto de entrada:
```python
from setuptools import setup
import os
from glob import glob

package_name = 'multi_turtles'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Multiple turtlesim nodes launcher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_movements = multi_turtles.turtle_movements:main'
        ],
    },
)
```

## 5. Hacer el Nodo Ejecutable

```bash
chmod +x ~/taller_multi_turtles/src/multi_turtles/multi_turtles/turtle_movements.py
```

## 6. Construir el Paquete

```bash
cd ~/taller_multi_turtles
colcon build --packages-select multi_turtles
source install/setup.bash
```

## 7. Ejecutar el Programa

En una terminal:
```bash
ros2 launch multi_turtles three_turtles.launch.py
```

En otra terminal:
```bash
source ~/taller_multi_turtles/install/setup.bash
ros2 run multi_turtles turtle_movements
```

## Resultados Esperados
- Se abrirá una ventana de turtlesim
- Aparecerán tres tortugas: una en el centro, una a la izquierda y otra a la derecha
- La tortuga central dibujará un cuadrado
- La tortuga de la izquierda dibujará un círculo
- La tortuga de la derecha dibujará un triángulo

## Solución de Problemas Comunes
1. Si el paquete no se encuentra, asegúrate de haber sourceado el setup.bash
2. Si los nodos no se ejecutan, verifica los permisos de los archivos
3. Si las tortugas no aparecen en las posiciones correctas, revisa las coordenadas en el launch file
4. Si las tortugas no se mueven, verifica que los topics de cmd_vel están correctamente configurados
