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