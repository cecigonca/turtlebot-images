#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, tty, termios
import os

# Configurando velocidade
MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.80
LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.get_logger().info( # Mensagem no terminal
            """
            Control Your Robot!
            ---------------------------
            Moving around:
                ↑
            ←       →
                ↓
            
            ↑ : Mover para frente
            ↓ : Mover para trás
            ← : Mover para esquerda
            → : Mover para direita
            Tecla 'Espaço' : zera as velocidades
            Tecla 'Enter' : encerra o programa
            """
        )

        self.timer = self.create_timer(0.1, self.update) # Atualização periódica

    def update(self):
        key = self.getKey() 
        if key:
            self.processKey(key)

    def getKey(self): 
        tty.setraw(sys.stdin.fileno()) # Leitura de tecla por tecla
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1) # Lê tecla pressionada
        if rlist:
            key = sys.stdin.read(1)
            print(f"Key pressed: {key} ({ord(key)})")  
            if key == '\x1b':  
                additional_chars = sys.stdin.read(2)  
                return key + additional_chars
            return key
        return ''

    def processKey(self, key): # Processa as teclas
        if key == '\x1b[A':  # Seta para cima
            self.current_linear_vel = min(self.current_linear_vel + LIN_VEL_STEP_SIZE, MAX_LIN_VEL)
        elif key == '\x1b[B':  # Seta para baixo
            self.current_linear_vel = max(self.current_linear_vel - LIN_VEL_STEP_SIZE, -MAX_LIN_VEL)
        elif key == '\x1b[D':  # Seta para esquerda
            self.current_angular_vel = min(self.current_angular_vel + ANG_VEL_STEP_SIZE, MAX_ANG_VEL)
            self.current_linear_vel = 0.0  # Zera a velocidade linear
        elif key == '\x1b[C':  # Seta para direita
            self.current_angular_vel = max(self.current_angular_vel - ANG_VEL_STEP_SIZE, -MAX_ANG_VEL)
            self.current_linear_vel = 0.0 # Zera a velocidade linear
        elif key == ' ':  # Tecla espaço
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
            self.get_logger().info("Velocities zeroed.")
        elif key == '\r' :  # Tecla Enter
            print("Enter key detected, exiting...")
            self.get_logger().info("Enter key pressed - shutting down.")
            rclpy.shutdown()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            os._exit(0)  # Encerra o programa 
        else:
            self.get_logger().info(f"Unrecognized key pressed: {key}")

        self.publishTwist()

    def publishTwist(self): # Mostra velocidade 
        twist = Twist()
        twist.linear.x = self.current_linear_vel
        twist.angular.z = self.current_angular_vel
        self.publisher.publish(twist)
        self.get_logger().info(f"Linear Vel: {self.current_linear_vel}, Angular Vel: {self.current_angular_vel}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
