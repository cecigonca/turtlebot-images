import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, tty, termios
import os
from std_srvs.srv import Empty

# Configurando velocidade
MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.80
LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.velocity = self.create_subscription(Twist,'/vel', self.callback_vel,10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.service = self.create_service(Empty, '/kill_teleop_node', self.kill_node_callback)
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.alive = True
        
    def callback_vel(self, msg):
        self.publishTwist(msg)


    def kill_node_callback(self, request, response):
        self.get_logger().info('Kill node service called. Shutting down...')
        self.alive = False
        return response

    def publishTwist(self,twist): # Mostra velocidade 
        if self.alive: 
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