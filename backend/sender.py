import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32  # Adicionado para publicar a latência
import cv2
import time

class Imagem(Node):
    def __init__(self):
        super().__init__('transmissao_imagem')
        self.publisher_ = self.create_publisher(CompressedImage, '/video_frames', 10)
        self.latency_publisher_ = self.create_publisher(Float32, '/image_latency', 10)  # Publisher para latência
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.cap = cv2.VideoCapture(0)

    def timer_callback(self):
        start_time = time.time()  # Marcar o tempo de início
        ret, frame = self.cap.read()
        if ret:
            _, buffer = cv2.imencode('.jpg', frame)
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.publisher_.publish(msg)
            
            latency = (time.time() - start_time) * 1000  # Calcular a latência em milissegundos
            latency_msg = Float32()
            latency_msg.data = latency
            self.latency_publisher_.publish(latency_msg)
            self.get_logger().info(f'Sending image with latency: {latency:.2f} ms')

def main(args=None):
    rclpy.init(args=args)
    transmissao_imagem = Imagem()
    try:
        rclpy.spin(transmissao_imagem)
    except KeyboardInterrupt:
        pass
    finally:
        transmissao_imagem.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
