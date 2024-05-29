import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class Imagem(Node):
    def __init__(self):
        super().__init__('transmissao_imagem')
        self.publisher_ = self.create_publisher(CompressedImage, '/video_frames', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.cap = cv2.VideoCapture(0)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            print("Sending img")
            _, buffer = cv2.imencode('.jpg', frame)
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    transmissao_imagem = Imagem()
    try:
        rclpy.spin(transmissao_imagem)
        print("spinning img")
    except KeyboardInterrupt:
        pass
    finally:
        transmissao_imagem.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()