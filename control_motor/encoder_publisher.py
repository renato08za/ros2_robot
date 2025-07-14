#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import RPi.GPIO as GPIO

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        # Encoder esquerdo
        self.LEFT_ENCA = 17
        self.LEFT_ENCB = 18
        # Encoder direito
        self.RIGHT_ENCA = 20
        self.RIGHT_ENCB = 21

        GPIO.setup(self.LEFT_ENCA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.LEFT_ENCB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.RIGHT_ENCA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.RIGHT_ENCB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Variáveis de contagem
        self.left_ticks = 0
        self.right_ticks = 0

        # Estados anteriores
        self.last_left_a = GPIO.input(self.LEFT_ENCA)
        self.last_right_a = GPIO.input(self.RIGHT_ENCA)

        # Timer de polling
        self.polling_rate = 0.001  # 1000 Hz
        self.create_timer(self.polling_rate, self.poll_encoders)

        # Timer de publicação
        self.publishing_rate = 0.05  # 20 Hz
        self.create_timer(self.publishing_rate, self.publish_encoder_ticks)

    def poll_encoders(self):
        # Encoder esquerdo
        current_left_a = GPIO.input(self.LEFT_ENCA)
        if current_left_a != self.last_left_a:
            if GPIO.input(self.LEFT_ENCB) != current_left_a:
                self.left_ticks += 1
            else:
                self.left_ticks -= 1
            self.last_left_a = current_left_a

        # Encoder direito
        current_right_a = GPIO.input(self.RIGHT_ENCA)
        if current_right_a != self.last_right_a:
            if GPIO.input(self.RIGHT_ENCB) != current_right_a:
                self.right_ticks += 1
            else:
                self.right_ticks -= 1
            self.last_right_a = current_right_a

    def publish_encoder_ticks(self):
        msg = Int32MultiArray()
        msg.data = [self.left_ticks, self.right_ticks]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
