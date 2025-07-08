#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import RPi.GPIO as GPIO
import time
from threading import Lock, Thread

# Configuração dos pinos
ENCA1, ENCB1 = 17, 18
ENCA2, ENCB2 = 20, 21

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENCA1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCB1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCA2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCB2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Inicializa contadores
        self.encoder_pos_1 = 0
        self.encoder_pos_2 = 0
        self.lock = Lock()

        # Inicia threads para polling
        self.running = True
        Thread(target=self.poll_encoder1, daemon=True).start()
        Thread(target=self.poll_encoder2, daemon=True).start()

        # Timer para publicação
        self.create_timer(0.05, self.publish_ticks)  # 20 Hz

    def poll_encoder1(self):
        last_a = GPIO.input(ENCA1)
        while self.running:
            a = GPIO.input(ENCA1)
            b = GPIO.input(ENCB1)
            if a != last_a and a == 1:
                with self.lock:
                    self.encoder_pos_1 += 1 if b == 1 else -1
            last_a = a
            time.sleep(0.0002)

    def poll_encoder2(self):
        last_a = GPIO.input(ENCA2)
        while self.running:
            a = GPIO.input(ENCA2)
            b = GPIO.input(ENCB2)
            if a != last_a and a == 1:
                with self.lock:
                    self.encoder_pos_2 += 1 if b == 1 else -1
            last_a = a
            time.sleep(0.0002)

    def publish_ticks(self):
        with self.lock:
            msg = Int32MultiArray()
            msg.data = [self.encoder_pos_1, self.encoder_pos_2]
            self.publisher_.publish(msg)

    def destroy_node(self):
        self.running = False
        time.sleep(0.1)
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
