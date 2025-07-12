#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import RPi.GPIO as GPIO
import time
from threading import Lock, Thread

# Pinos BCM dos encoders
ENCA1, ENCB1 = 17, 18
ENCA2, ENCB2 = 20, 21

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        self.pub = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENCA1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCB1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCA2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCB2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.pos1 = 0
        self.pos2 = 0
        self.lock = Lock()
        self.running = True

        Thread(target=self._poll, args=(ENCA1, ENCB1, 1), daemon=True).start()
        Thread(target=self._poll, args=(ENCA2, ENCB2, 2), daemon=True).start()

        self.create_timer(0.05, self._publish)  # 20 Hz

    def _poll(self, pin_a, pin_b, idx):
        last = GPIO.input(pin_a)
        while self.running:
            a = GPIO.input(pin_a); b = GPIO.input(pin_b)
            if a != last and a == 1:
                delta = 1 if b == 1 else -1
                with self.lock:
                    if idx == 1: self.pos1 += delta
                    else:         self.pos2 += delta
            last = a
            time.sleep(0.0002)

    def _publish(self):
        with self.lock:
            msg = Int32MultiArray()
            msg.data = [self.pos1, self.pos2]
        self.pub.publish(msg)

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
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
