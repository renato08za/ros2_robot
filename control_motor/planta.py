#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
import RPi.GPIO as GPIO
import time
from threading import Lock, Thread

# Configurações dos pinos (BCM)
ENCA1, ENCB1 = 17, 18
ENCA2, ENCB2 = 20, 21
PWM1, IN1_1, IN2_1 = 13, 5, 6
PWM2, IN1_2, IN2_2 = 19, 16, 26
PPR = 374  # Pulsos por revolução do encoder

class SimpleDualMotorControl(Node):
    def __init__(self):
        super().__init__('simple_dual_motor_control')

        # Inicializa GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Configura encoder e motores
        GPIO.setup(ENCA1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCB1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(IN1_1, GPIO.OUT)
        GPIO.setup(IN2_1, GPIO.OUT)
        GPIO.setup(PWM1, GPIO.OUT)
        self.pwm1 = GPIO.PWM(PWM1, 1000)
        self.pwm1.start(0)

        GPIO.setup(ENCA2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCB2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(IN1_2, GPIO.OUT)
        GPIO.setup(IN2_2, GPIO.OUT)
        GPIO.setup(PWM2, GPIO.OUT)
        self.pwm2 = GPIO.PWM(PWM2, 1000)
        self.pwm2.start(0)

        # Estado interno
        self.dir1 = 1
        self.dir2 = 1
        self.encoder_pos_1 = 0
        self.encoder_pos_2 = 0
        self.last_encoder_1 = 0
        self.last_encoder_2 = 0
        self.lock = Lock()
        self.last_time = time.time()
        self.running = True

        # Threads de leitura de encoder
        Thread(target=self._poll_encoder1, daemon=True).start()
        Thread(target=self._poll_encoder2, daemon=True).start()

        # Timer para cálculo de RPM
        self.create_timer(0.1, self._rpm_calc_loop)

        # Subscriptions
        self.create_subscription(Int32, '/motor1_pwm_set', self._pwm1_set_callback, 10)
        self.create_subscription(Int32, '/motor2_pwm_set', self._pwm2_set_callback, 10)
        self.create_subscription(Int32, '/motor1_direction', self._motor1_dir_cb, 10)
        self.create_subscription(Int32, '/motor2_direction', self._motor2_dir_cb, 10)

        # Publishers de RPM
        self.rpm_pub_1 = self.create_publisher(Float32, '/motor1_rpm', 10)
        self.rpm_pub_2 = self.create_publisher(Float32, '/motor2_rpm', 10)

        self.get_logger().info('Controle simples de dois motores iniciado')

    def _pwm1_set_callback(self, msg: Int32):
        pwm = max(0, min(255, msg.data))
        GPIO.output(IN1_1, GPIO.HIGH if self.dir1 == 1 else GPIO.LOW)
        GPIO.output(IN2_1, GPIO.LOW if self.dir1 == 1 else GPIO.HIGH)
        duty = pwm * 100.0 / 255.0
        self.pwm1.ChangeDutyCycle(duty)

    def _pwm2_set_callback(self, msg: Int32):
        pwm = max(0, min(255, msg.data))
        GPIO.output(IN1_2, GPIO.HIGH if self.dir2 == 1 else GPIO.LOW)
        GPIO.output(IN2_2, GPIO.LOW if self.dir2 == 1 else GPIO.HIGH)
        duty = pwm * 100.0 / 255.0
        self.pwm2.ChangeDutyCycle(duty)

    def _motor1_dir_cb(self, msg: Int32):
        self.dir1 = 1 if msg.data >= 0 else -1
        self.pwm1.ChangeDutyCycle(0)
        self.get_logger().info(f"Motor 1 direção: {'frente' if self.dir1 == 1 else 'ré'}")

    def _motor2_dir_cb(self, msg: Int32):
        self.dir2 = 1 if msg.data >= 0 else -1
        self.pwm2.ChangeDutyCycle(0)
        self.get_logger().info(f"Motor 2 direção: {'frente' if self.dir2 == 1 else 'ré'}")

    def _poll_encoder1(self):
        last = GPIO.input(ENCA1)
        while self.running:
            cur = GPIO.input(ENCA1)
            b = GPIO.input(ENCB1)
            # somente conta quando há mudança de estado de ENCA1
            if cur != last and cur == 1:
                with self.lock:
                    if b:
                        self.encoder_pos_1 += self.dir1
                    else:
                        self.encoder_pos_1 -= self.dir1
            last = cur
            time.sleep(0.0002)

    def _poll_encoder2(self):
        last = GPIO.input(ENCA2)
        while self.running:
            cur = GPIO.input(ENCA2)
            b = GPIO.input(ENCB2)
            # somente conta quando há mudança de estado de ENCA2
            if cur != last and cur == 1:
                with self.lock:
                    if b:
                        self.encoder_pos_2 += self.dir2
                    else:
                        self.encoder_pos_2 -= self.dir2
            last = cur
            time.sleep(0.0002)

    def _rpm_calc_loop(self):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            return
        self.last_time = now
        with self.lock:
            delta1 = self.encoder_pos_1 - self.last_encoder_1
            delta2 = self.encoder_pos_2 - self.last_encoder_2
            self.last_encoder_1 = self.encoder_pos_1
            self.last_encoder_2 = self.encoder_pos_2
        rpm1 = (delta1 / PPR) / dt * 60.0
        rpm2 = (delta2 / PPR) / dt * 60.0
        self.rpm_pub_1.publish(Float32(data=rpm1))
        self.rpm_pub_2.publish(Float32(data=rpm2))
        print(f"RPM1: {rpm1:.1f} | RPM2: {rpm2:.1f}")

    def destroy_node(self):
        self.running = False
        time.sleep(0.1)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm1.stop()
        self.pwm2.ChangeDutyCycle(0)
        self.pwm2.stop()
        GPIO.output(IN1_1, GPIO.LOW)
        GPIO.output(IN2_1, GPIO.LOW)
        GPIO.output(IN1_2, GPIO.LOW)
        GPIO.output(IN2_2, GPIO.LOW)
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDualMotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
