import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

import RPi.GPIO as GPIO
import time
from threading import Lock, Thread

# Configurações dos pinos
ENCA1, ENCB1 = 17, 18
ENCA2, ENCB2 = 20, 21
PWM1, IN1_1, IN2_1 = 13, 5, 6
PWM2, IN1_2, IN2_2 = 19, 16, 26
PPR = 374

class SimpleDualMotorControl(Node):
    def __init__(self):
        super().__init__('simple_dual_motor_control')

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Encoder e motores
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

        self.dir1 = 1
        self.dir2 = 1
        self.encoder_pos_1 = 0
        self.encoder_pos_2 = 0
        self.last_encoder_1 = 0
        self.last_encoder_2 = 0
        self.lock = Lock()

        self.last_time = time.time()
        self.running = True

        Thread(target=self._poll_encoder1, daemon=True).start()
        Thread(target=self._poll_encoder2, daemon=True).start()
        self.create_timer(0.1, self._rpm_calc_loop)

        # Recebe 0–255 para o PWM
        self.create_subscription(Int32, '/motor1_pwm_set', self._motor1_pwm_cb, 10)
        self.create_subscription(Int32, '/motor2_pwm_set', self._motor2_pwm_cb, 10)

        # Recebe 1 ou -1 para a direção
        self.create_subscription(Int32, '/motor1_direction', self._motor1_dir_cb, 10)
        self.create_subscription(Int32, '/motor2_direction', self._motor2_dir_cb, 10)

        # Publica RPM calculada a cada ciclo
        self.rpm_pub_1 = self.create_publisher(Float32, '/motor1_rpm', 10)
        self.rpm_pub_2 = self.create_publisher(Float32, '/motor2_rpm', 10)

        self.get_logger().info('Controle simples de dois motores iniciado')

    def _motor1_callback(self, msg):
        pwm = max(0, min(255, msg.data))
        if self.dir1 == 1:
            GPIO.output(IN1_1, GPIO.HIGH)
            GPIO.output(IN2_1, GPIO.LOW)
        else:
            GPIO.output(IN1_1, GPIO.LOW)
            GPIO.output(IN2_1, GPIO.HIGH)
        self.pwm1.ChangeDutyCycle(pwm * 100.0 / 255.0)

    def _motor2_callback(self, msg):
        pwm = max(0, min(255, msg.data))
        if self.dir2 == 1:
            GPIO.output(IN1_2, GPIO.HIGH)
            GPIO.output(IN2_2, GPIO.LOW)
        else:
            GPIO.output(IN1_2, GPIO.LOW)
            GPIO.output(IN2_2, GPIO.HIGH)
        self.pwm2.ChangeDutyCycle(pwm * 100.0 / 255.0)

    def _motor1_dir_callback(self, msg):
        self.dir1 = 1 if msg.data >= 0 else -1
        self.get_logger().info(f"Motor 1 direção: {'frente' if self.dir1 == 1 else 'ré'}")

    def _motor2_dir_callback(self, msg):
        self.dir2 = 1 if msg.data >= 0 else -1
        self.get_logger().info(f"Motor 2 direção: {'frente' if self.dir2 == 1 else 'ré'}")

        def _poll_encoder1(self):
        last = GPIO.input(ENCA1)
        while self.running:
            cur = GPIO.input(ENCA1)
            b = GPIO.input(ENCB1)
            # somente conta quando há mudança de estado de ENCA1
            if cur != last and cur == 1:
                with self.lock:
                    # incrementa ou decrementa baseada na leitura de ENCB1
                    if b:
                        # sinal B está alto: conta para frente
                        self.encoder_pos_1 += self.dir1
                    else:
                        # sinal B está baixo: conta para trás
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
                    # incrementa ou decrementa baseada na leitura de ENCB2
                    if b:
                        # sinal B está alto: conta para frente
                        self.encoder_pos_2 += self.dir2
                    else:
                        # sinal B está baixo: conta para trás
                        self.encoder_pos_2 -= self.dir2
            last = cur
            time.sleep(0.0002)

    def _rpm_calc_loop(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        with self.lock:
            delta1 = self.encoder_pos_1 - self.last_encoder_1
            self.last_encoder_1 = self.encoder_pos_1
            delta2 = self.encoder_pos_2 - self.last_encoder_2
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
      
if __name__ == '__main__':
    main()
