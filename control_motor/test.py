#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import RPi.GPIO as GPIO
import time

# ======== Definição dos pinos (igual ao seu sketch) ========
ENCA = 20
ENCB = 21
PWM  = 19
IN1  = 16
IN2  = 26

# ======== Nó ROS2 ========
class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Estado interno (volatile no Arduino)
        self.pos_i      = 0
        self.velocity_i = 0.0
        self.prevT_i    = time.time()

        # Filtros e variáveis do PID
        self.vFilt      = 0.0
        self.vPrev      = 0.0
        self.e_integral = 0.0

        # --- Setup GPIO ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENCA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(PWM,  GPIO.OUT)
        GPIO.setup(IN1,  GPIO.OUT)
        GPIO.setup(IN2,  GPIO.OUT)

        self.pwm_out = GPIO.PWM(PWM, 1000)
        self.pwm_out.start(0)

        # Interrupção no canal A do encoder
        GPIO.add_event_detect(ENCA, GPIO.RISING, callback=self._encoder_cb, bouncetime=2)

        # --- Publishers e Subscribers ROS2 ---
        self.pub_vel = self.create_publisher(Float32, '/motor_velocity', 10)
        self.sub_dir = self.create_subscription(Int32,   '/motor_direction', self._dir_cb, 10)
        self.sub_pwm = self.create_subscription(Int32,   '/motor_pwm',       self._pwm_cb, 10)

        # Timer de controle (100 Hz)
        self.create_timer(0.01, self.control_loop)

        self.dir = 1
        self.get_logger().info('MotorControlNode iniciado')

    def _encoder_cb(self, channel):
        """ Equivalente ao readEncoder() do Arduino """
        b = GPIO.input(ENCB)
        now = time.time()
        deltaT = now - self.prevT_i if self.prevT_i else 1e-3
        self.prevT_i = now

        increment = 1 if b == GPIO.HIGH else -1
        self.pos_i      += increment
        self.velocity_i  = increment / deltaT

    def _dir_cb(self, msg: Int32):
        """ Atualiza sentido do motor """
        self.dir = 1 if msg.data >= 0 else -1

    def _pwm_cb(self, msg: Int32):
        """ Atualiza duty do PWM """
        val = max(0, min(255, msg.data))
        duty = val * 100.0 / 255.0
        self.pwm_out.ChangeDutyCycle(duty)

    def control_loop(self):
        """ Executa cálculo de velocidade, filtragem e PID """
        # 1) Método 1 de velocidade (derivada suja)
        now   = time.time()
        # usando self.prevT_i e self.pos_i você já tem velocity_i do encoder

        # 2) Converter para RPM
        v1 = self.velocity_i / 600.0 * 60.0

        # 3) Filtro passa-baixas (~25 Hz)
        alpha = 0.854
        beta  = (1 - alpha) / 2
        self.vFilt = alpha * self.vFilt + beta * v1 + beta * self.vPrev
        self.vPrev = v1

        # 4) Exemplo de referência senoidal
        vt = 100.0 * (1 if (now % 2) < 1 else 0)

        # 5) PID simples
        kp = 5.0; ki = 10.0
        e  = vt - self.vFilt
        self.e_integral += e * 0.01
        u  = kp * e + ki * self.e_integral

        # 6) Determina direção e intensidade
        dir_cmd = 1 if u >= 0 else -1
        pwr     = min(255, int(abs(u)))

        # 7) Aciona motor
        self.set_motor(dir_cmd, pwr)

        # 8) Publica velocidade filtrada
        self.pub_vel.publish(Float32(data=self.vFilt))

    def set_motor(self, dir, pwm_val):
        """ Igual ao setMotor do Arduino """
        # PWM
        duty = max(0, min(255, pwm_val)) * 100.0 / 255.0
        self.pwm_out.ChangeDutyCycle(duty)

        # Direção
        if dir == 1:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
        else:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)

    def destroy_node(self):
        """ Cleanup ao desligar """
        self.pwm_out.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
