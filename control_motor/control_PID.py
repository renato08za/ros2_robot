#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import time
import numpy as np
from scipy.interpolate import interp1d

class DualPIDSyncController(Node):
    def __init__(self):
        super().__init__('dual_pid_sync_controller')

        # —––––––––––––––––––––––––––––––––––—
        # 1) Ganhos PID e de sincronização
        # —––––––––––––––––––––––––––––––––––—
        self.Kp1, self.Ki1, self.Kd1 = 1.0, 5.0, 0.1
        self.Kp2, self.Ki2, self.Kd2 = 1.0, 5.0, 0.1
        self.Ksync = 0.2       # quanto mais alto, mais força para igualar velocidades

        # —––––––––––––––––––––––––––––––––––—
        # 2) Estados internos
        # —––––––––––––––––––––––––––––––––––—
        self.err_int_1 = 0.0
        self.err_int_2 = 0.0
        self.err_ant_1 = 0.0
        self.err_ant_2 = 0.0
        self.last_time = time.time()

        # —––––––––––––––––––––––––––––––––––—
        # 3) Feed-forward (seu mapeamento PWM↔RPM)
        # —––––––––––––––––––––––––––––––––––—
        pwm_vals  = np.array([  0,  25,  50,  75, 100, 125, 150, 175, 200, 225, 255])
        rpm1_vals = np.array([  0,   0,   0,  25,  35,  45,  55,  60,  65,  70,  80])
        rpm2_vals = np.array([  0,   0,   0,   0,   0,  25,  30,  35,  40,  45,  50])
        self.ff1 = interp1d(rpm1_vals, pwm_vals, bounds_error=False, fill_value=(0,255))
        self.ff2 = interp1d(rpm2_vals, pwm_vals, bounds_error=False, fill_value=(0,255))

        # —––––––––––––––––––––––––––––––––––—
        # 4) Variáveis de domínio
        # —––––––––––––––––––––––––––––––––––—
        self.rpm1 = 0.0
        self.rpm2 = 0.0
        self.ref   = 0.0
        self.max_rpm = 60.0

        # —––––––––––––––––––––––––––––––––––—
        # 5) Publishers de direção e PWM
        # —––––––––––––––––––––––––––––––––––—
        self.pub_dir1  = self.create_publisher(Bool,    'motor1_direction',  10)
        self.pub_dir2  = self.create_publisher(Bool,    'motor2_direction',  10)
        self.pub_pwm1  = self.create_publisher(Float32, 'motor1_pwm_set',    10)
        self.pub_pwm2  = self.create_publisher(Float32, 'motor2_pwm_set',    10)

        # —––––––––––––––––––––––––––––––––––—
        # 6) Subscriptions: ref única e medições
        # —––––––––––––––––––––––––––––––––––—
        self.create_subscription(Float32, 'ref_rpm',      self.ref_cb,  10)
        self.create_subscription(Float32, 'motor1_rpm',   self.rpm1_cb, 10)
        self.create_subscription(Float32, 'motor2_rpm',   self.rpm2_cb, 10)

        # —––––––––––––––––––––––––––––––––––—
        # 7) Timer de controle (20 Hz)
        # —––––––––––––––––––––––––––––––––––—
        self.create_timer(1/20, self.control_loop)

    def ref_cb(self, msg: Float32):
        # set-point único, com clamp
        self.ref = min(msg.data, self.max_rpm)

    def rpm1_cb(self, msg: Float32):
        self.rpm1 = msg.data

    def rpm2_cb(self, msg: Float32):
        self.rpm2 = msg.data

    def control_loop(self):
        now = time.time()
        dt  = now - self.last_time
        self.last_time = now

        # ––––––––––––––––– Feed-forward
        pwm_ff1 = float(self.ff1(self.ref))
        pwm_ff2 = float(self.ff2(self.ref))

        # ––––––––––––––––– Erros individuais
        err1 = self.ref - self.rpm1
        err2 = self.ref - self.rpm2

        # ––––––––––––––––– Integral
        self.err_int_1 += err1 * dt
        self.err_int_2 += err2 * dt

        # ––––––––––––––––– Derivativo
        derr1 = (err1 - self.err_ant_1) / dt if dt>0 else 0.0
        derr2 = (err2 - self.err_ant_2) / dt if dt>0 else 0.0
        self.err_ant_1 = err1
        self.err_ant_2 = err2

        # ––––––––––––––––– Feedback PID
        u1 = self.Kp1*err1 + self.Ki1*self.err_int_1 + self.Kd1*derr1
        u2 = self.Kp2*err2 + self.Ki2*self.err_int_2 + self.Kd2*derr2

        # ––––––––––––––––– Sync coupling
        diff = self.rpm1 - self.rpm2
        u1 -= self.Ksync * diff
        u2 += self.Ksync * diff

        # ––––––––––––––––– PWM final e saturação
        pwm1 = max(0.0, min(255.0, pwm_ff1 + u1))
        pwm2 = max(0.0, min(255.0, pwm_ff2 + u2))

        # ––––––––––––––––– Publica direção (sempre pra frente)
        self.pub_dir1.publish(Bool(data=True))
        self.pub_dir2.publish(Bool(data=True))

        # ––––––––––––––––– Publica PWM
        self.pub_pwm1.publish(Float32(data=pwm1))
        self.pub_pwm2.publish(Float32(data=pwm2))

def main(args=None):
    rclpy.init(args=args)
    node = DualPIDSyncController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
