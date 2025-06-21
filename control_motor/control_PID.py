#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import numpy as np
from scipy.interpolate import interp1d

class DualPIController(Node):
    def __init__(self):
        super().__init__('dual_pi_controller')

        # — Ganhos do PI (iguais ao de antes)
        self.kp1, self.ki1 = 0.8, 4.5
        self.kp2, self.ki2 = 0.5, 3.0

        # — teto de RPM
        self.max_rpm = 60.0

        # — referências iniciais (podem começar em zero)
        self.ref1 = 0.0
        self.ref2 = 0.0

        # — estados internos
        self.err_int_1 = 0.0
        self.err_int_2 = 0.0
        self.t_prev_1  = time.time()
        self.t_prev_2  = time.time()

        # — feed-forward (igual ao seu mapeamento)
        pwm_vals  = np.array([  0,  25,  50,  75, 100, 125, 150, 175, 200, 225, 255])
        rpm1_vals = np.array([  0,   0,   0,  25,  35,  45,  55,  60,  65,  70,  80])
        rpm2_vals = np.array([  0,   0,   0,   0,   0,  25,  30,  35,  40,  45,  50])
        self.ff1 = interp1d(rpm1_vals, pwm_vals, bounds_error=False,
                            fill_value=(pwm_vals[0], pwm_vals[-1]))
        self.ff2 = interp1d(rpm2_vals, pwm_vals, bounds_error=False,
                            fill_value=(pwm_vals[0], pwm_vals[-1]))

        # Subscrições aos tópicos de RPM vindos do seu encoder
        self.sub1 = self.create_subscription(Float32, 'motor1_rpm', self.cb1, 10)
        self.sub2 = self.create_subscription(Float32, 'motor2_rpm', self.cb2, 10)

        # Publicadores para os tópicos que o driver espera  
        self.pub1 = self.create_publisher(Float32, 'motor1_pwm_set', 10)
        self.pub2 = self.create_publisher(Float32, 'motor2_pwm_set', 10)

        # — tópicos de medição e saída PWM
        self.sub1 = self.create_subscription(Float32, 'rpm1', self.cb1, 10)
        self.sub2 = self.create_subscription(Float32, 'rpm2', self.cb2, 10)
        self.pub1 = self.create_publisher(Float32, 'pwm1', 10)
        self.pub2 = self.create_publisher(Float32, 'pwm2', 10)

    def ref1_callback(self, msg: Float32):
        # aqui você define dinamicamente a referência, já respeitando o teto
        self.ref1 = min(msg.data, self.max_rpm)

    def ref2_callback(self, msg: Float32):
        self.ref2 = min(msg.data, self.max_rpm)

    def cb1(self, msg: Float32):
        now = time.time()
        dt  = now - self.t_prev_1; self.t_prev_1 = now

        rpm = msg.data
        pwm_ff = float(self.ff1(self.ref1))

        err      = self.ref1 - rpm
        self.err_int_1 += err * dt
        u_fb     = self.kp1 * err + self.ki1 * self.err_int_1

        pwm = pwm_ff + u_fb
        pwm = max(0.0, min(255.0, pwm))

        out = Float32(); out.data = pwm
        self.pub1.publish(out)

    def cb2(self, msg: Float32):
        now = time.time()
        dt  = now - self.t_prev_2; self.t_prev_2 = now

        rpm = msg.data
        pwm_ff = float(self.ff2(self.ref2))

        err      = self.ref2 - rpm
        self.err_int_2 += err * dt
        u_fb     = self.kp2 * err + self.ki2 * self.err_int_2

        pwm = pwm_ff + u_fb
        pwm = max(0.0, min(255.0, pwm))

        out = Float32(); out.data = pwm
        self.pub2.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = DualPIController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
