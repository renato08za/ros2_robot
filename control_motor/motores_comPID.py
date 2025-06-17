#!/usr/bin/env python3
"""
ROS2 node que implementa PID de velocidade para ambos motores
- Assina /motorX_rpm (Float32)
- Assina /desired_rpmX (Float32)
- Publica /motorX_pwm_set (Int32)
- Publica /motorX_direction (Int32)
Agora inclui feedforward de segunda ordem usando o modelo:
    G(s) = K·ωn² / (s² + 2ζωn·s + ωn²)
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32, Int32
import time

class DualMotorSecondOrderPid(Node):
    def __init__(self):
        super().__init__('dual_motor_second_order_pid')
        # declara parâmetros (PID + modelo 2ª ordem)
        for i in (1, 2):
            for p, default in [('kp', 0.5), ('ki', 0.1), ('kd', 0.0)]:
                self.declare_parameter(f'{p}{i}', default)
            # planta 2ª ordem identificada
            self.declare_parameter(f'K{i}', 1.0)
            self.declare_parameter(f'wn{i}', 1.0)
            self.declare_parameter(f'zeta{i}', 0.7)
        # estado interno
        self.setpoints = [0.0, 0.0]
        self.measured = [0.0, 0.0]
        self.integrals = [0.0, 0.0]
        self.prev_errors = [0.0, 0.0]
        self.d_filters = [0.0, 0.0]
        self.ff_states = [[0.0, 0.0], [0.0, 0.0]]  # histórico de r
        self.last_time = time.time()

        # publishers / subscribers
        self.pubs_pwm = []
        self.pubs_dir = []
        for i in (1, 2):
            self.pubs_pwm.append(self.create_publisher(Int32, f'/motor{i}_pwm_set', 10))
            self.pubs_dir.append(self.create_publisher(Int32, f'/motor{i}_direction', 10))
            self.create_subscription(Float32, f'/motor{i}_rpm', lambda m, idx=i-1: self._rpm_cb(idx, m), 10)
            self.create_subscription(Float32, f'/desired_rpm{i}', lambda m, idx=i-1: self._setpoint_cb(idx, m), 10)
        # controle a 50Hz
        self.create_timer(0.02, self._control_loop)
        self.get_logger().info('DualMotorSecondOrderPid iniciado')

    def _rpm_cb(self, idx, msg):
        self.measured[idx] = msg.data

    def _setpoint_cb(self, idx, msg):
        self.setpoints[idx] = msg.data
        # limpa integrador e estados de feedforward
        self.integrals[idx] = 0.0
        self.prev_errors[idx] = 0.0
        self.d_filters[idx] = 0.0
        self.ff_states[idx] = [0.0, 0.0]

    def _control_loop(self):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            return
        self.last_time = now
        for i in (0, 1):
            # lê parâmetros
            kp = self.get_parameter(f'kp{i+1}').value
            ki = self.get_parameter(f'ki{i+1}').value
            kd = self.get_parameter(f'kd{i+1}').value
            K  = self.get_parameter(f'K{i+1}').value
            wn = self.get_parameter(f'wn{i+1}').value
            zeta = self.get_parameter(f'zeta{i+1}').value
            # erro
            e = self.setpoints[i] - self.measured[i]
            # PID
            self.integrals[i] += e * dt
            de = (e - self.prev_errors[i]) / dt
            self.prev_errors[i] = e
            # filtro derivativo
            alpha = 1.0 / (1.0 + 1.0 / (zeta * wn * dt))
            self.d_filters[i] = alpha * self.d_filters[i] + (1 - alpha) * de
            u_pid = kp * e + ki * self.integrals[i] + kd * self.d_filters[i]
            # feedforward 2ª ordem (inverso Tustin)
            T = dt; w = wn
            A0 = 4 + 4*zeta*w*T + w*w*T*T
            b1 = (2*w*w*T*T - 8) / A0
            b2 = (4 - 4*zeta*w*T + w*w*T*T) / A0
            a0 = A0 / (w*w*T*T)
            a1 = -2 * (4 - w*w*T*T) / (w*w*T*T)
            a2 = (4 - 4*zeta*w*T + w*w*T*T) / (w*w*T*T)
            r0 = self.setpoints[i]
            r1, r2 = self.ff_states[i]
            u_ff = a0 * r0 + a1 * r1 + a2 * r2 - b1 * self.ff_states[i][0] - b2 * self.ff_states[i][1]
            # atualiza histórico
            self.ff_states[i] = [r0, r1]
            # sinal total
            u = u_pid + (u_ff / K)
            direction = 1 if u >= 0 else -1
            pwm = int(min(abs(u), 255))
            self.pubs_dir[i].publish(Int32(data=direction))
            self.pubs_pwm[i].publish(Int32(data=pwm))

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualMotorSecondOrderPid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
