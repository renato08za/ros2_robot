#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
import csv
import time

class SpeedMapper(Node):
    def __init__(self):
        super().__init__('speed_mapper')

        # Publishers de PWM e direção
        self.pub_pwm1       = self.create_publisher(Int32,   '/motor1_pwm_set', 10)
        self.pub_dir1       = self.create_publisher(Int32,   '/motor1_direction', 10)

        # Subscriber de RPM
        self.rpm1           = None
        self.create_subscription(
            Float32, '/motor1_rpm', self._rpm1_cb, 10
        )

        # Parâmetros de varredura
        self.pwm_values     = list(range(0, 256, 25))  # ex: 0,25,50,...,250
        self.stabilize_time = 1.0  # segundos para estabilizar
        self.results        = []   # para guardar (pwm, rpm)

    def _rpm1_cb(self, msg: Float32):
        # sempre atualiza o rpm atual
        self.rpm1 = msg.data

    def run_mapping(self):
        # 1) fixe direção pra frente
        self.pub_dir1.publish(Int32(data=1))
        self.get_logger().info('Direção: frente')

        # 2) varre cada PWM
        for pwm in self.pwm_values:
            self.get_logger().info(f'Testando PWM = {pwm}')
            # publique o PWM
            self.pub_pwm1.publish(Int32(data=pwm))
            # espere estabilizar
            time.sleep(self.stabilize_time)

            # capture o rpm atual (pode ser None se não chegou dado)
            rpm = self.rpm1 if self.rpm1 is not None else float('nan')
            self.get_logger().info(f'-> RPM medido: {rpm:.1f}')
            # armazene
            self.results.append((pwm, rpm))

        # 3) ao final, pare o motor
        self.pub_pwm1.publish(Int32(data=0))

        # 4) salve em CSV
        with open('mapa_pwm_rpm.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['pwm','rpm'])
            writer.writerows(self.results)

        self.get_logger().info('Mapeamento concluído! Arquivo mapa_pwm_rpm.csv gerado.')

def main(args=None):
    rclpy.init(args=args)
    mapper = SpeedMapper()
    try:
        # executa a varredura
        mapper.run_mapping()
    finally:
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
