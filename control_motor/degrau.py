#!/usr/bin/env python3
"""
Node to perform step response test on dual motor system.
Publishes a PWM step to both motors and logs RPM vs time to CSV.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
import csv
import time
import os

# Parameters for step test
STEP_PWM = 255        # PWM value for step (0-255)
DURATION = 5.0        # Duration of test in seconds
LOG_PATH = 'step_response.csv'  # CSV output file

class StepResponseNode(Node):
    def __init__(self):
        super().__init__('step_response')
        # Publishers: send step to both motors
        self.pub1 = self.create_publisher(Int32, '/motor1_pwm_set', 10)
        self.pub2 = self.create_publisher(Int32, '/motor2_pwm_set', 10)
        # Subscribers: record RPM
        self.rpm_data = []  # list of (timestamp, rpm1, rpm2)
        self.create_subscription(Float32, '/motor1_rpm', self.rpm1_cb, 10)
        self.create_subscription(Float32, '/motor2_rpm', self.rpm2_cb, 10)
        self.last_rpm1 = None
        self.last_rpm2 = None
        # Start test after short delay
        self.start_time = None
        self.create_timer(0.1, self._control_loop)
        self.get_logger().info('StepResponseNode initialized')

    def rpm1_cb(self, msg: Float32):
        self.last_rpm1 = msg.data

    def rpm2_cb(self, msg: Float32):
        self.last_rpm2 = msg.data

    def _control_loop(self):
        now = time.time()
        # on first call, record start and send step
        if self.start_time is None:
            self.start_time = now
            self.get_logger().info(f'Starting step: PWM={STEP_PWM} for {DURATION}s')
            # send PWM step
            self.pub1.publish(Int32(data=STEP_PWM))
            self.pub2.publish(Int32(data=STEP_PWM))
        elapsed = now - self.start_time
        # record data
        if self.last_rpm1 is not None and self.last_rpm2 is not None:
            self.rpm_data.append((elapsed, self.last_rpm1, self.last_rpm2))
        # if done, stop and flush
        if elapsed >= DURATION:
            # send zero PWM
            self.pub1.publish(Int32(data=0))
            self.pub2.publish(Int32(data=0))
            self.get_logger().info('Test complete, writing CSV')
            self._write_csv()
            rclpy.shutdown()

    def _write_csv(self):
        # ensure headers
        with open(LOG_PATH, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time_s', 'rpm1', 'rpm2'])
            writer.writerows(self.rpm_data)
        self.get_logger().info(f'CSV written to {os.path.abspath(LOG_PATH)}')


def main(args=None):
    rclpy.init(args=args)
    node = StepResponseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
