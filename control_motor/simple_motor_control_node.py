#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import RPi.GPIO as GPIO
import time

# Pinos de controle dos motores
PWM1, IN1_1, IN2_1 = 13, 5, 6
PWM2, IN1_2, IN2_2 = 19, 16, 26

class DualMotorControlNode(Node):
    def __init__(self):
        super().__init__('dual_motor_control_node')

        # Subscriptions
        self.create_subscription(Float32MultiArray, '/wheel_velocities', self.velocity_callback, 10)
        self.create_subscription(Int32MultiArray, 'encoder_ticks', self.encoder_callback, 10)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in [IN1_1, IN2_1, PWM1, IN1_2, IN2_2, PWM2]:
            GPIO.setup(pin, GPIO.OUT)

        self.pwm1 = GPIO.PWM(PWM1, 100)
        self.pwm2 = GPIO.PWM(PWM2, 100)
        self.pwm1.start(0)
        self.pwm2.start(0)

        # Estado inicial
        self.stop_motors()

        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = self.get_clock().now()

        self.desired_left_velocity = 0.0
        self.desired_right_velocity = 0.0

        # PID simplificado
        self.base_pwm = 7
        self.course_correction_factor = 0.8
        self.speed_correction_factor = 0.1
        self.target_tick_rate = 1
        self.tick_difference_offset = 0
        self.prev_movement_state = False

        # Timer de controle
        self.create_timer(0.1, self.control_loop)

    def stop_motors(self):
        for pin in [IN1_1, IN2_1, IN1_2, IN2_2]:
            GPIO.output(pin, GPIO.LOW)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)

    def velocity_callback(self, msg):
        new_left_velocity, new_right_velocity = msg.data
        current_movement_state = self.should_move_straight(new_left_velocity, new_right_velocity)
        if current_movement_state and not self.prev_movement_state:
            self.tick_difference_offset = self.left_ticks - self.right_ticks
            self.get_logger().info(f'Resetando offset de curso: {self.tick_difference_offset}')

        self.desired_left_velocity = new_left_velocity
        self.desired_right_velocity = new_right_velocity
        self.prev_movement_state = current_movement_state

    def encoder_callback(self, msg):
        self.left_ticks, self.right_ticks = msg.data

    def should_move_straight(self, left_velocity, right_velocity):
        return (left_velocity > 0 and right_velocity > 0 and abs(left_velocity - right_velocity) < 1e-6)

    def control_loop(self):
        if self.should_move_straight(self.desired_left_velocity, self.desired_right_velocity):
            self.move_straight_with_speed_control()
        else:
            self.set_motor_pwm(self.desired_left_velocity, self.desired_right_velocity)

    def move_straight_with_speed_control(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            return

        left_tick_rate = (self.left_ticks - self.prev_left_ticks) / dt
        right_tick_rate = (self.right_ticks - self.prev_right_ticks) / dt
        avg_tick_rate = (left_tick_rate + right_tick_rate) / 2
        tick_diff = (self.left_ticks - self.right_ticks) - self.tick_difference_offset
        speed_error = self.target_tick_rate - avg_tick_rate

        speed_correction = speed_error * self.speed_correction_factor
        course_correction = tick_diff * self.course_correction_factor

        pwm_left = max(0, min(100, self.base_pwm + speed_correction - course_correction))
        pwm_right = max(0, min(100, self.base_pwm + speed_correction + course_correction))

        self.set_motor_pwm(pwm_left, pwm_right)

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.prev_time = now

    def set_motor_pwm(self, pwm_left, pwm_right):
        # Motor esquerdo
        GPIO.output(IN1_1, GPIO.HIGH)
        GPIO.output(IN2_1, GPIO.LOW)
        self.pwm1.ChangeDutyCycle(min(abs(pwm_left), 100))

        # Motor direito
        GPIO.output(IN1_2, GPIO.HIGH)
        GPIO.output(IN2_2, GPIO.LOW)
        self.pwm2.ChangeDutyCycle(min(abs(pwm_right), 100))

    def destroy_node(self):
        self.stop_motors()
        self.pwm1.stop()
        self.pwm2.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualMotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
