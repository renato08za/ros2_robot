#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, Int32

import RPi.GPIO as GPIO
import time
from threading import Lock, Thread

ENCA1, ENCB1, PWM1, IN1_1, IN2_1 = 17, 18, 13, 5, 6
ENCA2, ENCB2, PWM2, IN1_2, IN2_2 = 20, 21, 19, 16, 26
PPR = 374


class DualMotorTwoPIDNode(Node):
    def __init__(self):
        super().__init__('dual_motor_two_pid_node')

        # 1) parâmetros PID e filtro
        self.declare_parameter('kp1', 0.5)
        self.declare_parameter('ki1', 0.05)
        self.declare_parameter('kd1', 0.0)
        self.declare_parameter('kp2', 0.5)
        self.declare_parameter('ki2', 0.06)
        self.declare_parameter('kd2', 0.0)
        self.declare_parameter('tsample', 0.05)
        self.declare_parameter('tau', 0.1)
        self.declare_parameter('use_interrupts', True)

        # 2) lê valores iniciais
        self.kp1 = self.get_parameter('kp1').value
        self.ki1 = self.get_parameter('ki1').value
        self.kd1 = self.get_parameter('kd1').value
        self.kp2 = self.get_parameter('kp2').value
        self.ki2 = self.get_parameter('ki2').value
        self.kd2 = self.get_parameter('kd2').value
        self.tsample = self.get_parameter('tsample').value
        self.tau = self.get_parameter('tau').value
        self.use_interrupts = self.get_parameter('use_interrupts').value

        self.add_on_set_parameters_callback(self._param_change_callback)

        # estado
        self.encoder_pos_1 = 0
        self.last_encoder_1 = 0
        self.prev_rpm_1 = 0.0
        self.desired_rpm_1 = 0.0
        self.integral_1 = 0.0
        self.prev_error_1 = 0.0

        self.encoder_pos_2 = 0
        self.last_encoder_2 = 0
        self.prev_rpm_2 = 0.0
        self.desired_rpm_2 = 0.0
        self.integral_2 = 0.0
        self.prev_error_2 = 0.0
        self.dir1 = 1
        self.dir2 = 1

        self.last_time = time.time()
        self.lock = Lock()
        self.running = True

        # inicializa GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # motor 1
        GPIO.setup(ENCA1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCB1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(IN1_1, GPIO.OUT)
        GPIO.setup(IN2_1, GPIO.OUT)
        GPIO.setup(PWM1, GPIO.OUT)
        self.pwm1 = GPIO.PWM(PWM1, 1000)
        self.pwm1.start(0)

        # motor 2
        GPIO.setup(ENCA2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCB2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(IN1_2, GPIO.OUT)
        GPIO.setup(IN2_2, GPIO.OUT)
        GPIO.setup(PWM2, GPIO.OUT)
        self.pwm2 = GPIO.PWM(PWM2, 1000)
        self.pwm2.start(0)

        # interrupção ou polling
        if self.use_interrupts:
            try:
                GPIO.remove_event_detect(ENCA1)
                GPIO.add_event_detect(ENCA1, GPIO.RISING,
                                      callback=self._encoder1_callback,
                                      bouncetime=2)
            except Exception:
                self.use_interrupts = False
            try:
                GPIO.remove_event_detect(ENCA2)
                GPIO.add_event_detect(ENCA2, GPIO.RISING,
                                      callback=self._encoder2_callback,
                                      bouncetime=2)
            except Exception:
                self.use_interrupts = False

        if not self.use_interrupts:
            Thread(target=self._poll_encoder1, daemon=True).start()
            Thread(target=self._poll_encoder2, daemon=True).start()

        # publishers / subscribers
        self.rpm_pub_1 = self.create_publisher(Float32, '/motor1_rpm', 10)
        self.rpm_pub_2 = self.create_publisher(Float32, '/motor2_rpm', 10)
        self.create_subscription(Int32, '/motor1_pwm_set',
                                 self._pwm1_set_callback, 10)
        self.create_subscription(Int32, '/motor2_pwm_set',
                                 self._pwm2_set_callback, 10)

        self.create_timer(self.tsample, self._control_loop)
        self.get_logger().info('DualMotorTwoPIDNode iniciado')

    def _param_change_callback(self, params):
        ok = True
        for p in params:
            if p.name == 'kp1' and p.type_ == Parameter.Type.DOUBLE:
                self.kp1 = p.value
            elif p.name == 'ki1' and p.type_ == Parameter.Type.DOUBLE:
                self.ki1 = p.value
            elif p.name == 'kd1' and p.type_ == Parameter.Type.DOUBLE:
                self.kd1 = p.value
            elif p.name == 'kp2' and p.type_ == Parameter.Type.DOUBLE:
                self.kp2 = p.value
            elif p.name == 'ki2' and p.type_ == Parameter.Type.DOUBLE:
                self.ki2 = p.value
            elif p.name == 'kd2' and p.type_ == Parameter.Type.DOUBLE:
                self.kd2 = p.value
            elif p.name == 'tsample' and p.type_ == Parameter.Type.DOUBLE:
                self.tsample = p.value
            elif p.name == 'tau' and p.type_ == Parameter.Type.DOUBLE:
                self.tau = p.value
            else:
                ok = False
        return SetParametersResult(successful=ok)


    def _pwm1_set_callback(self, msg):
        v = max(-255, min(255, msg.data))
        new_dir = 1 if v >= 0 else -1
        if new_dir != self.dir1:
            self.integral_1 = 0.0
            self.prev_error_1 = 0.0
            self.prev_rpm_1 = 0.0
        self.desired_rpm_1 = (abs(v) / 255.0) * 60.0
        self.dir1 = new_dir
        self.get_logger().info(f"Motor1 PWM={v} → RPM alvo={self.desired_rpm_1:.1f} dir={self.dir1}")

    def _pwm2_set_callback(self, msg):
        v = max(-255, min(255, msg.data))
        new_dir = 1 if v >= 0 else -1
        if new_dir != self.dir2:
            self.integral_2 = 0.0
            self.prev_error_2 = 0.0
            self.prev_rpm_2 = 0.0
        self.desired_rpm_2 = (abs(v) / 255.0) * 60.0
        self.dir2 = new_dir
        self.get_logger().info(f"Motor2 PWM={v} → RPM alvo={self.desired_rpm_2:.1f} dir={self.dir2}")

    def _encoder1_callback(self, ch):
        b = GPIO.input(ENCB1)
        with self.lock:
            delta = 1 if b else -1
            self.encoder_pos_1 += delta * self.dir1

    def _encoder2_callback(self, ch):
        b = GPIO.input(ENCB2)
        with self.lock:
            delta = 1 if b else -1
            self.encoder_pos_2 += delta * self.dir2

    def _poll_encoder1(self):
        last = GPIO.input(ENCA1)
        while self.running:
            cur = GPIO.input(ENCA1)
            b = GPIO.input(ENCB1)
            if cur != last and cur == 1:
                with self.lock:
                    delta = 1 if b else -1
                    self.encoder_pos_1 += delta * self.dir1
            last = cur
            time.sleep(0.0002)

    def _poll_encoder2(self):
        last = GPIO.input(ENCA2)
        while self.running:
            cur = GPIO.input(ENCA2)
            b = GPIO.input(ENCB2)
            if cur != last and cur == 1:
                with self.lock:
                    delta = 1 if b else -1
                    self.encoder_pos_2 += delta * self.dir2
            last = cur
            time.sleep(0.0002)

    def _control_loop(self):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            return
        alpha = self.tau / (self.tau + self.tsample)

        # Motor 1
        with self.lock:
            d1 = self.encoder_pos_1 - self.last_encoder_1
            self.last_encoder_1 = self.encoder_pos_1
        rpm1_inst = (d1 / PPR) / dt * 60.0
        rpm1 = alpha * self.prev_rpm_1 + (1 - alpha) * rpm1_inst
        self.prev_rpm_1 = rpm1

        err1 = self.desired_rpm_1 - rpm1
        self.integral_1 += err1 * dt
        der1 = (err1 - self.prev_error_1) / dt
        self.prev_error_1 = err1
        out1 = self.kp1 * err1 + self.ki1 * self.integral_1 + self.kd1 * der1

        if self.desired_rpm_1 == 0.0:
            duty1 = 0.0
            GPIO.output(IN1_1, GPIO.LOW)
            GPIO.output(IN2_1, GPIO.LOW)
        else:
            duty1 = max(0.0, min(100.0, abs(out1)))
            if self.dir1 > 0:
                GPIO.output(IN1_1, GPIO.HIGH)
                GPIO.output(IN2_1, GPIO.LOW)
            else:
                GPIO.output(IN1_1, GPIO.LOW)
                GPIO.output(IN2_1, GPIO.HIGH)
        self.pwm1.ChangeDutyCycle(duty1)
        self.rpm_pub_1.publish(Float32(data=rpm1))

        # Motor 2
        with self.lock:
            d2 = self.encoder_pos_2 - self.last_encoder_2
            self.last_encoder_2 = self.encoder_pos_2
        rpm2_inst = (d2 / PPR) / dt * 60.0
        rpm2 = alpha * self.prev_rpm_2 + (1 - alpha) * rpm2_inst
        self.prev_rpm_2 = rpm2

        err2 = self.desired_rpm_2 - rpm2
        self.integral_2 += err2 * dt
        der2 = (err2 - self.prev_error_2) / dt
        self.prev_error_2 = err2
        out2 = self.kp2 * err2 + self.ki2 * self.integral_2 + self.kd2 * der2

        if self.desired_rpm_2 == 0.0:
            duty2 = 0.0
            GPIO.output(IN1_2, GPIO.LOW)
            GPIO.output(IN2_2, GPIO.LOW)
        else:
            duty2 = max(0.0, min(100.0, abs(out2)))
            if self.dir2 > 0:
                GPIO.output(IN1_2, GPIO.HIGH)
                GPIO.output(IN2_2, GPIO.LOW)
            else:
                GPIO.output(IN1_2, GPIO.LOW)
                GPIO.output(IN2_2, GPIO.HIGH)
        self.pwm2.ChangeDutyCycle(duty2)
        self.rpm_pub_2.publish(Float32(data=rpm2))

        # Print de debug
        print(f"M1 → RPM={rpm1:6.1f} | Set={self.desired_rpm_1:6.1f} | Duty={duty1:5.1f}%  ||  "
              f"M2 → RPM={rpm2:6.1f} | Set={self.desired_rpm_2:6.1f} | Duty={duty2:5.1f}%")

        self.last_time = now

    def _control_loop(self):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            return
        alpha = self.tau / (self.tau + self.tsample)

        # Motor 1
        with self.lock:
            d1 = self.encoder_pos_1 - self.last_encoder_1
            self.last_encoder_1 = self.encoder_pos_1
        rpm1_inst = (d1 / PPR) / dt * 60.0
        rpm1 = alpha * self.prev_rpm_1 + (1 - alpha) * rpm1_inst
        self.prev_rpm_1 = rpm1

        err1 = self.desired_rpm_1 - rpm1
        self.integral_1 += err1 * dt
        der1 = (err1 - self.prev_error_1) / dt
        self.prev_error_1 = err1
        out1 = self.kp1 * err1 + self.ki1 * self.integral_1 + self.kd1 * der1

        if self.desired_rpm_1 == 0.0:
            duty1 = 0.0
            GPIO.output(IN1_1, GPIO.LOW)
            GPIO.output(IN2_1, GPIO.LOW)
        else:
            duty1 = max(0.0, min(255.0, abs(out1)))
            if self.dir1 > 0:
                GPIO.output(IN1_1, GPIO.HIGH)
                GPIO.output(IN2_1, GPIO.LOW)
            else:
                GPIO.output(IN1_1, GPIO.LOW)
                GPIO.output(IN2_1, GPIO.HIGH)
        self.pwm1.ChangeDutyCycle(duty1)
        self.rpm_pub_1.publish(Float32(data=rpm1))

        # Motor 2
        with self.lock:
            d2 = self.encoder_pos_2 - self.last_encoder_2
            self.last_encoder_2 = self.encoder_pos_2
        rpm2_inst = (d2 / PPR) / dt * 60.0
        rpm2 = alpha * self.prev_rpm_2 + (1 - alpha) * rpm2_inst
        self.prev_rpm_2 = rpm2

        err2 = self.desired_rpm_2 - rpm2
        self.integral_2 += err2 * dt
        der2 = (err2 - self.prev_error_2) / dt
        self.prev_error_2 = err2
        out2 = self.kp2 * err2 + self.ki2 * self.integral_2 + self.kd2 * der2

        if self.desired_rpm_2 == 0.0:
            duty2 = 0.0
            GPIO.output(IN1_2, GPIO.LOW)
            GPIO.output(IN2_2, GPIO.LOW)
        else:
            duty2 = max(0.0, min(255.0, abs(out2)))
            if self.dir2 > 0:
                GPIO.output(IN1_2, GPIO.HIGH)
                GPIO.output(IN2_2, GPIO.LOW)
            else:
                GPIO.output(IN1_2, GPIO.LOW)
                GPIO.output(IN2_2, GPIO.HIGH)
        self.pwm2.ChangeDutyCycle(duty2)
        self.rpm_pub_2.publish(Float32(data=rpm2))

        # Print de debug
        print(f"M1 → RPM={rpm1:6.1f} | Set={self.desired_rpm_1:6.1f} | Duty={duty1:5.1f}%  ||  "
              f"M2 → RPM={rpm2:6.1f} | Set={self.desired_rpm_2:6.1f} | Duty={duty2:5.1f}%")

        self.last_time = now
        

    def destroy_node(self):
        self.running = False
        time.sleep(0.1)
        self.pwm1.ChangeDutyCycle(0); self.pwm1.stop()
        self.pwm2.ChangeDutyCycle(0); self.pwm2.stop()
        GPIO.output(IN1_1, GPIO.LOW); GPIO.output(IN2_1, GPIO.LOW)
        GPIO.output(IN1_2, GPIO.LOW); GPIO.output(IN2_2, GPIO.LOW)
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualMotorTwoPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()        
