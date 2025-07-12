#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray
import RPi.GPIO as GPIO
import time
from threading import Lock, Thread

# PINOS BCM
ENCA1, ENCB1 = 17, 18
ENCA2, ENCB2 = 20, 21
PWM1, IN1_1, IN2_1 = 13, 5, 6
PWM2, IN1_2, IN2_2 = 19, 16, 26
PPR = 374

# Ganhos PID
Kp, Ki, Kd = 0.8, 0.1, 0.0
MAX_PWM = 255
CONTROL_HZ = 10

class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_err = 0.0

    def compute(self, err, dt):
        self.integral += err * dt
        d = (err - self.prev_err)/dt if dt>0 else 0.0
        self.prev_err = err
        return self.kp*err + self.ki*self.integral + self.kd*d

class DualMotorPIDNode(Node):
    def __init__(self):
        super().__init__('dual_motor_pid_node')
        self.lock = Lock()
        self.ticks = [0,0]
        self.prev_ticks = [0,0]
        self.ref = [0.0,0.0]
        self.rpm = [0.0,0.0]
        self.last_time = time.time()

        # GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for p in (ENCA1,ENCB1, ENCA2,ENCB2):
            GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        for p in (IN1_1,IN2_1,PWM1, IN1_2,IN2_2,PWM2):
            GPIO.setup(p, GPIO.OUT)
        self.pwm1 = GPIO.PWM(PWM1, 1000); self.pwm1.start(0)
        self.pwm2 = GPIO.PWM(PWM2, 1000); self.pwm2.start(0)

        # Threads de encoder
        Thread(target=self._poll, args=(ENCA1, ENCB1, 0), daemon=True).start()
        Thread(target=self._poll, args=(ENCA2, ENCB2, 1), daemon=True).start()

        # PID
        self.pid = [PID(Kp,Ki,Kd), PID(Kp,Ki,Kd)]

        # ROS
        self.create_subscription(Float32, '/ref_rpm1', lambda m: self._set_ref(m,0), 10)
        self.create_subscription(Float32, '/ref_rpm2', lambda m: self._set_ref(m,1), 10)
        self.create_subscription(Int32MultiArray, '/encoder_ticks', self._cb_enc, 10)
        self.create_timer(1.0/CONTROL_HZ, self._control_loop)

        self.get_logger().info('DualMotorPIDNode iniciado')

    def _poll(self, pa, pb, idx):
        last = GPIO.input(pa)
        while rclpy.ok():
            a=GPIO.input(pa); b=GPIO.input(pb)
            if a!=last and a==1:
                delta = 1 if b==1 else -1
                with self.lock:
                    self.ticks[idx] += delta
            last=a
            time.sleep(0.0002)

    def _cb_enc(self, msg: Int32MultiArray):
        with self.lock:
            self.ticks[0], self.ticks[1] = msg.data

    def _set_ref(self, msg, idx):
        with self.lock:
            self.ref[idx] = msg.data

    def _control_loop(self):
        now = time.time(); dt = now - self.last_time
        if dt<=0: return
        self.last_time = now

        with self.lock:
            # calcula RPM
            for i in (0,1):
                d = self.ticks[i] - self.prev_ticks[i]
                self.rpm[i] = (d/PPR)*(60.0/dt)
                self.prev_ticks[i] = self.ticks[i]
            ref = self.ref.copy()

        # PID e saída GPIO
        for i in (0,1):
            err = ref[i] - self.rpm[i]
            cmd = self.pid[i].compute(err, dt)
            pwm = int(min(abs(cmd), MAX_PWM))
            dir_ = 1 if cmd>=0 else -1

            IN1,IN2,pw = (IN1_1,IN2_1,self.pwm1) if i==0 else (IN1_2,IN2_2,self.pwm2)
            GPIO.output(IN1, GPIO.HIGH if dir_>0 else GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW  if dir_>0 else GPIO.HIGH)
            pw.ChangeDutyCycle(pwm * 100.0/255.0)

        self.get_logger().info(f"RPMs: {self.rpm[0]:.1f},{self.rpm[1]:.1f}")

    def destroy_node(self):
        self.pwm1.stop(); self.pwm2.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualMotorPIDNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
