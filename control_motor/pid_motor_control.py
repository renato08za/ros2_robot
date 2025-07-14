import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

MAX_PWM = 255

class PID:
    def __init__(self, Kp, Ki, Kd, Ko):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ko = Ko

        self.target = 0.0
        self.prev_input = 0
        self.ITerm = 0
        self.output = 0

    def update(self, input_val):
        perror = self.target - input_val
        output = (self.Kp * perror - self.Kd * (input_val - self.prev_input) + self.ITerm) / self.Ko
        output += self.output

        if output >= MAX_PWM:
            output = MAX_PWM
        elif output <= 0:
            output = 0
        else:
            self.ITerm += self.Ki * perror

        self.output = output
        self.prev_input = input_val
        return int(output)


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('pid_motor_control')

        self.pid = PID(Kp=20, Ki=1, Kd=12, Ko=50)
        self.current_rpm = 0.0

        self.create_subscription(Float32, '/motor1_rpm', self.rpm_callback, 10)
        self.create_subscription(Float32, '/ref_rpm1', self.ref_callback, 10)
        self.pwm_pub = self.create_publisher(Int32, '/motor1_pwm_set', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def rpm_callback(self, msg):
        self.current_rpm = msg.data

    def ref_callback(self, msg):
        self.pid.target = msg.data

    def control_loop(self):
        pwm = self.pid.update(self.current_rpm)
        self.pwm_pub.publish(Int32(data=pwm))
        self.get_logger().info(f"Ref: {self.pid.target:.1f} RPM | Medido: {self.current_rpm:.1f} | PWM: {pwm}")

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
