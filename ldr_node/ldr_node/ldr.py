import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

LDR_PIN = 18

class LDRPublisher(Node):
    def __init__(self):
        super().__init__('ldr_publisher')
        self.publisher_ = self.create_publisher(Int32, 'ldr', 10)
        self.timer = self.create_timer(1.0, self.publish_ldr_value) 
        GPIO.setmode(GPIO.BCM)
        self.get_logger().info("LDR Publisher Node has started.")

    def rc_time(self, pin):
        count = 0
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, False)
        time.sleep(0.1)
        GPIO.setup(pin, GPIO.IN)
        while GPIO.input(pin) == GPIO.LOW:
            count += 1
        return count

    def publish_ldr_value(self):
        brightness = self.rc_time(LDR_PIN)
        msg = Int32()
        msg.data = brightness
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Brightness Level: {brightness}')

    def cleanup(self):
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = LDRPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup()
        node.get_logger().info("LDR Publisher Node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
