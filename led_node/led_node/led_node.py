import rclpy
import time

from rclpy.node import Node
from std_msgs.msg import Float32
from rpi_ws281x import PixelStrip, Color

LED_COUNT = 8
LED_PIN = 12
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 25        #0-255
LED_INVERT = False 
LED_CHANNEL = 0

class LedNode(Node):
    def __init__(self):
        super().__init__('led_node')
        self.strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        self.strip.begin()
        
        self.startup_animation()

        self.subscription = self.create_subscription(
            Float32,
            '/percentage',
            self.percentage_callback,
            10)

    def startup_animation(self):
        for i in range(LED_COUNT):
            self.strip.setPixelColor(i, Color(255, 255, 255))
            self.strip.show()
            time.sleep(0.2)

        for i in range(LED_COUNT):
            self.strip.setPixelColor(i, Color(0, 0, 0))
            self.strip.show()
            time.sleep(0.2)

        for _ in range(2):
            for i in range(LED_COUNT):
                self.strip.setPixelColor(i, Color(255, 255, 255))
            self.strip.show()
            time.sleep(0.5)
            for i in range(LED_COUNT):
                self.strip.setPixelColor(i, Color(0, 0, 0))
            self.strip.show()
            time.sleep(0.5)

    def percentage_callback(self, msg):
        self.percentage = msg.data
        self.get_logger().info(f'Received percentage: {self.percentage}')
        self.set_led(self.percentage)

    def set_led(self, percentage):
        percentage = max(0, min(100, percentage))
        leds_to_turn_on = round((percentage / 100) * LED_COUNT)

        for i in range(LED_COUNT):
            if i < leds_to_turn_on:
                self.strip.setPixelColor(i, Color(255, 255, 255))
            else:
                self.strip.setPixelColor(i, Color(0, 0, 0))
        self.strip.show()

def main(args=None):
    rclpy.init(args=args)
    node = LedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
