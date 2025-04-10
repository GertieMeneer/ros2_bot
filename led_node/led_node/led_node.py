import rclpy
import time
import math

from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry, Path
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

        # self.test_led_progression()
        
        self.startup_animation()

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.percentage_subscription = self.create_subscription(Float32, '/percentage', self.percentage_callback, 10)
        self.navigation_subscription = self.create_subscription(Bool, '/navigating_to_brightest_point', self.navigation_started_callback, 10)

        self.navigating = False
        self.finished_navigating = False
        self.last_pose = None
        self.distance_driven = 0.0
        self.distance_remaining = 0.0
        self.initial_distance = 0.0
    
    def navigation_started_callback(self, msg):
        self.get_logger().info(f'Navigation to brightest point started')
        self.navigating = msg.data
        if self.navigating:
            self.distance_driven = 0.0
            self.last_pose = None

    def odom_callback(self, msg):
        if not self.navigating:
            return
        current_pose = msg.pose.pose
        if self.last_pose is not None:
            self.distance_driven += self.calculate_distance(self.last_pose, current_pose)
        self.last_pose = current_pose
        self.update_navigation_progress()

    def plan_callback(self, msg):
        if not self.navigating:
            return
        self.distance_remaining = self.calculate_path_length(msg.poses)
        if self.initial_distance == 0:
            self.initial_distance = self.distance_remaining
        self.update_navigation_progress()

    def calculate_distance(self, pose1, pose2):
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        return math.sqrt(dx**2 + dy**2)

    def calculate_path_length(self, poses):
        total_distance = 0.0
        for i in range(1, len(poses)):
            total_distance += self.calculate_distance(poses[i-1].pose, poses[i].pose)
        return total_distance

    def update_navigation_progress(self):
        if self.finished_navigating is True: return
        if self.initial_distance > 0:
            progress_percentage = (self.distance_driven / self.initial_distance) * 100
            progress_percentage = min(100, max(0, progress_percentage))
            self.get_logger().info(f'Navigation progress: {progress_percentage:.2f}%')
            self.set_led(progress_percentage)
            if progress_percentage is 100:
                self.finished_navigating = True
                time.sleep(3)
                self.stop_animation()

    def percentage_callback(self, msg):
        if not self.navigating:
            self.percentage = msg.data
            self.get_logger().info(f'Received percentage: {self.percentage}')
            self.set_led(self.percentage)

    def set_led(self, percentage):
        percentage = max(0, min(100, percentage))
        leds_to_turn_on = round((percentage / 100) * LED_COUNT)

        for i in range(LED_COUNT):
            if i < leds_to_turn_on:
                red = int(max(0, 255 * (1 - percentage / 100)))
                green = int(max(0, 255 * (percentage / 100)))
                color = Color(red, green, 0)
                self.strip.setPixelColor(i, color)
            else:
                self.strip.setPixelColor(i, Color(0, 0, 0))

        self.strip.show()
    
    def startup_animation(self):
        for i in range(LED_COUNT):
            self.strip.setPixelColor(i, Color(255, 255, 255))
            self.strip.show()
            time.sleep(0.1)
        
        for i in range(LED_COUNT):
            self.strip.setPixelColor(i, Color(0, 0, 255))
            self.strip.show()
            time.sleep(0.1)

        for i in range(LED_COUNT):
            self.strip.setPixelColor(i, Color(0, 0, 0))
            self.strip.show()
            time.sleep(0.1)

    def stop_animation(self):
        self.get_logger().info("Navigation finished")
        while True:
            for i in range(101):
                blue_value = int(255 * (i / 100))
                for j in range(LED_COUNT):
                    self.strip.setPixelColor(j, Color(0, 0, blue_value))
                self.strip.show()
                time.sleep(0.001)
            
            for i in range(100, -1, -1):
                blue_value = int(255 * (i / 100))
                for j in range(LED_COUNT):
                    self.strip.setPixelColor(j, Color(0, 0, blue_value))
                self.strip.show()
                time.sleep(0.001)

    def test_led_progression(self):
        for percentage in range(0, 101, 1):
            self.set_led(percentage)
            print(f"LED set to {percentage}%")
            time.sleep(0.5)
        
        self.set_led(0)
        print("LED test complete")

def main(args=None):
    rclpy.init(args=args)
    node = LedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()