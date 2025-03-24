import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import random

class AutonomousMapper(Node):
    def __init__(self):
        super().__init__('autonomous_mapper')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.mapped_percentage = 0.0

    def map_callback(self, msg):
        # Convert OccupancyGrid data to numpy array
        grid_data = np.array(msg.data)
        known_cells = np.count_nonzero(grid_data != -1)  # Exclude unknown (-1) values
        total_cells = grid_data.size
        self.mapped_percentage = (known_cells / total_cells) * 100

        self.get_logger().info(f'Mapped: {self.mapped_percentage:.2f}%')

        if self.mapped_percentage >= 65:
            self.get_logger().info('65% mapped! Stopping exploration.')
            rclpy.shutdown()
        else:
            self.explore()

    def explore(self):
        # Generate a random goal position for exploration
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = random.uniform(-2, 2)  # Adjust range as needed
        goal.pose.position.y = random.uniform(-2, 2)
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.get_logger().info(f'Navigating to new goal: ({goal.pose.position.x}, {goal.pose.position.y})')

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
