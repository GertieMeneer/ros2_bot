import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class AutonomousMapper(Node):
    def __init__(self):
        super().__init__('autonomous_mapper')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.percentage_pub = self.create_publisher(Float32, '/percentage', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.mapped_percentage = 0.0
        self.current_cell = 0
        self.grid_size = 3
        self.cell_size = 1.0
        self.exploring = False

    def map_callback(self, msg):
        if not self.exploring:
            self.explore()

    def explore(self):
        if self.current_cell >= self.grid_size * self.grid_size:
            self.get_logger().info('Exploration completed')
            return

        row = self.current_cell // self.grid_size
        col = self.current_cell % self.grid_size

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = -self.cell_size + (col * self.cell_size)
        goal_pose.pose.position.y = self.cell_size - (row * self.cell_size)
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Navigating to new goal: ({goal_pose.pose.position.x}, {goal_pose.pose.position.y})')

        self.exploring = True
        self.send_goal(goal_pose)

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_client.wait_for_server()
        
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.exploring = False
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal reached')

        self.current_cell += 1
        self.mapped_percentage = (self.current_cell / (self.grid_size * self.grid_size)) * 100
        self.percentage_pub.publish(Float32(data=self.mapped_percentage))
        self.get_logger().info(f'Mapped area: {self.mapped_percentage:.2f}%')

        self.exploring = False
        self.explore()
        
def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
