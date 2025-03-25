import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int32

class AutonomousMapper(Node):
    def __init__(self):
        super().__init__('autonomous_mapper')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.ldr_sub = self.create_subscription(Int32, '/ldr', self.ldr_callback, 10)
        self.percentage_pub = self.create_publisher(Float32, '/percentage', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.mapped_percentage = 0.0
        self.current_cell = 0
        self.grid_size = 3
        self.cell_size = 1.0
        self.exploring = False
        self.lowest_ldr_found = False
        
        self.ldr_value = None
        self.goals_list = []
        self.ldr_values_list = []

    def map_callback(self, msg):
        if not self.exploring:
            self.explore()
    
    def ldr_callback(self, msg):
        self.ldr_value = msg.data

    def explore(self):
        if self.current_cell >= self.grid_size * self.grid_size:
            self.get_logger().info('Exploration completed')
            if not self.lowest_ldr_found:
                self.find_lowest_ldr_location()
            return

        row = self.current_cell // self.grid_size
        col = self.current_cell % self.grid_size

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = -self.cell_size + (col * self.cell_size)
        goal_pose.pose.position.y = self.cell_size - (row * self.cell_size)
        goal_pose.pose.orientation.w = 1.0
        
        self.goals_list.append((goal_pose.pose.position.x, goal_pose.pose.position.y))
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

        if not self.lowest_ldr_found:
        	if self.ldr_value is not None:
            	self.ldr_values_list.append(self.ldr_value)
            	self.get_logger().info(f'LDR Value at goal: {self.ldr_value}')
        	else:
            	self.ldr_values_list.append(None)
            	self.get_logger().info('LDR Value not available')

        	self.current_cell += 1
        	self.mapped_percentage = (self.current_cell / (self.grid_size * self.grid_size)) * 100
        	self.percentage_pub.publish(Float32(data=self.mapped_percentage))
        	self.get_logger().info(f'Mapped area: {self.mapped_percentage:.2f}%')

        self.exploring = False
        self.explore()
    
    def find_lowest_ldr_location(self):
        if not self.ldr_values_list:
            self.get_logger().info('No LDR values recorded.')
            return

        min_ldr_value = min(self.ldr_values_list)
        min_index = self.ldr_values_list.index(min_ldr_value)

        min_goal_location = self.goals_list[min_index]

        self.get_logger().info(f'Lowest LDR Value: {min_ldr_value} at Location: {min_goal_location}')

        self.get_logger().info(f'Sending bot to the lowest LDR location: {min_goal_location}')
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = min_goal_location[0]
        goal_pose.pose.position.y = min_goal_location[1]
        goal_pose.pose.orientation.w = 1.0
        
        self.send_goal(goal_pose)

        self.lowest_ldr_found = True
        self.get_logger().info('Finished! Lowest LDR location has been processed.')

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
