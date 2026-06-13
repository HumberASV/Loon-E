import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Task(Node):
    def __init__(self):
        super().__init__('Task_Pub')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'Task', 10)
        timer_period = 0.25
        self.timer = self.create_timer(timer_period, self.run_task)

    def publish(self):
        msg = Float32MultiArray()
        msg.data = [self.action, self.target_heading, self.target_speed]
        self.publisher_.publish(msg)
        #self.get_logger().info(f"Task: {msg.data}")
    
    def run_task(self):
        self.action = 1.0
        self.target_heading = 0.0
        self.target_speed = 1.0
        
        self.publish()

def main(args = None):
    rclpy.init(args = args)
    task = Task()
    rclpy.spin(task)
    task.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()