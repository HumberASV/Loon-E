import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Task(Node):
    """
    A ROS2 node that publishes task commands to a topic.
    The node publishes a Float32MultiArray message containing 
        the command, 
        target heading, 
        target speed, 
        and direction 
    at a regular interval.
    """

    def __init__(self):
        """ Initialize the Task node and set up the publisher and timer. """
        super().__init__('Task_Pub')

        # Declare parameters with fallback/default values
        self.declare_parameter('timer_period', 0.25)

        # Retrieve parameters
        timer_period = self.get_parameter('timer_period').value
        
        self.publisher_ = self.create_publisher(Float32MultiArray, 'task', 10)
        self.timer = self.create_timer(timer_period, self.run_task)
        self.get_logger().info(f"Task node initialized with timer period: {timer_period} seconds.")

    def publish(self) -> None:
        """ Publish the task command as a Float32MultiArray message. """
        msg = Float32MultiArray()
        msg.data = [self.command, self.target_heading, self.target_speed, self.dir]
        self.publisher_.publish(msg)
    
    def run_task(self) -> None:
        """ Update the task command parameters and publish the message. """
        self.command = 1.0
        self.target_heading = 0.0
        self.target_speed = 1.0
        self.dir = None
        
        self.publish()

def main(args = None) -> None:
    """ Main function to initialize the ROS2 node and start spinning. """

    rclpy.init(args = args)
    task = Task()

    # Add a try-except block to handle KeyboardInterrupt gracefully
    try:
        rclpy.spin(task)
    except KeyboardInterrupt:
        task.get_logger().info("Task node interrupted by user.")
    finally:
        task.shutdown()
        task.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()