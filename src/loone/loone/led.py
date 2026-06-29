import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
import time
import threading

class LED(Node):
    """ 
    A ROS2 node that controls an LED light connected to a Jetson board.
    The node runs a separate thread to manage the LED light's behavior, cycling through red, yellow, and green colors.
    """

    def __init__(self):
        """ Initialize the LED node and set up GPIO pins for controlling the LED light. """
        super().__init__('LED_Node')

        # Declare parameters with fallback/default values
        self.declare_parameter('red_pin', 15)
        self.declare_parameter('green_pin', 32)
        self.declare_parameter('blue_pin', 33)
        self.declare_parameter('data_pin', 40)

        # Retrieve parameters
        self.red_pin   = self.get_parameter('red_pin').value
        self.green_pin = self.get_parameter('green_pin').value
        self.blue_pin  = self.get_parameter('blue_pin').value
        self.data_pin  = self.get_parameter('data_pin').value

        self.receiver = threading.Thread(target = self.light, daemon = True)
        self.receiver.start()

    def initialize(self):
        """ Set up the GPIO pins for controlling the LED light. """
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.red_pin, GPIO.OUT)
        GPIO.setup(self.green_pin, GPIO.OUT)
        GPIO.setup(self.blue_pin, GPIO.OUT)
        GPIO.setup(self.data_pin, GPIO.IN)

        self.red = GPIO.PWM(self.red_pin, 50)
        self.green = GPIO.PWM(self.green_pin, 50)
        self.blue = GPIO.PWM(self.blue_pin, 50)

        self.red.start(0)
        self.green.start(0)
        self.blue.start(0)

    def shutdown(self):
        """ Clean up the GPIO pins and stop the PWM signals for the LED light. """
        self.red.stop()
        self.green.stop()
        self.blue.stop()
        GPIO.cleanup()

    def rgb_to_duty(self, rgb: list) -> list:
        """
        Convert RGB values to duty cycle percentages.
        """
        dc = [value / 255.0 for value in rgb]
        return dc

    def light(self) -> None:
        """ Control the LED light's behavior, cycling through red, yellow, and green colors. """
        self.initialize()
        while rclpy.ok():
            self.red.ChangeDutyCycle(1)
            self.green.ChangeDutyCycle(0)
            time.sleep(1) #Red

            self.red.ChangeDutyCycle(1)
            self.green.ChangeDutyCycle(1)
            time.sleep(1) #Yellow

            self.red.ChangeDutyCycle(0)
            self.green.ChangeDutyCycle(1)
            time.sleep(1) #Green
        self.shutdown()

        
def main(args = None):
    """ Main function to initialize the ROS2 node and start spinning. """
    rclpy.init(args = args)
    led = LED()
    try:
        rclpy.spin(led)
    except KeyboardInterrupt:
        led.get_logger().info("LED node interrupted by user.")
    finally:
        led.shutdown()
        led.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()