import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
import time
import threading

class LED(Node):
    def __init__(self):
        self.initialize()

        self.receiver = threading.Thread(target = self.light, daemon = True)
        self.receiver.start()

    def initialize(self):
        red_pin = 15
        green_pin = 32
        blue_pin = 33

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(red_pin, GPIO.OUT)
        GPIO.setup(green_pin, GPIO.OUT)
        GPIO.setup(blue_pin, GPIO.OUT)

        self.red = GPIO.PWM(red_pin, 50)
        self.green = GPIO.PWM(green_pin, 50)
        self.blue = GPIO.PWM(blue_pin, 50)

        self.red.start(0)
        self.green.start(0)
        self.blue.start(0)

    def shutdown(self):
        self.red.stop()
        self.green.stop()
        self.blue.stop()
        GPIO.cleanup()

    def rgb_to_duty(self, rgb):
        dc = rgb/255

        return dc

    def light(self):
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
    rclpy.init(args = args)
    led = LED()
    rclpy.spin(led)
    led.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()