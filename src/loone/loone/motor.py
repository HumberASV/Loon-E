import busio
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import numpy as np
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Motor(Node):
    def __init__(self):
        super().__init__('Motor_Sub')
        self.phone_sub = self.create_subscription(Float32MultiArray, 'Phone', self.phone_callback, 10)
        self.task_sub = self.create_subscription(Float32MultiArray, 'Task', self.task_callback, 10)

        #Set up PCA board
        freq = 50
        i2c = busio.I2C(board.SCL_1, board.SDA_1)
        self.pca = PCA9685(i2c)
        self.pca.frequency = freq
        self.pulse = 1 / freq * 10**6

        #Set up PWM for motors
        self.prop_l = servo.Servo(self.pca.channels[0], min_pulse = 1120, max_pulse = 1880)
        self.prop_r = servo.Servo(self.pca.channels[0], min_pulse = 1120, max_pulse = 1880)
        self.rudder = servo.Servo(self.pca.channels[0], min_pulse = 1220, max_pulse = 1780)
        self.factor = 0.75

        #PID control variables
        self.kp = 1
        self.ki = 0
        self.kd = 0
        self.i = 0
        self.last_error = 0
        self.last_time = time.time()
        self.min = -45
        self.max = 45

        self.current_speed = None
        self.current_heading = None
        self.target_heading = None
        self.target_speed = None

    def remap(self, error):
        outMin = 1540
        outMax = 1880

        output = outMin + ((abs(error) - self.max) / (self.min - self.max) * (outMax - outMin)) #inMin and inMax swapped: Bigger error -> turn more
        return output
    
    def get_fraction(self, pulse, min_pulse = 1120, max_pulse = 1880): #Convert value in microseconds to duty cycle in %
        fraction = (pulse - min_pulse) / (max_pulse - min_pulse)

        return fraction

    def drive(self):
        #get current error, integral, time
        current_time = time.time()
        current_error = self.target_heading - self.current_heading
        dt = current_time - self.last_time
        de = (current_error - self.last_error) / dt

        #calculate integral and clamp
        self.i = self.i + self.ki * current_error
        if self.i < self.min:
            self.i = self.min

        elif self.i > self.max:
            self.i = self.max

        #calculate output and clamp
        output = self.kp * current_error + self.i * dt + self.kd * de
        if output < self.min:
            output = self.min
        
        elif output > self.max:
            output = self.max

        output = self.remap(current_error)
        
        #change speed as needed
        if self.current_speed < self.target_speed and self.factor < 1:
            self.factor += 0.5
        elif self.current_speed > self.target_speed and self.factor > 0.55:
            self.factor -= 0.5

        #set propeller speeds
        if current_error > 0: #right
            self.prop_l.fraction = self.factor #max forward
            self.prop_r.fraction = self.get_fraction(output) * self.factor #forward based on PID
        else: #left
            self.prop_l.fraction = self.get_fraction(output) * self.factor #forward based on PID
            self.prop_r.fraction = self.factor #max forward 

        #set rudder speeds
        if output < self.min / 2:
            self.rudder.fraction = 0 #35 degrees right
        elif output > self.max / 2:
            self.rudder.fraction = 1 #35 degrees left
        else:
            self.rudder.fraction = 0.5 #0 degrees

        #set previous error and time
        self.last_error = current_error
        self.last_time = current_time
    
    def check_data(self):
        data_good = True
        if self.current_heading is None or self.current_speed is None or self.target_heading is None or self.target_speed is None:
            data_good = False

        return data_good

    def phone_callback(self, msg):
        data = msg.data
        self.get_logger().info(f"Phone: {msg.data}")
        self.current_speed = data[2]
        self.current_heading = data[3]

    def task_callback(self, msg):
        data = msg.data
        self.get_logger().info(f"Task: {msg.data}")
        self.target_heading = data[1]
        self.target_speed = data[2]
        if self.check_data():
            self.drive()
    
    def shutdown(self):
        self.pca.deinit()

def main(args = None):
    rclpy.init(args = args)
    motor = Motor()
    rclpy.spin(motor)
    motor.shutdown()
    motor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()