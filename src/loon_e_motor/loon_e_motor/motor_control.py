import RPi.GPIO as GPIO
import numpy as np
import time

class MotorController():
    def __init__(self):
        #Set up PWM for motors
        prop_l_pin = 32
        prop_r_pin = 33
        rudder_pin = 15
        freq = 50
        self.pulse = 1/freq*10**6

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(prop_l_pin, GPIO.OUT)
        GPIO.setup(prop_r_pin, GPIO.OUT)
        GPIO.setup(rudder_pin, GPIO.OUT)

        self.prop_l = GPIO.PWM(prop_l_pin, freq)
        self.prop_r = GPIO.PWM(prop_r_pin, freq)
        self.rudder = GPIO.PWM(rudder_pin, freq)

        #start motors
        self.zero = self.getDuty(1500)
        self.prop_l.start(self.zero) #no motion left thruster
        self.prop_r.start(self.zero) #no motion right thruster
        self.rudder.start(self.zero) #rudder to 0 degrees

        #PID control variables
        self.kp = 1
        self.ki = 0
        self.kd = 0
        self.i = 0
        self.last_error = 0
        self.last_time = time.time()
        self.min = 0
        self.max = 4 #same as path_planning self.dist

    def remap(self, error):
        outMin = 1540
        outMax = 1880

        output = outMin + ((abs(error) - self.max)/(self.min - self.max)*(outMax - outMin)) #inMin and inMax swapped: Bigger error -> turn more
        return output
    
    def getDuty(self, us): #Convert value in microseconds to duty cycle in %
        dc = us/self.pulse

        return dc

    def search(self): #rotate clockwise
        self.prop_l.ChangeDutyCycle(self.getDuty(1540)) #minimum forward thrust left thruster
        self.prop_l.ChangeDutyCycle(self.getDuty(1460)) #minimum backward thrust right thruster
        self.rudder.ChangeDutyCycle(self.zero) #rudder to 0 degrees

        return

    def drive(self, path):
        #get current error, integral, time
        current_time = time.time()
        current_error = path[1][1] - path[0][1]
        dt = current_time - self.last_time
        de = (current_error - self.last_error)/dt

        #calculate integral and clamp
        self.i = self.i + self.ki*current_error
        if self.i < self.min:
            self.i = self.min

        elif self.i > self.max:
            self.i = self.max

        #calculate output and clamp
        output = self.kp*current_error + self.i*dt + self.kd*de
        if output < self.min:
            output = self.min
        
        elif output > self.max:
            output = self.max

        print(output)

        output = self.remap(current_error)
        #set propeller speeds
        if current_error > 0: #right
            self.prop_l.ChangeDutyCycle(self.getDuty(1880)) #maximum forward thrust left thruster
            self.prop_r.ChangeDutyCycle(self.getDuty(output)) #forward right thrust based on PID
        else: #left
            self.prop_l.ChangeDutyCycle(self.getDuty(output)) #forward left thrust based on PID
            self.prop_r.ChangeDutyCycle(self.getDuty(1880)) #maximum forward thrust right thruster

        #set rudder speeds
        if output < -2:
            self.rudder.ChangeDutyCycle(self.getDuty(1650)) #rudder to 20 degrees right
        elif output > 2:
            self.rudder.ChangeDutyCycle(self.getDuty(1300)) #rudder to 20 degrees left
        else:
            self.rudder.ChangeDutyCycle(self.zero) #rudder to 0 degrees

        #set previous error and time
        self.last_error = current_error
        self.last_time = current_time
        return

    def turn(self, dir):
        if np.sign(dir) == 1: #turn left
            self.prop_l.ChangeDutyCycle(self.getDuty(1540)) #minimum forward thrust left thruster
            self.prop_r.ChangeDutyCycle(self.getDuty(1880)) #maximum forward thrust right thruster
            self.rudder.ChangeDutyCycle(self.getDuty(1300)) #rudder to 20 degrees left
            
        else: #turn right
            self.prop_l.ChangeDutyCycle(self.getDuty(1880)) #maximum forward thrust left thruster
            self.prop_r.ChangeDutyCycle(self.getDuty(1540)) #minimum forward thrust right thruster
            self.rudder.ChangeDutyCycle(self.getDuty(1650)) #rudder to 20 degrees right

        return
    
    def stop(self):
        self.prop_l.ChangeDutyCycle(self.zero) #no motion left thruster
        self.prop_r.ChangeDutyCycle(self.zero) #no motion right thruster
        self.rudder.ChangeDutyCycle(self.zero) #rudder to 0 degrees

    def stop_pwm(self):
        self.prop_l.stop()
        self.prop_r.stop()
        self.rudder.stop()
        GPIO.cleanup()

        return