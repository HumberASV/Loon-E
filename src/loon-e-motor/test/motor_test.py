import RPi.GPIO as GPIO

def getDuty(us): #Convert value in microseconds to duty cycle in %
    dc = us/pulse

    return dc

#Set up PWM for motors
#prop_l_pin = 32
rudder_pin = 15
freq = 50
pulse = 1/freq*10**6

GPIO.setmode(GPIO.BOARD)
#GPIO.setup(prop_l_pin, GPIO.OUT)
GPIO.setup(rudder_pin, GPIO.OUT)

#prop_l = GPIO.PWM(prop_l_pin, freq)
rudder = GPIO.PWM(rudder_pin, freq)

#start motors
zero = getDuty(1500)
#prop_l.start(zero) #no motion left thruster
rudder.start(zero) #rudder to 0 degrees

#prop_l.ChangeDutyCycle(getDuty(1880)) #minimum forward thrust left thruster
rudder.ChangeDutyCycle(getDuty(1300)) #rudder to 20 degrees left