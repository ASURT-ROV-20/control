#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import time
import Adafruit_PCA9685
# import RPi.GPIO as GPIO
import ms5837

sensor = ms5837.MS5837_30BA()
hat = Adafruit_PCA9685.PCA9685()
hat.set_pwm_freq(50)
sensor.init()

offset = 300

def control_handler(data):
    # print (data)
    global offset
    hat.set_pwm(3,0,int(data+offset))
    hat.set_pwm(4,0,int(data+offset))


rospy.init_node("Hardware")
a = rospy.Publisher("state",Float64, queue_size=5)
b = rospy.Subscriber("control_effort",Float64,control_handler)

while not rospy.is_shutdown():
    sensor.read()
    a.publish(sensor.depth())
    time.sleep(0.05)

