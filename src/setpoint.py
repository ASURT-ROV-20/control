#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from dynamic_reconfigure.server import Server 
from Z_Control.cfg import pConfig

set_point_value = 0
 
def reconfigure(config, level):
    global a
    global set_point_value
    set_point_value = config["point"]
    a.publish(set_point_value)
    print(config["point"])
    return config



rospy.init_node("setpoint")
a = rospy.Publisher("setpoint",Float64, queue_size=5)
srv = Server(pConfig, reconfigure)
# b = rospy.Publisher("pid_enable", Bool, queue_size=5)

# while not rospy.is_shutdown():
    # depth = rospy.get_param("~fp")
    # depth = float(input("Enter Set_Point: "))
    # a.publish(set_point_value)
    # b.publish(True)
    # time.sleep(0.5)

rospy.spin()
print("Finished")
