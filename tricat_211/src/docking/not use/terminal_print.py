#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import time
from std_msgs.msg import String

class Terminal_Print:
    def __init__(self):
        rospy.Subscriber('/print', String, self.str_callback)

    def str_callback(self, msg):
        print(msg.data)

def main():   
    rospy.init_node('Terminal_Print', anonymous=False)
    rate = rospy.Rate(10)

    termianl_print = Terminal_Print()

    while not rospy.is_shutdown():
        
        rate.sleep() 
    
    rospy.spin()

if __name__ == '__main__':
    main()




