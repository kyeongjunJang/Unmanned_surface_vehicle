#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
import math

def Magnetic_now():
    pub = rospy.Publisher('/imu/mag', MagneticField, queue_size=10)

    rospy.init_node('fakeMagnetic', anonymous = False)
    rate = rospy.Rate(10)

    msg = MagneticField()

    while not rospy.is_shutdown():
        msg.magnetic_field.x = 3.0
        msg.magnetic_field.y = 2.0
        msg.magnetic_field.z = 1.0

        # rospy.loginfo(msg)

        pub.publish(msg)
        
        rospy.sleep(0.05) # 1 / Hz
    rospy.spin()
        #rate.sleep()

if __name__ =='__main__':
    try:
        Magnetic_now()
    except rospy.ROSInitException:
        pass
        

