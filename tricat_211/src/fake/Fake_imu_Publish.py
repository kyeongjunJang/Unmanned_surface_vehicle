#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import math

def IMU_now():
    pub = rospy.Publisher('/imu/data', Imu, queue_size=10)

    rospy.init_node('fakeIMU', anonymous = False)
    rate = rospy.Rate(10)

    msg = Imu()

    while not rospy.is_shutdown():
        msg.orientation.x = 2.0
        msg.orientation.y = 3.0
        msg.orientation.z = 5.0
        msg.orientation.w = 1.0
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 5.0

        # rospy.loginfo(msg)

        pub.publish(msg)
        
        rospy.sleep(0.05) # 1 / Hz
    rospy.spin()
        #rate.sleep()

if __name__ =='__main__':
    try:
        IMU_now()
    except rospy.ROSInitException:
        pass
        

