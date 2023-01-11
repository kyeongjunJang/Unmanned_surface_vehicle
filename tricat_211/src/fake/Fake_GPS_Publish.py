#!/usr/bin/env python
import rospy
from ublox_msgs.msg import NavPVT
from sensor_msgs.msg import NavSatFix
import time
import math

def GPS_now():
    pub1 = rospy.Publisher('/ublox_gps/navpvt', NavPVT, queue_size=10)
    pub2 = rospy.Publisher('/ublox_gps/fix', NavSatFix, queue_size=10)

    rospy.init_node('fakeGPS', anonymous = False)
    #rate = rospy.Rate(10)

    msg1 = NavPVT()
    msg2 = NavSatFix()
    while not rospy.is_shutdown():
        msg1.heading = 180*100000
        msg2.latitude, msg2.longitude = 37.451199251623244, 126.65554189666211 
        msg1.gSpeed = 1

        # rospy.loginfo(msg)
        pub1.publish(msg1)
        pub2.publish(msg2)
        rospy.sleep(0.05) # 1 / Hz
    rospy.spin()
        #rate.sleep()

if __name__ =='__main__':
    try:
        GPS_now()
    except rospy.ROSInitException:
        pass
        

