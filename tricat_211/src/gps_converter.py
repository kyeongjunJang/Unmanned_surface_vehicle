#!/usr/bin/env python

import rospy
import math
from ublox_msgs.msg import NavPVT
from sensor_msgs.msg import NavSatFix
import pymap3d as pm
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

class GPS_Converter:
    def __init__(self):
        self.gps_boat_lat = 0.0
        self.gps_boat_lon = 0.0
        self.gps_boat_alt = 0.0

        self.gps_boat_heading = 0.0

        self.cur_boat_x = 0.0
        self.cur_boat_y = 0.0
        
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.fix_callback)
        #rospy.Subscriber("/ublox_gps/navpvt", NavPVT, self.pvt_callback)

        self.enu_position_pub = rospy.Publisher("/enu_position", Point, queue_size=10)

    def fix_callback(self, msg):
        self.gps_boat_lat = msg.latitude
        self.gps_boat_lon = msg.longitude
        self.gps_boat_alt = msg.altitude

    #def pvt_callback(self, msg):
        #self.gps_boat_lat = msg.lat
        #self.gps_boat_lon = msg.lon
        #self.gps_boat_heading = msg.heading # hedveh chek plz

    def enuPositionPublisher(self):
        enu_position_msg = Point()
        e, n, u = get_xy(self.gps_boat_lat, self.gps_boat_lon, self.gps_boat_alt)
        enu_position_msg.x = e
        enu_position_msg.y = n
        enu_position_msg.z = u
        self.enu_position_pub.publish(enu_position_msg)
        #heading = Float32()
        #heading.data = self.gps_boat_heading * 0.00001  # deg/1e-5 -> deg
        #rospy.loginfo("heading degree : %f", heading.data)

map_list = rospy.get_param("map_dd")
lat_00, lon_00, alt_00 = map_list['map_00_lat'], map_list['map_00_lon'], map_list['map_00_alt']
#lat_0y, lon_0y = map_list['map_0y_lat'], map_list['map_0y_lon']
#lat_x0, lon_x0 = map_list['map_x0_lat'], map_list['map_x0_lon']
#lat_xy, lon_xy = map_list['map_xy_lat'], map_list['map_xy_lon']

def get_xy(lat, lon, alt):
    e, n, u = pm.geodetic2enu(lat, lon, alt, lat_00, lon_00, alt_00)

    return e, n, u


def main():
    rospy.init_node('GPS_Converter', anonymous=False)

    gpsconv = GPS_Converter()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        gpsconv.enuPositionPublisher()
        rate.sleep()
        #rospy.sleep(0.05) # 1 / Hz
    rospy.spin()

if __name__ == '__main__':
    main()

