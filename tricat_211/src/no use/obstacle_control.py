#!/usr/bin/env python

import rospy
import math
import pymap3d as pm
import numpy as np

from tricat_211.msg import Control, HeadingAngle  
from std_msgs.msg import UInt16
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]

def second_largest_number(arr):
    second = largest = -float('inf') 
    
    for n in arr:
        if n > largest:
            second = largest
            largest = n
        elif second < n < largest:
            second = n

    return second

def RAD2DEG(x):
    return x * 180. / math.pi

def DEG2RAD(x):
    return x / 180. * math.pi

def LiDARDEG2DEG(x):
    if -180 <= x < 0:
        x = -(180 + x)
    elif 0 <= x <= 180:
        x = 180 - x
    return x

class Goal:
    def __init__(self):
        self.t = 0.0

        self.boat_x = 0.0
        self.boat_y = 0.0

        rospy.Subscriber("/bearing", HeadingAngle, self.heading_callback)
        rospy.Subscriber("/enu_position", Point, self.enu_callback)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        self.map_list = rospy.get_param("map_dd")
        self.lat_00, self.lon_00, self.alt_00 = self.map_list['map_00_lat'], self.map_list['map_00_lon'], self.map_list['map_00_alt']

        self.waypoints = rospy.get_param("waypoint_List/waypoints")
        self.way_list_gps = np.empty((0,3), float)
        for i in range(len(self.waypoints)):
            self.way_list_gps = np.append(self.way_list_gps, np.array([self.waypoints[i]]), axis=0)
        
        self.way_list_gps = self.way_list_gps.astype(np.float64)

        self.goal_list = self.get_xy(self.way_list_gps)

        self.goal_range = rospy.get_param("goal_range")

        self.goal_x = self.goal_list[0][0]
        self.goal_y = self.goal_list[0][1]

        self.angle = 0.0
        self.target_angle = 0.0
        self.bearing = 0.0

        ## PID 
        self.init_servo = 93

        self.servo_control = 0

        self.kp_servo = rospy.get_param("kp_servo")

        self.rate = 20

        self.thruster_power = rospy.get_param("thruster_power")
    
        self.Servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

        ## local path plan
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.time_increment = 0.0
        self.scan_time = 0.0
        self.range_min = 0.0
        self.range_max = 0.0
        self.ranges = [] # list

        self.far_angle = 0.0
        self.avoid_angle = 0.0

        self.LPP_kp_servo = rospy.get_param("LPP_kp_servo")

        self.safe_radius = rospy.get_param("safe_radius")


    def test(self):
        print("asdf", self.boat_x)

    def get_xy(self, points):
        way_list = np.zeros_like(points)
        for i in range(len(points)):
            way_list[i][0], way_list[i][1] , way_list[i][2] = \
                pm.geodetic2enu(points[i][0], points[i][1], points[i][2], \
                 self.lat_00, self.lon_00, self.alt_00)
        way_list = np.delete(way_list, 2, axis=1)
        return way_list

    def heading_callback(self, data):
        self.bearing = data.bearing

    def enu_callback(self, data):
        self.boat_x = data.x  # East
        self.boat_y = data.y  # North
        #self.z = data.z

    def OPTIMAL_DIRECTION(self, b):
        dx = self.goal_x - self.boat_x
        dy = self.goal_y - self.boat_y
        self.angle = RAD2DEG(math.atan2(dx, dy))

        if dx >= 0 and dy >= 0: # Quadrant 1
            if b >= 0 : # right bearing
                self.t = self.angle - b
            elif b < 0: # left bearing
                if abs(b) < (180 - self.angle):
                    self.t = self.angle - b
                elif abs(b) >= (180 - self.angle):
                    self.t = -(360 - self.angle + b)

        elif dx < 0 and dy >= 0: # Quadrant 2
            if b >= 0 :
                if b < 180 + self.angle:
                    self.t = self.angle - b
                elif b >= 180 + self.angle:
                    self.t = 360 + self.angle - b
            elif b < 0:
                self.t = self.angle - b
                
        elif dx < 0 and dy < 0: # Quadrant 3
            if b >= 0 :
                if b < 180 + self.angle:
                    self.t = (self.angle - b)
                elif b >= 180 + self.angle:
                    self.t = 360 + (self.angle - b)
            elif b < 0:
                self.t = (self.angle - b)

        elif dx >= 0 and dy < 0: # Quadrant 4
            if b >= 0 :
                self.t = (self.angle - b)
            elif b < 0:
                if abs(b) < 180 - self.angle:
                    self.t = (self.angle - b)
                elif abs(b) >= 180 - self.angle:
                    self.t = self.angle - b - 360          

        return self.t


    def target(self):      

        return self.OPTIMAL_DIRECTION(self.bearing)

    def set_next_point(self):
        self.goal_list = np.delete(self.goal_list, 0, axis = 0)

    def arrival_check(self):
        self.dist_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)
        
        if self.dist_to_goal <= self.goal_range:
            return True
        else:
            return False
    
    def servo_pid_controller(self):
        self.error_angle = self.target()

        self.cp_servo = self.kp_servo * self.error_angle

        servo_pd = -(self.cp_servo)
        self.servo_control = self.init_servo + servo_pd

        if self.servo_control > 117: # 93+24 left turn
            self.servo_control = 117
        elif self.servo_control < 69: # 93-24 right turn
            self.servo_control = 69
        else:
            pass

        return self.servo_control

    def control_publisher(self):
        output_msg = Control()
        output_msg.thruster = self.thruster_power  #  param thruster value
        if self.LPP():
            output_msg.servo = round(self.LPP_Servo_Control())
        else:
            output_msg.servo = round(self.servo_pid_controller())
        self.thruster_pub.publish(output_msg.thruster)
        self.Servo_pub.publish(output_msg.servo)

    def lidar_callback(self, data):
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        self.time_increment = data.time_increment
        self.scan_time = data.scan_time
        self.range_min = data.range_min
        self.range_max = data.range_max
        self.ranges = data.ranges  # list
        
    def LPP(self):
        maaxx = 360.0
        idx = 0
        
        if len(self.ranges) == 0:
            pass
        elif min(self.ranges) < self.safe_radius: # x m in scan //LPP start
            for i in range(len(self.ranges)): # all scan data
                if 5 < self.ranges[i] : #< 25: # safe distance scan data
                    self.avoid_angle = LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * i))
                    if abs(self.OPTIMAL_DIRECTION(self.avoid_angle)) <= maaxx: #abs(self.avoid_angle - self.angle) <= maaxx:
                        maaxx = abs(self.OPTIMAL_DIRECTION(self.avoid_angle)) # abs(self.avoid_angle - self.angle)
                        idx = i
                


            if self.ranges[idx] == float("inf"):
                asd = [idx-4, idx-3, idx-2, idx-1, idx, idx+1, idx+2, idx+3, idx+4]
                
                a = []
                
                for j in asd:
                    a.append((j, LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * j))))
                    
                a.sort(key = lambda x: abs(x[1]))
                idx = a[0][0]
            else:
                pass

            self.target_avoid_angle = LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * idx))

            i_ob = self.ranges.index(min(self.ranges))
            ob_angle = LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * i_ob))
            ob2goal_angle = self.OPTIMAL_DIRECTION(ob_angle)

           
            if abs(ob2goal_angle) < 20: 
                if ob_angle <0:
                    self.target_avoid_angle = 93 - 8
                else:
                    self.target_avoid_angle = 93 + 8 
            
            # if 0 <= self.target_avoid_angle < 180:
            #     self.target_avoid_angle = - self.target_avoid_angle
            # elif 180 <= self.target_avoid_angle < 360:
            #     self.target_avoid_angle = abs(self.target_avoid_angle - 360)

            print("target avoid angle", self.target_avoid_angle)
            #self.far_angle = RAD2DEG(self.angle_min + self.angle_increment * self.ranges.index(second_largest_number(self.ranges))) # most 2nd far scan i
            #print(self.far_angle)
            return True
        else:
            return False
    
    def LPP_Servo_Control(self):
        self.error_angle = self.target_avoid_angle

        self.cp_servo = self.LPP_kp_servo * self.error_angle

        servo_pd = -(self.cp_servo)
        self.servo_control = self.init_servo + servo_pd

        if self.servo_control > 93+24: #93+20 left turn
            self.servo_control = 93+24
        elif self.servo_control < 93-24:# 93-20 right turn
            self.servo_control = 93-24
        else:
            pass

        return self.servo_control

    def prt(self):
        if self.servo_control > 93+3: # left turn
            turn = "left"
        elif self.servo_control < 93-3: # right turn
            turn = "right"
        else:
            turn = "mid"
        print("distance : ", self.dist_to_goal)
        print("my xy : ",self.boat_x, self.boat_y)
        print("way xy : ",self.goal_x, self.goal_y)
        print("bearing, target angle : ", self.bearing, self.target())
        print("servo : " + turn, round(self.servo_control))
        print('-------------------------------------')


def main():
    rospy.init_node('Obsacle_P_controller', anonymous=False)

    goal = Goal()
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        #goal.test()
        if goal.arrival_check():
            if len(goal.goal_list) == 0:
                goal.thruster_pub.publish(1500)
                goal.Servo_pub.publish(93)
                rospy.on_shutdown()
                print("arrived final goal")
                #break  #arrived final goal
            elif len(goal.goal_list) == 1:
                goal.goal_x = goal.goal_list[0][0]
                goal.goal_y = goal.goal_list[0][1]
                goal.set_next_point()
            else:
                goal.set_next_point()
                goal.goal_x = goal.goal_list[0][0]
                goal.goal_y = goal.goal_list[0][1]

        goal.control_publisher()

        goal.prt()
        
        rate.sleep()
        
    rospy.spin()


if __name__ == '__main__':
    main()