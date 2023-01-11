#!/usr/bin/env python

import rospy
import math
import pymap3d as pm
import numpy as np
import time

from tricat_211.msg import Control, HeadingAngle  
from std_msgs.msg import UInt16
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu

import skfuzzy as fuzz
from skfuzzy import control as ctrl        

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

class Fuzzy:
    def __init__(self):
        self.boat_x = 0.0
        self.boat_y = 0.0

        rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback)
        rospy.Subscriber("/bearing", HeadingAngle, self.heading_callback)
        rospy.Subscriber("/enu_position", Point, self.enu_callback)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        ## ENU & Waypoint List
        self.map_list = rospy.get_param("map_dd")
        self.lat_00, self.lon_00, self.alt_00 = self.map_list['map_00_lat'], self.map_list['map_00_lon'], self.map_list['map_00_alt']

        self.waypoints = rospy.get_param("waypoint_List/waypoints")
        self.way_list_gps = np.empty((0,3), float)
        for i in range(len(self.waypoints)):
            self.way_list_gps = np.append(self.way_list_gps, np.array([self.waypoints[i]]), axis=0)
        #self.way_list_gps = self.way_list_gps.astype(np.float64)
        self.goal_list = self.get_xy(self.way_list_gps) # ENU way list
        self.goal_x = self.goal_list[0][0]
        self.goal_y = self.goal_list[0][1]
        
        self.goal_range = rospy.get_param("goal_range")

        ## Direction Search
        self.angle = 0.0
        self.bearing = 0.0

        ## LiDAR
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.time_increment = 0.0
        self.scan_time = 0.0
        self.range_min = 0.0
        self.range_max = 0.0
        self.ranges = []

        self.idx = 0

        self.fuzzy_servo_control = 0

        self.target_servo_ang = None

        ## PID 
        self.init_servo = 94
        self.servo_control = 94

        self.kp_servo = rospy.get_param("kp_servo")
        self.kd_servo = rospy.get_param("kd_servo")
        self.thruster_power = rospy.get_param("thruster_power")
    
        self.Servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

        self.servo_pd = 0

    def yaw_rate_callback(self, data):
        self.yaw_rate = data.angular_velocity.z  # yaw_rate [rad/s]

    def heading_callback(self, data):
        self.bearing = data.bearing

    def enu_callback(self, data):
        self.boat_x = data.x  # East
        self.boat_y = data.y  # North
        #self.z = data.z
    
    def lidar_callback(self, data):
        self.angle_min = data.angle_min
        #self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        #self.time_increment = data.time_increment
        #self.scan_time = data.scan_time
        #self.range_min = data.range_min
        #self.range_max = data.range_max
        self.ranges = data.ranges  # list

    def get_xy(self, points):
        way_list = np.zeros_like(points)
        for i in range(len(points)):
            way_list[i][0], way_list[i][1] , way_list[i][2] = \
                pm.geodetic2enu(points[i][0], points[i][1], points[i][2], \
                 self.lat_00, self.lon_00, self.alt_00)
        way_list = np.delete(way_list, 2, axis=1) # axis z delete
        return way_list

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
        # P ctrl
        error_angle = self.target() # deg

        # D ctrl
        yaw_rate = RAD2DEG(self.yaw_rate) # deg/s

        cp_servo = self.kp_servo * error_angle
        cd_servo = self.kd_servo * -yaw_rate

        self.servo_pd = -(cp_servo + cd_servo)
        # self.servo_control = self.servo_control + servo_pd
        self.servo_control = self.init_servo + self.servo_pd

        if self.servo_control > 94+25: #94+24
            self.servo_control = 94+25
        elif self.servo_control < 94-25:
            self.servo_control = 94-25
        else:
            pass

        return self.servo_control

    def control_publisher(self):
        output_msg = Control()
        output_msg.thruster = self.thruster_power  #  param thruster value
        if self.fuzzy_control_avoidance():
            output_msg.servo = 94 + self.fuzzy_servo_control
        else:
            output_msg.servo = round(self.servo_pid_controller())
        self.thruster_pub.publish(output_msg.thruster)
        self.Servo_pub.publish(output_msg.servo)

    def fuzzy(self):
        distance = ctrl.Antecedent(np.arange(0, 4, 0.1), 'distance')
        angle = ctrl.Antecedent(np.arange(-50, 50, 1), 'angle')
        target_servo = ctrl.Consequent(np.arange(-30, 30, 1), 'target_servo')

        distance['ED'] = fuzz.trapmf(distance.universe, [0, 0, 1, 2])
        distance['D'] = fuzz.trimf(distance.universe, [1, 2, 3])
        distance['W'] = fuzz.trimf(distance.universe, [2, 3, 4])
        distance['B'] = fuzz.trimf(distance.universe, [3, 4, 4])

        angle['NL'] = fuzz.trapmf(angle.universe, [-70, -40, -30, -20])
        angle['NM'] = fuzz.trapmf(angle.universe, [-40, -30, -20, -10])
        angle['NS'] = fuzz.trimf(angle.universe, [-25, 0, 1])
        angle['PS'] = fuzz.trimf(angle.universe, [0, 1, 25])
        angle['PM'] = fuzz.trimf(angle.universe, [15, 25, 40])
        angle['PL'] = fuzz.trimf(angle.universe, [30, 40, 70])
        
        target_servo['RRRR'] = fuzz.trimf(target_servo.universe, [-30, -30, -23])
        target_servo['RRR'] = fuzz.trimf(target_servo.universe, [-26, -20, -13])
        target_servo['RR'] = fuzz.trimf(target_servo.universe, [-18, -13, -7])
        target_servo['R'] = fuzz.trimf(target_servo.universe, [-11, -6, 0])
        target_servo['N'] = fuzz.trimf(target_servo.universe, [0, 0, 0])
        target_servo['L'] = fuzz.trimf(target_servo.universe, [0, 6, 11])
        target_servo['LL'] = fuzz.trimf(target_servo.universe, [7, 13, 18])
        target_servo['LLL'] = fuzz.trimf(target_servo.universe, [13, 20, 26])
        target_servo['LLLL'] = fuzz.trimf(target_servo.universe, [23, 30, 30])

        #ob_distance = min(self.ranges)
        #ob_angle = LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * self.idx))

        rule_ED_NL = ctrl.Rule(distance['ED'] & angle['NL'], target_servo['RR'])
        rule_ED_NM = ctrl.Rule(distance['ED'] & angle['NM'], target_servo['RRR'])
        rule_ED_NS = ctrl.Rule(distance['ED'] & angle['NS'], target_servo['RRRR'])
        rule_ED_PS = ctrl.Rule(distance['ED'] & angle['PS'], target_servo['LLLL'])
        rule_ED_PM = ctrl.Rule(distance['ED'] & angle['PM'], target_servo['LLL'])
        rule_ED_PL = ctrl.Rule(distance['ED'] & angle['PL'], target_servo['LL'])

        rule_D_NL = ctrl.Rule(distance['D'] & angle['NL'], target_servo['R'])
        rule_D_NM = ctrl.Rule(distance['D'] & angle['NM'], target_servo['RR'])
        rule_D_NS = ctrl.Rule(distance['D'] & angle['NS'], target_servo['RRR'])
        rule_D_PS = ctrl.Rule(distance['D'] & angle['PS'], target_servo['LLL'])
        rule_D_PM = ctrl.Rule(distance['D'] & angle['PM'], target_servo['LL'])
        rule_D_PL = ctrl.Rule(distance['D'] & angle['PL'], target_servo['L'])

        rule_W_NL = ctrl.Rule(distance['W'] & angle['NL'], target_servo['N'])
        rule_W_NM = ctrl.Rule(distance['W'] & angle['NM'], target_servo['R'])
        rule_W_NS = ctrl.Rule(distance['W'] & angle['NS'], target_servo['RR'])
        rule_W_PS = ctrl.Rule(distance['W'] & angle['PS'], target_servo['LL'])
        rule_W_PM = ctrl.Rule(distance['W'] & angle['PM'], target_servo['L'])
        rule_W_PL = ctrl.Rule(distance['W'] & angle['PL'], target_servo['N']) 

        rule_B_NL = ctrl.Rule(distance['B'] & angle['NL'], target_servo['N'])
        rule_B_NM = ctrl.Rule(distance['B'] & angle['NM'], target_servo['N'])
        rule_B_NS = ctrl.Rule(distance['B'] & angle['NS'], target_servo['R'])
        rule_B_PS = ctrl.Rule(distance['B'] & angle['PS'], target_servo['L'])
        rule_B_PM = ctrl.Rule(distance['B'] & angle['PM'], target_servo['N'])
        rule_B_PL = ctrl.Rule(distance['B'] & angle['PL'], target_servo['N'])

        target_servo_ctrl = ctrl.ControlSystem([rule_ED_NL, rule_ED_NM, rule_ED_NS, rule_ED_PL, rule_ED_PM, rule_ED_PS, \
            rule_D_NL, rule_D_NM, rule_D_NS, rule_D_PL, rule_D_PM, rule_D_PS, \
            rule_W_NL, rule_W_NM, rule_W_NS, rule_W_PL, rule_W_PM, rule_W_PS, \
            rule_B_NL, rule_B_NM, rule_B_NS, rule_B_PL, rule_B_PM, rule_B_PS])
        self.target_servo_ang = ctrl.ControlSystemSimulation(target_servo_ctrl)

        # self.target_servo_ang.input['distance'] = float(min(self.ranges))
        # self.target_servo_ang.input['angle'] = float(LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * self.idx)))

        # self.target_servo_ang.compute()

    def fuzzy_control_avoidance(self):
        self.idx = self.ranges.index(min(self.ranges)) # min d point data idx
        #abs(LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * 0))) :
        #for i in range(len(self.ranges)):
            #if abs(LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * i))) < 1:
                #print(i)
        if min(self.ranges) < 2.5 and abs(LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * self.idx))) < 70:
        #     if abs(LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * self.idx))) > 45 and self.ranges[717] - min(self.ranges) < 0.5:
        #         print("except")
        #         self.target_servo_ang.input['distance'] = float(self.ranges[717]) #718 leftturn 717 right??
        #         self.target_servo_ang.input['angle'] = float(LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * 717)))
        #     else:
            self.target_servo_ang.input['distance'] = float(min(self.ranges))
            self.target_servo_ang.input['angle'] = float(LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * self.idx)))

            self.target_servo_ang.compute()

            self.fuzzy_servo_control = int(self.target_servo_ang.output['target_servo'])

            return True
        else:
            return False

    def prt(self):
        if self.fuzzy_control_avoidance():
            if self.fuzzy_servo_control > 3: # left turn
                turn = "left"
            elif self.fuzzy_servo_control < -3: # right turn
                turn = "right"
            else:
                turn = "mid"
            print("--FUZZY ON--")
            print("distance : ", self.dist_to_goal)
            print("my xy : ",self.boat_x, self.boat_y)
            print("way xy : ",self.goal_x, self.goal_y)
            print("near_obstacle d , ang ", min(self.ranges), LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * self.idx)))
            print("servo : " + turn,self.fuzzy_servo_control)
            print('-------------------------------------')
        else:
            if self.servo_control > 94+3: # left turn
                turn = "left"
            elif self.servo_control < 94-3: # right turn
                turn = "right"
            else:
                turn = "mid"
            print("--FUZZY OFF--")
            print("distance : ", self.dist_to_goal)
            print("my xy : ",self.boat_x, self.boat_y)
            print("way xy : ",self.goal_x, self.goal_y)
            print("a, b, t : ", self.angle, self.bearing, self.OPTIMAL_DIRECTION(self.bearing))
            print("near_obstacle d , ang ", min(self.ranges), LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * self.idx)))
            print("servo : " + turn, self.servo_pd)
            print('-------------------------------------')


def main():
    rospy.init_node('fuzzy_ctrl', anonymous=False)

    fuzz = Fuzzy()
    
    rate = rospy.Rate(10)
    time.sleep(3)
    fuzz.fuzzy()
    
    while not rospy.is_shutdown():
        if fuzz.arrival_check():
            if len(fuzz.goal_list) == 0:
                fuzz.thruster_pub.publish(1500)
                fuzz.Servo_pub.publish(94)
                #rospy.on_shutdown()
                print("arrived goal")
                break  #arrived final goal
            elif len(fuzz.goal_list) == 1:
                fuzz.goal_x = fuzz.goal_list[0][0]
                fuzz.goal_y = fuzz.goal_list[0][1]
                fuzz.set_next_point()
            else:
                fuzz.set_next_point()
                fuzz.goal_x = fuzz.goal_list[0][0]
                fuzz.goal_y = fuzz.goal_list[0][1]

        fuzz.control_publisher()
        fuzz.prt()
        
        rate.sleep()
        
    rospy.spin()


if __name__ == '__main__':
    main()