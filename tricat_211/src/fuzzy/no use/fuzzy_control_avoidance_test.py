#!/usr/bin/env python

import rospy
import math
import pymap3d as pm
import numpy as np

from tricat_211.msg import Control, HeadingAngle  
from std_msgs.msg import UInt16
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

import skfuzzy as fuzz
from skfuzzy import control as ctrl        

class Goal:
    def __init__(self):
        self.boat_x = 0.0
        self.boat_y = 0.0

        self.target_servo_ang = None
        self.ob_distance = None
        self.ob_angle = None

        rospy.Subscriber("/bearing", HeadingAngle, self.heading_callback)
        rospy.Subscriber("/enu_position", Point, self.enu_callback)

        # self.map_list = rospy.get_param("map_dd")
        # self.lat_00, self.lon_00, self.alt_00 = self.map_list['map_00_lat'], self.map_list['map_00_lon'], self.map_list['map_00_alt']

        # self.waypoints = rospy.get_param("waypoint_List/waypoints")
        # self.way_list_gps = np.empty((0,3), float)
        # for i in range(len(self.waypoints)):
            # self.way_list_gps = np.append(self.way_list_gps, np.array([self.waypoints[i]]), axis=0)
        
        # self.way_list_gps = self.way_list_gps.astype(np.float64)

        # self.goal_list = self.get_xy(self.way_list_gps)

        # self.goal_range = rospy.get_param("waypoint_distance_tolerance")

        # self.goal_x = self.goal_list[0][0]
        # self.goal_y = self.goal_list[0][1]

        self.angle = 0.0
        self.target_angle = 0.0
        self.bearing = 0.0

        ## PID 
        self.init_servo = 94

        self.servo_control = 0

        # self.kp_servo = rospy.get_param("kp_servo")

        self.rate = 20

        # self.thruster_power = rospy.get_param("thruster_power")
    
        # self.Servo_pub = rospy.Publisher("/Servo", Float32, queue_size=10)
        # self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

    def heading_callback(self, data):
        self.bearing = data.bearing

    def enu_callback(self, data):
        self.boat_x = data.x  # East
        self.boat_y = data.y  # North
        #self.z = data.z
    '''
    def get_xy(self, points):
        way_list = np.zeros_like(points)
        for i in range(len(points)):
            way_list[i][0], way_list[i][1] , way_list[i][2] = \
                pm.geodetic2enu(points[i][0], points[i][1], points[i][2], \
                 self.lat_00, self.lon_00, self.alt_00)
        way_list = np.delete(way_list, 2, axis=1)
        return way_list



    def target(self):
        self.angle = math.atan2((self.goal_x - self.boat_x), (self.goal_y - self.boat_y))
        self.angle = self.angle * 180 / math.pi
        self.target_angle = self.angle - self.bearing
        return self.target_angle

    def set_next_point(self):
        self.goal_list = np.delete(self.goal_list, 0, axis = 0)

    def arrival_check(self):
        self.dist_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)
        
        if self.dist_to_goal <= self.goal_range:
            return True
        else:
            return False

    def prt(self):
        print("distance : ", self.dist_to_goal)
        print("my xy : ",self.boat_x, self.boat_y)
        print("way xy : ",self.goal_x, self.goal_y)
        print("bearing, target angle : ", self.bearing, self.target())
        print("servo : ", round(self.servo_pid_controller(),1))
        print('-------------------------------------')
    
    def servo_pid_controller(self):
        self.error_angle = self.target()

        self.cp_servo = self.kp_servo * self.error_angle

        servo_pd = -(self.cp_servo)
        self.servo_control = self.init_servo + servo_pd

        if self.servo_control > 118: #94+24
            self.servo_control = 118
        elif self.servo_control < 70:# 94-24
            self.servo_control = 70
        else:
            pass

        return self.servo_control

    def control_publisher(self):
        output_msg = Control()
        output_msg.thruster = self.thruster_power  #  param thruster value
        output_msg.servo = round(self.servo_pid_controller(), 1)
        self.thruster_pub.publish(output_msg.thruster)
        self.Servo_pub.publish(output_msg.servo)
    '''

    def fuzzy(self):
        distance = ctrl.Antecedent(np.arange(0, 4, 0.1), 'distance')
        angle = ctrl.Antecedent(np.arange(-50, 50, 1), 'angle')
        target_servo = ctrl.Consequent(np.arange(-30, 30, 1), 'target_servo')

        distance['ED'] = fuzz.trapmf(distance.universe, [0, 0, 1, 2])
        distance['D'] = fuzz.trimf(distance.universe, [1, 2, 3])
        distance['W'] = fuzz.trimf(distance.universe, [2, 3, 4])
        distance['B'] = fuzz.trimf(distance.universe, [3, 4, 4])

        # distance.automf(6)
        # angle.automf(6)
        angle['NL'] = fuzz.trapmf(angle.universe, [-50, -40, -30, -20])
        angle['NM'] = fuzz.trapmf(angle.universe, [-40, -30, -20, -10])
        angle['NS'] = fuzz.trimf(angle.universe, [-25, -1, 0])
        angle['PS'] = fuzz.trimf(angle.universe, [-1, 0, 25])
        angle['PM'] = fuzz.trimf(angle.universe, [15, 25, 40])
        angle['PL'] = fuzz.trimf(angle.universe, [30, 40, 50])
        
        #print('target_angle universe :',target_angle.universe)
        target_servo['RRRR'] = fuzz.trimf(target_servo.universe, [-30, -30, -23])
        target_servo['RRR'] = fuzz.trimf(target_servo.universe, [-26, -19, -12])
        target_servo['RR'] = fuzz.trimf(target_servo.universe, [-16, -11, -5])
        target_servo['R'] = fuzz.trimf(target_servo.universe, [-9, -4, 0])
        target_servo['N'] = fuzz.trimf(target_servo.universe, [0, 0, 0])
        target_servo['L'] = fuzz.trimf(target_servo.universe, [0, 4, 9])
        target_servo['LL'] = fuzz.trimf(target_servo.universe, [5, 11, 16])
        target_servo['LLL'] = fuzz.trimf(target_servo.universe, [12, 19, 26])
        target_servo['LLLL'] = fuzz.trimf(target_servo.universe, [23, 30, 30])

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



    def fuzzy_control_avoidance(self):
        
        self.ob_distance = input("Input near ob distance\n")
        self.ob_angle = input("Input near ob angle\n")

        self.target_servo_ang.input['distance'] = float(self.ob_distance)
        self.target_servo_ang.input['angle'] = float(self.ob_angle)

        self.target_servo_ang.compute()
        print('target servo angle output:',self.target_servo_ang.output['target_servo'])
    

def main():
    rospy.init_node('fuzzy_ctrl', anonymous=False)

    goal = Goal()

    goal.fuzzy()
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        goal.fuzzy_control_avoidance()

        # if goal.arrival_check():
        #     if len(goal.goal_list) == 0:
        #         break  #arrived final goal
        #     elif len(goal.goal_list) == 1:
        #         goal.goal_x = goal.goal_list[0][0]
        #         goal.goal_y = goal.goal_list[0][1]
        #         goal.set_next_point()
        #     else:
        #         goal.set_next_point()
        #         goal.goal_x = goal.goal_list[0][0]
        #         goal.goal_y = goal.goal_list[0][1]
        #goal.target()
        #pid.servo_pid_controller()
        # goal.control_publisher()
        # goal.prt()
        
        rate.sleep()
        
    rospy.spin()


if __name__ == '__main__':
    main()