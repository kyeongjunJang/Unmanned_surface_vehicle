#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import math
import time
import numpy as np
import pymap3d as pm
from cv_bridge import CvBridge, CvBridgeError 

from tricat_211.msg import HeadingAngle, Control, DockingPrint
from std_msgs.msg import UInt16, String
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Point

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

        self.docking_config = rospy.get_param("docking_param")
        docking_zone_gps1 = self.docking_config['docking_zone_gps1']
        docking_zone_gps2 = self.docking_config['docking_zone_gps2']
        lat_00, lon_00, alt_00 = self.docking_config['map_00_lat'], self.docking_config['map_00_lon'], self.docking_config['map_00_alt']
        self.docking_zone_xy1 = pm.geodetic2enu(docking_zone_gps1[0], docking_zone_gps1[1], docking_zone_gps1[2], lat_00, lon_00, alt_00)[0:2]
        self.docking_zone_xy2 = pm.geodetic2enu(docking_zone_gps2[0], docking_zone_gps2[1], docking_zone_gps2[2], lat_00, lon_00, alt_00)[0:2]
        self.next_docking_xy = self.docking_zone_xy1
        self.docking_zone_range = self.docking_config['docking_zone_range']

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

        self.dist_to_goal = 10000

        self.fuzzy_servo_control = 0

        self.target_servo_ang = None

        ## PID 
        self.init_servo = 93
        self.servo_control = 93

        self.kp_servo = rospy.get_param("kp_servo")
        self.kd_servo = rospy.get_param("kd_servo")
        self.thruster_power = rospy.get_param("thruster_power")
    
        self.Servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

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
        self.angle_increment = data.angle_increment
        self.ranges = data.ranges  # list

    def docking_zone_check(self):
        if (self.next_docking_xy[0] == self.docking_zone_xy1[0]):
            #아직 첫 번째 지점 통과 못함
            if (self.boat_x - self.docking_zone_xy1[0])**2 + (self.boat_y - self.docking_zone_xy1[1])**2 <= self.docking_zone_range**2:
                #첫번째 지점 통과함
                self.next_docking_xy = self.docking_zone_xy2
            return False
        else: # elif (self.next_docking_xy[0] == self.docking_zone_xy2[0]):
            if (self.boat_x - self.docking_zone_xy2[0])**2 + (self.boat_y - self.docking_zone_xy2[1])**2 <= self.docking_zone_range**2:
                #두 번째 지점 통과함
                return True
            else:
                return False

    def OPTIMAL_DIRECTION(self, b):
        dx = self.next_docking_xy[0] - self.boat_x
        dy = self.next_docking_xy[1] - self.boat_y
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

    def arrival_check(self):
        self.dist_to_goal = math.hypot(self.boat_x - self.next_docking_xy[0], self.boat_y - self.next_docking_xy[1])
        
        if self.dist_to_goal <= self.docking_zone_range:
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

        servo_pd = -(cp_servo + cd_servo)
        # self.servo_control = self.servo_control + servo_pd
        self.servo_control = self.init_servo + servo_pd

        if self.servo_control > 93+25: #94+24
            self.servo_control = 93+25
        elif self.servo_control < 93-25:
            self.servo_control = 93-25
        else:
            pass

        return self.servo_control

    def control_publisher(self):
        output_msg = Control()
        output_msg.thruster = self.thruster_power  #  param thruster value
        if self.fuzzy_control_avoidance():
            output_msg.servo = 93 + self.fuzzy_servo_control
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
        angle['NS'] = fuzz.trimf(angle.universe, [-25, -1, 0])
        angle['PS'] = fuzz.trimf(angle.universe, [-1, 0, 25])
        angle['PM'] = fuzz.trimf(angle.universe, [15, 25, 40])
        angle['PL'] = fuzz.trimf(angle.universe, [30, 40, 70])
        
        target_servo['RRRR'] = fuzz.trimf(target_servo.universe, [-30, -30, -22])
        target_servo['RRR'] = fuzz.trimf(target_servo.universe, [-24, -18, -12])
        target_servo['RR'] = fuzz.trimf(target_servo.universe, [-16, -11, -5])
        target_servo['R'] = fuzz.trimf(target_servo.universe, [-9, -4, 0])
        target_servo['N'] = fuzz.trimf(target_servo.universe, [0, 0, 0])
        target_servo['L'] = fuzz.trimf(target_servo.universe, [0, 4, 9])
        target_servo['LL'] = fuzz.trimf(target_servo.universe, [5, 11, 16])
        target_servo['LLL'] = fuzz.trimf(target_servo.universe, [12, 18, 24])
        target_servo['LLLL'] = fuzz.trimf(target_servo.universe, [22, 30, 30])

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

    def fuzzy_control_avoidance(self):
        self.idx = self.ranges.index(min(self.ranges))
        if min(self.ranges) < 2.8 and abs(LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * self.idx))) < 70:

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
            print("my xy : ",round(self.boat_x,3), round(self.boat_y, 3))
            print("way xy : ",self.next_docking_xy[0], self.next_docking_xy[1])
            print("near_obstacle d , ang ", min(self.ranges), LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * self.idx)))
            print("servo : " + turn,self.fuzzy_servo_control)
            print('-------------------------------------')
        else:
            if self.servo_control > 93+3: # left turn
                turn = "left"
            elif self.servo_control < 93-3: # right turn
                turn = "right"
            else:
                turn = "mid"
            print("--FUZZY OFF--")
            print("distance : ", self.dist_to_goal)
            print("my xy : ",self.boat_x, self.boat_y)
            print("way xy : ",self.next_docking_xy[0], self.next_docking_xy[1])
            print("a, b, t : ", self.angle, self.bearing, self.OPTIMAL_DIRECTION(self.bearing))
            print("servo : " + turn, round(self.servo_control))
            print('-------------------------------------')


######################### docking ###############################
class Docking:
    def __init__(self):
        ### ROS Pub
        self.servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)
        self.print_node = rospy.Publisher("/print", DockingPrint, queue_size=10)

        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.boxes_callback)
        rospy.Subscriber('/darknet_ros/detection_image', Image, self.image_callback)
        #rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.convert_depth_image)

        self.docking_config = self.docking_config = rospy.get_param("docking_param")
        self.target_class = self.docking_config['target_class']
        

        self.checked_mark = 0
        self.classes = []
        self.img_width = 0
        self.hopping_ing = 0
        self.mark_state = ""
        self.forward_or_backward = ""
        self.is_target_result = ""
        

    def image_callback(self, msg):
        self.img_width = msg.width

    def boxes_callback(self, msg):
        boxes = msg.bounding_boxes
        if len(boxes)==0:
            ##### 아주 느린 속도로 전진?????
            pass
        else:
            self.classes = []
            for i in range(len(boxes)):
                self.classes.append(boxes[i].Class)
            if self.target_class in self.classes:
                self.target_center_x = abs(boxes[i].xmax + boxes[i].xmin)/2

    # def checked_mark_update(self):
    #     self.checked_mark.append(1)

    def search_station(self):
        if self.checked_mark==0: #if len(self.checked_mark)==0:
            #아직 아무데도 안 감. 7->4번 가야 함
            return 1
        elif self.checked_mark==1: #len(self.checked_mark)==1:
            #첫 번째 확인 마침. 두 번째 스테이션 가야 함
            return 2
        elif self.checked_mark==2:
            #두 번째 확인 마침. 세 번째 스테이션 가야 함
            return 3
        else:
            return 4

    def is_target(self):
        while 1:
            if len(self.classes) > 0 :
                if self.target_class in self.classes:
                    # self.target_center_x = abs(boxes[i].xmax + boxes[i].xmin)/2
                    if self.is_center():
                        ####우리가 원하는 마크임 --- 전진 명령
                        self.go_forward()
                    else:
                        ### 마크는 맞지만 이 스테이션이 아님--- 후진 명령
                        # self.go_backward()
                        self.checked_mark = self.checked_mark + 1
                        print(self.checked_mark)
                        
                else:
                    # 우리 마크가 아님-------후진 명령
                    # self.go_backward()
                    self.checked_mark = self.checked_mark + 1
                    print(self.checked_mark)
                break
            # else: #
            #     continue #

    def is_center(self):
        line1 = self.img_width/2 - self.img_width/8 #왼쪽 가운데선
        line2 = self.img_width/2 + self.img_width/8 #오른쪽 가운데선
        pixel_from_center = self.target_center_x - self.img_width/2 #박스 중점 x축이 중앙으로부터 얼마나 떨어졌나. 왼쪽에 있으면 (-)값
        if self.target_center_x > line1 and self.target_center_x < line2:
            return True
        else:
            return False

    def go_backward(self):
        print("go backward")
        #pwm = 1400 (5~7 sec)
        start_time = time.time()
        end_time = time.time()
        delta_time = end_time - start_time

        self.forward_or_backward="go backward"
        
        while delta_time <= 4:
            control = Control()
            control.thruster = 1400
            control.servo = 93
            self.thruster_pub.publish(control.thruster)
            self.servo_pub.publish(control.servo)

            end_time = time.time()
            delta_time = end_time - start_time
        self.thruster_pub.publish(1500)
        self.servo_pub.publish(93)
        print("back move done")


    def go_forward(self):
        print("go forward")
        #pwm = 1700, servo = 93 (직진)
        start_time = time.time()
        end_time = time.time()
        delta_time = end_time - start_time
        
        self.forward_or_backward="go forward"

        while delta_time <= 10:
            control = Control()
            control.thruster = 1620
            control.servo = 93
            self.thruster_pub.publish(control.thruster)
            self.servo_pub.publish(control.servo)
            
            end_time = time.time()
            delta_time = end_time - start_time
        print("move done")
        #self.thruster_pub.publish(1500)
        #self.servo_pub.publish(93)

    def prt(self): ####main에 추가
        print_list = []
        #print_list.append("now gps: {0}".format(gps좌표))
            
        if self.checked_mark==0: 
            print_list.append("should move: point1")
        elif self.checked_mark==1: #len(self.checked_mark)==1:
            print_list.append("should move: point2")
        elif self.checked_mark==2:
            print_list.append("should move: point3")
        
        if self.hopping_ing ==0:
            print_list.append("hopping X")
        else:
            print_list.append("hopping O")
        
        print_list.append(self.mark_state)
        print_list.append(self.is_target_result)
        print_list.append(self.forward_or_backward)

        self.print_node.publish(print_list)

################## Hopping #############################
class Hopping:
    def __init__(self):
        self.boat_x = 0.0
        self.boat_y = 0.0

        rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback)
        rospy.Subscriber("/bearing", HeadingAngle, self.heading_callback)
        rospy.Subscriber("/enu_position", Point, self.enu_callback)

        ## ENU & Waypoint List
        self.map_list = rospy.get_param("map_dd")
        self.lat_00, self.lon_00, self.alt_00 = self.map_list['map_00_lat'], self.map_list['map_00_lon'], self.map_list['map_00_alt']

        # self.waypoints = rospy.get_param("waypoint_List/waypoints")
        # self.way_list_gps = np.empty((0,3), float)
        # for i in range(len(self.waypoints)):
        #     self.way_list_gps = np.append(self.way_list_gps, np.array([self.waypoints[i]]), axis=0)
        # #self.way_list_gps = self.way_list_gps.astype(np.float64)
        # self.goal_list = self.get_xy(self.way_list_gps) # ENU way list
        # self.goal_x = self.goal_list[0][0]
        # self.goal_y = self.goal_list[0][1]
        
        self.goal_range = rospy.get_param("goal_range")
        
        ## Direction Search
        self.angle = 0.0
        self.bearing = 0.0

        ## PID 
        self.init_servo = 93
        self.servo_control = 0
        self.errSum = 0.0
        self.yaw_rate = 0.0

        self.thrust = 0

        self.kp_servo = rospy.get_param("kp_servo")
        self.ki_servo = rospy.get_param("ki_servo")
        self.kd_servo = rospy.get_param("kd_servo")
        self.thruster_power = rospy.get_param("thruster_power")
        self.kp_distance = rospy.get_param("kp_distance")


        #### 호핑 goal list 먹이기
        self.isDone = False

        self.docking_config = rospy.get_param("docking_param")
        self.point_1 = self.docking_config['point_1']
        self.point_2 = self.docking_config['point_2']
        self.point_3 = self.docking_config['point_3']
        
        self.point_1_list = np.empty((0,3), float)
        for i in range(len(self.point_1)):
            self.point_1_list = np.append(self.point_1_list, np.array([self.point_1[i]]), axis=0)
        self.goal_list_1 = self.get_xy(self.point_1_list) # ENU way list
        

        self.point_2_list = np.empty((0,3), float)
        for i in range(len(self.point_2)):
            self.point_2_list = np.append(self.point_2_list, np.array([self.point_2[i]]), axis=0)
        self.goal_list_2 = self.get_xy(self.point_2_list) # ENU way list

        self.point_3_list = np.empty((0,3), float)
        for i in range(len(self.point_3)):
            self.point_3_list = np.append(self.point_3_list, np.array([self.point_3[i]]), axis=0)
        self.goal_list_3 = self.get_xy(self.point_3_list) # ENU way list

        self.goal_x = 0.0
        self.goal_y = 0.0
        
        self.servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

    def get_xy(self, points):
        way_list = np.zeros_like(points)
        for i in range(len(points)):
            way_list[i][0], way_list[i][1] , way_list[i][2] = \
                pm.geodetic2enu(points[i][0], points[i][1], points[i][2], \
                 self.lat_00, self.lon_00, self.alt_00)
        way_list = np.delete(way_list, 2, axis=1) # axis z delete
        return way_list

    def yaw_rate_callback(self, data):
        self.yaw_rate = data.angular_velocity.z  # yaw_rate [rad/s]

    def heading_callback(self, data):
        self.bearing = data.bearing

    def enu_callback(self, data):
        self.boat_x = data.x  # East
        self.boat_y = data.y  # North

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

    #####################추가##################
    def next_waypoint(self, state):
        if state==1:
            self.goal_list = self.goal_list_1
            print("station1")
        elif state==2:
            self.goal_list = self.goal_list_2
            print("station2")
        elif state==3:
            self.goal_list = self.goal_list_3
            print("station3")
        
        self.goal_x = self.goal_list[0][0]
        self.goal_y = self.goal_list[0][1]
        print("goal: ", self.goal_x, self.goal_y)

    def prt(self):
        if self.servo_control > 93+3: # left turn
            turn = "left"
        elif self.servo_control < 93-3: # right turn
            turn = "right"
        else:
            turn = "mid"
        print("distance, thruster : ", self.dist_to_goal, self.thrust)
        print("my xy : ",self.boat_x, self.boat_y)
        print("way xy : ",self.goal_x, self.goal_y)
        print("a, b, t : ", self.angle, self.bearing, self.OPTIMAL_DIRECTION(self.bearing))
        print("servo : " + turn, round(self.servo_control))
        print("errSum:", self.errSum)
        print('-------------------------------------')
    
    def servo_pid_controller(self):
        # P ctrl
        error_angle = self.target() # deg

        # I ctrl
        self.errSum += (error_angle * 0.1)
        
        if self.errSum > 90:
            self.errSum = 90
        elif self.errSum < -90:
            self.errSum = -90
        else:
            pass

        # D ctrl
        yaw_rate = RAD2DEG(self.yaw_rate) # deg/s


        cp_servo = self.kp_servo * error_angle
        ci_servo = self.ki_servo * self.errSum
        cd_servo = self.kd_servo * -yaw_rate

        servo_pid = -(cp_servo + ci_servo + cd_servo)
        self.servo_control = self.init_servo + servo_pid

        if self.servo_control > 93+45: #94+24
            self.servo_control = 93+45
        elif self.servo_control < 93-45:
            self.servo_control = 93-45
        else:
            pass

        return self.servo_control

    def control_publisher(self):
        ## propotional ctrl for thruster
        self.thrust = int(1590 + self.kp_distance * self.dist_to_goal)
        if self.thrust > self.thruster_power:
            self.thrust = self.thruster_power
        else:
            pass
        output_msg = Control()
        output_msg.thruster = self.thrust # self.thruster_power
        output_msg.servo = round(self.servo_pid_controller())
        self.thruster_pub.publish(output_msg.thruster)
        self.servo_pub.publish(output_msg.servo)


def main():
    rospy.init_node('DockingMission', anonymous=False)
    rate = rospy.Rate(10)

    fuzz = Fuzzy()
    time.sleep(3)
    fuzz.fuzzy()

    while not rospy.is_shutdown():
        if fuzz.docking_zone_check():
            break
        # else:
        #     if fuzz.arrival_check():
        #         if len(fuzz.goal_list) == 0:
        #             break  #arrived final goal
        #         elif len(fuzz.goal_list) == 1:
        #             fuzz.goal_x = fuzz.goal_list[0][0]
        #             fuzz.goal_y = fuzz.goal_list[0][1]
        #             fuzz.set_next_point()
        #         else:
        #             fuzz.set_next_point()
        #             fuzz.goal_x = fuzz.goal_list[0][0]
        #             fuzz.goal_y = fuzz.goal_list[0][1]

        fuzz.control_publisher()
        fuzz.prt()
        
        rate.sleep() 
    
 #waypoint2에 왔다고 가정--------------------
    docking = Docking()
    goal = Hopping()
    state = 0 #7->4가야 함=1, ...2, ...3

    state = docking.search_station()
    print("state : ", state)
    goal.next_waypoint(state)
    docking.prt()

    while not rospy.is_shutdown():
        if not goal.isDone:
            # print("hopping ------------")
            if goal.arrival_check():
                if len(goal.goal_list) == 0:
                    goal.thruster_pub.publish(1500)
                    goal.servo_pub.publish(93)
                    goal.isDone = True
                    #rospy.on_shutdown()
                    print("arrived station entry -> isDocking")
                elif len(goal.goal_list) == 1:
                    goal.goal_x = goal.goal_list[0][0]
                    goal.goal_y = goal.goal_list[0][1]
                    goal.set_next_point()
                    goal.errSum = 0.0 # error initialization
                    print("done second point")
                else:
                    goal.set_next_point()
                    goal.goal_x = goal.goal_list[0][0]
                    goal.goal_y = goal.goal_list[0][1]
                    goal.errSum = 0.0 # error initialization
                    print("done first point, go second")

            goal.control_publisher()
        else:
            print("target????")
            docking.is_target()
            print("search next station")
            state = docking.search_station()
            print("state : ", state)
            goal.next_waypoint(state)
            goal.isDone = False
        docking.prt()
        
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
