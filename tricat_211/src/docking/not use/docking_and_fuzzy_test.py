#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import math
import time
import numpy as np
import pymap3d as pm

from tricat_211.msg import HeadingAngle, Control, DockingPrint
from std_msgs.msg import UInt16, String
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount 
    #if error, 'darknet_ros_msg.msg'
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Point

import skfuzzy as fuzz
from skfuzzy import control as ctrl

####checklist
# 1) confidence 조정
# 2) GPS 변환 잘 되는가
# 3) 장애물 잘 피하는가
# 4) 퍼지와 합치기 - 잘 작동하나
# 5) PID 넣을까


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
        docking_zone_gps = self.docking_config['docking_zone_gps']
        lat_00, lon_00, alt_00 = self.docking_config['map_00_lat'], self.docking_config['map_00_lon'], self.docking_config['map_00_alt']
        self.docking_zone_xy = pm.geodetic2enu(docking_zone_gps[0], docking_zone_gps[1], docking_zone_gps[2], lat_00, lon_00, alt_00)[0:2]
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
        if (self.boat_x - self.docking_zone_xy[0])**2 + (self.boat_y - self.docking_zone_xy[1])**2 <= self.docking_zone_range**2:
            return True
        else:
            return False

    def OPTIMAL_DIRECTION(self, b):
        dx = self.docking_zone_xy[0] - self.boat_x
        dy = self.docking_zone_xy[1] - self.boat_y
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
        self.dist_to_goal = math.hypot(self.boat_x - self.docking_zone_xy[0], self.boat_y - self.docking_zone_xy[1])
        
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
        
        target_servo['RRRR'] = fuzz.trimf(target_servo.universe, [-30, -30, -23])
        target_servo['RRR'] = fuzz.trimf(target_servo.universe, [-26, -19, -12])
        target_servo['RR'] = fuzz.trimf(target_servo.universe, [-16, -11, -5])
        target_servo['R'] = fuzz.trimf(target_servo.universe, [-9, -4, 0])
        target_servo['N'] = fuzz.trimf(target_servo.universe, [0, 0, 0])
        target_servo['L'] = fuzz.trimf(target_servo.universe, [0, 4, 9])
        target_servo['LL'] = fuzz.trimf(target_servo.universe, [5, 11, 16])
        target_servo['LLL'] = fuzz.trimf(target_servo.universe, [12, 19, 26])
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

    def fuzzy_control_avoidance(self):
        self.idx = self.ranges.index(min(self.ranges))
        if min(self.ranges) < 4 and abs(LiDARDEG2DEG(RAD2DEG(self.angle_min + self.angle_increment * self.idx))) < 69:

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
            print("way xy : ",self.docking_zone_xy[0], self.docking_zone_xy[1])
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
            print("way xy : ",self.docking_zone_xy[0], self.docking_zone_xy[1])
            print("a, b, t : ", self.angle, self.bearing, self.OPTIMAL_DIRECTION(self.bearing))
            print("servo : " + turn, round(self.servo_control))
            print('-------------------------------------')


class Docking:
    def __init__(self):
        ### ROS Pub
        self.servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)
        self.print_node = rospy.Publisher("/print", DockingPrint, queue_size=10)

        ### Hyper-Parameters
        self.docking_config = rospy.get_param("docking_param")
        self.target_class = self.docking_config['target_class']

        ### Camera(mark)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.boxes_callback)
        rospy.Subscriber('/darknet_ros/detection_image', Image, self.image_callback)

        self.classes = []
        self.img_width = 0
        self.img_height = 0
        self.box_center_x = 0
        self.box_center_y = 0

        self.extra_span = self.docking_config['extra_span'] # 0~1
        self.close_distance = self.docking_config['close_distance'] #5m default -- too far??
        
        self.mark_state = 0 
            #0:None, 1:mark exist&not target, 2:target far, 3:target close
        self.target_loc = "None"
        
        ### LiDAR(obstacle)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.ranges = []
        self.ob_state = 0
            #0:None or far, 1:ob close

        ### Control
        self.servo = 94
        self.thruster = 1600

    def boxes_callback(self, msg):
        boxes = msg.bounding_boxes
        if len(boxes)==0:
            self.mark_state = 0
        else:
            self.classes = []
            for i in range(len(boxes)):
                self.classes.append(boxes[i].Class)

            if self.target_class in self.classes:
                self.box_size = (boxes[i].xmax-boxes[i].xmin) * (boxes[i].ymax-boxes[i].ymin)
                self.box_center_x = float((boxes[i].xmax+boxes[i].xmin)/2)
                self.box_center_y = float((boxes[i].ymax+boxes[i].ymin)/2)

                self.get_target_distance()
                if self.target_distance <= self.close_distance:
                    self.mark_state = 3
                else:
                    self.mark_state = 2
                self.target_center_x = abs(boxes[i].xmax + boxes[i].xmin)/2
            else:
                self.mark_state = 1
            
    def image_callback(self, msg):
        self.img_width = msg.width
        self.img_height = msg.height

    def lidar_callback(self, msg):
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.ranges = msg.ranges
        if len(self.ranges)!=0 and min(self.ranges) <= self.close_distance:
            idx = self.ranges.index(min(self.ranges))
            self.ob_angle = math.degrees(self.angle_min + self.angle_increment * self.idx)
            triangle_area = abs((min(self.ranges)**2) * math.tan(self.ob_angle))
            if triangle_area <= 35.8: #angle=20, d=2
                self.ob_state = 1
            else:
                self.ob_state = 0
        else:
            self.ob_state = 0

    def get_target_distance(self):
        self.target_distance = -(3E-05)*(self.box_size) + 2.3483    
        
    def state_judge(self):
        if self.ob_state==0:
            if self.mark_state==0:
                self.control_publish(thruster=1700)
            elif self.mark_state==1:
                self.control_publish(thruster=1400)
            else:
                self.target_tracking()
        else:
            self.obstacle_avoid()

    def target_tracking(self):
        line1 = self.img_width/2 - self.img_width*(1/6 + self.extra_span)
        line2 = self.img_width/2 + self.img_width*(1/6 + self.extra_span)
        servo_line1 = 94 - (8 + 48*self.extra_span)
        servo_line2 = 94 + (8 + 48*self.extra_span)
        if self.target_center_x < line1:
            angle_servo = self.map_func(self.target_center_x, line2, 1280, 94, servo_line2)
            self.target_loc = "Left"
        elif self.target_center_x > line2:
            angle_servo = self.map_func(self.target_center_x, 0, line1, 70, servo_line1)
            self.target_loc = "Right"
        else:
            angle_servo = self.map_func(self.target_center_x, line1, line2, servo_line1, servo_line2)
            self.target_loc = "Center"
        self.control_publish(servo=angle_servo)

    def obstacle_avoid(self):
        angle_servo = map_func(self.ob_angle, self.angle_min, self.angle_max, 70, 118)
            #check whether it works well
        self.control_publish(servo=angle_servo)

    def map_func(self, x, input_min, input_max, output_min, output_max):
        return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min

    def control_publish(self, servo=94, thruster=1600):
        self.servo = servo
        self.thruster = thruster
        control = Control()
        control.thruster = thruster
        control.servo = round(servo) #PID control add?
        self.thruster_pub.publish(control.thruster)
        self.servo_pub.publish(control.servo)


    def dock_finished(self):
        if self.mark_state == 0:
            return False
        else:
            return (self.target_distance <= 1) # finished-> True
    
    def reset_state(self):
        self.mark_state = 0 
        self.ob_state = 0
    
    def board_print(self):
        print_list = []

        # print("{0:*<27}".format("State"))
        # print("# State")
        # print("{0:^5} | {1:^15} | {2:^5}".format("", "target", ""))
        # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format("", "cls", "far", "not", "none"))
        # print("{0:-^5}-+-{1:-^15}-+-{2:-^5}".format("", "", ""))

        print_list.append("{0:^5} | {1:^15} | {2:^5}".format("", "target", ""))
        print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format("", "cls", "far", "not", "none"))
        print_list.append("{0:-^5}-+-{1:-^15}-+-{2:-^5}".format("", "", ""))

        if self.mark_state==0:
            if self.ob_state==0:
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format("ob", "", "", "", ""))
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format("none", "", "", "", "O"))

                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format("ob", "", "", "", ""))
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format("none", "", "", "", "O"))
            else:
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "", "O"))
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "", ""))

                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "", "O"))
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "", ""))
        elif self.mark_state==1:
            if self.ob_state==0:
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "", ""))
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "O", ""))
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "", ""))
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "O", ""))
            else:
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "O", ""))
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "", ""))
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "O", ""))
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "", ""))
        elif self.mark_state==2:
            if self.ob_state==0:
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "", ""))
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "O", "", ""))
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "", ""))
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "O", "", ""))
            else:
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "O", "", ""))
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "", ""))
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "O", "", ""))
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "", ""))
        elif self.mark_state==3:
            if self.ob_state==0:
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "", ""))
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "O", "", "", ""))
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "", ""))
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "O", "", "", ""))
            else:
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "O", "", "", ""))
                # print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "", ""))  
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "O", "", "", ""))
                print_list.append("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "", ""))

        if self.mark_state > 1:
            # print("# {0:10} : {1}".format("tgt dist", round(self.target_distance, 2)))
            # print("# {0:10} : {1}".format("tgt loc", self.target_loc))
            print_list.append("# {0:10} : {1}".format("tgt dist", round(self.target_distance, 2)))
            print_list.append("# {0:10} : {1}".format("tgt loc", self.target_loc))
        elif self.mark_state == 1:
            # print("# {0:10} : {1}".format("watching", self.classes))
            print_list.append("# {0:10} : {1}".format("watching", self.classes))
        if self.ob_state==1:
            # print("# {0:10} : {1}".format("ob dist", round(min(self.ranges), 2)))
            print_list.append("# {0:10} : {1}".format("ob dist", round(min(self.ranges), 2)))
        
        # print("# {0:10} : {1}".format("servo", self.servo))
        # print("# {0:10} : {1}".format("thruster", self.thruster))
        print_list.append("# {0:10} : {1}".format("servo", self.servo))
        print_list.append("# {0:10} : {1}".format("thruster", self.thruster))

        self.print_node.publish(print_list)


def main():   
    rospy.init_node('DockingMission', anonymous=False)
    rate = rospy.Rate(10)

    fuzz = Fuzzy()
    time.sleep(3)
    fuzz.fuzzy()

    while not rospy.is_shutdown():
        if fuzz.docking_zone_check():
            break
        else:
            if fuzz.arrival_check():
                if len(fuzz.goal_list) == 0:
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
    
    #rospy.spin()빼버림

    docking = Docking()
    print("{:=^40}".format(" Docking Start "))

    tick = 0
    docking.board_print()
    
    docking.state_judge()
    docking.reset_state()

    while not rospy.is_shutdown():
        if docking.dock_finished():
            print("{:=^40}".format(" Docking Finished "))
            break
        else:
            print("{:=^40}".format(" Not Finished "))
            if tick>=10:
                docking.board_print()
                tick=0
            docking.state_judge()
            docking.reset_state()
        tick+=1
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()




