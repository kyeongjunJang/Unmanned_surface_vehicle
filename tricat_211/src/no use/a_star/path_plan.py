#!/usr/bin/env python

import numpy as np
import math
import sys
import rospy
import pymap3d as pm
import time

from std_msgs.msg import String, UInt16, Float32
from geometry_msgs.msg import Point
from tricat_211.msg import FilteredObstacles, ObstaclePoint, Control, HeadingAngle
from tricat_211.msg import Info, EnuObstacle, EnuTrajectory


class Obstacle:
    def __init__(self):
        rospy.Subscriber('/filtered_obstacles', FilteredObstacles, self.filtered_obstacle_callback)
        self.ob_list = np.empty((0, 3))

    def filtered_obstacle_callback(self, msg):
        self.ob_list = np.empty((0, 3)) #init
        filtered_obstacle_list = msg.obstacle
        for i in range(len(filtered_obstacle_list)):
            ob_x = filtered_obstacle_list[i].x
            ob_y = filtered_obstacle_list[i].y
            ob_theta = filtered_obstacle_list[i].theta
            self.ob_list = np.append(self.ob_list, np.array([[ob_x, ob_y, ob_theta]]), axis = 0)


class Boat:
    def __init__(self):
        self.cur_pos = np.zeros((0, 2))
        self.cur_heading = np.zeros((0, 2))
        self.bearing = 0.0

        rospy.Subscriber("/enu_position", Point, self.cur_pos_callback)
        rospy.Subscriber("/bearing", HeadingAngle, self.cur_bearing_callback)

    def cur_pos_callback(self, msg):
        self.cur_pos = np.array([msg.x, msg.y])
    
    def cur_bearing_callback(self, msg):
        self.bearing = msg.bearing #degree
        theta = math.radians(msg.bearing)
        self.cur_heading = np.array([math.sin(theta), math.cos(theta)])
        self.cur_heading /= math.sqrt((self.cur_heading[0] ** 2 + self.cur_heading[1] ** 2))

class PathPlanner:
    def __init__(self):
        ## ROS
        path_plan_config = rospy.get_param("path_plan")
        self.servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)
        self.info_pub = rospy.Publisher("/info", Info, queue_size=20)

        ## PID
        self.init_servo = 94
        self.kp_servo = path_plan_config["kp_servo"]
        self.rate = 20

        ## edge points
        self.origin_point = list(path_plan_config["origin_point"]) #x, y, z
        self.edge_points = self.points_coordinate(path_plan_config["edge_points"])

        ## boat
        self.boat = Boat()
        self.start_pos = self.boat.cur_pos
        self.start_heading = self.boat.cur_heading
        self.cur_pos = self.start_pos
        self.cur_heading = self.start_heading
        self.bearing = self.boat.bearing

        ## obstacles
        self.obstacle = Obstacle()
        self.obstacle_update()
        self.obstacle_search_range = path_plan_config["obstacle_search_range"]

        ## goal
        self.end_points = self.points_coordinate(path_plan_config["end_points"])
        self.next_goal = self.end_points[0]

        ## trajectory prediction
        self.predict_step = path_plan_config["predict_step"]
        self.predict_step_size = path_plan_config["predict_step_size"]
        self.g_value_rotate_gain_0 = path_plan_config["g_value_rotate_gain_0"]
        self.g_value_rotate_gain_30 = path_plan_config["g_value_rotate_gain_30"]
        self.g_value_rotate_gain_60 = path_plan_config["g_value_rotate_gain_60"]
        self.g_value_rotate_gain_90 = path_plan_config["g_value_rotate_gain_90"]
        self.h_value_gain = path_plan_config["h_value_gain"]
        self.past_path = np.array([self.start_pos])  #ckeck and compare with path_paln_simulation.py
        self.trajectory = np.zeros((1, 2))
        self.search_center_pub = []
        self.best_point_pub = []
        self.calc_point_num_pub = 0

        ## real moving
        self.arrival_range = path_plan_config["arrival_range"]
        self.thruster_power = path_plan_config["thruster_power"]
        self.rotate_angle = 0.0
        self.error_angle = 0.0
        self.servo_control = 0.0

    def points_coordinate(self, points):
        converted = np.zeros_like(points)
        for i in range(len(points)):
            converted[i][0], converted[i][1] , converted[i][2] = \
                pm.geodetic2enu(points[i][0], points[i][1], points[i][2], \
                    self.origin_point[0], self.origin_point[1], self.origin_point[2])
        converted = np.delete(converted, 2, axis=1)
        return converted

    def is_finished(self):
        dist_to_goal = math.sqrt((self.cur_pos[0] - self.next_goal[0]) ** 2 + (self.cur_pos[1] - self.next_goal[1]) ** 2)
        if dist_to_goal <= self.arrival_range:
            return True
        else:
            return False

    def make_trajectory(self):
        print("|{:-<82}+".format(" make_trajectory()"))
        search_center = self.cur_pos
        predict_heading = self.cur_heading
        self.trajectory = np.zeros((1, 2))

        for i in range(self.predict_step):
            print("| {0:<16} | {1:>8} , {2:>8} || {3:<16} | {4:>8} , {5:>8} |".format(\
                "search_center", str(round(search_center[0], 2)), str(round(search_center[1], 2)),\
                "predict_heading", str(round(predict_heading[0], 2)), str(round(predict_heading[1], 2))))
            search_center, predict_heading = self.best_point(search_center, predict_heading)
            print("| {0} {1:<14} | {2:>8} , {3:>8}    {4:>38} |".format(\
                i, "best_point", str(round(search_center[0], 2)), str(round(search_center[1], 2)), ""))
            self.trajectory = np.append(self.trajectory, np.array([search_center]), axis=0)
        self.trajectory = np.delete(self.trajectory, 0, axis=0)
        print("| {0:<16} | {1:61} |".format("trajectory", ""))
        for x, y in self.trajectory:
            print("| {0:16} | {1:>8} , {2:>8} {3:41} |".format("", str(round(x, 2)), str(round(y, 2)), ""))

    def best_point(self, search_center, predict_heading):
        min_f_value = 10000000
        best_point = np.array([search_center[0] + predict_heading[0] * self.predict_step_size,
                     search_center[1] + predict_heading[1] * self.predict_step_size])
        best_heading = predict_heading

        points = self.predict_step_size * np.array([
            [1, 0], [0.87, 0.50], [0.50, 0.87], [0, 1], [-0.50, 0.87], [-0.87, 0.50],
            [-1, 0], [-0.87, -0.50], [-0.50, -0.87], [0, -1], [0.50, -0.87], [0.87, -0.50]])
        points += search_center

        cnt = 0
        for p in points:
            if self.is_line_out(p):
                continue
            if self.is_obstacle_in(p):
                continue

            cnt += 1
            # g
            distance_to_cur = (p[0] - self.cur_pos[0]) ** 2 + (p[1] - self.cur_pos[1]) ** 2

            # g
            vec = p - search_center
            vec = vec / math.sqrt((vec[0] ** 2 + vec[1] ** 2))
            sin_value = round(predict_heading[0] * vec[1] - predict_heading[1] * vec[0], 5)
            rotate = abs(math.degrees(math.asin(sin_value)))  # degree

            distance_to_search = (p[0] - search_center[0]) ** 2 + (p[1] - search_center[1]) ** 2
            if 0 <= rotate < 30:
                rotate_cost = distance_to_search * self.g_value_rotate_gain_0
            elif 30 <= rotate < 60:
                rotate_cost = distance_to_search * self.g_value_rotate_gain_30
            elif 60 <= rotate < 90:
                rotate_cost = distance_to_search * self.g_value_rotate_gain_60
            else: #if 90 < rotate <= 180
                rotate_cost = distance_to_search * self.g_value_rotate_gain_90

            # g
            g = distance_to_cur + rotate_cost

            # h
            h = ((p[0] - self.next_goal[0]) ** 2 + (p[1] - self.next_goal[1]) ** 2) * self.h_value_gain

            # f
            f = g + h

            # min f
            if min_f_value > f:
                min_f_value = f
                if np.dot(vec, predict_heading) == -1: #not working
                    print("|{:-<82}+".format(" make_trajectory() / best_point() / It Stucked!"))
                    best_point = np.array([search_center[0] + predict_heading[0] * self.predict_step_size,
                     search_center[1] + predict_heading[1] * self.predict_step_size])
                    best_heading = predict_heading
                else:
                    best_point = np.array([p[0], p[1]])
                    best_heading = vec
        print("| make_trajectory() / best_point() / {0} of 8 points are calculated{1:-<18}+".format(cnt, ""))

        return best_point, best_heading

    def is_line_out(self, point):
        crossed = 0
        for i in range(len(self.edge_points)):
            j = (i + 1) % len(self.edge_points)
            if (self.edge_points[i][1] > point[1]) != (self.edge_points[j][1] > point[1]):
                intersection = float((self.edge_points[j][0] - self.edge_points[i][0]) * (
                        point[1] - self.edge_points[i][1]) / (
                                             self.edge_points[j][1] - self.edge_points[i][1]) +
                                     self.edge_points[i][0])
                if point[0] < intersection:
                    crossed = crossed + 1
        return (crossed % 2) == 0 #line out = True

    def is_obstacle_in(self, point):
        if len(self.ob_list) == 0:
            return False

        for i in range(len(self.ob_list)):
            distance_to_ob = math.sqrt((point[0] - self.ob_list[i][0]) ** 2 + (point[1] - self.ob_list[i][1]) ** 2)
            if distance_to_ob < self.obstacle_search_range:
                return True #obstacle in = True

        return False

    def save_past_path(self):
        self.past_path = np.append(self.past_path, np.array([self.cur_pos]), axis=0)
        
    def obstacle_update(self):
        print("|{:-<82}+".format(" obstacle_update()"))
        self.ob_list = self.obstacle.ob_list
        for i in range(len(self.ob_list)):
            bearing = self.bearing
            theta = self.ob_list[i][2]
            x = self.cur_pos[0]
            y = self.cur_pos[1]
            d = math.sqrt(self.ob_list[i][0]**2 + self.ob_list[i][1]**2)
            if 0 <= bearing <= 180: # bearing Quadrant 1, 4
                if 0 <= theta <= (180 - bearing):
                    x += d * math.sin(math.radians(abs(theta) + abs(bearing)))
                    y += d * math.cos(math.radians(abs(theta) + abs(bearing)))
                elif (180 - bearing) <= theta < 180:
                    x += d * math.sin(math.radians(-360 + abs(theta) + abs(bearing)))
                    y += d * math.cos(math.radians(-360 + abs(theta) + abs(bearing)))
                else:
                    x += d * math.sin(math.radians(-abs(theta) + abs(bearing)))
                    y += d * math.cos(math.radians(-abs(theta) + abs(bearing)))
            else: # -180 < bearing < 0 # bearing Quadrant 2, 3
                if 0 <= theta <= 180:
                    x += d * math.sin(math.radians(abs(theta) - abs(bearing)))
                    y += d * math.cos(math.radians(abs(theta) - abs(bearing)))
                elif -180 < theta < (-180 - bearing):
                    x += d * math.sin(math.radians(360 - abs(theta) - abs(bearing)))
                    y += d * math.cos(math.radians(360 - abs(theta) - abs(bearing)))
                else:
                    x += d * math.sin(math.radians(-abs(theta) - abs(bearing)))
                    y += d * math.cos(math.radians(-abs(theta) - abs(bearing)))
            self.ob_list[i][0] = x
            self.ob_list[i][1] = y
        self.ob_list = self.ob_list[:, 0:2]
        print("| {0:<16} | {1:61} |".format("ob_list", ""))
        for x, y in self.ob_list:
            print("| {0:16} | {1:>8} , {2:>8} {3:41} |".format("", str(round(x, 2)), str(round(y, 2)), ""))
    
    def servo_pid_controller(self):
        dx = self.trajectory[0][0] - self.cur_pos[0]
        dy = self.trajectory[0][1] - self.cur_pos[1]
        self.angle = math.degrees(math.atan2(dx, dy))

        if dx >= 0 and dy >= 0: # Quadrant 1
            if self.bearing >= 0 : # right bearing
                error_angle = self.angle - self.bearing
            elif self.bearing < 0: # left bearing
                if abs(self.bearing) < (180 - self.angle):
                    error_angle = self.angle - self.bearing
                elif abs(self.bearing) >= (180 - self.angle):
                    error_angle = -(360 - self.angle + self.bearing)

        elif dx < 0 and dy >= 0: # Quadrant 2
            if self.bearing >= 0 :
                if self.bearing < 180 + self.angle:
                    error_angle = self.angle - self.bearing
                elif self.bearing >= 180 + self.angle:
                    error_angle = 360 + self.angle - self.bearing
            elif self.bearing < 0:
                error_angle = self.angle - self.bearing
                
        elif dx < 0 and dy < 0: # Quadrant 3
            if self.bearing >= 0 :
                if self.bearing < 180 + self.angle:
                    error_angle = (self.angle - self.bearing)
                elif self.bearing >= 180 + self.angle:
                    error_angle = 360 + (self.angle - self.bearing)
            elif self.bearing < 0:
                error_angle = (self.angle - self.bearing)

        elif dx >= 0 and dy < 0: # Quadrant 4
            if self.bearing >= 0 :
                error_angle = (self.angle - self.bearing)
            elif self.bearing < 0:
                if abs(self.bearing) < 180 - self.angle:
                    error_angle = (self.angle - self.bearing)
                elif abs(self.bearing) >= 180 - self.angle:
                    error_angle = self.angle - self.bearing - 360

        self.cp_servo = self.kp_servo * error_angle
        servo_pd = -(self.cp_servo)
        servo_control = self.init_servo + servo_pd

        if servo_control > 118: #94+24
            servo_control = 118
        elif servo_control < 70:# 94-24
            servo_control = 70
        else:
            pass

        self.rotate_angle = self.angle
        self.error_angle = error_angle
        self.servo_control = servo_control

        return servo_control

    def control_publisher(self):
        print("|{:-<82}+".format(" control_publisher()"))
        control = Control()
        control.thruster = self.thruster_power
        control.servo = round(self.servo_pid_controller())
        self.thruster_pub.publish(control.thruster)
        self.servo_pub.publish(control.servo)

    def print_info(self):
        print("+" + "-"*82 + "+")
        print("+" + "-"*82 + "+")
        print("| {0:<16} | {1:>8} , {2:>8} || {3:<16} | {4:>19} |".format(\
            "cur_pos", str(round(self.cur_pos[0], 2)), str(round(self.cur_pos[1], 2)),\
            "bearing", str(round(self.bearing))))
        print("| {0:<16} | {1:>8} , {2:>8} || {3:<39}|".format(\
            "next_goal", str(round(self.next_goal[0], 3)), str(round(self.next_goal[1], 3)), ""))
        print("| {0:<16} | {1:>8} , {2:>8} || {3:<16} | {4:>8} , {5:>8} |".format(\
            "trajectory[0]", str(round(self.trajectory[0][0], 2)), str(round(self.trajectory[0][1], 2)),\
            "heading", str(round(self.cur_heading[0], 2)), str(round(self.cur_heading[1], 2))))
        print("| rotate_angle   {0:>8}  | error_angle   {1:>8}  | servo_control   {2:>11} |".format(str(round(self.rotate_angle)), str(round(self.error_angle)), str(round(self.servo_control))))

    def topic_pub(self):
        info = Info()
        info.curPosX = round(self.cur_pos[0], 2)
        info.curPosY = round(self.cur_pos[1], 2)
        info.bearing = round(self.bearing)
        info.nextGoalX = round(self.next_goal[0], 3)
        info.nextGoalY = round(self.next_goal[1], 3)
        info.trajectory0X = round(self.trajectory[0][0], 2)
        info.trajectory0Y = round(self.trajectory[0][1], 2)
        info.rotateAngle = round(self.rotate_angle)
        info.errorAngle = round(self.error_angle)
        info.servoControl = round(self.servo_control)
        for i in range(len(self.trajectory)):
            info.trajectory.append(EnuTrajectory(x=self.trajectory[i][0], y=self.trajectory[i][1]))
        for i in range(len(self.ob_list)):
            info.obList.append(EnuObstacle(x=round(self.ob_list[i][0], 2), y=round(self.ob_list[i][1], 2)))
        # searchCenterX = 
        # searchCenterY
        # bestPointX
        # bestPointY
        # calcPointNum
        # float64 headingX
        # float64 headingY
        self.info_pub.publish(info)

if __name__ == '__main__':
    rospy.init_node('Path_Plan', anonymous=False)
    pp = PathPlanner()
    rate = rospy.Rate(10)
    hz = 1 # hz * 0.1 sec
    time.sleep(5)
    pp.start_pos = pp.boat.cur_pos
    while not rospy.is_shutdown():
        pp.cur_pos = pp.boat.cur_pos
        pp.cur_heading = pp.boat.cur_heading
        pp.bearing = pp.boat.bearing

        hz -= 1
        
        if pp.is_finished():
            print("|{:-<82}+".format(" main loop / Arrived Next Goal"))
            if len(pp.end_points) > 1:
                print("|{:-<82}+".format(" main loop / Arrived Next Goal / Set Next Goal"))
                pp.end_points = np.delete(pp.end_points, 0, axis=0)
                pp.next_goal = pp.end_points[0]
            elif len(pp.end_points) == 1:
                print("|{:-<82}+".format(" main loop / Arrived Next Goal / Set Next Goal"))
                pp.next_goal = pp.end_points[0]
                pp.end_points = np.delete(pp.end_points, 0, axis=0)
            else:
                print("|{:-<82}+".format(" main loop / Arrived Next Goal / Arrived Final Goal"))
                pp.thruster_pub.publish(1500)
                pp.servo_pub.publish(93)
                break
        else:
            print("|{:-<82}+".format(" main loop / Not Arrived"))
            pp.obstacle_update()
            if hz < 1:
                pp.make_trajectory()
                hz = 10
            pp.control_publisher()
            
        pp.print_info()
        pp.topic_pub()

        rate.sleep()

    rospy.spin()